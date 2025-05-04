#include "myrobot.h"
#include "kinematics.h"
#include "poseseq.h"
#include "planner_wholebody.h"
#include "planner_centroid.h"

#include <rollpitchyaw.h>

#include <cnoid/ExecutablePath>

using namespace std;

namespace dymp{
namespace mpc{

MyRobot::MyRobot(){
    base_state_from_simulator = true;
    base_actuation            = false;

    timer.dt = 0.001;
    param.control_cycle = 1;
    param.nominal_inertia = Vector3(10,10,10);

    kinematics       = new Kinematics();
    poseseq          = new Poseseq();
    planner_wb       = new PlannerWholebody();
    planner_centroid = new PlannerCentroid();

    kinematics      ->robot = this;
    poseseq         ->robot = this;
    planner_wb      ->robot = this;
    planner_centroid->robot = this;
}

void MyRobot::Read(const YAML::Node& node){
    ReadDouble (param.com_height     , node["com_height"]     );
    ReadDouble (param.total_mass     , node["total_mass"]     );
	ReadVector3(param.nominal_inertia, node["nominal_inertia"]);
    ReadDouble(timer.dt, node["dt"]);
    ReadInt(param.control_cycle, node["control_cycle"]);
    ReadInt(save_cycle, node["save_cycle"]);

    vector<double> pgain, dgain, ulim;
    ReadVectorDouble(pgain, node["pgain"]);
    ReadVectorDouble(dgain, node["dgain"]);
    ReadVectorDouble(ulim , node["ulim" ]);
    for(int i = 0; i < (int)joint.size(); i++){
        joint[i].Set(pgain[i], dgain[i], ulim[i]);
    }
}

void MyRobot::Init(cnoid::SimpleControllerIO* io){
    // two hands and two feet
    foot.resize(2);
    hand.resize(2);

    // 30 joints
    joint.resize(32);
    
    string path = cnoid::shareDir() + "/project/dymp_mpc/dance_controller/config/";
    string robotFilename            = path + "robot.yaml";
    string plannerWholebodyFilename = path + "planner_wholebody.yaml";
    string plannerCentroidFilename  = path + "planner_centroid.yaml";
    string pseqConfFilename         = path + "poseseq.yaml";
    string kinematicsFilename       = path + "kinematics.yaml";

    Read(YAML::LoadFile(robotFilename));

    planner_wb      ->Read(YAML::LoadFile(plannerWholebodyFilename));
    planner_centroid->Read(YAML::LoadFile(plannerCentroidFilename));
    
    kinematics->Read(YAML::LoadFile(kinematicsFilename));

    poseseq->Read(YAML::LoadFile(pseqConfFilename));
    poseseq->Load(YAML::LoadFile(cnoid::shareDir() + "/motion/" + poseseq->pseqFile));

    Robot::Init(io, timer, joint);

    planner_centroid->CreateThreads();
    planner_wb      ->CreateThreads();

    poseseq         ->Init();
    
    planner_centroid->InitState();
    planner_wb      ->InitState();
    
    planner_centroid->InitThreads();
    planner_wb      ->InitThreads();

    planner_centroid->RunThreads();
    planner_wb      ->RunThreads();
    
    viz = new cnoid::vnoid::Visualizer();
    viz->header.numMaxFrames       = 1500;
    viz->header.numMaxLines        = 10;
    viz->header.numMaxSpheres      = 100;
    viz->header.numMaxBoxes        = 100;
    viz->header.numMaxCylinders    = 100;
    viz->header.numMaxLineVertices = 4000;
    viz->Open();

    const char* basename[] = {"log_psq", "log_cen", "log_ref", "log_sim"};
    stringstream ss;
    for(int i = 0; i < 4; i++){
        ss.str("");
        ss << basename[i] << ".csv";
        fileLog[i] = fopen(ss.str().c_str(), "w");
        fprintf(fileLog[i],
            "time, "
            "com_x, com_y, com_z, "
            "vel_x, vel_y, vel_z, "
            "ori_x, ori_y, ori_z, "
            "angvel_x, angvel_y, angvel_z, "
            "mom_abs_x, mom_abs_y, mom_abs_z, "
            "mom_x, mom_y, mom_z, "
            "zmp_x, zmp_y, zmp_z, "
        );
        for(int j = 0; j < 2; j++){
            fprintf(fileLog[i],
                "foot%d_pos_x, foot%d_pos_y, foot%d_pos_z, "
                "foot%d_ori_x, foot%d_ori_y, foot%d_ori_z, "
                "foot%d_force_x, foot%d_force_y, foot%d_force_z, "
                "foot%d_moment_x, foot%d_moment_y, foot%d_moment_z, "
                "foot%d_zmp_x, foot%d_zmp_y, foot%d_zmp_z, "
                "foot%d_state, ",
                j, j, j,
                j, j, j,
                j, j, j,
                j, j, j,
                j, j, j,
                j
            );
        }
        fprintf(fileLog[i], "\n");

        for(int j = 0; j < 2; j++){
            ss.str("");
            ss << basename[i] << "_step" << j << ".csv";
            fileFootstep[i][j] = fopen(ss.str().c_str(), "w");
            contactPrev[i][j] = false;
        }
    }

    fileTiming = fopen("timing.csv", "w");

    FILE* file = fopen("disturbance.csv", "r");
    disturbance_magnitude = 0.0;
    disturbance_angle     = 0.0;
    disturbance_time      = 0.0;
    disturbance_duration  = 0.0;
    fscanf(file, "%lf %lf %lf %lf", &disturbance_magnitude, &disturbance_angle, &disturbance_time, &disturbance_duration);
    fclose(file);

    tprev = 0.0;
    tprevSwitch = -1.0;
}

void MyRobot::Visualize(const cnoid::vnoid::Timer& timer){
    cnoid::vnoid::VizInfo  info;
    info.iframe    = timer.time/0.025;
    info.ilines    = 0;
    info.isphere   = 0;
    info.ibox      = 0;
    info.icylinder = 0;
    cnoid::vnoid::Visualizer::Frame* fr = viz->data->GetFrame(info.iframe);
    fr->time = timer.time;

    // visualize wholebody
    planner_wb->Visualize(viz, info);
    planner_centroid->Visualize(viz, info);
}

void MyRobot::Control(){
    if(timer.count % param.control_cycle == 0){
        Sense(timer, base, foot, joint);

        // set current state of robot and restart mpc
        planner_wb->FromRobot();
        planner_centroid->FromRobot();
    
        // get control input from mpc
        planner_wb->ToRobot();
        planner_centroid->ToRobot();

        Visualize(timer);

        Robot::Actuate(timer, base, joint);

        cnoid::Link* target = io_body->link(0);
        if( disturbance_time <= timer.time && timer.time <= disturbance_time + disturbance_duration ){
            Vector3 f = (param.total_mass/(param.T*disturbance_duration))*disturbance_magnitude*Vector3(cos((3.1415/180.0)*disturbance_angle), sin((3.1415/180.0)*disturbance_angle), 0.0);
            target->f_ext  () = f;
	        target->tau_ext() = target->p().cross(f);
        }
        else{
            target->f_ext  () = Vector3::Zero();
	        target->tau_ext() = Vector3::Zero();
        }
        /*
        */

        timer.control_count++;
    }
    if(timer.count % save_cycle == 0){
        SaveLog();
    }
	timer.Countup();
}

void MyRobot::SaveLog(){
    // get centroidal states from simulator
    io_body->calcForwardKinematics(true);
	Vector3 com_pos_sim = io_body->calcCenterOfMass();
	Vector3 P, momentum_sim;
    io_body->calcTotalMomentum(P, momentum_sim);

    //
    dymp::Wholebody* wb = planner_wb->GetWholebody();
    poseseq->Setup(timer.time, wb, planner_wb->data_des_poseseq);
    poseseq->Setup(timer.time, wb, planner_wb->data_des_centroid);
        
    // overwrite with centroid if ready
    if(planner_wb->useCentroid && planner_centroid->mpcInputReady){
        kinematics->SetupFromCentroid(
            timer.time,
            planner_centroid->GetCentroid(), 
            planner_centroid->data_traj,
            wb, planner_wb->data_des_centroid);
    }

    for(int i = 0; i < 4; i++){
        fprintf(fileLog[i],
            "%f, ",
            timer.time
        );
        dymp::vec3_t p, v, a, w, L_abs, L_local, c;
        dymp::quat_t q;
        dymp::vec3_t pe[2], ae[2], fe[2], me[2], ce[2];
        dymp::quat_t qe[2];
        bool   se[2];

        dymp::WholebodyData* d;
        if(i == 0){
            d = &planner_wb->data_des_poseseq;
        }
        if(i == 1){
            d = &planner_wb->data_des_centroid;
        }
        if(i == 2){
            d = &planner_wb->data_ref;
        }
        if(i == 3){
            d = &planner_wb->data_cur;
        }
        p    = d->centroid.pos_t;
        v    = d->centroid.vel_t;
        q    = d->centroid.pos_r;
        a    = cnoid::vnoid::ToRollPitchYaw(q);
        w    = d->centroid.vel_r;
        L_abs   = d->centroid.L_abs;
        L_local = d->centroid.L_local;
        
        for(int j = 0; j < 2; j++){
            dymp::WholebodyData::End& dend = d->ends[j];
            if(i == 0 || i == 1){
                pe[j] = dend.pos_t_abs;
                qe[j] = dend.pos_r_abs;
            }
            else{
                pe[j] = p + q*dend.pos_t;
                qe[j] = q*dend.pos_r;
            }
            if(i == 3){
                // actual force from simulator
                fe[j] = dend.pos_r*foot[j].force ;
                me[j] = dend.pos_r*foot[j].moment;
            }
            else{
                fe[j] = dend.force_t;
                me[j] = dend.force_r;
            }
            ae[j] = cnoid::vnoid::ToRollPitchYaw(qe[j]);
            //se[j] = (dend.state != DiMP::Wholebody::ContactState::Free);
            const dymp::real_t fzmin = 10.0;
            se[j] = (fe[j].z() > fzmin);
            if(se[j])
                 ce[j] = dymp::vec3_t(-me[j].y()/fe[j].z(), me[j].x()/fe[j].z(), 0.0);
            else ce[j] = dymp::zero3;
        }

        if(i == 0){
            dymp::vec3_t f  = wb->param.totalMass*((v - vprev[i])/(timer.time - tprev) + dymp::vec3_t(0.0, 0.0, wb->param.gravity));
            dymp::vec3_t m  = (L_abs - Lprev[i])/(timer.time - tprev);

            /*
            (c - p) % f = m
                (cy - py)*fz - (cz - pz)*fy = mx
            -(cx - px)*fz + (cz - pz)*fx = my
            cx = px + (-my - pz*fx)/fz
            cy = py + ( mx - pz*fy)/fz
            */
            c.x() = p.x() + (-m.y() - p.z()*f.x())/f.z();
            c.y() = p.y() + ( m.x() - p.z()*f.y())/f.z();
            c.z() = 0.0;

            vprev[i] = v;
            Lprev[i] = L_abs;
        }
        else{
            // calculate zmp from wrench
            dymp::real_t fzsum = fe[0].z() + fe[1].z();
            c = (fe[0].z()/fzsum)*(pe[0] + ce[0]) + (fe[1].z()/fzsum)*(pe[1] + ce[1]);
        }

        fprintf(fileLog[i],
            "%f, %f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, ",
            p.x(), p.y(), p.z(),
            v.x(), v.y(), v.z(),
            a.x(), a.y(), a.z(),
            w.x(), w.y(), w.z(),
            L_abs.x(), L_abs.y(), L_abs.z(),
            L_local.x(), L_local.y(), L_local.z(),
            c.x(), c.y(), c.z()
        );
        for(int j = 0; j < 2; j++){
            fprintf(fileLog[i],
                "%f, %f, %f, "
                "%f, %f, %f, "
                "%f, %f, %f, "
                "%f, %f, %f, "
                "%f, %f, %f, "
                "%d, ",
                pe[j].x(), pe[j].y(), pe[j].z(),
                ae[j].x(), ae[j].y(), ae[j].z(),
                fe[j].x(), fe[j].y(), fe[j].z(),
                me[j].x(), me[j].y(), me[j].z(),
                pe[j].x() + ce[j].x(), pe[j].y() + ce[j].y(), pe[j].z() + ce[j].z(),
                se[j]
            );
        }
        fprintf(fileLog[i], "\n");
        fflush(fileLog[i]);

        dymp::vec3_t footprint[4];
        footprint[0] = dymp::vec3_t( 0.145,  0.06, 0.0);
        footprint[1] = dymp::vec3_t( 0.145, -0.06, 0.0);
        footprint[2] = dymp::vec3_t(-0.105, -0.06, 0.0);
        footprint[3] = dymp::vec3_t(-0.105,  0.06, 0.0);
        for(int j = 0; j < 2; j++){
            if(!contactPrev[i][j] && se[j]){
                for(int k = 0; k <= 4; k++){
                    dymp::vec3_t v = pe[j] + qe[j]*footprint[k%4];
                    fprintf(fileFootstep[i][j], "%f, %f, %f\n", timer.time, v.x(), v.y());
                }
                fprintf(fileFootstep[i][j], "\n");
                fflush(fileFootstep[i][j]);
            }
            contactPrev[i][j] = se[j];
        }
    }

    if(planner_wb->useCentroid && planner_centroid->mpcInputReady){
        dymp::real_t t = planner_centroid->data_traj[0].time;
        dymp::vec3_t L = planner_centroid->data_traj[0].L;
        if(t > tprevSwitch){
            fprintf(fileTiming, "%f, %f, %f, %f\n", t, L.x(), L.y(), L.z());
            tprevSwitch = t;
        }
    }
    
    tprev = timer.time;
}

}
}
