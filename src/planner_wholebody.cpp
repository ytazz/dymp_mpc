#include "myrobot.h"
#include "planner_wholebody.h"
#include "planner_centroid.h"
#include "kinematics.h"

namespace dymp{
namespace mpc{

PlannerThreadWholebody::PlannerThreadWholebody(Planner* _planner):PlannerThread(_planner){
    planner_wb = (PlannerWholebody*)_planner;

	wb    = new dymp::Wholebody(world, "wholebody");
    wb->param.nominalInertia = planner->robot->param.nominal_inertia;
    wb->param.useLd      = planner_wb->useLd;
    wb->param.inputMode  = planner_wb->inputMode;
    wb->param.dt         = planner_wb->mpcTimestep;
    wb->callback         = planner_wb;

    planner->robot->kinematics->Init(wb);

    wb->SetScaling();
}

void PlannerThreadWholebody::Init(){
    PlannerThread::Init();
    //graph->solver->Enable(ID(DiMP::ConTag::WholebodyPosT         ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::WholebodyPosR         ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::WholebodyVelT         ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::WholebodyVelR         ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::WholebodyAccT         ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::WholebodyAccR         ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::WholebodyForceT       ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::WholebodyForceR       ), false);
    //graph->solver->Enable(ID(DiMP::ConTag::WholebodyLimit        ), false);
	//world->solver->Enable(dymp::ID(dymp::ConTag::WholebodyContactPosT  ), false);
	//world->solver->Enable(dymp::ID(dymp::ConTag::WholebodyContactPosR  ), false);
	//world->solver->Enable(dymp::ID(dymp::ConTag::WholebodyContactVelT  ), false);
	//world->solver->Enable(dymp::ID(dymp::ConTag::WholebodyContactVelR  ), false);
	//world->solver->Enable(dymp::ID(dymp::ConTag::WholebodyNormalForce  ), false);
	//world->solver->Enable(dymp::ID(dymp::ConTag::WholebodyFrictionForce), false);
	//world->solver->Enable(dymp::ID(dymp::ConTag::WholebodyMoment       ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::WholebodyMomentum       ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::WholebodyComPos       ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::WholebodyComVel       ), false);

    //graph->solver->ddpCallback = this;
    nx = planner_wb->nx;
    nu = planner_wb->nu;
    nj = (nx - 12)/2;

    // timestep is one after scaling
    h  = 1.0;//planner_wb->mpcTimestep;
    h2 = h*h;
}

void PlannerThreadWholebody::Setup(){
    wb->Shift(/*planner->mpcUpdateCycle*planner->robot->timer.dt*/planner->mpcTimestep);
	wb->Setup();
    wb->Reset(false);
    //wb->Reset(true);
    world->solver->InitDDP();

    if(planner_wb->useCentroid && planner_wb->robot->planner_centroid->mpcInputReady){
        // use custom quadratic weight for terminal cost
        int N  = planner_wb->mpcPredictionSteps;
        int ny = world->solver->cost[N]->dim;
        world->solver->cost[N]->useQuadWeight = true;
        world->solver->cost[N]->Vy .Allocate(ny);
        world->solver->cost[N]->Vyy.Allocate(ny, ny);

        // get value function from centroidal mpc
        dymp::real_t tf = planner_wb->mpcInitialTime + N*planner_wb->mpcTimestep;
        planner_wb->robot->planner_centroid->CalcQuadWeight(
            tf,
            world->solver->cost[N]->Vconst,
            world->solver->cost[N]->Vy,
            world->solver->cost[N]->Vyy
        );

        // apply scale conversion and terminal weight
        for(int i = 0; i < ny; i++){
            world->solver->cost[N]->Vy(i) *= planner_wb->terminalWeight*planner_wb->sconv(i);
            for(int j = 0; j < ny; j++){
                world->solver->cost[N]->Vyy(i,j) *= planner_wb->terminalWeight*planner_wb->sconv(i)*planner_wb->sconv(j);
            }
        }
    }
}

void PlannerThreadWholebody::mat_fx_mul  (dymp::Matrix& V, dymp::Matrix& fx, dymp::Matrix& y, bool add){
    mat_mat_mul(V.SubMatrix(0,0,nx,12), fx.SubMatrix(0,0,12,nx), y, 1.0, (add ? 1.0 : 0.0));
    mat_add(V.SubMatrix(0,12,nx,nx-12), y.SubMatrix(0,12,nx,nx-12));
    mat_add(V.SubMatrix(0,12,nx,nj), y.SubMatrix(0,12+nj,nx,nj), h);
}
void PlannerThreadWholebody::mat_fu_mul  (dymp::Matrix& V, dymp::Matrix& fu, dymp::Matrix& y, bool add){
    mat_mat_mul(V.SubMatrix(0,0,nx,12), fu.SubMatrix(0,0,12,nu), y, 1.0, (add ? 1.0 : 0.0));
    mat_add(V.SubMatrix(0,12+nj,nx,nj), y.SubMatrix(0,6,nx,nj), h);
    mat_add(V.SubMatrix(0,12,nx,nj), y.SubMatrix(0,6,nx,nj), h2/2);
}
void PlannerThreadWholebody::fxtr_mat_mul(dymp::Matrix& fx, dymp::Matrix& V, dymp::Matrix& y, bool add){
    mattr_mat_mul(fx.SubMatrix(0,0,12,nx), V.SubMatrix(0,0,12,nx), y, 1.0, (add ? 1.0 : 0.0));
    mat_add(V.SubMatrix(12,0,nx-12,nx), y.SubMatrix(12,0,nx-12,nx));
    mat_add(V.SubMatrix(12,0,nj,nx), y.SubMatrix(12+nj,0,nj,nx), h);    
}
void PlannerThreadWholebody::futr_mat_mul(dymp::Matrix& fu, dymp::Matrix& V, dymp::Matrix& y, bool add){
    mattr_mat_mul(fu.SubMatrix(0,0,12,nu), V.SubMatrix(0,0,12,V.n), y, 1.0, (add ? 1.0 : 0.0));
    mat_add(V.SubMatrix(12+nj,0,nj,V.n), y.SubMatrix(6,0,nj,V.n), h);
    mat_add(V.SubMatrix(12,0,nj,V.n), y.SubMatrix(6,0,nj,V.n), h2/2);
}
void PlannerThreadWholebody::fxtr_vec_mul(dymp::Matrix& fx, dymp::Vector& v, dymp::Vector& y, bool add){
    mattr_vec_mul(fx.SubMatrix(0,0,12,nx), v.SubVector(0,12), y, 1.0, (add ? 1.0 : 0.0));
    vec_add(v.SubVector(12,nx-12), y.SubVector(12,nx-12));
    vec_add(v.SubVector(12,nj), y.SubVector(12+nj,nj), h);
}
void PlannerThreadWholebody::futr_vec_mul(dymp::Matrix& fu, dymp::Vector& v, dymp::Vector& y, bool add){
    mattr_vec_mul(fu.SubMatrix(0,0,12,nu), v.SubVector(0,12), y, 1.0, (add ? 1.0 : 0.0));
    vec_add(v.SubVector(12+nj,nj), y.SubVector(6,nj), h);
    vec_add(v.SubVector(12,nj), y.SubVector(6,nj), h2/2);
}

PlannerWholebody::PlannerWholebody(){
    name = "planner_wholebody";
    useLd       = true;
    //useJerk     = false;
    inputMode   = dymp::Wholebody::InputMode::Acceleration;
    usePoseseq  = true;
    useCentroid = true;

    initialWeight      = 1.0;
    terminalWeight     = 1.0;
	freeWeight         = 1.0;
	centroidPosWeight    = Vector3(1.0, 1.0, 1.0);
	centroidVelWeight    = Vector3(1.0, 1.0, 1.0);
	centroidOriWeight    = Vector3(1.0, 1.0, 1.0);
	centroidAngvelWeight = Vector3(1.0, 1.0, 1.0);
    centroidLWeight      = Vector3(1.0, 1.0, 1.0);
    torsoOriWeight       = Vector3(1.0, 1.0, 1.0);
	torsoAngvelWeight    = Vector3(1.0, 1.0, 1.0);
	torsoAngaccWeight    = Vector3(1.0, 1.0, 1.0);
	footPosWeight        = Vector3(1.0, 1.0, 1.0);
	footOriWeight        = Vector3(1.0, 1.0, 1.0);
	footVelWeight        = Vector3(1.0, 1.0, 1.0);
	footAngvelWeight     = Vector3(1.0, 1.0, 1.0);
	footAccWeight        = Vector3(1.0, 1.0, 1.0);
	footAngaccWeight     = Vector3(1.0, 1.0, 1.0);
	handPosWeight        = Vector3(1.0, 1.0, 1.0);
	handOriWeight        = Vector3(1.0, 1.0, 1.0);
	handVelWeight        = Vector3(1.0, 1.0, 1.0);
	handAngvelWeight     = Vector3(1.0, 1.0, 1.0);
	handAccWeight        = Vector3(1.0, 1.0, 1.0);
	handAngaccWeight     = Vector3(1.0, 1.0, 1.0);
	endForceWeight       = Vector3(1.0, 1.0, 1.0);
	endMomentWeight      = Vector3(1.0, 1.0, 1.0);
}

PlannerThread* PlannerWholebody::CreateThread(){
	return new PlannerThreadWholebody(this);
}

void PlannerWholebody::Read(const YAML::Node& node){
    Planner::Read(node);

    const YAML::Node& wbNode = node["wholebody"];

    ReadBool(useLd      , wbNode["use_Ld"]);
    
    string str;
    ReadString(str, wbNode["input_mode"]);
    if(str == "velocity"    ) inputMode = dymp::Wholebody::InputMode::Velocity;
    if(str == "acceleration") inputMode = dymp::Wholebody::InputMode::Acceleration;
    if(str == "jerk"        ) inputMode = dymp::Wholebody::InputMode::Jerk;
    //ReadBool(useJerk    , wbNode["use_jerk"]);

    ReadBool(usePoseseq , wbNode["use_poseseq"]);
    ReadBool(useCentroid, wbNode["use_centroid"]);

    ReadDouble(initialWeight        , wbNode["initial_weight" ]);
	ReadDouble(terminalWeight       , wbNode["terminal_weight"]);
	ReadDouble(freeWeight           , wbNode["free_weight"    ]);
	ReadVector3(centroidPosWeight   , wbNode["centroid_pos_weight"   ]);
	ReadVector3(centroidVelWeight   , wbNode["centroid_vel_weight"   ]);
	ReadVector3(centroidOriWeight   , wbNode["centroid_ori_weight"   ]);
	ReadVector3(centroidAngvelWeight, wbNode["centroid_angvel_weight"]);
    ReadVector3(centroidLWeight     , wbNode["centroid_L_weight"     ]);
	ReadVector3(torsoOriWeight      , wbNode["torso_ori_weight"      ]);
	ReadVector3(torsoAngvelWeight   , wbNode["torso_angvel_weight"   ]);
	ReadVector3(torsoAngaccWeight   , wbNode["torso_angacc_weight"   ]);
	ReadVector3(footPosWeight       , wbNode["foot_pos_weight"       ]);
	ReadVector3(footOriWeight       , wbNode["foot_ori_weight"       ]);
	ReadVector3(footVelWeight       , wbNode["foot_vel_weight"       ]);
	ReadVector3(footAngvelWeight    , wbNode["foot_angvel_weight"    ]);
	ReadVector3(footAccWeight       , wbNode["foot_acc_weight"       ]);
	ReadVector3(footAngaccWeight    , wbNode["foot_angacc_weight"    ]);
	ReadVector3(handPosWeight       , wbNode["hand_pos_weight"       ]);
	ReadVector3(handOriWeight       , wbNode["hand_ori_weight"       ]);
	ReadVector3(handVelWeight       , wbNode["hand_vel_weight"       ]);
	ReadVector3(handAngvelWeight    , wbNode["hand_angvel_weight"    ]);
	ReadVector3(handAccWeight       , wbNode["hand_acc_weight"       ]);
	ReadVector3(handAngaccWeight    , wbNode["hand_angacc_weight"    ]);
	ReadVector3(endForceWeight      , wbNode["end_force_weight"      ]);
	ReadVector3(endMomentWeight     , wbNode["end_moment_weight"     ]);
    ReadVectorDouble(qWeight        , wbNode["q_weight"   ]);
    ReadVectorDouble(qdWeight       , wbNode["qd_weight"  ]);
    ReadVectorDouble(qddWeight      , wbNode["qdd_weight" ]);
    ReadVectorDouble(qdddWeight     , wbNode["qddd_weight" ]);
    //ReadVectorDouble(qMin           , wbNode["q_min"      ]);
    //ReadVectorDouble(qMax           , wbNode["q_max"      ]);
    //ReadVectorDouble(qdMin          , wbNode["qd_min"     ]);
    //ReadVectorDouble(qdMax          , wbNode["qd_max"     ]);
    //ReadVectorDouble(qddMin         , wbNode["qdd_min"    ]);
    //ReadVectorDouble(qddMax         , wbNode["qdd_max"    ]);
    ReadVectorDouble(qRangeWeight   , wbNode["q_range_weight"   ]);
    ReadVectorDouble(qdRangeWeight  , wbNode["qd_range_weight"  ]);
    ReadVectorDouble(qddRangeWeight , wbNode["qdd_range_weight" ]);
}

dymp::Wholebody* PlannerWholebody::GetWholebody(){
    return ((PlannerThreadWholebody*)threads[threadIndex])->wb;
}

void PlannerWholebody::InitState(){
    dymp::Wholebody* wb = GetWholebody();

    int nend   = (int)wb->ends  .size();
    int njoint = (int)wb->joints.size();

    nx = 12;
    if(wb->param.inputMode == dymp::Wholebody::InputMode::Velocity)
        nx += njoint;
    if(wb->param.inputMode == dymp::Wholebody::InputMode::Acceleration)
        nx += 2*njoint;
    if(wb->param.inputMode == dymp::Wholebody::InputMode::Jerk)
        nx += 3*njoint;

    nu =  6 + njoint;
    for(int i = 0; i < nend; i++){
        nu += ((wb->ends[i].enableForce ? 3 : 0) + (wb->ends[i].enableMoment ? 3 : 0));
    }
    
    dx.Allocate(nx);
    du.Allocate(nu);
    u .Allocate(nu);
    Quuinv_Qux.Allocate(nu, nx);
    mat_clear(Quuinv_Qux);

    int N = mpcPredictionSteps;
    data_cur.Init(wb);
    data_cur.InitJacobian(wb);
    data_ref.Init(wb);
    data_des.Init(wb);
    data_des_poseseq .Init(wb);
    data_des_centroid.Init(wb);
    data_traj    .resize(N+1);
    data_traj_des.resize(N+1);
    for(int k = 0; k <= N; k++){
        data_traj    [k].Init(wb);
        data_traj_des[k].Init(wb);
    }

    GetDesiredState(0, 0.0, data_cur);
    GetDesiredState(0, 0.0, data_ref);
    qd_prev.resize(njoint);
    for(int i = 0; i < njoint; i++){
        qd_prev[i] = data_cur.joints[i].qd;
    }
    time_prev = robot->timer.time;

    // calc scaling matrix
    dymp::Centroid* cen = robot->planner_centroid->GetCentroid();
    sconv.Allocate(robot->planner_centroid->nx);
    int idx = 0;
    for(int i = 0; i < 3; i++)sconv(idx++) = wb->scale.pt/cen->scale.pt;
    for(int i = 0; i < 3; i++)sconv(idx++) = wb->scale.pr/cen->scale.pr;
    for(int i = 0; i < 3; i++)sconv(idx++) = wb->scale.vt/cen->scale.vt;
    for(int i = 0; i < 3; i++)sconv(idx++) = wb->scale.L/cen->scale.L;
    //sconv(idx++) = wb->scale.t/cen->scale.t;
    for(int i = 0; i < 2; i++){
        for(int i = 0; i < 3; i++)sconv(idx++) = wb->scale.pt/cen->scale.pt;
        for(int i = 0; i < 3; i++)sconv(idx++) = wb->scale.pr/cen->scale.pr;
    }
}

void PlannerWholebody::Observe(){
    dymp::Wholebody* wb = GetWholebody();
    
    dymp::vec3_t p0 = robot->base.pos;
    dymp::vec3_t v0 = robot->base.vel;
    dymp::quat_t q0 = robot->base.ori;
    dymp::vec3_t w0 = robot->base.angvel;
    data_cur.centroid.pos_r = q0;
    data_cur.centroid.vel_r = w0;

    int njoint = (int)wb->joints.size();
    for(int i = 0; i < njoint; i++){
        data_cur.joints[i].q  = robot->joint[i].q;
        data_cur.joints[i].qd = robot->joint[i].dq;
    }
    for(int i = 0; i < 2; i++){
        dymp::WholebodyData::End& dend = data_cur.ends[i];

        dend.force_t = dend.pos_r*robot->foot[i].force ;
        dend.force_r = dend.pos_r*robot->foot[i].moment;
    }
    /*
    */
    // pb_abs = pc + qb*pb
    // vb_abs = vc + qb*vb + wb % qb*pb

    wb->CalcPosition(data_cur);
    wb->CalcVelocity(data_cur);

    data_cur.centroid.pos_t = p0 - q0*data_cur.links[0].pos_t;
    data_cur.centroid.vel_t = v0 - q0*data_cur.links[0].vel_t - w0.cross(q0*data_cur.links[0].pos_t);

    wb->CalcAcceleration           (data_cur);
    wb->CalcInertia                (data_cur);
    wb->CalcInertiaDerivative      (data_cur);
    wb->CalcLocalMomentum          (data_cur);
    wb->CalcLocalMomentumDerivative(data_cur);
    wb->CalcAbsoluteMomentum       (data_cur);

    //DSTR << "com: " << data_cur.centroid.pos_t << endl;
}

void PlannerWholebody::InterpolateReference(){
    dymp::Wholebody* wb = GetWholebody();
    dymp::WholebodyData& d0 = data_traj[(mpcDelayMode ? 1 : 0)];
    dymp::WholebodyData& d1 = data_traj[(mpcDelayMode ? 2 : 1)];
    
    int nend   = (int)wb->ends  .size();
    int njoint = (int)wb->joints.size(); 

    dymp::real_t t   = std::min(std::max(0.0, robot->timer.time - mpcInitialTime), mpcTimestep);
    dymp::real_t h   = mpcTimestep;
    dymp::real_t t2  = t*t;
    dymp::real_t t3  = t*t2;
    dymp::real_t s   = t/h;
    
    {
        dymp::vec3_t pc = d0.centroid.pos_t;
        dymp::vec3_t vc = d0.centroid.vel_t;
        dymp::vec3_t ac = (d1.centroid.vel_t - d0.centroid.vel_t)/h;

        dymp::quat_t qc = d0.centroid.pos_r;
        dymp::vec3_t wc = d0.centroid.vel_r;
        dymp::vec3_t uc = (d1.centroid.vel_r - d0.centroid.vel_r)/h;
    
        data_ref.centroid.pos_t = pc + vc*t + (0.5*t2)*ac;
        data_ref.centroid.vel_t = vc + ac*t;
    
        //data_ref.centroid.pos_r = quat_t::Rot(t*wc + (0.5*t2)*uc)*qc;
        data_ref.centroid.pos_r = qc*dymp::rot_quat(qc.conjugate()*(t*wc + (0.5*t2)*uc));
        data_ref.centroid.vel_r = wc + t*uc;
    }

    for(int i = 0; i < njoint; i++){
        dymp::WholebodyData::Joint& djnt_ref  = data_ref.joints[i];
        dymp::WholebodyData::Joint& djnt_ref0 = d0      .joints[i];
        djnt_ref.q    = djnt_ref0.q    + djnt_ref0.qd  *t + djnt_ref0.qdd *(0.5*t2) + djnt_ref0.qddd*((1.0/6.0)*t3);
        djnt_ref.qd   = djnt_ref0.qd   + djnt_ref0.qdd *t + djnt_ref0.qddd*(0.5*t2);
        djnt_ref.qdd  = djnt_ref0.qdd  + djnt_ref0.qddd*t;
        djnt_ref.qddd = djnt_ref0.qddd;
    }

    for(int i = 0; i < nend; i++){
        dymp::WholebodyData::End & dend_ref  = data_ref.ends[i];
        dymp::WholebodyData::End & dend_ref0 = d0      .ends[i];

        dend_ref.state   = dend_ref0.state;
        dend_ref.force_t = dend_ref0.force_t;
        dend_ref.force_r = dend_ref0.force_r;
    }

    wb->CalcPosition(data_ref);
    wb->CalcVelocity(data_ref);
    wb->CalcInertia(data_ref);
    wb->CalcLocalMomentum(data_ref);
    wb->CalcAbsoluteMomentum(data_ref);
}

void PlannerWholebody::UpdateInput(){
    dymp::Wholebody* wb = GetWholebody();
    dymp::WholebodyData& ddes0 = data_traj_des[(mpcDelayMode ? 1 : 0)];
    dymp::WholebodyData& ddes1 = data_traj_des[(mpcDelayMode ? 2 : 1)];
    
    int nend   = (int)wb->ends  .size();
    int njoint = (int)wb->joints.size(); 
    int idx;

    // calc dx
    idx = 0;
    vec_clear(dx);

    subvec3_set(dx, idx, (data_cur.centroid.pos_t - data_ref.centroid.pos_t)/wb->scale.pt);
    subvec3_set(dx, idx, quat_diff(data_ref.centroid.pos_r, data_cur.centroid.pos_r)/wb->scale.pr);
    subvec3_set(dx, idx, (data_cur.centroid.vel_t - data_ref.centroid.vel_t)/wb->scale.vt);
    subvec3_set(dx, idx, (data_cur.centroid.L_abs - data_ref.centroid.L_abs)/wb->scale.L);
    //subvec3_set(dx, idx, (data_cur.centroid.vel_r - data_ref.centroid.vel_r)/wb->scale.vr);

    for(int i = 0; i < njoint; i++){
        dx(idx++) = (data_cur.joints[i].q - data_ref.joints[i].q)/wb->scale.pr;
    }
    if( wb->param.inputMode == dymp::Wholebody::InputMode::Acceleration ||
        wb->param.inputMode == dymp::Wholebody::InputMode::Jerk ){
        for(int i = 0; i < njoint; i++){
            dx(idx++) = (data_cur.joints[i].qd - data_ref.joints[i].qd)/wb->scale.vr;
        }
    }
    if(wb->param.inputMode == dymp::Wholebody::InputMode::Jerk){
        for(int i = 0; i < njoint; i++){
            dx(idx++) = (data_cur.joints[i].qdd - data_ref.joints[i].qdd)/wb->scale.ar;
        }
    }

    // calc u
    mat_vec_mul(Quuinv_Qux, dx, du, (mpcFeedback ? -1.0 : 0.0), 0.0);

    // decode u
    idx = 0;

    // skip virtual acceleration inputs
    idx += 6;

    if(wb->param.inputMode == dymp::Wholebody::InputMode::Velocity){
        for(int i = 0; i < njoint; i++){
            data_cur.joints[i].qd = data_ref.joints[i].qd + wb->scale.vr*du(idx++);
            
            // enforce joint velocity limit
            //data_cur.joints[i].qd = std::min(std::max(ddes0.joints[i].qd_min, data_cur.joints[i].qd), ddes0.joints[i].qd_max);
            data_cur.joints[i].qd = std::min(std::max(wb->joints[i].vel_range[0], data_cur.joints[i].qd), wb->joints[i].vel_range[1]);

            // calc acceleration by difference
            data_cur.joints[i].qdd = (data_cur.joints[i].qd - qd_prev[i])/(robot->timer.time - time_prev);
            qd_prev[i] = data_cur.joints[i].qd;
        }
        time_prev = robot->timer.time;
    }
    if(wb->param.inputMode == dymp::Wholebody::InputMode::Acceleration){
        for(int i = 0; i < njoint; i++){
            data_cur.joints[i].qdd = data_ref.joints[i].qdd + wb->scale.ar*du(idx++);
            
            // enforce joint acceleration limit
            //data_cur.joints[i].qdd = std::min(std::max(ddes0.joints[i].qdd_min, data_cur.joints[i].qdd), ddes0.joints[i].qdd_max);
            data_cur.joints[i].qdd = std::min(std::max(wb->joints[i].acc_range[0], data_cur.joints[i].qdd), wb->joints[i].acc_range[1]);
        }
    }
    if(wb->param.inputMode == dymp::Wholebody::InputMode::Jerk){
        for(int i = 0; i < njoint; i++){
            data_cur.joints[i].qddd = data_ref.joints[i].qddd + wb->scale.jr*du(idx++);
        }
    }
    
    for(int i = 0; i < nend; i++){
        dymp::WholebodyData::End&  dend_ref = data_ref.ends[i];
        dymp::WholebodyData::End&  dend_des = ddes0   .ends[i];
        dymp::WholebodyData::End&  dend     = data_cur.ends[i];
        
        if(wb->ends[i].enableForce)
             dend.force_t = dend_ref.force_t + wb->scale.ft*subvec3_get(du, idx);
        else dend.force_t = dymp::zero3;

        if(wb->ends[i].enableMoment)
             dend.force_r = dend_ref.force_r + wb->scale.fr*subvec3_get(du, idx);
        else dend.force_r = dymp::zero3;

        if(dend_des.state == dymp::Wholebody::ContactState::Free){
            dend.force_t = dymp::zero3;
            dend.force_r = dymp::zero3;
        }
        else{
            dymp::real_t fz = dend.force_t.z();
            
            // limit by measured force
            //if(i == 3 || i == 4){
            fz = std::max(0.0, std::min(fz, robot->foot[i].force.z()));
            //}
            // enforce contact force constraint
            dymp::vec3_t flocal = dend.pos_r.conjugate()*dend.force_t;
            dymp::vec3_t mlocal = dend.pos_r.conjugate()*dend.force_r;
            flocal.z() = std::max(0.0, flocal.z());
            flocal.x() = std::min(std::max(-dend_des.mu*fz, flocal.x()),  dend_des.mu*fz);
            flocal.y() = std::min(std::max(-dend_des.mu*fz, flocal.y()),  dend_des.mu*fz);
            mlocal.x() = std::min(std::max( dend_des.cop_min.y()*fz, mlocal.x()),  dend_des.cop_max.y()*fz);
            mlocal.y() = std::min(std::max(-dend_des.cop_max.x()*fz, mlocal.y()), -dend_des.cop_min.x()*fz);
            mlocal.z() = std::min(std::max( dend_des.cop_min.z()*fz, mlocal.z()),  dend_des.cop_max.z()*fz);
            dend.force_t = dend.pos_r*flocal;
            dend.force_r = dend.pos_r*mlocal;
            /*
            */
        }
    }
    
    wb->CalcPosition          (data_cur);
    wb->CalcVelocity          (data_cur);
    wb->CalcAcceleration      (data_cur);
    wb->CalcInertia          (data_cur);
    wb->CalcInertiaDerivative(data_cur);
    wb->CalcLocalMomentum          (data_cur);
    wb->CalcLocalMomentumDerivative(data_cur);
    wb->CalcComAcceleration   (data_cur);
    wb->CalcBaseAngularAcceleration  (data_cur);
    wb->CalcForce             (data_cur);    

    //DSTR << data_cur.links[0].force_t_par << " " << data_cur.links[0].force_r_par << endl;
}

void PlannerWholebody::UpdateGain(){
    dymp::Wholebody* wb = GetWholebody();
    
    // copy data
    int N = mpcPredictionSteps;
    for(int k = 0; k < N; k++){
        auto key = (dymp::WholebodyKey*)wb->traj.GetKeypoint(k);
        key->data.CopyVars    (data_traj    [k]);
        key->data_des.CopyVars(data_traj_des[k]);
        //if(k == 0){
        //    key->data    .CopyVars(data_ref);
        //}
    }
    
    // calc input dimension
    int nend   = (int)wb->ends  .size();
    int njoint = (int)wb->joints.size();
    
    // store matrices
    mat_copy(wb->world->solver->Quuinv_Qux[mpcDelayMode ? 1 : 0], Quuinv_Qux);

    if(savePlan)
        SavePlan();

    mpcInputReady = true;
}

void PlannerWholebody::GetInitialState(dymp::WholebodyData& d){
    d.centroid.pos_t = data_cur.centroid.pos_t;
    d.centroid.vel_t = data_cur.centroid.vel_t;
    d.centroid.pos_r = data_cur.centroid.pos_r;
    d.centroid.vel_r = data_cur.centroid.vel_r;
    d.centroid.L_abs = data_cur.centroid.L_abs;
    
    int nend   = (int)d.ends  .size();
    int njoint = (int)d.joints.size();

    for(int i = 0; i < njoint; i++){
        d.joints[i].q   = data_cur.joints[i].q  ;
        d.joints[i].qd  = data_cur.joints[i].qd ;
        d.joints[i].qdd = data_cur.joints[i].qdd;
    }
}

void PlannerWholebody::GetDesiredState(int k, dymp::real_t t, dymp::WholebodyData& d){
    dymp::Wholebody* wb = GetWholebody();
    int N = mpcPredictionSteps;
    
    if(usePoseseq){
        // setup from poseseq
        robot->poseseq->Setup(t + mpcInitialTime, wb, d);
        
        // overwrite with centroid if ready
        if(useCentroid && robot->planner_centroid->mpcInputReady){
            robot->kinematics->SetupFromCentroid(
                t + mpcInitialTime,
                robot->planner_centroid->GetCentroid(), 
                robot->planner_centroid->data_traj,
                wb, d);
            //DSTR << k << "pos_r: " << d.centroid.pos_r << " vel_r: " << d.centroid.vel_r << " L: " << d.centroid.Labs << endl;
        }
    }
    else{
        robot->kinematics->SetupStandStill(wb, d);
    }

    bool is_initial  = k == 0;
    bool is_terminal = k == N;
    bool is_flight   = (d.ends[0].state == dymp::Wholebody::ContactState::Free &&
                        d.ends[1].state == dymp::Wholebody::ContactState::Free);

    int nend   = (int)d.ends.size();

    // weight
    if(is_terminal){
        d.centroid.pos_t_weight = dymp::one3;
        d.centroid.pos_r_weight = dymp::one3;
        d.centroid.vel_t_weight = dymp::one3;
        d.centroid.L_weight     = dymp::one3;

        for(int i = 0; i < nend; i++){
            dymp::WholebodyData::End&  dend = d.ends[i];
            if(wb->ends[i].enableTerminalCost){
                dend.pos_t_weight = dymp::one3;
                dend.pos_r_weight = dymp::one3;
            }
        }
    }
    else{
        dymp::real_t wf = (is_flight  ? 1.0 : 1.0);
        d.centroid.pos_t_weight = wf*centroidPosWeight;
        d.centroid.pos_r_weight = wf*centroidOriWeight;
        d.centroid.vel_t_weight = wf*centroidVelWeight;
        d.centroid.L_weight     = wf*centroidLWeight;
        //d.centroid.vel_r_weight = wf*ToSpr(centroidAngvelWeight);
        d.centroid.acc_t_weight = 10000.0*dymp::one3;
        d.centroid.Ld_weight    = 10000.0*dymp::one3;
        //d.centroid.acc_r_weight = 10000.0*one;

        int nend   = (int)d.ends  .size();
        int njoint = (int)d.joints.size();

        for(int i = 0; i < njoint; i++){
            dymp::WholebodyData::Joint& djnt = d.joints[i];
		    djnt.q_weight    = qWeight  [i];
		    djnt.qd_weight   = qdWeight [i];
		    djnt.qdd_weight  = qddWeight[i];
            djnt.qddd_weight = qdddWeight[i];

		    //djnt.q_min   = qMin  [i];
		    //djnt.q_max   = qMax  [i];
		    //djnt.qd_min  = qdMin [i];
		    //djnt.qd_max  = qdMax [i];
		    //djnt.qdd_min = qddMin[i];
		    //djnt.qdd_max = qddMax[i];

            djnt.q_range_weight =   qRangeWeight  [i];
            djnt.qd_range_weight  = qdRangeWeight [i];
            djnt.qdd_range_weight = qddRangeWeight[i];
	    }

        for(int i = 0; i < nend; i++){
            dymp::WholebodyData::End&  dend = d.ends[i];
        
            dymp::real_t wf = (dend.state == dymp::Wholebody::ContactState::Free ? freeWeight : 1.0);
        
            // end-specific weight scale
            //if(i == Kinematics::End::ChestP){
            //    dend.pos_t_weight = dymp::zero3;
            //    dend.pos_r_weight = torsoOriWeight;
            //    dend.vel_t_weight = dymp::zero3;
            //    dend.vel_r_weight = torsoAngvelWeight;
            //}
            //if(i == Kinematics::End::HandR || i == Kinematics::End::HandL){
            //    dend.pos_t_weight = handPosWeight   ;
            //    dend.pos_r_weight = handOriWeight   ;
            //    dend.vel_t_weight = handVelWeight   ;
            //    dend.vel_r_weight = handAngvelWeight;
            //}
            if(i == Kinematics::End::FootR || i == Kinematics::End::FootL){
                dend.pos_t_weight = wf*(dend.state == dymp::Wholebody::ContactState::Free ? 1.0 : 1.0)*footPosWeight   ;
                dend.pos_r_weight = wf*(dend.state == dymp::Wholebody::ContactState::Free ? 1.0 : 1.0)*footOriWeight   ;
                dend.vel_t_weight = wf*(dend.state == dymp::Wholebody::ContactState::Free ? 1.0 : 1.0)*footVelWeight   ;
                dend.vel_r_weight = wf*(dend.state == dymp::Wholebody::ContactState::Free ? 1.0 : 1.0)*footAngvelWeight;
            }
            dend.force_t_weight     = (dend.state == dymp::Wholebody::ContactState::Free ? 10000.0*dymp::one3 : endForceWeight );
            dend.force_r_weight     = (dend.state == dymp::Wholebody::ContactState::Free ? 10000.0*dymp::one3 : endMomentWeight);
        }
    }
}

void PlannerWholebody::ToRobot(){
    if(mpcInputReady){
        Observe();
        
        // update input
        InterpolateReference();
        UpdateInput();

        // set joint commands
        int njoint = (int)data_cur.joints.size();
        for(int i = 0; i < njoint; i++){
            robot->joint[i].q_ref  = data_ref.joints[i].q  ;
            robot->joint[i].dq_ref = data_ref.joints[i].qd ;
            robot->joint[i].u_ref  = data_cur.joints[i].tau;
        }
        /*
        // prevent joint buckling
        const int knee_joints[] = {
            Kinematics::Joint::LowerLegLP, 
            Kinematics::Joint::LowerLegRP
        };
        for(int i : knee_joints){
            if(robot->joint[i].q <= qMin[i]){
                robot->joint[i].q_ref  = std::max(robot->joint[i].q_ref, qMin[i]);
                robot->joint[i].dq_ref = std::max(robot->joint[i].dq_ref, 0.0);
                robot->joint[i].u_ref  = std::max(robot->joint[i].u_ref, 0.0);
                printf("buckle warning!\n");
            }
        }
        */
    }

	//Planner::ToRobot();
}

void PlannerWholebody::SavePlan(){
    dymp::Wholebody* wb = GetWholebody();
    dymp::World* world  = wb->world;
    
    static int idx = 0;
    char filename[256];
    //sprintf(filename, "plan_mpc%d.csv", idx);
    sprintf(filename, "plan_mpc.csv");
    FILE* file = fopen(filename, "w");
    idx++;

    fprintf(file, 
        "k, "
        "cen_pos_t_x, cen_pos_t_y, cen_pos_t_z, "
        "cen_pos_r_x, cen_pos_r_y, cen_pos_r_z, "
        "cen_vel_t_x, cen_vel_t_y, cen_vel_t_z, "
        "cen_vel_r_x, cen_vel_r_y, cen_vel_r_z, "
        "cen_acc_t_x, cen_acc_t_y, cen_acc_t_z, "
        "cen_acc_r_x, cen_acc_r_y, cen_acc_r_z, "
        "cen_L_x, cen_L_y, cen_L_z, "
        "cen_Ld_x, cen_Ld_y, cen_Ld_z, "
    );
    for(int i = 0; i < wb->joints.size(); i++){
        fprintf(file, "joint%d_q, ", i);
    }
    for(int i = 0; i < wb->joints.size(); i++){
        fprintf(file, "joint%d_qd, ", i);
    }
    for(int i = 0; i < wb->joints.size(); i++){
        fprintf(file, "joint%d_qdd, ", i);
    }
    for(int i = 0; i < wb->joints.size(); i++){
        fprintf(file, "joint%d_qddd, ", i);
    }
    for(int i = 0; i < wb->joints.size(); i++){
        fprintf(file, "joint%d_tau, ", i);
    }
    for(int i = 0; i < wb->ends.size(); i++){
        fprintf(file,
            "end%d_pos_t_x, end%d_pos_t_y, end%d_pos_t_z, "
            "end%d_pos_r_x, end%d_pos_r_y, end%d_pos_r_z, end%d_pos_r_w, "
            "end%d_vel_t_x, end%d_vel_t_y, end%d_vel_t_z, "
            "end%d_vel_r_x, end%d_vel_r_y, end%d_vel_r_z, "
            "end%d_acc_t_x, end%d_acc_t_y, end%d_acc_t_z, "
            "end%d_acc_r_x, end%d_acc_r_y, end%d_acc_r_z, "
            "end%d_pos_t_des_x, end%d_pos_t_des_y, end%d_pos_t_des_z, "
            "end%d_pos_r_des_x, end%d_pos_r_des_y, end%d_pos_r_des_z, end%d_pos_r_des_w, "
            "end%d_force_t_x, end%d_force_t_y, end%d_force_t_z, "
            "end%d_force_r_x, end%d_force_r_y, end%d_force_r_z, ",
            i, i, i,
            i, i, i, i,
            i, i, i,
            i, i, i,
            i, i, i,
            i, i, i,
            i, i, i,
            i, i, i, i,
            i, i, i,
            i, i, i
            );
    }
    fprintf(file, "\n");

    for(int k = 0; k < world->ticks.size(); k++){
        auto key = (dymp::WholebodyKey*)wb->traj.GetKeypoint(k);
        auto& d = key->data;
        auto& d_des = key->data_des;

        dymp::vec3_t pc = d.centroid.pos_t;
        dymp::vec3_t vc = d.centroid.vel_t;
        dymp::vec3_t ac = d.centroid.acc_t;
        dymp::quat_t q0 = d.centroid.pos_r;
        dymp::vec3_t w0 = d.centroid.vel_r;
        dymp::vec3_t u0 = d.centroid.acc_r;

        fprintf(file,
            "%d, "
            "%f, %f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, ",
            k, 
            pc.x(), pc.y(), pc.z(), 
            q0.x(), q0.y(), q0.z(), 
            vc.x(), vc.y(), vc.z(), 
            w0.x(), w0.y(), w0.z(),
            ac.x(), ac.y(), ac.z(), 
            u0.x(), u0.y(), u0.z(),
            d.centroid.L_abs.x(), d.centroid.L_abs.y(), d.centroid.L_abs.z(),
            d.centroid.Ld_local.x(), d.centroid.Ld_local.y(), d.centroid.Ld_local.z()
        );
        for(int i = 0; i < wb->joints.size(); i++){
            fprintf(file, "%f, ", d.joints[i].q);
        }
        for(int i = 0; i < wb->joints.size(); i++){
            fprintf(file, "%f, ", d.joints[i].qd);
        }
        for(int i = 0; i < wb->joints.size(); i++){
            fprintf(file, "%f, ", d.joints[i].qdd);
        }
        for(int i = 0; i < wb->joints.size(); i++){
            fprintf(file, "%f, ", d.joints[i].qddd);
        }
        for(int i = 0; i < wb->joints.size(); i++){
            fprintf(file, "%f, ", d.joints[i].tau);
        }
        for(int i = 0; i < wb->ends.size(); i++){
            dymp::WholebodyData::End& dend = d.ends[i];
            dymp::WholebodyData::End& dend_des = d_des.ends[i];
            
            dymp::vec3_t pe_abs = pc + q0*dend.pos_t;
            dymp::quat_t qe_abs = q0*dend.pos_r;
            dymp::vec3_t ve_abs = vc + q0*dend.vel_t + w0.cross(q0*dend.pos_t);
            dymp::vec3_t we_abs = w0 + q0*dend.vel_r;
            dymp::vec3_t ae_abs = ac + q0*dend.acc_t + 2.0*(w0.cross(q0*dend.vel_t)) + w0.cross(w0.cross(q0*dend.pos_t));
            dymp::vec3_t ue_abs = u0 + q0*dend.acc_r + w0.cross(q0*dend.vel_r);

            fprintf(file,
                "%f, %f, %f, "
                "%f, %f, %f, %f, "
                "%f, %f, %f, "
                "%f, %f, %f, "
                "%f, %f, %f, "
                "%f, %f, %f, "
                "%f, %f, %f, "
                "%f, %f, %f, %f, "
                "%f, %f, %f, "
                "%f, %f, %f, ",
                pe_abs.x(), pe_abs.y(), pe_abs.z(), 
                qe_abs.x(), qe_abs.y(), qe_abs.z(), qe_abs.w(),
                ve_abs.x(), ve_abs.y(), ve_abs.z(), 
                we_abs.x(), we_abs.y(), we_abs.z(), 
                ae_abs.x(), ae_abs.y(), ae_abs.z(), 
                ue_abs.x(), ue_abs.y(), ue_abs.z(), 
                dend_des.pos_t_abs.x(), dend_des.pos_t_abs.y(), dend_des.pos_t_abs.z(), 
                dend_des.pos_r_abs.x(), dend_des.pos_r_abs.y(), dend_des.pos_r_abs.z(), dend_des.pos_r_abs.w(),
                //dend.pos_t.x, dend.pos_t.y, dend.pos_t.z, 
                //dend.pos_r.x, dend.pos_r.y, dend.pos_r.z, 
                //dend.vel_t.x, dend.vel_t.y, dend.vel_t.z, 
                //dend.vel_r.x, dend.vel_r.y, dend.vel_r.z, 
                dend.force_t.x(), dend.force_t.y(), dend.force_t.z(), 
                dend.force_r.x(), dend.force_r.y(), dend.force_r.z()
            );    
        }
        fprintf(file, "\n");
    }

    fclose(file);
}

void PlannerWholebody::Visualize(cnoid::vnoid::Visualizer* viz, cnoid::vnoid::VizInfo& info){
    dymp::Wholebody* wb = GetWholebody();
    int N = mpcPredictionSteps;
    int nlink = (int)wb->links.size();
    int nend  = (int)wb->ends .size();

    // get poseseq snapshot
    robot->poseseq->Setup(mpcInitialTime, wb, data_des);
    //DSTR << "wholebody time: " << mpcInitialTime << endl;

    // visualize planned and reference trajectory, and current state
    for(int item = 0; item < 3; item++){
        cnoid::vnoid::Visualizer::Lines* lines = viz->data->GetLines(info.iframe, info.ilines);
        if(item == 0) lines->color = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
        if(item == 1) lines->color = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
        if(item == 2) lines->color = Eigen::Vector3f(1.0f, 0.0f, 1.0f);
        lines->alpha = 0.5f;
        lines->width = (item == 2/*(item == 1 || item == 2)*/ ? 5.0f : 1.0f);
        
        int iv = 0;
        int ii = 0;
        for(int k = 0; k < N; k++){
            dymp::WholebodyData& d = (item == 0 ? data_traj[k] : (item == 1 ? /*data_des*/data_traj_des[k] : data_cur));
        
            dymp::vec3_t pc = d.centroid.pos_t;
            dymp::quat_t q0 = d.centroid.pos_r;

            if(item == 0 || item == 1){
                cnoid::vnoid::Visualizer::Sphere* sphere = viz->data->GetSphere(info.iframe, info.isphere++);
                sphere->pos    = pc;
                sphere->radius = 0.02f;
                if(item == 0)
                    sphere->color  = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
                if(item == 1)
                    sphere->color  = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
                sphere->alpha  = 0.5f;
            }

            for(int i = 1; i < nlink; i++){
                // skip leg links of desired pose
                /*
                if(item == 1 && (Kinematics::Link::UpperLegRY <= i && i <= Kinematics::Link::FootLP))
                    continue;
                */

                int ip = wb->links[i].iparent;

                dymp::vec3_t v0 = pc + q0*d.links[ip].pos_t;
                dymp::vec3_t v1 = pc + q0*d.links[i ].pos_t;
                viz->data->GetLineVertices(info.iframe, info.ilines)[iv+0] = Eigen::Vector3f((float)v0.x(), (float)v0.y(), (float)v0.z());
                viz->data->GetLineVertices(info.iframe, info.ilines)[iv+1] = Eigen::Vector3f((float)v1.x(), (float)v1.y(), (float)v1.z());
                viz->data->GetLineIndices (info.iframe, info.ilines)[ii+0] = iv+0;
                viz->data->GetLineIndices (info.iframe, info.ilines)[ii+1] = iv+1;
                iv += 2;
                ii += 2;
            }

            const dymp::real_t r = 0.1;
            for(int i = 0; i < nend; i++){
                // skip upper ends of desired pose
                if(item == 1 && (0 <= i && i <= 2))
                    continue;

                dymp::vec3_t pe;
                dymp::quat_t qe;
                if(item == 0 || item == 2){
                    pe = pc + q0*d.ends[i].pos_t;
                    qe = q0*d.ends[i].pos_r;
                }
                else{
                    pe = d.ends[i].pos_t_abs;
                    qe = d.ends[i].pos_r_abs;
                }
                dymp::vec3_t v0 = pe;
                dymp::vec3_t v1 = pe + qe*dymp::vec3_t(r, 0, 0);
                dymp::vec3_t v2 = pe + qe*dymp::vec3_t(0, r, 0);
                dymp::vec3_t v3 = pe + qe*dymp::vec3_t(0, 0, r);
                viz->data->GetLineVertices(info.iframe, info.ilines)[iv+0] = Eigen::Vector3f((float)v0.x(), (float)v0.y(), (float)v0.z());
                viz->data->GetLineVertices(info.iframe, info.ilines)[iv+1] = Eigen::Vector3f((float)v1.x(), (float)v1.y(), (float)v1.z());
                viz->data->GetLineVertices(info.iframe, info.ilines)[iv+2] = Eigen::Vector3f((float)v0.x(), (float)v0.y(), (float)v0.z());
                viz->data->GetLineVertices(info.iframe, info.ilines)[iv+3] = Eigen::Vector3f((float)v2.x(), (float)v2.y(), (float)v2.z());
                viz->data->GetLineVertices(info.iframe, info.ilines)[iv+4] = Eigen::Vector3f((float)v0.x(), (float)v0.y(), (float)v0.z());
                viz->data->GetLineVertices(info.iframe, info.ilines)[iv+5] = Eigen::Vector3f((float)v3.x(), (float)v3.y(), (float)v3.z());
                viz->data->GetLineIndices (info.iframe, info.ilines)[ii+0] = iv+0;
                viz->data->GetLineIndices (info.iframe, info.ilines)[ii+1] = iv+1;
                viz->data->GetLineIndices (info.iframe, info.ilines)[ii+2] = iv+2;
                viz->data->GetLineIndices (info.iframe, info.ilines)[ii+3] = iv+3;
                viz->data->GetLineIndices (info.iframe, info.ilines)[ii+4] = iv+4;
                viz->data->GetLineIndices (info.iframe, info.ilines)[ii+5] = iv+5;
                iv += 6;
                ii += 6;
            }

            //if(item == 1 || item == 2)
            if(item == 2)
                break;
        }
        lines->numVertices = iv;
        lines->numIndices  = ii;
        info.ilines++;
    }
}

}
}
