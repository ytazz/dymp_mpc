#include "myrobot.h"
#include "planner_centroid.h"
#include "planner_wholebody.h"
#include "poseseq.h"
#include "kinematics.h"

namespace dymp{
namespace mpc{

PlannerThreadCentroid::PlannerThreadCentroid(Planner* _planner):PlannerThread(_planner){
	planner_centroid = (PlannerCentroid*)_planner;
	centroid = new dymp::Centroid(world, "centroid");
    centroid->callback  = planner_centroid;
    centroid->param     = planner_centroid->param;
    centroid->ends      = planner_centroid->ends;
    centroid->faces     = planner_centroid->faces;
    
    centroid->param.m = planner->robot->param.total_mass;
    centroid->param.I(0,0) = 10.0;//planner->robot->nominal_inertia.x();
    centroid->param.I(1,1) = 10.0;//planner->robot->nominal_inertia.y();
    centroid->param.I(2,2) =  5.0;//planner->robot->nominal_inertia.z();
    centroid->param.g  = 9.8;
    centroid->param.mu = 1.0;
    
    centroid->param.contactMargin = 0.0;
    
    //world->scale.Set(1.0, 1.0, 1.0);
    centroid->SetScaling();
}

void PlannerThreadCentroid::Init(){
    PlannerThread::Init();

    centroid->Setup();
    centroid->Reset(true, true, true);
    
    //graph->solver->Enable(ID(DiMP::ConTag::CentroidPosT      ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::CentroidPosR      ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::CentroidVelT      ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::CentroidVelR      ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::CentroidTime      ), false);
    //graph->solver->Enable(ID(DiMP::ConTag::CentroidEndPos    ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndVel    ), false);
	//graph->solver->Enable(ID(DiMP::ConTag::CentroidEndStiff  ), false);
	world->solver->Enable(dymp::ID(dymp::ConTag::CentroidEndPosRange   ), false);
	//world->solver->Enable(dymp::ID(dymp::ConTag::CentroidEndContact    ), false);
    //world->solver->Enable(dymp::ID(dymp::ConTag::CentroidEndFriction   ), false);
	//world->solver->Enable(dymp::ID(dymp::ConTag::CentroidEndMomentRange), false);

    //graph->solver->SetCorrection(ID(DiMP::ConTag::CentroidPosT    ), 0.0);
    //graph->solver->SetCorrection(ID(DiMP::ConTag::CentroidPosR    ), 0.0);
    //graph->solver->SetCorrection(ID(DiMP::ConTag::CentroidVelT    ), 0.0);
    //graph->solver->SetCorrection(ID(DiMP::ConTag::CentroidMomentum), 0.0);
    //graph->solver->SetCorrection(ID(DiMP::ConTag::CentroidTime    ), 0.0);
    //graph->solver->SetCorrection(ID(DiMP::ConTag::CentroidEndPosT ), 0.0);
    //graph->solver->SetCorrection(ID(DiMP::ConTag::CentroidEndPosR ), 0.0);

    //graph->solver->Lock(ID(DiMP::VarTag::CentroidPosT    ), 0.0);
    //graph->solver->Lock(ID(DiMP::VarTag::CentroidPosR    ), true);
    //graph->solver->Lock(ID(DiMP::VarTag::CentroidVelT    ), 0.0);
    //graph->solver->Lock(ID(DiMP::VarTag::CentroidMomentum), true);
    
}

void PlannerThreadCentroid::Setup(){
    //wb->Shift(planner->mpcTimestep);
    if(planner_centroid->phaseSwitched){
        centroid->Shift();
        planner_centroid->phaseSwitched = false;
        planner_centroid->nextPlanTime  = planner_centroid->data_ref.time + planner_centroid->data_ref.duration;
    }
	centroid->Setup();
    centroid->Reset(true, true, true);
    world->solver->InitDDP();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

PlannerCentroid::PlannerCentroid(){
    name = "planner_centroid";

    desDuration        = 0.3;
    initialWeight      = 1.0;
    terminalWeight     = 1.0;
	centroidPosWeight  = Vector3(1.0, 1.0, 1.0);
	centroidVelWeight  = Vector3(1.0, 1.0, 1.0);
	centroidOriWeight  = Vector3(1.0, 1.0, 1.0);
	centroidLWeight    = Vector3(1.0, 1.0, 1.0);
    timeWeight         = 1.0;
    durationWeight     = 1.0;
    endPosWeight       = Vector3(1.0, 1.0, 1.0);
	endVelWeight       = Vector3(1.0, 1.0, 1.0);
	endOriWeight       = Vector3(1.0, 1.0, 1.0);
	endAngvelWeight    = Vector3(1.0, 1.0, 1.0);
    endStiffnessWeight = 1.0;
    endCmpWeight       = Vector2(1.0, 1.0);
    endMomentWeight    = Vector3(1.0, 1.0, 1.0);
    savePlan           = false;
    enableFeedback     = false;
}

void PlannerCentroid::Read(const YAML::Node& node){
	Planner::Read(node);

    const YAML::Node& cenNode = node["centroid"];

    ReadVector3(param.bodyRangeMin   , cenNode["body_range_min" ]);
    ReadVector3(param.bodyRangeMax   , cenNode["body_range_max" ]);
    ReadDouble (param.durationMin    , cenNode["duration_min"   ]);
    ReadDouble (param.durationMax    , cenNode["duration_max"   ]);
    ReadDouble (param.swingHeight    , cenNode["swing_height"   ]);
    ReadDouble (param.swingSlope     , cenNode["swing_slope"    ]);
    ReadDouble (param.swingLiftMargin, cenNode["swing_lift_margin"]);
    ReadDouble (param.swingLandMargin, cenNode["swing_land_margin"]);
    ReadDouble (param.swingTurnMargin, cenNode["swing_turn_margin"]);
    ReadBool   (param.enableRotation , cenNode["enable_rotation"]);
    ReadInt    (param.rotationResolution, cenNode["rotation_resolution"]);
    ReadDouble (param.complWeight    , cenNode["compl_weight"   ]);

    string str;
    ReadString(str, cenNode["end_wrench_parametrization"]);
    if(str == "stiffness") param.endWrenchParametrization = dymp::Centroid::EndWrenchParametrization::Stiffness;
    if(str == "direct"   ) param.endWrenchParametrization = dymp::Centroid::EndWrenchParametrization::Direct;

    ReadString(str, cenNode["end_interpolation"]);
    if(str == "polynomial"    ) param.endInterpolation = dymp::Centroid::EndInterpolation::Polynomial;
    if(str == "cycloid_global") param.endInterpolation = dymp::Centroid::EndInterpolation::CycloidGlobal;
    if(str == "cycloid_local" ) param.endInterpolation = dymp::Centroid::EndInterpolation::CycloidLocal ;

    for(int i = 0; i < cenNode["end"].size(); i++){
        YAML::Node endNode = cenNode["end"][i];
        dymp::Centroid::End end;
        
        ReadVector3(end.basePos     , endNode["base_pos"     ]);	
        ReadVector3(end.copMin      , endNode["cop_min"      ]);	
        ReadVector3(end.copMax      , endNode["cop_max"      ]);	
        ReadDouble (end.stiffnessMax, endNode["stiffness_max"]);	
        ReadBool   (end.lockOri     , endNode["lock_ori"     ]);
        ReadBool   (end.lockCmp     , endNode["lock_cmp"     ]);
        ReadBool   (end.lockMoment  , endNode["lock_moment"  ]);
        ReadVector2(end.cmpOffset   , endNode["cmp_offset"   ]);
        ReadVector3(end.posMin      , endNode["pos_min"      ]);	
        ReadVector3(end.posMax      , endNode["pos_max"      ]);	
                
        ends.push_back(end);
    }

    for(int i = 0; i < cenNode["face"].size(); i++){
        YAML::Node faceNode = cenNode["face"][i];

        dymp::Centroid::Face face;
        ReadVector3(face.normal, faceNode["normal"]);
        for(int j = 0; j < faceNode["vertex"].size(); j++){
            YAML::Node vtxNode = faceNode["vertex"][j];
            dymp::vec3_t v;
            ReadVector3(v, vtxNode["pos"]);
            face.vertices.push_back(v);
        }
        faces.push_back(face);
    }

    ReadDouble (initialWeight     , cenNode["initial_weight"     ]);
	ReadDouble (terminalWeight    , cenNode["terminal_weight"    ]);
	ReadVector3(centroidPosWeight , cenNode["centroid_pos_weight"]);
	ReadVector3(centroidVelWeight , cenNode["centroid_vel_weight"]);
	ReadVector3(centroidOriWeight , cenNode["centroid_ori_weight"]);
	ReadVector3(centroidLWeight   , cenNode["centroid_L_weight"  ]);
	ReadDouble (timeWeight        , cenNode["time_weight"        ]);
	ReadDouble (durationWeight    , cenNode["duration_weight"    ]);
	ReadVector3(endPosWeight      , cenNode["end_pos_weight"     ]);
	ReadVector3(endVelWeight      , cenNode["end_vel_weight"     ]);
	ReadVector3(endOriWeight      , cenNode["end_ori_weight"     ]);
	ReadVector3(endAngvelWeight   , cenNode["end_angvel_weight"  ]);
	ReadDouble (endStiffnessWeight, cenNode["end_stiffness_weight"]);
	ReadVector2(endCmpWeight      , cenNode["end_cmp_weight"     ]);
	ReadVector3(endMomentWeight   , cenNode["end_moment_weight"  ]);

    ReadBool(enableFeedback, cenNode["enable_feedback"]);
}

PlannerThread* PlannerCentroid::CreateThread(){
    return new PlannerThreadCentroid(this);
}

dymp::Centroid* PlannerCentroid::GetCentroid(){
    return ((PlannerThreadCentroid*)threads[threadIndex])->centroid;
}

void PlannerCentroid::InitState(){
    dymp::Centroid* centroid = GetCentroid();
    dymp::Wholebody* wb = robot->planner_wb->GetWholebody();

    int nend = (int)centroid->ends.size();
    nx = 6 + (centroid->param.enableRotation ? 6 : 0) + 1;
    nu = 1;
    for(int i = 0; i < nend; i++){
        nx += 3 + (centroid->ends[i].lockOri ? 0 : 3);
        nu += 3 + (centroid->ends[i].lockOri ? 0 : 3) + 1 + (centroid->ends[i].lockCmp ? 0 : 2) + (centroid->ends[i].lockMoment ? 0 : 3);
    }
    
    dx.Allocate(nx);
    du.Allocate(nu);
    u .Allocate(nu);
    Quuinv_Qux.Allocate(nu, nx);
    mat_clear(Quuinv_Qux);

    int N = mpcPredictionSteps;
    data_cur.Init(centroid);
    data_ref.Init(centroid);
    data_traj    .resize(N+1);
    data_traj_des.resize(N+1);
    V .resize (N+1);
    Vx.resize (N+1);
    Vxx.resize(N+1);
    fx .resize(N);
    for(int k = 0; k <= N; k++){
        data_traj    [k].Init(centroid);
        data_traj_des[k].Init(centroid);

        Vx [k].Allocate(nx);
        Vxx[k].Allocate(nx,nx);

        if(k < N){
            fx[k].Allocate(nx,nx);
        }
    }
    I       .Allocate(nx,nx);
    fxt     .Allocate(nx,nx);
    fxtr_Vxx.Allocate(nx,nx);

    data_wb.Init(wb);

    cur_time  = 0.0;
    cur_phase = 0;
    data_ref.time     = 0.0;
    data_ref.duration = 0.0;
    GetDesiredState(0, 0.0, data_ref);

    // trigger 0-th update immediately
    phaseSwitched = true;

    // time to trigger 1-th update
    nextPlanTime = robot->poseseq->GetTime(cur_phase + 1);
}

void PlannerCentroid::UpdateState(){
    dymp::Centroid* centroid = GetCentroid();
    int nend = (int)centroid->ends.size();
    
    dymp::real_t dt  = robot->timer.dt*robot->param.control_cycle;

    cur_time += dt;
    //if(cur_time >= data_ref.time + data_ref.duration){
    if(cur_time >= nextPlanTime){
        cur_phase++;
        phaseSwitched = true;
    }
    /*
    real_t dt2 = dt*dt;

    centroid->CalcWrench          (data_cur);
    centroid->CalcComAcceleration (data_cur);
    centroid->CalcBaseAcceleration(data_cur);
    
    data_cur.pos_t += data_cur.vel_t*dt + data_cur.acc_t*(0.5*dt2);
    data_cur.vel_t += data_cur.acc_t*dt;
    data_cur.pos_r = quat_t::Rot(data_cur.vel_r*dt + data_cur.acc_r*(0.5*dt2))*data_cur.pos_r;
    data_cur.pos_r.unitize();
    data_cur.vel_r += data_cur.acc_r*dt;

    for(int i = 0; i < nend; i++){
        DiMP::CentroidData::End& dend = data_cur.ends[i];

        dend.pos_t += dend.vel_t*dt;
        dend.pos_r = quat_t::Rot(dend.vel_r*dt)*dend.pos_r;
        dend.pos_r.unitize();
    }

    data_cur.time += dt;
    data_cur.duration -= dt;

    DSTR << "centroid time: " << data_cur.time << " duration: " << data_cur.duration << endl;

    if(data_cur.duration <= 0.0){
        data_cur.time     = std::max(data_cur.time, data_traj[1].time);
        data_cur.duration = data_traj[1].duration;
        for(int i = 0; i < nend; i++){
            data_cur.ends[i].iface = data_traj_des[1].ends[i].iface;
        }
        phaseSwitched = true;
        //phase = (phase + 1)%Phase::Num;
        //data_cur.duration = (phase == Phase::R || phase == Phase::L) ? desDurationSsp : desDurationDsp;
        //for(int i = 0; i < nend; i++){
        //    data_cur.ends[i].iface = ((i == 0 && phase != Phase::L) || (i == 1 && phase != Phase::R)) ? 0 : -1;
        //}
    }
    */
}

void PlannerCentroid::UpdateInput(){
    /*
    DiMP::Centroid* centroid = GetCentroid();
    int nend = (int)centroid->ends.size();
    int idx;

    // calc dx
    idx = 0;
    vec_clear(dx);

    subvec3_set(dx, idx, (data_cur.pos_t - data_ref.pos_t)/centroid->scale.pt);
    if(centroid->param.enableRotation)
        subvec3_set(dx, idx, quat_diff(data_ref.pos_r, data_cur.pos_r)/centroid->scale.pr);
    subvec3_set(dx, idx, (data_cur.vel_t - data_ref.vel_t)/centroid->scale.vt);
    if(centroid->param.enableRotation)
        subvec3_set(dx, idx, (data_cur.vel_r - data_ref.vel_r)/centroid->scale.vr);
    dx(idx++) = (data_cur.time - data_ref.time)/centroid->scale.t;

    for(int i = 0; i < nend; i++){
        DiMP::CentroidData::End&  dend     = data_cur.ends[i];
        DiMP::CentroidData::End&  dend_ref = data_ref.ends[i];
        
        subvec3_set(dx, idx, (dend.pos_t - dend_ref.pos_t)/centroid->scale.pt);
        if(!centroid->ends[i].lockOri)
            subvec3_set(dx, idx, quat_diff(dend_ref.pos_r, dend.pos_r)/centroid->scale.pr);
    }

    // calc u
    mat_vec_mul(Quuinv_Qux, dx, du, (mpcFeedback ? -1.0 : 0.0), 0.0);
    
    // decode u
    idx = 0;

    // duration is not updated
    //data_cur.duration = data_ref.duration + cen->scale.t*du(idx++);
    idx++;

    for(int i = 0; i < nend; i++){
        DiMP::CentroidData::End&  dend     = data_cur.ends[i];
        DiMP::CentroidData::End&  dend_ref = data_ref.ends[i];

        dend.vel_t  = dend_ref.vel_t + centroid->scale.vt*subvec3_get(du, idx);
        if(!centroid->ends[i].lockOri)
            dend.vel_r  = dend_ref.vel_r + centroid->scale.vr*subvec3_get(du, idx);
        
        dend.stiff  = dend_ref.stiff  + centroid->scale.tinv*du(idx++);
        if(!centroid->ends[i].lockCmp)
            dend.cmp    = dend_ref.cmp    + centroid->scale.pt  *subvec2_get(du, idx);
        if(!centroid->ends[i].lockMoment)
            dend.moment = dend_ref.moment + centroid->scale.pt2 *subvec3_get(du, idx);
    }
    */
}

void PlannerCentroid::UpdateGain(){
    dymp::Centroid* centroid = GetCentroid();
    
    // copy data
    int N = mpcPredictionSteps;
    for(int k = 0; k <= N; k++){
        auto key = (dymp::CentroidKey*)centroid->traj.GetKeypoint(k);
        key->data.CopyVars    (data_traj    [k]);
        key->data_des.CopyVars(data_traj_des[k]);
        if(k == 1){
            key->data.CopyVars(data_ref);
        }
    }
    
    // store matrices
    mat_copy(centroid->world->solver->Quuinv_Qux[mpcDelayMode ? 1 : 0], Quuinv_Qux);

    // store value function
    for(int k = 0; k <= N; k++){
        V[k] = centroid->world->solver->V[k];
        vec_copy(centroid->world->solver->Vx [k], Vx [k]);
        mat_copy(centroid->world->solver->Vxx[k], Vxx[k]);

        if(k < N){
            mat_copy(centroid->world->solver->fx[k], fx[k]);
        }
    }

    if(savePlan)
        SavePlan();

    mpcInputReady = true;
}

void PlannerCentroid::GetInitialState(dymp::CentroidData& d){
    //d = data_ref;
    if(enableFeedback){
        d.pos_t = data_cur.pos_t;
        d.vel_t = data_cur.vel_t;
        d.pos_r = data_cur.pos_r;
        d.L     = data_cur.L;
    }
    else{
        d.pos_t = data_ref.pos_t;
        d.vel_t = data_ref.vel_t;
        d.pos_r = data_ref.pos_r;
        d.L     = data_ref.L;
    }
    
    d.time  = data_ref.time;

    int nend = (int)d.ends.size();
    
    for(int i = 0; i < nend; i++){
        dymp::CentroidData::End&  dend     = d.ends[i];
        dymp::CentroidData::End&  dend_cur = data_cur.ends[i];
        dymp::CentroidData::End&  dend_ref = data_ref.ends[i];
        
        if(enableFeedback){
            dend.pos_t = dend_cur.pos_t;
            dend.pos_r = dend_cur.pos_r;
        }
        else{
            dend.pos_t = dend_ref.pos_t;
            dend.pos_r = dend_ref.pos_r;
        }
        
        dend.iface = dend_ref.iface;
        dend.state = dend_ref.state;
    }
    /*
    d.pos_t    = data_cur.pos_t;
    d.vel_t    = data_cur.vel_t;
    d.pos_r    = data_cur.pos_r;
    d.vel_r    = data_cur.vel_r;
    d.time     = data_cur.time;
    
    d.duration = data_cur.duration;

    int nend = (int)d.ends.size();
    
    for(int i = 0; i < nend; i++){
        DiMP::CentroidData::End&  dend     = d.ends[i];
        DiMP::CentroidData::End&  dend_cur = data_cur.ends[i];
        
        dend.pos_t = dend_cur.pos_t;
        dend.pos_r = dend_cur.pos_r;
        dend.vel_t = dend_cur.vel_t;
        dend.vel_r = dend_cur.vel_r;

        dend.iface = dend_cur.iface;
    }
    */
}

void PlannerCentroid::GetDesiredState(int k, dymp::real_t t, dymp::CentroidData& d){
    dymp::Centroid* centroid = GetCentroid();
    dymp::Wholebody* wb = robot->planner_wb->GetWholebody();
    int N = mpcPredictionSteps;
    
    if(true){
        int ndiv = centroid->param.rotationResolution;
        dymp::real_t t0 = robot->poseseq->GetTime(cur_phase+k+0);
        dymp::real_t t1 = robot->poseseq->GetTime(cur_phase+k+1);
        dymp::real_t tau  = t1 - t0;
        dymp::real_t dtau = tau/ndiv;
        //real_t t0 = robot->poseseq->keys[robot->poseseq->initialPhase+cur_phase+k+0].time - robot->poseseq->keys[robot->poseseq->initialPhase].time;
        //real_t t1 = robot->poseseq->keys[robot->poseseq->initialPhase+cur_phase+k+1].time - robot->poseseq->keys[robot->poseseq->initialPhase].time;
        for(int idiv = 0; idiv <= ndiv; idiv++){
            robot->poseseq->Setup(t0 + idiv*dtau, wb, data_wb);
            robot->kinematics->Convert(data_wb, d, idiv);

            // set desired angular momentum as zero
            //d.L = dymp::zero3;
        }
        d.time     = t0;
        d.duration = tau;
    }
    else{
        robot->kinematics->SetupStandStill(centroid, d);
        d.time     = data_ref.time + k*desDuration;
        d.duration = desDuration;
    }
    
    bool is_initial  = k == 0;
    bool is_terminal = k == N;
    dymp::real_t wt = (is_initial ? initialWeight : (is_terminal ? terminalWeight : 1.0));
    d.pos_t_weight = wt*centroidPosWeight;
    d.pos_r_weight = wt*centroidOriWeight;
    d.vel_t_weight = wt*centroidVelWeight;
    d.L_weight     = wt*centroidLWeight;
    
    d.time_weight     = wt*timeWeight;
    d.duration_weight = wt*durationWeight;

    int nend = (int)d.ends.size();
    for(int i = 0; i < nend; i++){
        dymp::CentroidData::End& dend = d.ends[i];
        
        dend.pos_t_weight = wt*endPosWeight;
        dend.pos_r_weight = wt*endOriWeight;
        dend.vel_t_weight = wt*endVelWeight;
        dend.vel_r_weight = wt*endAngvelWeight;

        dend.stiff_weight  = endStiffnessWeight;
        dend.cmp_weight    = endCmpWeight;
        dend.moment_weight = endMomentWeight;
    }

    centroid->CalcStiffness(d);
}

bool PlannerCentroid::IsMpcUpdateCycle(){
    //return /*!mpcInputReady &&*/ data_cur.duration > 0.05 && Planner::IsMpcUpdateCycle();
    //return !mpcInputReady || phaseSwitched;
    return phaseSwitched;
}

void PlannerCentroid::Observe(){
    dymp::Centroid* centroid = GetCentroid();
    
    data_cur.pos_t = robot->planner_wb->data_cur.centroid.pos_t;
    data_cur.vel_t = robot->planner_wb->data_cur.centroid.vel_t;
    data_cur.pos_r = robot->planner_wb->data_cur.centroid.pos_r;
    data_cur.vel_r = robot->planner_wb->data_cur.centroid.vel_r;        
    data_cur.L     = robot->planner_wb->data_cur.centroid.L_abs;

    int nend = (int)centroid->ends.size();
    for(int i = 0; i < nend; i++){
        data_cur.ends[i].pos_t = data_cur.pos_t + data_cur.pos_r*robot->planner_wb->data_cur.ends[i].pos_t;
        data_cur.ends[i].pos_r = data_cur.pos_r*robot->planner_wb->data_cur.ends[i].pos_r;
        data_cur.ends[i].vel_t = data_cur.vel_t + data_cur.vel_r.cross(data_cur.pos_r*robot->planner_wb->data_cur.ends[i].pos_t) + data_cur.pos_r*robot->planner_wb->data_cur.ends[i].vel_t;
        data_cur.ends[i].vel_r = data_cur.vel_r + data_cur.pos_r*robot->planner_wb->data_cur.ends[i].vel_r;
    }
    /*
    */
}

void PlannerCentroid::SavePlan(){
    dymp::Centroid* centroid = GetCentroid();

    static int idx = 0;
    char filename[256];
    sprintf(filename, "plan_centroid.csv");
    FILE* file = fopen(filename, "w");
    idx++;

    fprintf(file, 
        "k, "
        "time, duration, "
        "time_des, duration_des, "
        "cen_pos_t_x, cen_pos_t_y, cen_pos_t_z, "
        "cen_vel_t_x, cen_vel_t_y, cen_vel_t_z, "
        "cen_pos_r_x, cen_pos_r_y, cen_pos_r_z, cen_pos_r_w, "
        "cen_L_x, cen_L_y, cen_L_z, "
        "cen_des_pos_t_x, cen_des_pos_t_y, cen_des_pos_t_z, "
        "cen_des_vel_t_x, cen_des_vel_t_y, cen_des_vel_t_z, "
        "cen_des_pos_r_x, cen_des_pos_r_y, cen_des_pos_r_z, cen_des_pos_r_w, "
        "cen_des_L_x, cen_des_L_y, cen_des_L_z, "
        "cen_I_x, cen_I_y, cen_I_z, "
    );
    for(int i = 0; i < centroid->ends.size(); i++){
        fprintf(file,
            "end%d_pos_t_x, end%d_pos_t_y, end%d_pos_t_z, "
            "end%d_vel_t_x, end%d_vel_t_y, end%d_vel_t_z, "
            "end%d_pos_r_x, end%d_pos_r_y, end%d_pos_r_z, end%d_pos_r_w, "
            "end%d_vel_r_x, end%d_vel_r_y, end%d_vel_r_z, "
            "end%d_des_pos_t_x, end%d_des_pos_t_y, end%d_des_pos_t_z, "
            "end%d_des_vel_t_x, end%d_des_vel_t_y, end%d_des_vel_t_z, "
            "end%d_des_pos_r_x, end%d_des_pos_r_y, end%d_des_pos_r_z, end%d_des_pos_r_w, "
            "end%d_des_vel_r_x, end%d_des_vel_r_y, end%d_des_vel_r_z, "
            "end%d_stiff, "
            "end%d_cmp_x, end%d_cmp_y, "
            "end%d_mom_x, end%d_mom_y, end%d_mom_z, "
            "end%d_des_stiff, "
            "end%d_des_cmp_x, end%d_des_cmp_y, "
            "end%d_des_mom_x, end%d_des_mom_y, end%d_des_mom_z, ",
            i, i, i,
            i, i, i,
            i, i, i, i,
            i, i, i,
            i, i, i,
            i, i, i,
            i, i, i, i,
            i, i, i,
            i,
            i, i,
            i, i, i,
            i,
            i, i,
            i, i, i
            );
    }
    for(int i = 0; i < centroid->ends.size(); i++){
        fprintf(file,
            "end%d_iface, ",
            i
            );
    }
    fprintf(file, "\n");

    int N = mpcPredictionSteps;
    for(int k = 0; k < N; k++){
        dymp::CentroidData& d = data_traj[k];
        dymp::CentroidData& d_des = data_traj_des[k];
        
        fprintf(file,
            "%d, "
            "%f, %f, "
            "%f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, %f, "
            "%f, %f, %f, "
            "%f, %f, %f, ",
            k, 
            d.time, d.duration,
            d_des.time, d_des.duration,
            d.pos_t.x(), d.pos_t.y(), d.pos_t.z(), 
            d.vel_t.x(), d.vel_t.y(), d.vel_t.z(),
            d.pos_r.x(), d.pos_r.y(), d.pos_r.z(), d.pos_r.w(),
            d.L.x(), d.L.y(), d.L.z(),
            d_des.pos_t.x(), d_des.pos_t.y(), d_des.pos_t.z(), 
            d_des.vel_t.x(), d_des.vel_t.y(), d_des.vel_t.z(),
            d_des.pos_r.x(), d_des.pos_r.y(), d_des.pos_r.z(), d_des.pos_r.w(),
            d_des.L.x(), d_des.L.y(), d_des.L.z(),
            d_des.I[0](0,0), d_des.I[0](1,1), d_des.I[0](2,2)
        );
        for(int i = 0; i < d.ends.size(); i++){
            dymp::CentroidData::End& dend = d.ends[i];
            dymp::CentroidData::End& dend_des = d_des.ends[i];
            fprintf(file,
                "%f, %f, %f, "
                "%f, %f, %f, "
                "%f, %f, %f, %f, "
                "%f, %f, %f, "
                "%f, %f, %f, "
                "%f, %f, %f, "
                "%f, %f, %f, %f, "
                "%f, %f, %f, ",
                dend.pos_t.x(), dend.pos_t.y(), dend.pos_t.z(), 
                dend.vel_t.x(), dend.vel_t.y(), dend.vel_t.z(),
                dend.pos_r.x(), dend.pos_r.y(), dend.pos_r.z(), dend.pos_r.w(), 
                dend.vel_r.x(), dend.vel_r.y(), dend.vel_r.z(),
                dend_des.pos_t.x(), dend_des.pos_t.y(), dend_des.pos_t.z(), 
                dend_des.vel_t.x(), dend_des.vel_t.y(), dend_des.vel_t.z(),
                dend_des.pos_r.x(), dend_des.pos_r.y(), dend_des.pos_r.z(), dend_des.pos_r.w(), 
                dend_des.vel_r.x(), dend_des.vel_r.y(), dend_des.vel_r.z()
            );
            fprintf(file,
                "%f, "
                "%f, %f, "
                "%f, %f, %f, "
                "%f, "
                "%f, %f, "
                "%f, %f, %f, ",
                dend.stiff,
                dend.cmp[0], dend.cmp[1],
                dend.moment.x(), dend.moment.y(), dend.moment.z(),
                dend_des.stiff,
                dend_des.cmp[0], dend_des.cmp[1],
                dend_des.moment.x(), dend_des.moment.y(), dend_des.moment.z()
            );
        }
        for(int i = 0; i < d.ends.size(); i++){
            dymp::CentroidData::End& dend_des = d_des.ends[i];
            fprintf(file,
                "%d, ",
                dend_des.iface
            );    
        }
        fprintf(file, "\n");
    }

    fclose(file);
}

void PlannerCentroid::Visualize(cnoid::vnoid::Visualizer* viz, VizInfo& info){
    if(!mpcInputReady)
        return;

    dymp::Centroid* centroid = GetCentroid();
    dymp::World*    world    = centroid->world;
    int N    = (int)world->ticks.size()-1;
    int nend = (int)centroid->ends.size();

    //DSTR << "centroid time: " << cur_time << endl;
    
    for(int k = 0; k < N; k++){
        //planner_lip->biped->CreateSnapshot(planner_lip->world->ticks[k]->time);
        //auto key = (DiMP::CentroidKey*)centroid->traj.GetKeypoint(graph->ticks[k]);
        dymp::CentroidData& d = data_traj[k];

        cnoid::vnoid::Visualizer::Sphere* sphereCom = viz->data->GetSphere(info.iframe, info.isphere++);
        sphereCom->color  = Eigen::Vector3f(1.0f, 0.0f, 1.0f);
        sphereCom->alpha  = 0.5f;
        //sphereCom->pos    = FromSpr(key->var_pos_t->val);
        sphereCom->pos    = d.pos_t;
        sphereCom->radius = 0.015f;

        for(int i = 0; i < nend; i++){
            // skip floating ends
            if(data_traj_des[k].ends[i].iface == -1)
                continue;

            cnoid::vnoid::Visualizer::Box*  boxEnd = viz->data->GetBox(info.iframe, info.ibox++);        
            if(data_traj_des[k].ends[i].iface == -1)
                 boxEnd->color = Eigen::Vector3f(0.5f, 0.0f, 0.5f);
            else boxEnd->color = Eigen::Vector3f(1.0f, 0.0f, 1.0f);
            boxEnd->alpha = 0.5f;
            //boxEnd->pos   = FromSpr(key->ends[i].var_pos_t->val);
            //boxEnd->ori   = FromSpr(key->ends[i].var_pos_r->val);
            boxEnd->pos   = d.ends[i].pos_t;
            boxEnd->ori   = d.ends[i].pos_r;
            boxEnd->size  = Vector3(0.2, 0.1, 0.01);
        }
    }
    
    cnoid::vnoid::Visualizer::Lines* lines = viz->data->GetLines(info.iframe, info.ilines);
    lines->color = Eigen::Vector3f(1.0f, 0.0f, 1.0f);
    lines->alpha = 0.5f;
    lines->width = 1.0f;
    int iv = 0;
    int ii = 0;
    for(int k = 0; k < N-1; k++){
        dymp::real_t t0 = data_traj[k+0].time;
        dymp::real_t t1 = data_traj[k+1].time;

        const int ndiv = 10;
        for(int j = 0; j < ndiv; j++){
            dymp::real_t a  = (dymp::real_t)j/(dymp::real_t)ndiv;
            dymp::real_t t  = (1-a)*t0 + a*t1;
            centroid->CalcState(t, data_traj[k+0], data_traj[k+1], dtmp[1]);

            if(j > 0){
                viz->data->GetLineVertices(info.iframe, info.ilines)[iv+0] = Eigen::Vector3f((float)dtmp[0].pos_t.x(), (float)dtmp[0].pos_t.y(), (float)dtmp[0].pos_t.z());
                viz->data->GetLineVertices(info.iframe, info.ilines)[iv+1] = Eigen::Vector3f((float)dtmp[1].pos_t.x(), (float)dtmp[1].pos_t.y(), (float)dtmp[1].pos_t.z());
                viz->data->GetLineIndices (info.iframe, info.ilines)[ii+0] = iv+0;
                viz->data->GetLineIndices (info.iframe, info.ilines)[ii+1] = iv+1;
                iv += 2;
                ii += 2;

                for(int i = 0; i < nend; i++){
                    viz->data->GetLineVertices(info.iframe, info.ilines)[iv+0] = Eigen::Vector3f((float)dtmp[0].ends[i].pos_t.x(), (float)dtmp[0].ends[i].pos_t.y(), (float)dtmp[0].ends[i].pos_t.z());
                    viz->data->GetLineVertices(info.iframe, info.ilines)[iv+1] = Eigen::Vector3f((float)dtmp[1].ends[i].pos_t.x(), (float)dtmp[1].ends[i].pos_t.y(), (float)dtmp[1].ends[i].pos_t.z());
                    viz->data->GetLineIndices (info.iframe, info.ilines)[ii+0] = iv+0;
                    viz->data->GetLineIndices (info.iframe, info.ilines)[ii+1] = iv+1;
                    iv += 2;
                    ii += 2;
                }
            }
            dtmp[0] = dtmp[1];
        }
    }
    lines->numVertices = iv;
    lines->numIndices  = ii;
    info.ilines++;
    /*
    auto key_f = (DiMP::CentroidKey*)centroid->traj.GetKeypoint(graph->ticks.back());
    real_t tf = key_f->tick->time;
    Visualizer::Lines* lines = viz->data->GetLines(info.iframe, info.ilines);
    lines->color = Vector3f(1.0f, 0.0f, 1.0f);
    lines->alpha = 0.5f;
    lines->width = 1.0f;
    int iv = 0;
    int ii = 0;
    vec3_t pc, pcprev;
    vector<vec3_t> pe, peprev;
    pe    .resize(nend);
    peprev.resize(nend);
    for(real_t t = 0.0; t <= tf; t += 0.05){
        vec3_t p, v, w, a;
        quat_t q;
		centroid->ComState(t, p, v, a);
        pc = p;

        for(int i = 0; i < nend; i++){
	        centroid->EndState(t, i, p, q, v, w);
            pe[i] = p;
        }
        
        if(t != 0.0){
            viz->data->GetLineVertices(info.iframe, info.ilines)[iv+0] = Vector3f((float)pcprev.x, (float)pcprev.y, (float)pcprev.z);
            viz->data->GetLineVertices(info.iframe, info.ilines)[iv+1] = Vector3f((float)pc    .x, (float)pc    .y, (float)pc    .z);
            viz->data->GetLineIndices (info.iframe, info.ilines)[ii+0] = iv+0;
            viz->data->GetLineIndices (info.iframe, info.ilines)[ii+1] = iv+1;
            pcprev = pc;
            iv += 2;
            ii += 2;

            for(int i = 0; i < nend; i++){
                viz->data->GetLineVertices(info.iframe, info.ilines)[iv+0] = Vector3f((float)peprev[i].x, (float)peprev[i].y, (float)peprev[i].z);
                viz->data->GetLineVertices(info.iframe, info.ilines)[iv+1] = Vector3f((float)pe    [i].x, (float)pe    [i].y, (float)pe    [i].z);
                viz->data->GetLineIndices (info.iframe, info.ilines)[ii+0] = iv+0;
                viz->data->GetLineIndices (info.iframe, info.ilines)[ii+1] = iv+1;
                iv += 2;
                ii += 2;
                peprev[i] = pe[i];
            }
        }
    }
    lines->numVertices = iv;
    lines->numIndices  = ii;
    info.ilines++;
    */
}

void PlannerCentroid::CalcQuadWeight(dymp::real_t tf, dymp::real_t& Vconst, dymp::Vector& Vy, dymp::Matrix& Vyy){
    int N = mpcPredictionSteps;
    int k = 0;
    for( ; k < N; k++){
        if(tf < data_traj[k+1].time)
            break;
    }

    dymp::real_t s;
    if(k == N){
        k = N-1;
        s = 1.0;
    }
    else{
        dymp::real_t t0 = data_traj[k+0].time;
        dymp::real_t t1 = data_traj[k+1].time;
        dymp::real_t h = t1 - t0;
        s = (tf - t0)/h;
    }

    int ny = Vy.n;

    Vconst = s*V[k+1] + (1-s)*V[k+0];
    //Vconst = 0.0;
    
    // last element of Vx and last row/column of Vxx correspond to time variable of centroid
    // these parts are not copied to Vy, Vyy
    vec_copy(Vx[k+0].SubVector(0,ny), Vy, 1-s);
    vec_add (Vx[k+1].SubVector(0,ny), Vy, s  );
    //vec_clear(Vy);
    
    mat_copy(Vxx[k+0].SubMatrix(0,0,ny,ny), Vyy, 1-s);
    mat_add (Vxx[k+1].SubMatrix(0,0,ny,ny), Vyy, s  );
    
    /* 
    // clear entries outside (12,12) upper-left block
    for(int i = 0; i < Vyy.n; i++){
        if(i >= 12)
            Vy(i) = 0.0;
        for(int j = 0; j < Vyy.m; j++){
            if(i >= 12 || j >= 12)
                Vyy(i,j) = 0.0;
        }
    }
    */
    /*
    // fxt = s*I + (1-s)*fxk
    mat_identity(I);
    mat_copy(I, fxt, s);
    mat_add (fx[k], fxt, 1-s);
    
    //
    Vconst = V[k+1];
    mattr_vec_mul(fxt, Vx[k+1], Vy, 1.0, 0.0);
    mattr_mat_mul(fxt, Vxx[k+1], fxtr_Vxx, 1.0, 0.0);
    mat_mat_mul(fxtr_Vxx, fxt, Vyy, 1.0, 0.0);
    */
}

}
}
