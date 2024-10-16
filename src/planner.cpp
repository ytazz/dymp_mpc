#include "base.h"
#include "planner.h"
#include "myrobot.h"

#include <visualizer.h>
#include <rollpitchyaw.h>

const dymp::real_t pi  = 3.1415926535;

namespace dymp{
namespace mpc{

PlannerThread::PlannerThread(Planner* _planner){
    planner = _planner;
	//graph = new DiMP::Graph();
    world = new dymp::World();

    evRunning.Create(true);
    evInitial.Create(true);
}

void PlannerThread::Init(){
    int    N   = planner->mpcPredictionSteps;
    real_t dt  = planner->mpcTimestep;
    real_t dtc = planner->mpcTimestepScale;
    real_t t   = 0.0;
	for(int k = 0; k <= N; k++){
		//new DiMP::Tick(graph, t, "");
        new dymp::Tick(world, k, t);
        t += dt;
        dt *= dtc;
        dt = std::min(0.1, dt);
	}
    
    world->solver->param.hastyStepSize = true;
	world->solver->param.method = dymp::Solver::Method::DDP;
	world->solver->param.minStepSize     = planner->minStepSize    ;
	world->solver->param.maxStepSize     = planner->maxStepSize    ;
	world->solver->param.cutoffStepSize  = planner->cutoffStepSize ;
    world->solver->param.regularization  = planner->regularization ;
	world->solver->param.stateRegularization  = planner->stateRegularization ;
	world->solver->param.fixInitialState = planner->fixInitialState;
    world->solver->param.fixInitialInput = planner->fixInitialInput;
    world->solver->param.parallelize     = planner->parallelize    ;
    world->solver->param.verbose         = id == 0 ? planner->verbose : false;

    world->Init();

    world->solver->SetCorrection(dymp::ID(), planner->correctionRate);

    if(planner->saveComptime){
        stringstream ss;
        ss << planner->name << id << ".csv";
        fileComptime = fopen(ss.str().c_str(), "w");
    }
}

void PlannerThread::Func(){
    cnt = 0;
    while(true){
        //printf("planner: %s  id: %d  wait start\n", planner->name.c_str(), id);
        if(!evStart.Wait())
            continue;
        //printf("planner: %s  id: %d  start running\n", planner->name.c_str(), id);
        evRunning.Set();

        //csMpc.Enter();
        timeTotal = 0;
        while(mpcIterCount < planner->mpcNumIter){
	        timer.CountUS();
            world->Step();
            timeIter = timer.CountUS();
            timeTotal += timeIter;
            //printf("planner: %s  id: %d  time: %d\n", planner->name.c_str(), id, T);
        
            mpcIterCount++;
            //printf("planner: %s  id: %d  iter: %d cnt: %d\n", planner->name.c_str(), id, mpcIterCount, cnt);

            if(planner->saveComptime){
                SaveComptime();
            }
            //costHistory.push_back(graph->solver->status.obj);
        }
        //csMpc.Leave();
        evRunning.Reset();

        //printf("planner: %s  id: %d  done  time: %d\n", planner->name.c_str(), id, T);
        evDone.Set();
        cnt++;
    }
}

void PlannerThread::SaveComptime(){
    fprintf(fileComptime, "%d, %d, %d, ", mpcIterCount, timeIter, timeTotal);
    fprintf(fileComptime,
        "%d, %f, %f, %d, %d, %d, %d, "
        "%d, %d, %d, %d, "
        "%d, %d, %d, %d\n",
        world->solver->status.iterCount,
		world->solver->status.stepSize,
		world->solver->status.obj,
		world->solver->status.timePre,
		world->solver->status.timeDir,
		world->solver->status.timeStep,
		world->solver->status.timeMod,
        world->solver->status.timeTrans,
        world->solver->status.timeCost,
        world->solver->status.timeCostGrad,
        world->solver->status.timeBack,
        world->TPrepare,
		world->TPrepareStep,
		world->TStep,
		world->TFinish
        );
}

Planner::Planner(){
    correctionRate     = 0.0;
    mpcUpdateCycle     = 25;
    mpcNumThreads      = 1;
    mpcNumIter         = 10;
	mpcPredictionSteps = 10;
	mpcTimestep        = 0.025;
    mpcTimestepScale   = 1.0;
    mpcDelayMode       = false;
    mpcFeedback        = false;

    mpcInputReady  = false;
	mpcInitialTime = 0.0;

    savePlan     = false;
    saveComptime = false;
}

void Planner::Read(const YAML::Node& node){
	ReadInt   (mpcUpdateCycle    , node["mpc_update_cycle"    ]);
    ReadInt   (mpcNumThreads     , node["mpc_num_threads"     ]);
	ReadInt   (mpcPredictionSteps, node["mpc_prediction_steps"]);
	ReadDouble(mpcTimestep       , node["mpc_timestep"        ]);
    ReadDouble(mpcTimestepScale  , node["mpc_timestep_scale"  ]);
    ReadBool  (mpcDelayMode      , node["mpc_delay_mode"      ]);
    ReadBool  (mpcFeedback       , node["mpc_feedback"        ]);

    int niter;
    ReadInt   (niter, node["mpc_num_iter"        ]);
    mpcNumIter = niter;

    ReadDouble(correctionRate    , node["correction_rate"    ]);

    ReadDouble(minStepSize    , node["min_step_size"    ]);
	ReadDouble(maxStepSize    , node["max_step_size"    ]);
	ReadDouble(cutoffStepSize , node["cutoff_step_size" ]);
    ReadDouble(regularization , node["regularization"   ]);
	ReadDouble(stateRegularization , node["state_regularization"   ]);
	ReadBool  (fixInitialState, node["fix_initial_state"]);
    ReadBool  (fixInitialInput, node["fix_initial_input"]);
    ReadBool  (parallelize    , node["parallelize"      ]);
	ReadBool  (enableSparse   , node["enable_sparse"    ]);
	ReadBool  (verbose        , node["verbose"          ]);
    ReadBool  (savePlan       , node["save_plan"        ]);
    ReadBool  (saveComptime   , node["save_comptime"    ]);
}

void Planner::Init(){        
	threads.resize(mpcNumThreads);
	for(int i = 0; i < mpcNumThreads; i++){
		threads[i] = CreateThread();
		threads[i]->id = i;
	}
    threadIndex = 0;
}

void Planner::InitState(){

}

void Planner::RunThreads(){
    threadIndex = 0;
    for(int i = 0; i < mpcNumThreads; i++){
        threads[i]->Init();
        threads[i]->mpcIterCount = 0;
        threads[i]->evInitial.Set();
        threads[i]->Run();
    }
}

void Planner::SetupThread(){
    mpcInitialTime = robot->timer.time;
    Observe();
   
    PlannerThread* th = threads[threadIndex];
    th->Setup();
    th->mpcIterCount = 0;
    th->evStart.Set();
}

void Planner::WaitThread(){
    PlannerThread* th = threads[threadIndex];
    while(!th->evDone.Wait())
        ;
}

void Planner::GetThread(){
    PlannerThread* th = threads[threadIndex];
    
    UpdateGain();
    //UpdateInput();

    mpcInputReady = true;
}

bool Planner::IsMpcUpdateCycle(){
    return robot->timer.control_count % mpcUpdateCycle == 0;
}

void Planner::Observe(){

}

void Planner::InterpolateReference(){

}

void Planner::UpdateGain(){

}

void Planner::UpdateInput(){

}

void Planner::UpdateState(){

}

void Planner::FromRobot(){
    if(!IsMpcUpdateCycle())
        return;

    if(mpcDelayMode){
        PlannerThread* th = threads[threadIndex];
        if(th->evInitial.IsSet()){
            th->evInitial.Reset();
        }
        else{
            WaitThread();
            GetThread();
        }

        SetupThread();

        if(++threadIndex == mpcNumThreads)
            threadIndex = 0;
    }
    else{
        SetupThread();

        WaitThread();
        GetThread();
    }
}

void Planner::ToRobot(){
    Observe();
    InterpolateReference();
    if(mpcInputReady){
        UpdateInput();
    }
    UpdateState();
}

}
}
