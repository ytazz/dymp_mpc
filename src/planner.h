#pragma once

#include <robot.h>
#include <visualizer.h>

#include "base.h"

#include "criticalsection.h"
#include "event.h"
#include "thread.h"
#include "timer.h"

#include <dymp.h>

namespace dymp{
namespace mpc{

class MyRobot;

const dymp::real_t pi  = 3.1415926535;

inline dymp::vec3_t quat_diff(const dymp::quat_t& q0, const dymp::quat_t& q1){
    /*
	quat_t qdiff = q0.Conjugated()*q1;
    real_t theta = qdiff.Theta();
    if(theta > pi)
        theta -= 2*pi;
    vec3_t w = theta*qdiff.Axis();
	*/
	Eigen::AngleAxisd qdiff(q0.conjugate()*q1);
	dymp::real_t theta = qdiff.angle();
	if(theta > pi)
        theta -= 2*pi;
    dymp::vec3_t w = theta*qdiff.axis();
	
    //return w;
    //return q0*w;
    return (1.0/2.0)*(q0*w + q1*w);
}

inline void subvec3_set(dymp::Vector& v, int& idx, const dymp::vec3_t& sv){
    v(idx++) = sv[0];
    v(idx++) = sv[1];
    v(idx++) = sv[2];
}

inline dymp::vec2_t subvec2_get(dymp::Vector& v, int& idx){
    dymp::vec2_t sv(v(idx+0), v(idx+1));
    idx += 2;
    return sv;
}

inline dymp::vec3_t subvec3_get(dymp::Vector& v, int& idx){
    dymp::vec3_t sv(v(idx+0), v(idx+1), v(idx+2));
    idx += 3;
    return sv;
}

class Planner;
class PlannerThread : public Thread{
public:
	int       id;
	int       cnt;
	Planner*  planner;

	CriticalSection  csMpc;
	Event            evStart;
	Event            evDone;
	Event            evInitial;
	Event            evRunning;

	dymp::World*  world;

	volatile int     mpcIterCount;
    int              timeIter;
    int              timeTotal;

	dymp::Timer  timer;

    FILE*  fileComptime;
    
public:
	virtual void Func();

	virtual void Init ();
	virtual void Setup() = 0;
    virtual void SaveComptime();

	PlannerThread(Planner* _planner);
};

class Planner{
public:
	int     mpcPredictionSteps;
	double  mpcTimestep;
	double  mpcTimestepScale;
    int     mpcUpdateCycle;
	int     mpcNumThreads;
	bool    mpcDelayMode;
	bool    mpcFeedback;
	double  correctionRate;
	double  minStepSize    ;
	double  maxStepSize    ;
	double  cutoffStepSize ;
	double  regularization ;
	double  stateRegularization ;
	bool    fixInitialState;
	bool    fixInitialInput;
	bool    parallelize    ;
	bool    enableSparse   ;
	bool    verbose        ;
	bool    savePlan       ;
    bool    saveComptime   ;

	MyRobot*  robot;

	string  name;

	vector<PlannerThread*>  threads;
	int     threadIndex;
	bool    mpcInputReady;
	double  mpcInitialTime;  //< initial time of MPC prediction horizon
	volatile int     mpcNumIter;

    int     nx, ny, nu;
	dymp::Vector  dx, du, u, uref;
	dymp::Matrix  Quuinv_Qux;

public:
	void  RunThreads ();
	void  SetupThread();
	void  WaitThread ();
	void  GetThread  ();
	
	virtual PlannerThread* CreateThread() = 0;

	virtual void Read     (const YAML::Node& node);
	virtual void Init     ();
	virtual void InitState();
	virtual void Observe  ();
	virtual void InterpolateReference();
	virtual void UpdateInput();
	virtual void UpdateState();
	virtual void UpdateGain ();
	virtual bool IsMpcUpdateCycle();
	virtual void FromRobot();
	virtual void ToRobot  ();
    
	Planner();
};

}
}
