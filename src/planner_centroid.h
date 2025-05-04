#pragma once

#include "planner.h"

namespace dymp{
namespace mpc{

class PlannerCentroid;

class PlannerThreadCentroid : public PlannerThread{
public:
	PlannerCentroid*  planner_centroid;
	dymp::Centroid*     centroid;

public:
	virtual void Init ();
	virtual void Setup();

	PlannerThreadCentroid(Planner* _planner);
};

class PlannerCentroid : public Planner,  public dymp::CentroidCallback{
public:
	dymp::Centroid::Param             param;
	vector<dymp::Centroid::End>       ends;
	vector<dymp::Centroid::Face>      faces;
	
	dymp::real_t   desDuration;
	dymp::real_t   initialWeight;
	dymp::real_t   terminalWeight;
	dymp::vec3_t   centroidPosWeight;
	dymp::vec3_t   centroidVelWeight;
	dymp::vec3_t   centroidOriWeight;
	dymp::vec3_t   centroidLWeight;
	dymp::real_t   timeWeight;
	dymp::real_t   durationWeight;
	dymp::vec2_t   endPosWeight;
	dymp::vec2_t   endVelWeight;
	dymp::real_t   endOriWeight;
	dymp::real_t   endAngvelWeight;
	dymp::real_t   endStiffnessWeight;
	dymp::vec2_t   endCopWeight;
	dymp::vec2_t   endCmpWeight;
	dymp::real_t   endTorsionWeight;
	dymp::vec3_t   endForceWeight;
	dymp::vec3_t   endMomentWeight;
    bool           enableFeedback;
	bool           saveTraj;
	
	dymp::real_t  cur_time;
	int     cur_phase;
	dymp::CentroidData  data_cur, data_ref, dtmp[2];
	vector<dymp::CentroidData>  data_traj, data_traj_des;
	vector<dymp::real_t> V;
	vector<dymp::Vector> Vx;
	vector<dymp::Matrix> Vxx;
	dymp::Vector  Vx_int, sconv;
	dymp::Matrix  Vxx_int, S, Str_Vxx;
	
	dymp::WholebodyData  data_wb;

	bool    phaseSwitched;
	dymp::real_t  nextPlanTime;

public:
	dymp::Centroid* GetCentroid();
	void SavePlan();
	void SaveTraj();
	void CalcQuadWeight(dymp::real_t tf, dymp::real_t wf, dymp::real_t& Vconst, dymp::Vector& Vy, dymp::Matrix& Vyy);

	virtual PlannerThread*  CreateThread();
	virtual void  UpdateInput  ();
	virtual void  UpdateState();
	virtual void  UpdateGain   ();
	virtual bool  IsMpcUpdateCycle();

	virtual void GetInitialState(dymp::CentroidData& d);
	virtual void GetDesiredState(int k, dymp::real_t t, dymp::CentroidData& d);

	virtual void Read     (const YAML::Node& node);
	virtual void InitState();
	virtual void Observe  ();
	virtual void Visualize(cnoid::vnoid::Visualizer* viz, cnoid::vnoid::VizInfo& info);

	PlannerCentroid();
};

}
}
