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
	dymp::vec3_t   endPosWeight;
	dymp::vec3_t   endVelWeight;
	dymp::vec3_t   endOriWeight;
	dymp::vec3_t   endAngvelWeight;
	dymp::real_t   endStiffnessWeight;
	dymp::vec2_t   endCmpWeight;
	dymp::vec3_t   endMomentWeight;
	
	dymp::real_t  cur_time;
	int     cur_phase;
	dymp::CentroidData  data_ref, dtmp[2];
	vector<dymp::CentroidData>  data_traj, data_traj_des;
	vector<dymp::real_t> V;
	vector<dymp::Vector> Vx;
	vector<dymp::Matrix> Vxx, fx;
	dymp::Matrix  fxt, fxtr_Vxx, I;
	
	dymp::WholebodyData  data_wb;

	bool    phaseSwitched;
	dymp::real_t  nextPlanTime;

public:
	dymp::Centroid* GetCentroid();
	void SavePlan();
	void CalcQuadWeight(dymp::real_t tf, dymp::real_t& Vconst, dymp::Vector& Vy, dymp::Matrix& Vyy);

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
	virtual void Visualize(cnoid::vnoid::Visualizer* viz, VizInfo& info);

	PlannerCentroid();
};

}
}
