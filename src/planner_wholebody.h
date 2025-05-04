#pragma once

#include "planner.h"

namespace dymp{
namespace mpc{

class PlannerCentroid;
class PlannerWholebody;

class PlannerThreadWholebody : public PlannerThread, public dymp::DDPCallback{
public:
	PlannerWholebody*  planner_wb;
	//dymp::Wholebody*   wb;
	dymp::Wholebody*   wb;
	int    nx, nu, nj;
	dymp::real_t h, h2;

public:
	virtual void Init ();
	virtual void Setup();

	virtual void mat_fx_mul  (dymp::Matrix& V , dymp::Matrix& fx, dymp::Matrix& y, bool add);
	virtual void mat_fu_mul  (dymp::Matrix& V , dymp::Matrix& fu, dymp::Matrix& y, bool add);
	virtual void fxtr_mat_mul(dymp::Matrix& fx, dymp::Matrix& V , dymp::Matrix& y, bool add);
	virtual void futr_mat_mul(dymp::Matrix& fu, dymp::Matrix& V , dymp::Matrix& y, bool add);
	virtual void fxtr_vec_mul(dymp::Matrix& fx, dymp::Vector& v , dymp::Vector& y, bool add);
	virtual void futr_vec_mul(dymp::Matrix& fu, dymp::Vector& v , dymp::Vector& y, bool add);

	PlannerThreadWholebody(Planner* _planner);
};

class PlannerWholebody : public Planner,  public dymp::WholebodyCallback{
public:
	bool    useLd;
	//bool    useJerk;
	int     inputMode;
	bool    usePoseseq;
	bool    useCentroid;
	bool    enableWarmstart;

	double   initialWeight;
	double   terminalWeight;
	double   freeWeight;
	Vector3  centroidPosWeight;
	Vector3  centroidVelWeight;
	Vector3  centroidOriWeight;
	Vector3  centroidLWeight;
	Vector3  baseOriWeight;
	Vector3  baseAngvelWeight;
	Vector3  baseAngaccWeight;
	Vector3  footPosWeight;
	Vector3  footOriWeight;
	Vector3  footVelWeight;
	Vector3  footAngvelWeight;
	Vector3  footAccWeight;
	Vector3  footAngaccWeight;
	Vector3  endForceWeight;
	Vector3  endMomentWeight;
	Vector3  endForcerateWeight;
	Vector3  endMomentrateWeight;
	vector<double>  qWeight;
	vector<double>  qdWeight;
	vector<double>  qddWeight;
	vector<double>  qdddWeight;
	//vector<double>  qMin, qdMin, qddMin;
	//vector<double>  qMax, qdMax, qddMax;
    vector<double>  qRangeWeight, qdRangeWeight, qddRangeWeight;

	dymp::WholebodyData  data_cur, data_ref, data_des;
	vector<dymp::WholebodyData>  data_traj, data_traj_des;
	vector<double>  qd_prev;
	double  time_prev;
	//vector<vec3_t>  force_mod, moment_mod;
	
	// used for comparison only
	dymp::WholebodyData  data_des_poseseq, data_des_centroid;

public:	
	dymp::Wholebody* GetWholebody();
	void SavePlan();

	virtual PlannerThread*  CreateThread();

	virtual void GetInitialState(dymp::WholebodyData& d);
	virtual void GetDesiredState(int k, dymp::real_t t, dymp::WholebodyData& d);

	virtual void Read     (const YAML::Node& node);
	virtual void InitState();
	virtual void Observe  ();
	virtual void InterpolateReference();
	virtual void UpdateInput();
	virtual void UpdateGain ();
	virtual void ToRobot  ();
	virtual void Visualize(cnoid::vnoid::Visualizer* viz, cnoid::vnoid::VizInfo& info);

	PlannerWholebody();
};

}
}
