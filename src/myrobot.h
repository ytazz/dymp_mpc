#pragma once

#include <robot.h>
#include <visualizer.h>

#include "planner.h"

namespace dymp{
namespace mpc{

class PlannerCentroid;
class PlannerWholebody;
class Kinematics;
class Poseseq;

class MyRobot : public cnoid::vnoid::Robot{
public:
	cnoid::vnoid::Timer          timer;
    cnoid::vnoid::Param          param;
    cnoid::vnoid::Centroid       centroid;
    cnoid::vnoid::Base           base;
    vector<cnoid::vnoid::Hand>   hand;
    vector<cnoid::vnoid::Foot>   foot;
    vector<cnoid::vnoid::Joint>  joint;
    int            save_cycle;

    PlannerWholebody*  planner_wb;
    PlannerCentroid*   planner_centroid;
  	Kinematics*        kinematics;
	Poseseq*           poseseq;

    cnoid::vnoid::Visualizer* viz;

    FILE*  fileLog[4];
    FILE*  fileFootstep[4][2];
    FILE*  fileTiming;
    bool   contactPrev[4][2];

    dymp::vec3_t vprev[4];
    dymp::vec3_t Lprev[4];
    dymp::real_t tprev;
    dymp::real_t tprevSwitch;
	
public:
    void  Read     (const YAML::Node& node);
    void  Visualize(const cnoid::vnoid::Timer& timer);
    void  SaveLog  ();

    virtual void  Init   (cnoid::SimpleControllerIO* io);
	virtual void  Control();
	
	MyRobot();

};

}
}