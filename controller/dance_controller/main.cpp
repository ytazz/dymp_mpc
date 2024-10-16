#include <cnoid/SimpleController>
#include <cnoid/Body>

#include <vector>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

#include <myrobot.h>

using namespace cnoid;
using namespace cnoid::vnoid;

class DympMpcDanceController : public SimpleController{
public:
	dymp::mpc::MyRobot*  robot;

public:
    virtual bool configure(SimpleControllerConfig* config){
        return true;
    }

	virtual bool initialize(SimpleControllerIO* io){
		robot = new dymp::mpc::MyRobot();
		robot->Init(io);

		return true;
	}

	virtual bool control()	{
		robot->Control();
		return true;
	}
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(DympMpcDanceController)
