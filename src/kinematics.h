#pragma once

#include "base.h"
#include "poseseq.h"

#include <dymp.h>

namespace dymp{
namespace mpc{

class MyRobot;

class Kinematics{
public:
	struct End{
		enum{
			FootR,
			FootL,
			Num
		};
	};

    dymp::real_t verticalHipJointOffset;
    dymp::real_t lateralHipJointOffset ;
    dymp::real_t lateralKneeOffset     ;
    dymp::real_t thighLength           ;
    dymp::real_t shankLength           ;

    std::string  modelFilename;
    std::vector<int>     legJointIndices[2];
    int                  footLinkIndex[2];
    dymp::vec3_t         ankleToFoot[2];
	std::vector<real_t>  standStillJointAngle;

	MyRobot*  robot;

public:
	void LoadLinksFromBody(dymp::Wholebody* wb);
    void SetupStandStill  (dymp::Wholebody* wb, dymp::WholebodyData& d);
	void SetupStandStill  (dymp::Centroid* centroid, dymp::CentroidData& d);
	void SetupFromCentroid(dymp::real_t t, dymp::Centroid* centroid, const vector<dymp::CentroidData>& d_cen_array, dymp::Wholebody* wb, dymp::WholebodyData& d);
	void SetupWeights     (dymp::WholebodyData& d, bool is_initial, bool is_terminal);
	void SetupWeights     (dymp::CentroidData&  d, bool is_initial, bool is_terminal);
	void Convert          (const dymp::WholebodyData& d_wb, dymp::Centroid* cen, dymp::CentroidData& d, int idiv);
	bool CalcIK           (const dymp::vec3_t& pos, const dymp::quat_t& ori, double sign, vector<double>& q);
    
	virtual void Read  (const YAML::Node& node);
	virtual void Init  (dymp::Wholebody* wb);

	Kinematics();
};

}
}
