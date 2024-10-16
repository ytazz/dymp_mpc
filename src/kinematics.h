#pragma once

#include "base.h"
#include "poseseq.h"

#include <dymp.h>

namespace dymp{
namespace mpc{

class MyRobot;

class Kinematics{
public:
	struct Link{
		enum{
			Hips, ChestY, ChestP, HeadY, HeadP,
			UpperArmRP, UpperArmRR, UpperArmRY, LowerArmRP, LowerArmRY, WristRP, WristRY, HandRR,
			UpperArmLP, UpperArmLR, UpperArmLY, LowerArmLP, LowerArmLY, WristLP, WristLY, HandLR,
			UpperLegRY, UpperLegRR, UpperLegRP, LowerLegRP, FootRR, FootRP,
			UpperLegLY, UpperLegLR, UpperLegLP, LowerLegLP, FootLR, FootLP,		
			Num
		};
	};
	
	struct Joint{
		enum{
			UpperLegLY, UpperLegLR, UpperLegLP, LowerLegLP, FootLR, FootLP,		
			ChestY, ChestP, 
			UpperLegRY, UpperLegRR, UpperLegRP, LowerLegRP, FootRR, FootRP,
			HeadY, HeadP,
			UpperArmLP, UpperArmLR, UpperArmLY, LowerArmLP, LowerArmLY, WristLP, WristLY, HandLR,
			UpperArmRP, UpperArmRR, UpperArmRY, LowerArmRP, LowerArmRY, WristRP, WristRY, HandRR,
			Num
		};
	};
	struct End{
		enum{
			ChestP,
			HandR,
			HandL,
			FootR,
			FootL,
			Num
		};
	};
	struct Chain{
		enum{
			Torso,
			ArmR,
			ArmL,
			LegR,
			LegL,
			Num
		};
	};

	dymp::vec3_t armBase[2];
	dymp::vec3_t legBase[2];
	dymp::vec3_t wristToHand[2];
	dymp::vec3_t ankleToFoot[2];
	dymp::real_t elbowYaw[2];
	dymp::real_t torsoLength;
	dymp::real_t headOffset;
	dymp::real_t upperArmLength;
	dymp::real_t lowerArmLength;
	dymp::real_t upperLegLength;
	dymp::real_t lowerLegLength;
	dymp::vec3_t handOffset[2];

	dymp::real_t verticalHipJointOffset;
    dymp::real_t lateralHipJointOffset ;
    dymp::real_t lateralKneeOffset     ;
    dymp::real_t thighLength           ;
    dymp::real_t shankLength           ;

	MyRobot*  robot;

public:
	void SetupStandStill  (dymp::Wholebody* wb, dymp::WholebodyData& d);
	void SetupStandStill  (dymp::Centroid* centroid, dymp::CentroidData& d);
	void SetupFromCentroid(dymp::real_t t, dymp::Centroid* centroid, const vector<dymp::CentroidData>& d_cen_array, dymp::Wholebody* wb, dymp::WholebodyData& d);
	void SetupWeights     (dymp::WholebodyData& d, bool is_initial, bool is_terminal);
	void SetupWeights     (dymp::CentroidData&  d, bool is_initial, bool is_terminal);
	void Convert          (const dymp::WholebodyData& d_wb, dymp::CentroidData& d, int idiv);
	bool CalcIK           (const dymp::vec3_t& pos, const dymp::quat_t& ori, double sign, vector<double>& q);

	virtual void Read  (const YAML::Node& node);
	virtual void Init  (dymp::Wholebody* wb);

	Kinematics();
};

}
}
