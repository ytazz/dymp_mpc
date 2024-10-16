#pragma once

#include "base.h"

#include <dymp.h>

#include <string>
using namespace std;

namespace dymp{
namespace mpc{

class MyRobot;

class Poseseq{
public:
    struct IKLink{
        std::string name;
        int    index;
        bool   isBaseLink;
        bool   isTouching;
        bool   isToeContact;
        bool   isHeelContact;

        dymp::vec3_t   pos;
        dymp::quat_t   ori;
        dymp::vec3_t   angle;
        dymp::vec3_t   vel;
        dymp::vec3_t   angvel;
        dymp::vec3_t   partingDirection;

        IKLink();
    };
    struct Pose{
        string  type;
        string  name;
        vector<int>     joints;
        vector<int>     spJoints;
        vector<double>  q, qd;
        vector<IKLink>  ikLinks;

        vector<int>     jointIndex;
        vector<bool>    isSpJoint;
        vector<int>     endIndex;
        int             baseIndex;

        Pose();
    };
    struct Key{
        double  time;
        double  maxTransitionTime;
        Pose    pose;

        int     basePrev, baseNext;
        vector<int>  endPrev, endNext;
        vector<int>  jointPrev, jointNext;

        Key();
    };

    double  playSpeed;
    double  minContactDuration;
    double  maxDuration;
    int     initialPhase;
    string  pseqFile;
    
    MyRobot*     robot;
    vector<Key>  keys;

public:
    void           Read   (const YAML::Node& node);
    bool           Load   (const YAML::Node& node);
    void           Init   ();
    pair<int,int>  Find   (double t);
    //double         GetTime(double t0, int k);
    double         GetTime(int k);
    void           Setup  (double t, dymp::Wholebody* wb, dymp::WholebodyData& d);
	
	Poseseq();

};

}
}