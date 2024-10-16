#include "poseseq.h"
#include "myrobot.h"
#include "kinematics.h"
#include "planner_wholebody.h"

#include <rollpitchyaw.h>

using namespace std;

const dymp::real_t pi   = 3.1415926535;
const dymp::real_t inf  = numeric_limits<dymp::real_t>::max();
const int          iinf = numeric_limits<int>::max();

namespace dymp{
namespace mpc{

Poseseq::Poseseq(){
    playSpeed          = 1.0;
    minContactDuration = 0.2;
    maxDuration        = 0.5;
    initialPhase       = 0;
}

Poseseq::IKLink::IKLink(){            
    pos              = dymp::zero3;
    ori              = dymp::unit_quat();
    angle            = dymp::zero3;
    vel              = dymp::zero3;
    angvel           = dymp::zero3;
    partingDirection = dymp::ez;
    
    name = "";
    index = 0;
    isBaseLink    = false;
    isTouching    = false;
    isToeContact  = false;
    isHeelContact = false;
}

Poseseq::Pose::Pose(){
    type = "";
    name = "";
}

Poseseq::Key::Key(){
    time = 0.0;
    maxTransitionTime = 0.0;
}

void Poseseq::Read(const YAML::Node& node){
    ReadDouble(playSpeed         , node["play_speed"]          );
    ReadDouble(minContactDuration, node["min_contact_duration"]);
    ReadDouble(maxDuration       , node["max_duration"]        );
    ReadInt   (initialPhase      , node["initial_phase"]       );
    ReadString(pseqFile          , node["pseq_file"]           );
}

bool Poseseq::Load(const YAML::Node& node){
    string type, name, targetBody;
    ReadString(type, node["type"]);
    ReadString(name, node["name"]);
    ReadString(targetBody, node["targetBody"]);

    YAML::Node refsNode = node["refs"];
    for(auto&& keyNode : refsNode){
        Key key;
        ReadDouble(key.time, keyNode["time"]);
        ReadDouble(key.maxTransitionTime, keyNode["maxTransitionTime"]);

        YAML::Node poseNode = keyNode["refer"];

        ReadString(key.pose.type, poseNode["type"]);
        ReadString(key.pose.name, poseNode["name"]);
        ReadVectorInt   (key.pose.joints  , poseNode["joints"  ]);
        ReadVectorInt   (key.pose.spJoints, poseNode["spJoints"]);
        ReadVectorDouble(key.pose.q, poseNode["q"]);

        YAML::Node ikLinksNode = poseNode["ikLinks"];
        for(auto&& ikLinkNode : ikLinksNode){
            IKLink lnk;
            ReadString (lnk.name, ikLinkNode["name"]);
            ReadInt    (lnk.index, ikLinkNode["index"]);
            ReadBool   (lnk.isBaseLink, ikLinkNode["isBaseLink"]);

            Vector3 trn;
            Matrix3 rot;
            ReadVector3(trn, ikLinkNode["translation"]);
            ReadMatrix3(rot, ikLinkNode["rotation"]);
            lnk.pos   = trn;
            lnk.ori   = Quaternion(rot);
            lnk.angle = cnoid::vnoid::ToRollPitchYaw(lnk.ori);

            ReadBool   (lnk.isTouching , ikLinkNode["isTouching"]);
            ReadVector3(lnk.partingDirection, ikLinkNode["partingDirection"]);

            // detect toe/heel contact
            const dymp::real_t th = 0.01;
            lnk.isToeContact  = lnk.isTouching && (lnk.angle.y() >  th);
            lnk.isHeelContact = lnk.isTouching && (lnk.angle.y() < -th);

            key.pose.ikLinks.push_back(lnk);
        }

        //YAML::Node jointsNode = poseNode["joints"];

        keys.push_back(key);
    }

    return true;
}

void Poseseq::Init(){
    dymp::Wholebody* wb = robot->planner_wb->GetWholebody();

    int njoint = wb->joints.size();
    int nend   = wb->ends.size();

    string baseName = "WAIST";
    vector<string>  endNames = {"", "", "", "R_ANKLE_P", "L_ANKLE_P"};

    for(Key& key : keys){
        key.pose.endIndex  .resize(nend  , -1);
        key.pose.baseIndex = -1;

        for(int j = 0; j < (int)key.pose.ikLinks.size(); j++){
            for(int i = 0; i < (int)endNames.size(); i++){
                if(key.pose.ikLinks[j].name == endNames[i])
                    key.pose.endIndex[i] = j;
            }
            if(key.pose.ikLinks[j].name == baseName){
                key.pose.baseIndex = j;
            }
        }
    }

    for(Key& key : keys){
        key.pose.jointIndex.resize(njoint, -1);
        key.pose.isSpJoint .resize(njoint, false);

        for(int j = 0; j < (int)key.pose.joints.size(); j++){
            key.pose.jointIndex[key.pose.joints[j]] = j;
        }
        for(int j = 0; j < (int)key.pose.spJoints.size(); j++){
            key.pose.isSpJoint[key.pose.spJoints[j]] = true;
        }
    }

    int basePrev = -1;
    vector<int> endPrev(nend, -1);
    vector<int> jointPrev(njoint, -1);

    for(int k = 0; k < (int)keys.size(); k++){
        Key& key = keys[k];
        if(key.pose.baseIndex != -1)
            basePrev = k;
        for(int i = 0; i < nend; i++)
            if(key.pose.endIndex[i] != -1)
                endPrev[i] = k;
        for(int i = 0; i < njoint; i++)
            if(key.pose.jointIndex[i] != -1)
                jointPrev[i] = k;

        key.basePrev  = basePrev;
        key.endPrev   = endPrev;
        key.jointPrev = jointPrev;
    }

    int baseNext = -1;
    vector<int> endNext(nend, -1);
    vector<int> jointNext(njoint, -1);

    for(int k = (int)keys.size() - 1; k >= 0; k--){
        Key& key = keys[k];
        if(key.pose.baseIndex != -1)
            baseNext = k;
        for(int i = 0; i < nend; i++)
            if(key.pose.endIndex[i] != -1)
                endNext[i] = k;
        for(int i = 0; i < njoint; i++)
            if(key.pose.jointIndex[i] != -1)
                jointNext[i] = k;

        key.baseNext  = baseNext;
        key.endNext   = endNext;
        key.jointNext = jointNext;
    }

    // modify timing according to play speed
    for(int k = 0; k < (int)keys.size(); k++){
        Key& key = keys[k];
        key.time /= playSpeed;
        key.maxTransitionTime /= playSpeed;
    }

    // calc velocity by difference
    int N = (int)keys.size()-1;
    dymp::real_t t0 = 0.0, t1 = 0.0, s0 = 0.0, s1 = 0.0;
    dymp::vec3_t p0 = dymp::zero3;
    dymp::vec3_t p1 = dymp::zero3;
    dymp::quat_t q0 = dymp::unit_quat();
    dymp::quat_t q1 = dymp::unit_quat();
    for(int k = 0; k <= N; k++){
        Key& key = keys[k];
    
        if(key.pose.baseIndex != -1){
            IKLink& base = key.pose.ikLinks[key.pose.baseIndex];
            if(k > 0 && keys[k-1].basePrev != -1){
                Key& keyPrev = keys[keys[k-1].basePrev];
                IKLink& basePrev = keyPrev.pose.ikLinks[keyPrev.pose.baseIndex];
                t0 = keyPrev.time;
                p0 = basePrev.pos;
                q0 = basePrev.ori;
            }
            else{
                t0 = key.time;
                p0 = base.pos;
                q0 = base.ori;
            }
            if(k < N && keys[k+1].baseNext != -1){
                Key& keyNext = keys[keys[k+1].baseNext];
                IKLink& baseNext = keyNext.pose.ikLinks[keyNext.pose.baseIndex];
                t1 = keyNext.time;
                p1 = baseNext.pos;
                q1 = baseNext.ori;
            }
            else{
                t1 = key.time;
                p1 = base.pos;
                q1 = base.ori;
            }

            //base.vel = (p1 - p0)/(t1 - t0);
            //quat_t qrel = q0.Conjugated()*q1;
            //base.angvel = (qrel.Theta()*qrel.Axis())/(t1 - t0);
            base.vel    = dymp::zero3;
            base.angvel = dymp::zero3;
        }

        for(int i = 0; i < nend; i++){
            if(key.pose.endIndex[i] != -1){
                IKLink& end = key.pose.ikLinks[key.pose.endIndex[i]];
                if(end.isTouching){
                    end.vel    = dymp::zero3;
                    end.angvel = dymp::zero3;
                    continue;
                }

                if(k > 0 && keys[k-1].endPrev[i] != -1){
                    Key& keyPrev = keys[keys[k-1].endPrev[i]];
                    IKLink& endPrev = keyPrev.pose.ikLinks[keyPrev.pose.endIndex[i]];
                    t0 = keyPrev.time;
                    p0 = endPrev.pos;
                    q0 = endPrev.ori;
                }
                else{
                    t0 = key.time;
                    p0 = end.pos;
                    q0 = end.ori;
                }
                if(k < N && keys[k+1].endNext[i] != -1){
                    Key& keyNext = keys[keys[k+1].endNext[i]];
                    IKLink& endNext = keyNext.pose.ikLinks[keyNext.pose.endIndex[i]];
                    t1 = keyNext.time;
                    p1 = endNext.pos;
                    q1 = endNext.ori;
                }
                else{
                    t1 = key.time;
                    p1 = end.pos;
                    q1 = end.ori;
                }

                //end.vel = (p1 - p0)/(t1 - t0);
                //quat_t qrel = q0.Conjugated()*q1;
                //end.angvel = (qrel.Theta()*qrel.Axis())/(t1 - t0);
                end.vel    = dymp::zero3;
                end.angvel = dymp::zero3;
            }
        }

        key.pose.qd.resize(key.pose.q.size(), 0.0);

        for(int i = 0; i < njoint; i++){
            if(key.pose.jointIndex[i] != -1){
                // joint velocity is zero if sp
                if(key.pose.isSpJoint[i]){
                    key.pose.qd[key.pose.jointIndex[i]] = 0.0;
                    continue;
                }

                if(k > 0 && keys[k-1].jointPrev[i] != -1){
                    Key& keyPrev = keys[keys[k-1].jointPrev[i]];
                    t0 = keyPrev.time;
                    s0 = keyPrev.pose.q[keyPrev.pose.jointIndex[i]];
                }
                else{
                    t0 = key.time;
                    s0 = key.pose.q[key.pose.jointIndex[i]];
                }
                if(k < N && keys[k+1].jointNext[i] != -1){
                    Key& keyNext = keys[keys[k+1].jointNext[i]];
                    t1 = keyNext.time;
                    s1 = keyNext.pose.q[keyNext.pose.jointIndex[i]];
                }
                else{
                    t1 = key.time;
                    s1 = key.pose.q[key.pose.jointIndex[i]];
                }

                //key.pose.qd[key.pose.jointIndex[i]] = (s1 - s0)/(t1 - t0);
                key.pose.qd[key.pose.jointIndex[i]] = 0.0;
            }
        }
    }
    /*
    // print info
    for(int k = 0; k < (int)keys.size(); k++){
        Key& key = keys[k];
        DSTR << "k: " << k << " time: " << key.time << endl;
        for(int i = 0; i < nend; i++){
            DSTR << " end" << i << ": " << "prev: " << key.endPrev[i] << " next: " << key.endNext[i] << endl;
            if(key.endPrev[i] != -1){
                Key& kprev = keys[key.endPrev[i]];
                DSTR << "  touch prev: " << kprev.pose.ikLinks[kprev.pose.endIndex[i]].isTouching << endl;
            }
            if(key.endNext[i] != -1){
                Key& knext = keys[key.endNext[i]];
                DSTR << "  touch next: " << knext.pose.ikLinks[knext.pose.endIndex[i]].isTouching << endl;
            }
        }       
    }
    */
}

pair<int, int> Poseseq::Find(double t){
    if(t < keys.front().time){
        return make_pair(0, 0);
    }
    int N = (int)keys.size();
    for(int k = 0; k < N-1; k++){
        Key& key0 = keys[k+0];
        Key& key1 = keys[k+1];
        if(key0.time <= t && t < key1.time)
            return make_pair(k, k+1);
    }
    return make_pair(N-1, N-1);
}
/*
double Poseseq::GetTime(double t0, int k){
    if(k == 0)
        return t0;

    int N = (int)keys.size();
    pair<int, int> kp = Find(t0);

    return keys[std::min(kp.first + k, N-1)].time;
}
*/

double Poseseq::GetTime(int k){
    /*
    int iphase = initialPhase;
    int k0   = 0;
    int idiv = 0;
    int ndiv;
    real_t tau;
    real_t dtau;
    //while(iphase < (int)keys.size()-1){
    while(true){
        if(idiv == 0){
            if(iphase < (int)keys.size()-1){
                tau = keys[iphase+1].time - keys[iphase+0].time;
                ndiv = (int)(ceil(tau/maxDuration));
                dtau = tau/ndiv;
            }
            else{
                dtau = 0.3;
                tau  = inf;
                ndiv = iinf;
                DSTR << "end of keys" << endl;
            }
        }
        if(k0 == k)
            break;

        if(++idiv == ndiv){
            iphase++;
            idiv = 0;
        }
        k0++;
    }
    return (keys[iphase].time + idiv*dtau) - keys[initialPhase].time;
    */
    return keys[std::min(initialPhase + k, (int)keys.size()-1)].time - keys[initialPhase].time;
}       

void Poseseq::Setup(double t, dymp::Wholebody* wb, dymp::WholebodyData& d){
    t += keys[initialPhase].time;

    int njoint = wb->joints.size();
    int nend   = wb->ends.size();

    pair<int, int> kp = Find(t);

    Key& k0 = keys[kp.first ];
    Key& k1 = keys[kp.second];
    
    dymp::real_t t0, t1, h, s;

    // base
    dymp::vec3_t pbase = dymp::zero3;
    dymp::vec3_t vbase = dymp::zero3;
    dymp::quat_t qbase = dymp::unit_quat();
    dymp::vec3_t wbase = dymp::zero3;
    if(k0.basePrev == -1){
        // should not come here
    }
    else if(k0.basePrev == k1.baseNext || k1.baseNext == -1){
        Key& kprev = keys[k0.basePrev];
        pbase = kprev.pose.ikLinks[kprev.pose.baseIndex].pos;
        qbase = kprev.pose.ikLinks[kprev.pose.baseIndex].ori;
        vbase = dymp::zero3;
        wbase = dymp::zero3;
    }
    else{
        Key& kprev = keys[k0.basePrev];
        Key& knext = keys[k1.baseNext];

        dymp::vec3_t p0 = kprev.pose.ikLinks[kprev.pose.baseIndex].pos;
        dymp::quat_t q0 = kprev.pose.ikLinks[kprev.pose.baseIndex].ori;
        dymp::vec3_t v0 = kprev.pose.ikLinks[kprev.pose.baseIndex].vel;
        dymp::vec3_t w0 = kprev.pose.ikLinks[kprev.pose.baseIndex].angvel;

        dymp::vec3_t p1 = knext.pose.ikLinks[knext.pose.baseIndex].pos;
        dymp::quat_t q1 = knext.pose.ikLinks[knext.pose.baseIndex].ori;
        dymp::vec3_t v1 = knext.pose.ikLinks[knext.pose.baseIndex].vel;
        dymp::vec3_t w1 = knext.pose.ikLinks[knext.pose.baseIndex].angvel;

        t1 = knext.time;
        t0 = std::max(kprev.time, knext.time - knext.maxTransitionTime);
        /*t0 = kprev.time;
        pbase = InterpolatePos   (t, t0, p0, v0, t1, p1, v1, Interpolate::LinearDiff);
        vbase = InterpolateVel   (t, t0, p0, v0, t1, p1, v1, Interpolate::LinearDiff);
        qbase = InterpolateOri   (t, t0, q0, w0, t1, q1, w1, Interpolate::SlerpDiff);
        wbase = InterpolateAngvel(t, t0, q0, w0, t1, q1, w1, Interpolate::SlerpDiff);
        */
        h  = t1 - t0;
        s  = std::min(std::max(0.0, (t - t0)/(t1 - t0)), 1.0);
    
        pbase = (1-s)*p0 + s*p1;
        vbase = (p1 - p0)/h;

        Eigen::AngleAxisd qrel(q0.conjugate()*q1);
        dymp::vec3_t axis  = qrel.axis ();
        dymp::real_t theta = qrel.angle();
        if(theta > pi)
            theta -= 2*pi;

        qbase = q0*dymp::rot_quat((s*theta)*axis);
        wbase = q0*((theta*axis)/h);
    }

    // joint
    for(int i = 0; i < njoint; i++){
        if(k0.jointPrev [i] == -1){
            d.q  [i] = 0.0;
            d.qd [i] = 0.0;
        }
        else if(k0.jointPrev[i] == k1.jointNext[i] || k1.jointNext[i] == -1){
            Key& kprev = keys[k0.jointPrev[i]];
            d.q [i] = kprev.pose.q[kprev.pose.jointIndex[i]];
            d.qd[i] = 0.0;
        }
        else{
            Key& kprev = keys[k0.jointPrev[i]];
            Key& knext = keys[k1.jointNext[i]];

            dymp::real_t q0  = kprev.pose.q [kprev.pose.jointIndex[i]];
            dymp::real_t q1  = knext.pose.q [knext.pose.jointIndex[i]];
            dymp::real_t qd0 = kprev.pose.qd[kprev.pose.jointIndex[i]];
            dymp::real_t qd1 = knext.pose.qd[knext.pose.jointIndex[i]];
        
            //h = (knext.time - kprev.time)/playSpeed;
            //s = std::min(std::max(0.0, (playSpeed*t - kprev.time)/(knext.time - kprev.time)), 1.0);
            t1 = knext.time;
            t0 = std::max(kprev.time, knext.time - knext.maxTransitionTime);
            //t0 = kprev.time;
            //d.q [i] = InterpolatePos(t, t0, q0, qd0, t1, q1, qd1, Interpolate::Cubic);
            //d.qd[i] = InterpolateVel(t, t0, q0, qd0, t1, q1, qd1, Interpolate::Cubic);
            h  = t1 - t0;
            s  = std::min(std::max(0.0, (t - t0)/(t1 - t0)), 1.0);
    
            d.q [i] = (1-s)*q0 + s*q1;
            d.qd[i] = (q1 - q0)/h;
            //d.qd[i] = 0.0;
        }
        d.qdd [i] = 0.0;
        d.qddd[i] = 0.0;
    }

    // end
    dymp::vec3_t pe = dymp::zero3;
    dymp::quat_t qe = dymp::unit_quat();
    dymp::vec3_t ve = dymp::zero3;
    dymp::vec3_t we = dymp::zero3;
    bool   contact = false;
    bool   toe     = false;
    bool   heel    = false;
    for(int i = 0; i < nend; i++){
        if(k0.endPrev[i] == -1){
            // should not come here
        }
        else if(k0.endPrev[i] == k1.endNext[i] || k1.endNext[i] == -1){
            Key& kprev = keys[k0.endPrev[i]];
            pe = kprev.pose.ikLinks[kprev.pose.endIndex[i]].pos;
            qe = kprev.pose.ikLinks[kprev.pose.endIndex[i]].ori;
            ve = dymp::zero3;
            we = dymp::zero3;

            contact = kprev.pose.ikLinks[kprev.pose.endIndex[i]].isTouching;
            toe     = kprev.pose.ikLinks[kprev.pose.endIndex[i]].isToeContact;
            heel    = kprev.pose.ikLinks[kprev.pose.endIndex[i]].isHeelContact;
        }
        else{
            Key& kprev = keys[k0.endPrev[i]];
            Key& knext = keys[k1.endNext[i]];

            dymp::vec3_t p0 = kprev.pose.ikLinks[kprev.pose.endIndex[i]].pos;
            dymp::quat_t q0 = kprev.pose.ikLinks[kprev.pose.endIndex[i]].ori;
            dymp::vec3_t v0 = kprev.pose.ikLinks[kprev.pose.endIndex[i]].vel;
            dymp::vec3_t w0 = kprev.pose.ikLinks[kprev.pose.endIndex[i]].angvel;

            dymp::vec3_t p1 = knext.pose.ikLinks[knext.pose.endIndex[i]].pos;
            dymp::quat_t q1 = knext.pose.ikLinks[knext.pose.endIndex[i]].ori;
            dymp::vec3_t v1 = knext.pose.ikLinks[knext.pose.endIndex[i]].vel;
            dymp::vec3_t w1 = knext.pose.ikLinks[knext.pose.endIndex[i]].angvel;

            //h = (knext.time - kprev.time)/playSpeed;
            //s = std::min(std::max(0.0, (playSpeed*t - kprev.time)/(knext.time - kprev.time)), 1.0);
            t1 = knext.time;
            t0 = std::max(kprev.time, knext.time - knext.maxTransitionTime);
            //t0 = kprev.time;
        
            /*
            pe = InterpolatePos   (t, t0, p0, v0, t1, p1, v1, Interpolate::Cubic);
            ve = InterpolateVel   (t, t0, p0, v0, t1, p1, v1, Interpolate::Cubic);
            qe = InterpolateOri   (t, t0, q0, w0, t1, q1, w1, Interpolate::SlerpDiff);
            we = InterpolateAngvel(t, t0, q0, w0, t1, q1, w1, Interpolate::SlerpDiff);
            */
            // before contact transition
            if(t < t0){
                pe = p0;
                ve = dymp::zero3;
                qe = q0;
                we = dymp::zero3;
            }
            // during transition
            else{
                h  = t1 - t0;
                s  = std::min(std::max(0.0, (t - t0)/(t1 - t0)), 1.0);

                pe = (1-s)*p0 + s*p1;
                ve = (p1 - p0)/h;

                Eigen::AngleAxisd qrel(q0.conjugate()*q1);
                dymp::vec3_t axis  = qrel.axis ();
                dymp::real_t theta = qrel.angle();
                if(theta > pi)
                    theta -= 2*pi;

                qe = q0*dymp::rot_quat((s*theta)*axis);
                we = q0*((theta*axis)/h);
            }
            /*
            */
            // k0 touch and k1 touch -> touch
            // k0 float and k1 touch -> float
            // k0 touch and k1 float -> (t >= t0 ? float : touch)
            contact = (  kprev.pose.ikLinks[kprev.pose.endIndex[i]].isTouching && 
                        (knext.pose.ikLinks[knext.pose.endIndex[i]].isTouching || (knext.time > k1.time)/*(t < t0)*/)
                       );
            toe = contact && (kprev.pose.ikLinks[kprev.pose.endIndex[i]].isToeContact ||
                              knext.pose.ikLinks[knext.pose.endIndex[i]].isToeContact);
            heel = contact && (kprev.pose.ikLinks[kprev.pose.endIndex[i]].isHeelContact ||
                               knext.pose.ikLinks[knext.pose.endIndex[i]].isHeelContact);
        }

        dymp::WholebodyData::End& dend = d.ends[i];

 		if(!contact){
            dend.force_t = dymp::zero3;
            dend.force_r = dymp::zero3;
        }
        else{
            dend.force_t = dymp::vec3_t(0.0, 0.0, wb->param.totalMass*9.8/2.0); //wb->param.totalMass*le*le*(pc - pe - vec3_t(re.x, re.y, 0.0));
            dend.force_r = dymp::zero3; //wb->param.totalMass*le*le*me;
        }
         
        if(!contact){
            dend.state  = dymp::Wholebody::ContactState::Free;
        }
        if(contact){
            if(toe){
                dend.state   = dymp::Wholebody::ContactState::Line;
                dend.cop_min = dymp::vec3_t( 0.15, -0.05, -0.1);
                dend.cop_max = dymp::vec3_t( 0.15,  0.05,  0.1);
                dend.pos_te  = dymp::vec3_t( 0.15,  0.0 ,  0.0);
            }
            else if(heel){
                dend.state   = dymp::Wholebody::ContactState::Line;
                dend.cop_min = dymp::vec3_t(-0.10, -0.05, -0.1);
                dend.cop_max = dymp::vec3_t(-0.10,  0.05,  0.1);
                dend.pos_te  = dymp::vec3_t( 0.1 ,  0.0 ,  0.0);
            }
            else{
                dend.state   = dymp::Wholebody::ContactState::Surface;
                dend.cop_min = dymp::vec3_t(-0.10, -0.05, -0.1);
                dend.cop_max = dymp::vec3_t( 0.15,  0.05,  0.1);
                dend.pos_te  = dymp::vec3_t( 0.0, 0.0, 0.0);
            }
            dend.mu      = 1.0;
            dend.pos_tc  = pe + qe*(wb->ends[i].offset + dend.pos_te);
            dend.pos_rc  = dymp::unit_quat();
        }

        dend.pos_t_abs = pe + qe*wb->ends[i].offset;
        dend.pos_r_abs = qe;
        dend.vel_t_abs = ve;
        dend.vel_r_abs = we;

        if(i == 3 || i == 4){
            // calc leg joint angles by IK
            int jleg[2][6] = { {8,9,10,11,12,13}, {0,1,2,3,4,5} };

            vector<double> qleg;
            robot->kinematics->CalcIK(qbase.conjugate()*(pe - pbase), qbase.conjugate()*dend.pos_r_abs, (i == 3 ? -1.0 : +1.0), qleg);
            for(int j = 0; j < 6; j++){
                d.q[jleg[i-3][j]] = qleg[j];
            }
        }
    }
    
    // calc com
    d.centroid.pos_r = qbase;
    d.centroid.vel_r = wbase;

    wb->CalcPosition(d);
    wb->CalcVelocity(d);

    d.centroid.pos_t = pbase - qbase*d.links[0].pos_t;
    d.centroid.vel_t = vbase - qbase*d.links[0].vel_t - wbase.cross(qbase*d.links[0].pos_t);

    wb->CalcInertia(d);
    wb->CalcLocalMomentum(d);
    wb->CalcAbsoluteMomentum(d);
}

}
}
