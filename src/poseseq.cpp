#include "poseseq.h"
#include "myrobot.h"
#include "kinematics.h"
#include "planner_wholebody.h"

#include <dymp/rollpitchyaw.h>

using namespace std;

const dymp::real_t pi   = 3.1415926535;
const dymp::real_t inf  = numeric_limits<dymp::real_t>::max();
const int          iinf = numeric_limits<int>::max();

namespace dymp{
namespace mpc{

Poseseq::Poseseq(){
    playSpeed    = 1.0;
    initialPhase = 0;
}

Poseseq::IKLink::IKLink(){            
    pos              = dymp::zero3;
    ori              = dymp::unit_quat();
    angle            = dymp::zero3;
    vel              = dymp::zero3;
    angvel           = dymp::zero3;
    angled           = dymp::zero3;
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
    ReadInt   (initialPhase      , node["initial_phase"]       );
    ReadString(pseqFile          , node["pseq_file"]           );
    ReadString(baseLinkName      , node["base_link_name"]      );
    ReadVectorString(endLinkName , node["end_link_name"]       );
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
            lnk.angle = dymp::ToRollPitchYaw(lnk.ori);

            ReadBool   (lnk.isTouching , ikLinkNode["isTouching"]);
            ReadVector3(lnk.partingDirection, ikLinkNode["partingDirection"]);

            // detect toe/heel contact
            const dymp::real_t th = 0.01;
            lnk.isToeContact  = lnk.isTouching && (lnk.angle.y() >  th);
            lnk.isHeelContact = lnk.isTouching && (lnk.angle.y() < -th);

            key.pose.ikLinks.push_back(lnk);
        }

        keys.push_back(key);
    }

    return true;
}

void Poseseq::Init(){
    dymp::Wholebody* wb = robot->planner_wb->GetWholebody();

    int njoint = wb->joints.size();
    int nend   = wb->ends.size();

    for(Key& key : keys){
        key.pose.endIndex  .resize(nend  , -1);
        key.pose.baseIndex = -1;

        for(int j = 0; j < (int)key.pose.ikLinks.size(); j++){
            for(int i = 0; i < (int)endLinkName.size(); i++){
                if(key.pose.ikLinks[j].name == endLinkName[i])
                    key.pose.endIndex[i] = j;
            }
            if(key.pose.ikLinks[j].name == baseLinkName){
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

    // modify yaw angle considering multiple turns
    int N = (int)keys.size()-1;
    for(int k = 0; k <= N; k++){
        Key& key = keys[k];

        if(key.pose.baseIndex != -1){
            IKLink& base = key.pose.ikLinks[key.pose.baseIndex];
            if(k > 0 && keys[k-1].basePrev != -1){
                Key& keyPrev = keys[keys[k-1].basePrev];
                IKLink& basePrev = keyPrev.pose.ikLinks[keyPrev.pose.baseIndex];
                while(base.angle.z() < basePrev.angle.z() - pi)
                    base.angle.z() += 2*pi;
                while(base.angle.z() > basePrev.angle.z() + pi)
                    base.angle.z() -= 2*pi;                
            }
        }    
        for(int i = 0; i < nend; i++){
            if(key.pose.endIndex[i] != -1){
                IKLink& end = key.pose.ikLinks[key.pose.endIndex[i]];
                if(k > 0 && keys[k-1].endPrev[i] != -1){
                    Key& keyPrev = keys[keys[k-1].endPrev[i]];
                    IKLink& endPrev = keyPrev.pose.ikLinks[keyPrev.pose.endIndex[i]];
                    while(end.angle.z() < endPrev.angle.z() - pi)
                        end.angle.z() += 2*pi;
                    while(end.angle.z() > endPrev.angle.z() + pi)
                        end.angle.z() -= 2*pi;                
                }
            }
        }
    }

    // calc velocity by difference
    dymp::real_t t0 = 0.0, t1 = 0.0, s0 = 0.0, s1 = 0.0;
    dymp::real_t TT = 0.0;  //< transition time
    dymp::vec3_t p0 = dymp::zero3;
    dymp::vec3_t p1 = dymp::zero3;
    dymp::vec3_t a0 = dymp::zero3;
    dymp::vec3_t a1 = dymp::zero3;
    for(int k = 0; k <= N; k++){
        Key& key = keys[k];
    
        if(key.pose.baseIndex != -1){
            IKLink& base = key.pose.ikLinks[key.pose.baseIndex];
            if(k > 0 && keys[k-1].basePrev != -1){
                Key& keyPrev = keys[keys[k-1].basePrev];
                IKLink& basePrev = keyPrev.pose.ikLinks[keyPrev.pose.baseIndex];
                t0 = keyPrev.time;
                p0 = basePrev.pos;
                a0 = basePrev.angle;
            }
            else{
                t0 = key.time;
                p0 = base.pos;
                a0 = base.angle;
            }
            if(k < N && keys[k+1].baseNext != -1){
                Key& keyNext = keys[keys[k+1].baseNext];
                IKLink& baseNext = keyNext.pose.ikLinks[keyNext.pose.baseIndex];
                t1 = keyNext.time;
                TT = keyNext.maxTransitionTime;
                p1 = baseNext.pos;
                a1 = baseNext.angle;
            }
            else{
                t1 = key.time;
                TT = 0.0;
                p1 = base.pos;
                a1 = base.angle;
            }

            if(key.time < t1 - TT){
                base.vel    = dymp::zero3;
                base.angled = dymp::zero3;
            }
            else{
                base.vel    = (p1 - p0)/(t1 - t0);
                base.angled = (a1 - a0)/(t1 - t0);
            }
        }

        for(int i = 0; i < nend; i++){
            if(key.pose.endIndex[i] != -1){
                IKLink& end = key.pose.ikLinks[key.pose.endIndex[i]];

                // zero velocity if surface contact
                if(end.isTouching && !(end.isToeContact || end.isHeelContact)){
                    end.vel    = dymp::zero3;
                    end.angled = dymp::zero3;
                    continue;
                }

                if(k > 0 && keys[k-1].endPrev[i] != -1){
                    Key& keyPrev = keys[keys[k-1].endPrev[i]];
                    IKLink& endPrev = keyPrev.pose.ikLinks[keyPrev.pose.endIndex[i]];
                    t0 = keyPrev.time;
                    p0 = endPrev.pos;
                    a0 = endPrev.angle;
                }
                else{
                    t0 = key.time;
                    p0 = end.pos;
                    a0 = end.angle;
                }
                if(k < N && keys[k+1].endNext[i] != -1){
                    Key& keyNext = keys[keys[k+1].endNext[i]];
                    IKLink& endNext = keyNext.pose.ikLinks[keyNext.pose.endIndex[i]];
                    t1 = keyNext.time;
                    TT = keyNext.maxTransitionTime;
                    p1 = endNext.pos;
                    a1 = endNext.angle;
                }
                else{
                    t1 = key.time;
                    TT = 0.0;
                    p1 = end.pos;
                    a1 = end.angle;
                }

                if(key.time < t1 - TT){
                    end.vel    = dymp::zero3;
                    end.angled = dymp::zero3;
                }
                else{
                    end.vel    = (p1 - p0)/(t1 - t0);
                    end.angled = (a1 - a0)/(t1 - t0);
                }
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
                    TT = keyNext.maxTransitionTime;
                    s1 = keyNext.pose.q[keyNext.pose.jointIndex[i]];
                }
                else{
                    t1 = key.time;
                    TT = 0.0;
                    s1 = key.pose.q[key.pose.jointIndex[i]];
                }

                if(key.time < t1 - TT){
                    key.pose.qd[key.pose.jointIndex[i]] = 0.0;
                }
                else{
                    key.pose.qd[key.pose.jointIndex[i]] = (s1 - s0)/(t1 - t0);
                }
            }
        }
    }
    /*
    // generate contact face info
    faces.clear();
    for(int k = 0; k < (int)keys.size(); k++){
        for(int i = 0; i < nend; i++){
            if(key.pose.endIndex[i] != -1){
                IKLink& end = key.pose.ikLinks[key.pose.endIndex[i]];

            }
        }
    }
    */
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

double Poseseq::GetTime(int k){
    return keys[std::min(initialPhase + k, (int)keys.size()-1)].time - keys[initialPhase].time;
}       

void Poseseq::Setup(double t, dymp::Wholebody* wb, dymp::WholebodyData& d){
    t += keys[initialPhase].time;

    int njoint = wb->joints.size();
    int nend   = wb->ends.size();

    pair<int, int> kp = Find(t);

    Key& k0 = keys[kp.first ];
    Key& k1 = keys[kp.second];
    
    dymp::real_t t0, t1, tclip;

    // base
    dymp::vec3_t pbase  = dymp::zero3;
    dymp::vec3_t vbase  = dymp::zero3;
    dymp::quat_t qbase  = dymp::unit_quat();
    dymp::vec3_t wbase  = dymp::zero3;
    dymp::vec3_t abase  = dymp::zero3;
    dymp::vec3_t adbase = dymp::zero3;
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
        IKLink& basePrev = kprev.pose.ikLinks[kprev.pose.baseIndex];
        IKLink& baseNext = knext.pose.ikLinks[knext.pose.baseIndex];
        dymp::vec3_t p0  = basePrev.pos;
        dymp::vec3_t v0  = basePrev.vel;
        dymp::vec3_t a0  = basePrev.angle;
        dymp::vec3_t ad0 = basePrev.angled;
        
        dymp::vec3_t p1  = baseNext.pos;
        dymp::vec3_t v1  = baseNext.vel;
        dymp::vec3_t a1  = baseNext.angle;
        dymp::vec3_t ad1 = baseNext.angled;
        
        t0 = std::max(kprev.time, knext.time - knext.maxTransitionTime);
        t1 = knext.time;
        tclip = std::max(t, t0);
        
        pbase  = interpolate_pos_cubic(tclip, t0, p0, v0 , t1, p1, v1 );
        vbase  = interpolate_vel_cubic(tclip, t0, p0, v0 , t1, p1, v1 );
        abase  = interpolate_pos_cubic(tclip, t0, a0, ad0, t1, a1, ad1);
        adbase = interpolate_vel_cubic(tclip, t0, a0, ad0, t1, a1, ad1);
        qbase  = dymp::FromRollPitchYaw(abase);
        wbase  = dymp::VelocityFromRollPitchYaw(abase, adbase);
    }

    // joint
    for(int i = 0; i < njoint; i++){
        if(k0.jointPrev [i] == -1){
            d.joints[i].q  = 0.0;
            d.joints[i].qd = 0.0;
        }
        else if(k0.jointPrev[i] == k1.jointNext[i] || k1.jointNext[i] == -1){
            Key& kprev = keys[k0.jointPrev[i]];
            d.joints[i].q  = kprev.pose.q[kprev.pose.jointIndex[i]];
            d.joints[i].qd = 0.0;
        }
        else{
            Key& kprev = keys[k0.jointPrev[i]];
            Key& knext = keys[k1.jointNext[i]];

            dymp::real_t q0  = kprev.pose.q [kprev.pose.jointIndex[i]];
            dymp::real_t q1  = knext.pose.q [knext.pose.jointIndex[i]];
            dymp::real_t qd0 = kprev.pose.qd[kprev.pose.jointIndex[i]];
            dymp::real_t qd1 = knext.pose.qd[knext.pose.jointIndex[i]];
        
            t0 = std::max(kprev.time, knext.time - knext.maxTransitionTime);
            t1 = knext.time;
            tclip = std::max(t, t0);
            
            d.joints[i].q  = interpolate_pos_cubic(tclip, t0, q0, qd0, t1, q1, qd1);
            d.joints[i].qd = interpolate_vel_cubic(tclip, t0, q0, qd0, t1, q1, qd1);
        }
        d.joints[i].qdd  = 0.0;
        d.joints[i].qddd = 0.0;
    }

    // end
    dymp::vec3_t pend  = dymp::zero3;
    dymp::vec3_t vend  = dymp::zero3;
    dymp::quat_t qend  = dymp::unit_quat();
    dymp::vec3_t wend  = dymp::zero3;
    dymp::vec3_t aend  = dymp::zero3;
    dymp::vec3_t adend = dymp::zero3;
    bool   contact = false;
    bool   toe     = false;
    bool   heel    = false;
    dymp::vec3_t normal;

    for(int i = 0; i < nend; i++){
        if(k0.endPrev[i] == -1){
            // should not come here
        }
        else if(k0.endPrev[i] == k1.endNext[i] || k1.endNext[i] == -1){
            Key& kprev = keys[k0.endPrev[i]];
            IKLink& endPrev = kprev.pose.ikLinks[kprev.pose.endIndex[i]];
            
            pend = endPrev.pos;
            qend = endPrev.ori;
            vend = dymp::zero3;
            wend = dymp::zero3;

            contact = endPrev.isTouching;
            toe     = endPrev.isToeContact;
            heel    = endPrev.isHeelContact;
            normal  = FromRollPitchYaw(endPrev.angle)*endPrev.partingDirection;
        }
        else{
            Key& kprev = keys[k0.endPrev[i]];
            Key& knext = keys[k1.endNext[i]];
            IKLink& endPrev = kprev.pose.ikLinks[kprev.pose.endIndex[i]];
            IKLink& endNext = knext.pose.ikLinks[knext.pose.endIndex[i]];

            dymp::vec3_t p0  = endPrev.pos;
            dymp::vec3_t v0  = endPrev.vel;
            dymp::vec3_t a0  = endPrev.angle;
            dymp::vec3_t ad0 = endPrev.angled;
        
            dymp::vec3_t p1  = endNext.pos;
            dymp::vec3_t v1  = endNext.vel;
            dymp::vec3_t a1  = endNext.angle;
            dymp::vec3_t ad1 = endNext.angled;
            
            t0 = std::max(kprev.time, knext.time - knext.maxTransitionTime);
            t1 = knext.time;
            tclip = std::max(t, t0);
        
            pend  = interpolate_pos_cubic(tclip, t0, p0, v0 , t1, p1, v1 );
            vend  = interpolate_vel_cubic(tclip, t0, p0, v0 , t1, p1, v1 );
            aend  = interpolate_pos_cubic(tclip, t0, a0, ad0, t1, a1, ad1);
            adend = interpolate_vel_cubic(tclip, t0, a0, ad0, t1, a1, ad1);
            qend  = dymp::FromRollPitchYaw(aend);
            wend  = dymp::VelocityFromRollPitchYaw(aend, adend);
            
            // k0 touch and k1 touch -> touch
            // k0 float and k1 touch -> float
            // k0 touch and k1 float -> (t1 < tnext ? float : touch)
            contact = (  endPrev.isTouching && 
                        (endNext.isTouching || (k1.time < knext.time) )
                       );
            toe  = contact && (endPrev.isToeContact  || endNext.isToeContact );
            heel = contact && (endPrev.isHeelContact || endNext.isHeelContact);
            normal = FromRollPitchYaw(endPrev.angle)*endPrev.partingDirection;
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
                dend.cop_min = dymp::vec3_t( 0.15, -0.05, -0.01);
                dend.cop_max = dymp::vec3_t( 0.15,  0.05,  0.01);
                dend.pos_tc  = dymp::vec3_t( 0.15,  0.0 ,  0.0  );
            }
            else if(heel){
                dend.state   = dymp::Wholebody::ContactState::Line;
                dend.cop_min = dymp::vec3_t(-0.1, -0.05, -0.01);
                dend.cop_max = dymp::vec3_t(-0.1,  0.05,  0.01);
                dend.pos_tc  = dymp::vec3_t( 0.1 , 0.0 ,  0.0  );
            }
            else{
                dend.state   = dymp::Wholebody::ContactState::Surface;
                dend.cop_min = dymp::vec3_t(-0.10, -0.05, -0.01);
                dend.cop_max = dymp::vec3_t( 0.15,  0.05,  0.01);
                dend.pos_tc  = dymp::vec3_t( 0.0 ,  0.0 ,  0.0);
            }
            dend.mu      = 2.0;
            dend.normal  = normal;
            //dend.pos_tc  = pend + qend*(wb->ends[i].offset + dend.pos_te);
            //dend.pos_rc  = dymp::unit_quat();
        }

        // take care of ankle-to-foot offset here
        dend.pos_t_abs = pend + qend*wb->ends[i].offset;
        dend.pos_r_abs = qend;
        dend.vel_t_abs = vend + wend.cross(qend*wb->ends[i].offset);
        dend.vel_r_abs = wend;

        vector<double> qleg;
        robot->kinematics->CalcIK(qbase.conjugate()*(pend - pbase), qbase.conjugate()*qend, (i == 0 ? -1.0 : +1.0), qleg);
        for(int j = 0; j < 6; j++){
            d.joints[robot->kinematics->legJointIndices[i][j]].q = qleg[j];
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
