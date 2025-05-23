﻿#include "kinematics.h"
#include "myrobot.h"

#include <dymp/rollpitchyaw.h>

#include <cnoid/ExecutablePath>

namespace dymp{
namespace mpc{

Kinematics::Kinematics(){
    ankleToFoot[0] = dymp::vec3_t(0.0,  0.0 , -0.05);
	ankleToFoot[1] = dymp::vec3_t(0.0,  0.0 , -0.05);
}

void Kinematics::Read(const YAML::Node& node){
	ReadDouble(verticalHipJointOffset, node["vertical_hip_joint_offset"]);
    ReadDouble(lateralHipJointOffset , node["lateral_hip_joint_offset"]);
    ReadDouble(lateralKneeOffset     , node["lateral_knee_offset"]);
    ReadDouble(thighLength           , node["thigh_length"]);
    ReadDouble(shankLength           , node["shank_length"]);

    ReadString(modelFilename, node["model_filename"]);
    ReadVectorInt(legJointIndices[0], node["leg_joint_indices0"]);
    ReadVectorInt(legJointIndices[1], node["leg_joint_indices1"]);
    ReadInt(footLinkIndex[0], node["foot_link_index0"]);
    ReadInt(footLinkIndex[1], node["foot_link_index1"]);
    ReadVector3(ankleToFoot[0], node["ankle_to_foot0"]);
	ReadVector3(ankleToFoot[1], node["ankle_to_foot1"]);
	ReadVectorDouble(standStillJointAngle, node["stand_still_joint_angle"]);
}

void Kinematics::Init(dymp::Wholebody* wb){
    LoadLinksFromBody(wb);
    /*
    // manual setup
    // kinematic and inertial parameters of sample robot
    wb->links .resize(Link ::Num);
	wb->joints.resize(Link ::Num - 1);
	wb->links[ 0] = dymp::Wholebody::Link( 4.200, dymp::vec3_t(0.030, 0.030, 0.030), dymp::vec3_t( 0.000,  0.000,  0.000), -1, -1, -1, dymp::vec3_t( 0.000,  0.000,  0.000), dymp::zero3);
	wb->links[ 1] = dymp::Wholebody::Link( 0.600, dymp::vec3_t(0.001, 0.001, 0.001), dymp::vec3_t( 0.000,  0.000,  0.000), -1,  0,  6, dymp::vec3_t( 0.000,  0.000,  0.145), dymp::ez);
	wb->links[ 2] = dymp::Wholebody::Link(18.800, dymp::vec3_t(1.000, 1.000, 0.300), dymp::vec3_t(-0.050,  0.000,  0.200),  0,  1,  7, dymp::vec3_t( 0.000,  0.000,  0.000), dymp::ey);
	wb->links[ 3] = dymp::Wholebody::Link( 0.100, dymp::vec3_t(0.000, 0.000, 0.000), dymp::vec3_t( 0.000,  0.000,  0.000), -1,  2, 14, dymp::vec3_t( 0.000,  0.000,  0.450), dymp::ez);
	wb->links[ 4] = dymp::Wholebody::Link( 1.400, dymp::vec3_t(0.005, 0.005, 0.005), dymp::vec3_t( 0.000,  0.000,  0.040), -1,  3, 15, dymp::vec3_t( 0.000,  0.000,  0.000), dymp::ey);
		
	wb->links[ 5] = dymp::Wholebody::Link( 1.900, dymp::vec3_t(0.003, 0.003, 0.003), dymp::vec3_t( 0.000,  0.000,  0.000), -1,  2, 24, dymp::vec3_t( 0.000, -0.180,  0.330), dymp::ey);
	wb->links[ 6] = dymp::Wholebody::Link( 0.300, dymp::vec3_t(0.000, 0.000, 0.000), dymp::vec3_t( 0.000,  0.000,  0.000), -1,  5, 25, dymp::vec3_t( 0.000,  0.000, -0.040), dymp::ex);
	wb->links[ 7] = dymp::Wholebody::Link( 2.400, dymp::vec3_t(0.100, 0.100, 0.005), dymp::vec3_t( 0.000,  0.000, -0.200), -1,  6, 26, dymp::vec3_t( 0.000,  0.000,  0.000), dymp::ez);
	wb->links[ 8] = dymp::Wholebody::Link( 0.200, dymp::vec3_t(0.000, 0.000, 0.000), dymp::vec3_t( 0.000,  0.000,  0.000), -1,  7, 27, dymp::vec3_t( 0.020,  0.000, -0.260), dymp::ey);
	wb->links[ 9] = dymp::Wholebody::Link( 1.400, dymp::vec3_t(0.050, 0.050, 0.001), dymp::vec3_t( 0.000,  0.000, -0.150), -1,  8, 28, dymp::vec3_t( 0.000,  0.000,  0.000), dymp::ez);
	wb->links[10] = dymp::Wholebody::Link( 0.060, dymp::vec3_t(0.000, 0.000, 0.000), dymp::vec3_t( 0.000,  0.000,  0.000), -1,  9, 29, dymp::vec3_t( 0.000,  0.000, -0.230), dymp::ex);
	wb->links[11] = dymp::Wholebody::Link( 1.600, dymp::vec3_t(0.020, 0.020, 0.003), dymp::vec3_t( 0.000,  0.000, -0.100), -1, 10, 30, dymp::vec3_t( 0.000,  0.000,  0.000), dymp::ez);
    wb->links[12] = dymp::Wholebody::Link( 0.040, dymp::vec3_t(0.001, 0.001, 0.001), dymp::vec3_t( 0.000,  0.000,  0.000),  1, 11, 31, dymp::vec3_t( 0.000, -0.020, -0.083), dymp::ex);
		
	wb->links[13] = dymp::Wholebody::Link( 1.900, dymp::vec3_t(0.003, 0.003, 0.003), dymp::vec3_t( 0.000,  0.000,  0.000), -1,  2, 16, dymp::vec3_t( 0.000,  0.180,  0.330), dymp::ey);
	wb->links[14] = dymp::Wholebody::Link( 0.300, dymp::vec3_t(0.000, 0.000, 0.000), dymp::vec3_t( 0.000,  0.000,  0.000), -1, 13, 17, dymp::vec3_t( 0.000,  0.000, -0.040), dymp::ex);
	wb->links[15] = dymp::Wholebody::Link( 2.400, dymp::vec3_t(0.100, 0.100, 0.005), dymp::vec3_t( 0.000,  0.000, -0.200), -1, 14, 18, dymp::vec3_t( 0.000,  0.000,  0.000), dymp::ez);
	wb->links[16] = dymp::Wholebody::Link( 0.200, dymp::vec3_t(0.000, 0.000, 0.000), dymp::vec3_t( 0.000,  0.000,  0.000), -1, 15, 19, dymp::vec3_t( 0.020,  0.000, -0.260), dymp::ey);
	wb->links[17] = dymp::Wholebody::Link( 1.400, dymp::vec3_t(0.050, 0.050, 0.001), dymp::vec3_t( 0.000,  0.000, -0.150), -1, 16, 20, dymp::vec3_t( 0.000,  0.000,  0.000), dymp::ez);
	wb->links[18] = dymp::Wholebody::Link( 0.060, dymp::vec3_t(0.000, 0.000, 0.000), dymp::vec3_t( 0.000,  0.000,  0.000), -1, 17, 21, dymp::vec3_t( 0.000,  0.000, -0.230), dymp::ex);
	wb->links[19] = dymp::Wholebody::Link( 1.600, dymp::vec3_t(0.020, 0.020, 0.003), dymp::vec3_t( 0.000,  0.000, -0.100), -1, 18, 22, dymp::vec3_t( 0.000,  0.000,  0.000), dymp::ez);
	wb->links[20] = dymp::Wholebody::Link( 0.040, dymp::vec3_t(0.001, 0.001, 0.001), dymp::vec3_t( 0.000,  0.000,  0.000),  2, 19, 23, dymp::vec3_t( 0.000,  0.020, -0.083), dymp::ex);

	wb->links[21] = dymp::Wholebody::Link( 0.700, dymp::vec3_t(0.001, 0.001, 0.001), dymp::vec3_t( 0.000,  0.000,  0.000), -1,  0,  8, dymp::vec3_t( 0.000, -0.080, -0.000), dymp::ez);
	wb->links[22] = dymp::Wholebody::Link( 0.050, dymp::vec3_t(0.000, 0.000, 0.000), dymp::vec3_t( 0.000,  0.000,  0.000), -1, 21,  9, dymp::vec3_t( 0.000,  0.000, -0.055), dymp::ex);
	wb->links[23] = dymp::Wholebody::Link( 3.000, dymp::vec3_t(0.150, 0.150, 0.007), dymp::vec3_t( 0.000,  0.000, -0.200), -1, 22, 10, dymp::vec3_t( 0.000,  0.000,  0.000), dymp::ey);
	wb->links[24] = dymp::Wholebody::Link( 3.100, dymp::vec3_t(0.100, 0.100, 0.006), dymp::vec3_t( 0.000,  0.000, -0.140), -1, 23, 11, dymp::vec3_t( 0.000,  0.000, -0.360), dymp::ey);
	wb->links[25] = dymp::Wholebody::Link( 0.060, dymp::vec3_t(0.000, 0.000, 0.000), dymp::vec3_t( 0.000,  0.000,  0.000), -1, 24, 12, dymp::vec3_t( 0.000,  0.000, -0.360), dymp::ex);
	wb->links[26] = dymp::Wholebody::Link( 1.300, dymp::vec3_t(0.010, 0.010, 0.005), dymp::vec3_t( 0.000,  0.000,  0.000),  3, 25, 13, dymp::vec3_t( 0.000,  0.000,  0.000), dymp::ey);

	wb->links[27] = dymp::Wholebody::Link( 0.700, dymp::vec3_t(0.001, 0.001, 0.001), dymp::vec3_t( 0.000,  0.000,  0.000), -1,  0,  0, dymp::vec3_t( 0.000,  0.080,  0.000), dymp::ez);
	wb->links[28] = dymp::Wholebody::Link( 0.050, dymp::vec3_t(0.000, 0.000, 0.000), dymp::vec3_t( 0.000,  0.000,  0.000), -1, 27,  1, dymp::vec3_t( 0.000,  0.000, -0.055), dymp::ex);
	wb->links[29] = dymp::Wholebody::Link( 3.000, dymp::vec3_t(0.150, 0.150, 0.007), dymp::vec3_t( 0.000,  0.000, -0.200), -1, 28,  2, dymp::vec3_t( 0.000,  0.000,  0.000), dymp::ey);
	wb->links[30] = dymp::Wholebody::Link( 3.100, dymp::vec3_t(0.100, 0.100, 0.006), dymp::vec3_t( 0.000,  0.000, -0.140), -1, 29,  3, dymp::vec3_t( 0.000,  0.000, -0.360), dymp::ey);
	wb->links[31] = dymp::Wholebody::Link( 0.060, dymp::vec3_t(0.000, 0.000, 0.000), dymp::vec3_t( 0.000,  0.000,  0.000), -1, 30,  4, dymp::vec3_t( 0.000,  0.000, -0.360), dymp::ex);
	wb->links[32] = dymp::Wholebody::Link( 1.300, dymp::vec3_t(0.010, 0.010, 0.005), dymp::vec3_t( 0.000,  0.000,  0.000),  4, 31,  5, dymp::vec3_t( 0.000,  0.000,  0.000), dymp::ey);

    const real_t Ir = 0.1;
    for(auto& jnt : wb->joints)
        jnt.rotor_inertia = Ir;
    */

    // ilink, offset, rotation, force, moment
	//wb->ends  .resize(End  ::Num);
	//wb->ends[0] = dymp::Wholebody::End(Link::ChestP, dymp::vec3_t( 0.000,  0.000,  0.000), true , true , false, false, false);
	//wb->ends[1] = dymp::Wholebody::End(Link::HandRR, dymp::vec3_t( 0.000,  0.000,  0.000), true , true , false, false, false);
	//wb->ends[2] = dymp::Wholebody::End(Link::HandLR, dymp::vec3_t( 0.000,  0.000,  0.000), true , true , false, false, false);
	//wb->ends[3] = dymp::Wholebody::End(Link::FootRP, dymp::vec3_t( 0.000,  0.000, -0.104), true , true , true , true , true );
	//wb->ends[4] = dymp::Wholebody::End(Link::FootLP, dymp::vec3_t( 0.000,  0.000, -0.104), true , true , true , true , true );
    wb->ends.resize(End  ::Num);
	wb->ends[0] = dymp::Wholebody::End(footLinkIndex[0], ankleToFoot[0], true , true , true , true , true );
	wb->ends[1] = dymp::Wholebody::End(footLinkIndex[1], ankleToFoot[1], true , true , true , true , true );

}

void Kinematics::LoadLinksFromBody(dymp::Wholebody* wb){
    string path = cnoid::shareDir() + "/model/" + modelFilename;

    YAML::Node node = YAML::LoadFile(path);

    std::map<std::string, int>  name_to_id;
    std::vector<std::string>    parents;

    YAML::Node linksNode = node["links"];
    for(auto&& linkNode : linksNode){
        dymp::Wholebody::Link  lnk;

        string jointType;
        ReadString(jointType, linkNode["joint_type"]);
        if(jointType == "free"){
            // this must be root link
            lnk.iparent = -1;
            lnk.ijoint  = -1;

        }
        if(jointType == "revolute"){
            ReadVector3(lnk.trn   , linkNode["translation"]);
            ReadVector3(lnk.axis  , linkNode["joint_axis" ]);
            ReadInt    (lnk.ijoint, linkNode["joint_id"   ]);

            if(wb->joints.size() <= lnk.ijoint){
                wb->joints.resize(lnk.ijoint + 1);
            }
            ReadDouble (wb->joints[lnk.ijoint].rotor_inertia, linkNode["joint_axis_inertia"]);

            vec2_t r;
            ReadVector2(r, linkNode["joint_range"]);

            // deg to rad
            wb->joints[lnk.ijoint].pos_range[0] = dymp::deg_to_rad(r[0]);
            wb->joints[lnk.ijoint].pos_range[1] = dymp::deg_to_rad(r[1]);
        }
        if(jointType == "fixed"){
            // skip fix joints
            continue;
        }

        string name, parent;
        ReadString(name  , linkNode["name"  ]);
        ReadString(parent, linkNode["parent"]);

        ReadDouble (lnk.mass   , linkNode["mass"]);
        ReadVector3(lnk.center , linkNode["center_of_mass"]);
        ReadMatrix3(lnk.inertia, linkNode["inertia"]);

        name_to_id[name] = (int)wb->links.size();
        parents.push_back(parent);

        wb->links.push_back(lnk);
    }

    for(int i = 0; i < (int)wb->links.size(); i++){
        auto it = name_to_id.find(parents[i]);
        wb->links[i].iparent = (it != name_to_id.end() ? it->second : -1);
        wb->links[i].iend    = -1;
    }
}

void Kinematics::SetupStandStill(dymp::Wholebody* wb, dymp::WholebodyData& d){
    dymp::vec3_t pc = dymp::vec3_t(0.0, 0.0, robot->param.com_height);
    dymp::vec3_t vc = dymp::zero3;
    dymp::vec3_t ac = dymp::zero3;
    dymp::vec3_t wf = dymp::zero3;
    dymp::quat_t qf = dymp::unit_quat();
    d.centroid.pos_t = pc;
    d.centroid.vel_t = vc;
    d.centroid.pos_r = qf;
    d.centroid.vel_r = wf;
    d.centroid.L_abs = dymp::zero3;
    
    int njoint = (int)wb->joints.size();
    for(int i = 0; i < njoint; i++){
        dymp::WholebodyData::Joint& djnt = d.joints[i];
        djnt.q    = standStillJointAngle[i];
        djnt.qd   = 0.0;
        djnt.qdd  = 0.0;
        djnt.qddd = 0.0;
    }

    dymp::vec3_t pe = dymp::zero3;
    dymp::quat_t qe = dymp::unit_quat();
    dymp::vec3_t ve = dymp::zero3;
    dymp::vec3_t we = dymp::zero3;
    
    for(int i = 0; i < End::Num; i++){
        dymp::WholebodyData::End&  dend = d.ends[i];
        
        if(i == End::FootR || i == End::FootL){
            bool   contact = true;
            pe = dymp::vec3_t(0.0, (i == End::FootR ? -0.08 : 0.08), 0.0);

			if(!contact){
                dend.force_t = dymp::zero3;
                dend.force_r = dymp::zero3;
            }
            else{
                dend.force_t = dymp::vec3_t(0.0, 0.0, wb->param.totalMass*9.8/2.0);
                dend.force_r = dymp::zero3;
            }
         
            if(!contact){
                dend.state  = dymp::Wholebody::ContactState::Free;
            }
            if(contact){
                dend.state   = dymp::Wholebody::ContactState::Surface;
                dend.mu      = 1.0;
                dend.cop_min = dymp::vec3_t(-0.10, -0.05, -0.1);
                dend.cop_max = dymp::vec3_t(+0.15,  0.05,  0.1);
                dend.pos_tc  = dymp::vec3_t( 0.0, 0.0, 0.0);
                dend.normal  = ez;
            }

            dend.pos_t_abs = pe;
            dend.pos_r_abs = qe;
            dend.vel_t_abs = ve;
            dend.vel_r_abs = we;
        }
    }
}

void Kinematics::SetupStandStill(dymp::Centroid* centroid, dymp::CentroidData& d){
    d.pos_t = dymp::vec3_t(0.0, 0.0, robot->param.com_height);
    d.pos_r = dymp::unit_quat();
    d.vel_t = dymp::zero3;
    d.vel_r = dymp::zero3;
    
    for(int i = 0; i < 2; i++){
        dymp::CentroidData::End&  dend = d.ends[i];
        
        dend.pos_t   = dymp::vec2_t(0.0, (i == 0 ? -0.08 : 0.08));
        dend.pos_r   = 0.0;
        dend.vel_t   = dymp::zero2;
        dend.vel_r   = 0.0;
        dend.contact = true;
        dend.iface   = 0;
    }
}

void Kinematics::SetupFromCentroid(real_t t, dymp::Centroid* centroid, const vector<dymp::CentroidData>& d_cen_array, dymp::Wholebody* wb, dymp::WholebodyData& d){
    dymp::CentroidData d_cen;
    centroid->CalcState(t, d_cen_array, d_cen);
    
    d.centroid.pos_t = d_cen.pos_t;
    d.centroid.vel_t = d_cen.vel_t;
    
    d.centroid.pos_r = d_cen.pos_r;
    d.centroid.vel_r = d_cen.vel_r;
    
    d.centroid.L_abs   = d_cen.L;
    d.centroid.L_local = d_cen.Llocal[0];

    for(int i = 0; i < End::Num; i++){
        dymp::WholebodyData::End&  dend = d.ends[i];
        
        if(i == End::FootR || i == End::FootL){
            dymp::CentroidData::End&  dend_cen = d_cen.ends[i];

            if(dend_cen.contact == false){
                dend.state  = dymp::Wholebody::ContactState::Free;
            }
            else{
                const double eps = 0.01;
                if(std::abs(dend_cen.tilt) < eps)
                    dend.state = dymp::Wholebody::ContactState::Surface;
                else 
                    dend.state = dymp::Wholebody::ContactState::Line;
                dend.mu      = dend_cen.mu;
                dend.cop_min = dend_cen.cop_min;
                dend.cop_max = dend_cen.cop_max;
                dend.pos_tc  = dend_cen.pos_tc;
                dend.normal  = centroid->faces[dend_cen.iface].normal;
                //dend.pos_tc  = dymp::zero3;
                //dend.pos_rc  = dymp::unit_quat();
            }
            /*
            */

            dend.pos_t_abs = dend_cen.pos_t_abs;
            dend.pos_r_abs = dend_cen.pos_r_abs;
            dend.vel_t_abs = dend_cen.vel_t_abs;
            dend.vel_r_abs = dend_cen.vel_r_abs;

			dend.force_t = dend_cen.force_t;
            dend.force_r = dend_cen.force_r;            
        }
    }
}

void Kinematics::Convert(const dymp::WholebodyData& d_wb, dymp::Centroid* cen, dymp::CentroidData& d, int idiv){
    if(idiv == 0){
        d.pos_t = d_wb.centroid.pos_t;
        d.vel_t = d_wb.centroid.vel_t;
        d.pos_r = d_wb.centroid.pos_r;
        d.vel_r = d_wb.centroid.vel_r;
        d.L     = d_wb.centroid.L_abs;
        //d.L = zero3;

        int nend = (int)d.ends.size();
        for(int i = 0; i < nend; i++){
            d.ends[i].pos_t_abs = d_wb.ends[i].pos_t_abs;
            d.ends[i].pos_r_abs = d_wb.ends[i].pos_r_abs;
            d.ends[i].vel_t_abs = d_wb.ends[i].vel_t_abs;
            d.ends[i].vel_r_abs = d_wb.ends[i].vel_r_abs;

            d.ends[i].contact = (d_wb.ends[i].state != dymp::Wholebody::ContactState::Free);
            d.ends[i].mu      = d_wb.ends[i].mu;
            d.ends[i].cop_min = d_wb.ends[i].cop_min;
            d.ends[i].cop_max = d_wb.ends[i].cop_max;
            d.ends[i].pos_tc  = d_wb.ends[i].pos_tc;
        }

        // find contact face
        for(int i = 0; i < nend; i++){
            //if(d.ends[i].contact){
                vec3_t pos_tc_abs = d.ends[i].pos_t_abs + d.ends[i].pos_r_abs*d.ends[i].pos_tc;
                int iface = cen->FindFace(pos_tc_abs, d_wb.ends[i].normal);
                if(iface == -1){
                    iface = cen->CreateFace(pos_tc_abs, d_wb.ends[i].normal);
                }
                d.ends[i].iface = iface;
            //}
            //else{
            //    d.ends[i].iface = 0;
            //}

            cen->CalcEndState(i, d, false);
        }
    }

    d.I     [idiv] = d_wb.centroid.I_abs;
    d.Iinv  [idiv] = d_wb.centroid.I_abs_inv;
    d.Llocal[idiv] = d_wb.centroid.pos_r*d_wb.centroid.L_local;
    //d.Llocal[idiv] = dymp::zero3;
}

bool Kinematics::CalcIK(const dymp::vec3_t& pos, const dymp::quat_t& ori, double sign, vector<double>& q){
    q.resize(6);

    double L0 = verticalHipJointOffset;
    double L1 = lateralHipJointOffset;
    double L2 = lateralKneeOffset;
    double L3 = thighLength;
    double L4 = shankLength;

    dymp::mat3_t R_ha = ori.toRotationMatrix();
    auto R_ah = R_ha.transpose();
    dymp::vec3_t p = pos;
    p.y() -= sign*L1;
    p.z() -= L0;
    dymp::vec3_t p_ah = -R_ah * p;
    
    // knee bend
    double c_k = (p_ah.dot(p_ah) - (L2 * L2 + L3 * L3 + L4 * L4)) / (2.0 * L3 * L4);
    //  knee fully stretched
    if(c_k > 1.0){
        return false;
    }
    q[3] = acos(c_k);
    
    double s_k = sqrt(1.0 - c_k * c_k);
    // position of hip relative to ankle
    dymp::vec3_t p2(-L3 * s_k, -1.0 * sign * L2, L3 * c_k + L4);

    q[4] =  atan2(p2.z(), p2.y()) - acos(p_ah.y()/sqrt(p2.y()*p2.y() + p2.z()*p2.z()));

    double c_ap = cos(q[4]);
    double s_ap = sin(q[4]);
    q[5] =  atan2(p_ah.z(), p_ah.x()) - atan2(-s_ap*p2.y() + c_ap*p2.z(), p2.x());

    double c_kap = cos(q[3] + q[4]);
    double s_kap = sin(q[3] + q[4]);
    double c_ar = cos(q[5]);
    double s_ar = sin(q[5]);

    // Ry(q3)*Rx(q4)*Ry(q5)
    dymp::mat3_t R_ak = (Eigen::AngleAxisd(q[3], dymp::ey)*Eigen::AngleAxisd(q[4], dymp::ex)*Eigen::AngleAxisd(q[5], dymp::ey)).toRotationMatrix();
    dymp::mat3_t R_kh = R_ak * R_ah;

    q[0] = atan2(-R_kh(1,0), R_kh(1,1));
    q[1] = atan2( R_kh(1,2), sqrt(R_kh(1,0) * R_kh(1,0) + R_kh(1,1) * R_kh(1,1)));
    q[2] = atan2(-R_kh(0,2), R_kh(2,2));

    return true;
}

}
}
