/* Copyright 2020 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/GetGazeboModelParamMsgHandler.hpp"

namespace robotics {
namespace gazebotransport {

GetGazeboModelParamMsgHandler::GetGazeboModelParamMsgHandler(gazebo::physics::WorldPtr ptr)
    : m_ptr(ptr)
    , m_utils(std::make_shared<GazeboMLUtils>()) {
}
GetGazeboModelParamMsgHandler::~GetGazeboModelParamMsgHandler() {
}

void GetGazeboModelParamMsgHandler::GetLinkParam(
    mw::internal::robotics::gazebotransport::Packet& replyMsg,
    gazebo::physics::LinkPtr link) {

    // Get Link message from packet
    mw::internal::robotics::gazebotransport::ML_Links* link_message =
        replyMsg.mutable_gazebo_model()->add_links();

    // retrieve Model-Link parameter details and add to Link message
    link_message->set_name(link->GetName());

    link_message->set_gravity(link->GetGravityMode());

    link_message->set_self_collide(link->GetSelfCollide());

    link_message->set_kinematic(link->GetKinematic());

    link_message->set_enabled_wind(link->WindMode());

    link_message->set_is_static(link->IsStatic());

    link_message->set_canonical(link->IsCanonicalLink());

    // retrieve and add Model-Link Pose message
    ignition::math::Pose3d world_pose = link->WorldPose();
    mw::internal::robotics::gazebotransport::ML_Pose* ml_pose = link_message->mutable_pose();
    m_utils->ConvertMLPose(ml_pose, world_pose);

    mw::internal::robotics::gazebotransport::ML_Inertial* ml_intertial =
        link_message->mutable_inertial();
    gazebo::physics::InertialPtr intertial_ptr = link->GetInertial();

    // retrieve and add Model-Link Inertial parameter message
    ml_intertial->set_mass(intertial_ptr->Mass());
    ml_intertial->set_ixx(intertial_ptr->IXX());
    ml_intertial->set_ixy(intertial_ptr->IXY());
    ml_intertial->set_ixz(intertial_ptr->IXZ());
    ml_intertial->set_iyy(intertial_ptr->IYY());
    ml_intertial->set_iyz(intertial_ptr->IYZ());
    ml_intertial->set_izz(intertial_ptr->IZZ());
}

void GetGazeboModelParamMsgHandler::GetJointParam(
    mw::internal::robotics::gazebotransport::Packet& replyMsg,
    gazebo::physics::JointPtr joint) {
    mw::internal::robotics::gazebotransport::ML_Joints* joint_message =
        replyMsg.mutable_gazebo_model()->add_joints();

    {
        // get joint name
        joint_message->set_name(joint->GetName());
        // set dof
        joint_message->set_dof(static_cast<uint32_t>(joint->DOF()));
        // get initial anchor joint pose
        ignition::math::Pose3d init_anchor_pose = joint->InitialAnchorPose();

        // retrieve and set Model-Joint Pose message
        mw::internal::robotics::gazebotransport::ML_Pose* ml_pose = joint_message->mutable_pose();
        m_utils->ConvertMLPose(ml_pose, init_anchor_pose);

        if (joint->DOF() > 0) {
            joint_message->set_fudge_factor(joint->GetParam("fudge_factor", 0));
            joint_message->set_cfm(joint->GetParam("cfm", 0));
            joint_message->set_suspension_cfm(joint->GetParam("suspension_cfm", 0));
            joint_message->set_suspension_erp(joint->GetParam("suspension_erp", 0));
        }

        {
            unsigned int index = 0;
            // for axis index 0
            if (joint->DOF() > 0) {
                // get and add joint parameters
                mw::internal::robotics::gazebotransport::ML_Axis* ml_axis1 =
                    joint_message->mutable_axis1();

                ml_axis1->set_angle(joint->Position(index));
                {
                    ignition::math::Vector3d axis1_xyz = joint->GlobalAxis(index);
                    mw::internal::robotics::gazebotransport::ML_Point* ml_axis1_xyz =
                        ml_axis1->mutable_xyz();
                    m_utils->ConvertMLPose(ml_axis1_xyz, axis1_xyz);

                    ml_axis1->set_damping(joint->GetDamping(index));
                    ml_axis1->set_friction(joint->GetParam("friction", index));
                }
            }

            index = 1;
            // for axis index 1
            if (joint->DOF() > 1) {
                // get and add joint parameters
                mw::internal::robotics::gazebotransport::ML_Axis* ml_axis2 =
                    joint_message->mutable_axis2();

                ml_axis2->set_angle(joint->Position(index));
                {
                    ignition::math::Vector3d axis2_xyz = joint->GlobalAxis(index);
                    mw::internal::robotics::gazebotransport::ML_Point* ml_axis2_xyz =
                        ml_axis2->mutable_xyz();
                    m_utils->ConvertMLPose(ml_axis2_xyz, axis2_xyz);

                    ml_axis2->set_damping(joint->GetDamping(index));
                    ml_axis2->set_friction(joint->GetParam("friction", index));
                }
            }
        }
    }
}

std::string GetGazeboModelParamMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    mw::internal::robotics::gazebotransport::Packet replyMsg;

    replyMsg.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                          PacketHeader_MsgID_GAZEBO_MODEL);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds((uint64_t)m_ptr->SimTime().sec);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(
        (uint64_t)m_ptr->SimTime().nsec);
    replyMsg.mutable_gazebo_model()->set_name("");

    std::string model_name = msgContent.get_gazebo_model_param().model_name();

    gazebo::physics::ModelPtr model = m_ptr->ModelByName(model_name); // Retrieve model

    if (model) {
        // get and add Model parameters
        replyMsg.mutable_gazebo_model()->set_name(model_name);

        replyMsg.mutable_gazebo_model()->set_enable_wind(model->WindMode());

        replyMsg.mutable_gazebo_model()->set_self_collide(model->GetSelfCollide());

        replyMsg.mutable_gazebo_model()->set_is_static(model->IsStatic());

        // get joint pose relative to world frame
        ignition::math::Pose3d world_pose = model->WorldPose();

        mw::internal::robotics::gazebotransport::ML_Pose* ml_pose =
            replyMsg.mutable_gazebo_model()->mutable_pose();

        m_utils->ConvertMLPose(ml_pose, world_pose);

        //  if link or joint parameter details are requested
        if (msgContent.get_gazebo_model_param().has_is_link()) {
            // get requested link or joint name
            std::string linkJointName = msgContent.get_gazebo_model_param().link_joint_name();

            if (msgContent.get_gazebo_model_param().is_link()) {
                // get link parameter details
                auto link_list = model->GetLinks();
                for (size_t linkId = 0; linkId < link_list.size(); linkId++) {
                    gazebo::physics::LinkPtr link = link_list[linkId];
                    if (linkJointName == link->GetName()) {
                        GetLinkParam(replyMsg, link);
                    }
                }
            } else {
                // get joint parameter details
                auto joint_list = model->GetJoints();
                for (size_t jointId = 0; jointId < joint_list.size(); jointId++) {
                    gazebo::physics::JointPtr joint = joint_list[jointId];
                    if (linkJointName == joint->GetName()) {
                        GetJointParam(replyMsg, joint);
                    }
                }
            }
        }
    }

    return replyMsg.SerializeAsString();
}

uint32_t GetGazeboModelParamMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_GET_GAZEBO_MODEL_PARAM; // Message ID from MsgTable
}
} // namespace gazebotransport
} // namespace robotics
