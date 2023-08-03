/* Copyright 2020 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/GetLinkStateMsgHandler.hpp"

namespace robotics {
namespace gazebotransport {

GetLinkStateMsgHandler::GetLinkStateMsgHandler(gazebo::physics::WorldPtr ptr)
    : m_ptr(ptr) {
}

GetLinkStateMsgHandler::~GetLinkStateMsgHandler() {
}

std::string GetLinkStateMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    mw::internal::robotics::gazebotransport::Packet replyMsg;
    replyMsg.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_LINK_STATE);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds((uint64_t)m_ptr->SimTime().sec);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(
        (uint64_t)m_ptr->SimTime().nsec);

    // initialization
    replyMsg.mutable_link_state()->set_model_name("");
    replyMsg.mutable_link_state()->set_link_name("");
    replyMsg.mutable_link_state()->set_link_id(0);
    replyMsg.mutable_link_state()->mutable_world_linear_velocity()->set_x(0);
    replyMsg.mutable_link_state()->mutable_world_linear_velocity()->set_y(0);
    replyMsg.mutable_link_state()->mutable_world_linear_velocity()->set_z(0);
    replyMsg.mutable_link_state()->mutable_world_angular_velocity()->set_x(0);
    replyMsg.mutable_link_state()->mutable_world_angular_velocity()->set_y(0);
    replyMsg.mutable_link_state()->mutable_world_angular_velocity()->set_z(0);
    replyMsg.mutable_link_state()->mutable_relative_linear_velocity()->set_x(0);
    replyMsg.mutable_link_state()->mutable_relative_linear_velocity()->set_y(0);
    replyMsg.mutable_link_state()->mutable_relative_linear_velocity()->set_z(0);
    replyMsg.mutable_link_state()->mutable_relative_angular_velocity()->set_x(0);
    replyMsg.mutable_link_state()->mutable_relative_angular_velocity()->set_y(0);
    replyMsg.mutable_link_state()->mutable_relative_angular_velocity()->set_z(0);

    /// Separate model name and link name from input message
    gazebo::physics::ModelPtr _model = m_ptr->ModelByName(
        msgContent.get_link_state().model_name()); // Retrieve model from Gazebo world pointer

    if (_model) // Validate input model available
    {
        gazebo::physics::LinkPtr _link =
            _model->GetLink(msgContent.get_link_state().link_name()); // Retrieve joint

        if (_link) {
            // get model name
            replyMsg.mutable_link_state()->set_model_name(_model->GetName());

            // get link name
            replyMsg.mutable_link_state()->set_link_name(_link->GetName());

            // get link id
            replyMsg.mutable_link_state()->set_link_id(_link->GetId());

            // get world linear velocity
            ignition::math::Vector3d world_linear_vel =
                _link->WorldLinearVel(); // Get the linear velocity of the body in world frame
            replyMsg.mutable_link_state()->mutable_world_linear_velocity()->set_x(
                world_linear_vel.X());
            replyMsg.mutable_link_state()->mutable_world_linear_velocity()->set_y(
                world_linear_vel.Y());
            replyMsg.mutable_link_state()->mutable_world_linear_velocity()->set_z(
                world_linear_vel.Z());

            // get world angular velocity
            ignition::math::Vector3d world_angular_vel =
                _link->WorldAngularVel(); // Get the angular velocity of the body in world frame
            replyMsg.mutable_link_state()->mutable_world_angular_velocity()->set_x(
                world_angular_vel.X());
            replyMsg.mutable_link_state()->mutable_world_angular_velocity()->set_y(
                world_angular_vel.Y());
            replyMsg.mutable_link_state()->mutable_world_angular_velocity()->set_z(
                world_angular_vel.Z());

            // get relative linear velocity
            ignition::math::Vector3d rel_linear_vel =
                _link->RelativeLinearVel(); // Get the linear velocity of the body
            replyMsg.mutable_link_state()->mutable_relative_linear_velocity()->set_x(
                rel_linear_vel.X());
            replyMsg.mutable_link_state()->mutable_relative_linear_velocity()->set_y(
                rel_linear_vel.Y());
            replyMsg.mutable_link_state()->mutable_relative_linear_velocity()->set_z(
                rel_linear_vel.Z());

            // get relative angular velocity
            ignition::math::Vector3d rel_angular_vel =
                _link->RelativeAngularVel(); // Get the angular velocity of the body
            replyMsg.mutable_link_state()->mutable_relative_angular_velocity()->set_x(
                rel_angular_vel.X());
            replyMsg.mutable_link_state()->mutable_relative_angular_velocity()->set_y(
                rel_angular_vel.Y());
            replyMsg.mutable_link_state()->mutable_relative_angular_velocity()->set_z(
                rel_angular_vel.Z());

            // get pose of link in world frame
            ignition::math::Pose3d world_pose = _link->WorldPose();
            replyMsg.mutable_link_state()->mutable_world_pose()->mutable_position()->set_x(
                world_pose.Pos()[0]);
            replyMsg.mutable_link_state()->mutable_world_pose()->mutable_position()->set_y(
                world_pose.Pos()[1]);
            replyMsg.mutable_link_state()->mutable_world_pose()->mutable_position()->set_z(
                world_pose.Pos()[2]);
            replyMsg.mutable_link_state()->mutable_world_pose()->mutable_orientation()->set_w(
                world_pose.Rot().W());
            replyMsg.mutable_link_state()->mutable_world_pose()->mutable_orientation()->set_x(
                world_pose.Rot().X());
            replyMsg.mutable_link_state()->mutable_world_pose()->mutable_orientation()->set_y(
                world_pose.Rot().Y());
            replyMsg.mutable_link_state()->mutable_world_pose()->mutable_orientation()->set_z(
                world_pose.Rot().Z());

            // get relative link pose
            ignition::math::Pose3d rel_pose = _link->RelativePose();
            replyMsg.mutable_link_state()->mutable_relative_pose()->mutable_position()->set_x(
                rel_pose.Pos()[0]);
            replyMsg.mutable_link_state()->mutable_relative_pose()->mutable_position()->set_y(
                rel_pose.Pos()[1]);
            replyMsg.mutable_link_state()->mutable_relative_pose()->mutable_position()->set_z(
                rel_pose.Pos()[2]);
            replyMsg.mutable_link_state()->mutable_relative_pose()->mutable_orientation()->set_w(
                rel_pose.Rot().W());
            replyMsg.mutable_link_state()->mutable_relative_pose()->mutable_orientation()->set_x(
                rel_pose.Rot().X());
            replyMsg.mutable_link_state()->mutable_relative_pose()->mutable_orientation()->set_y(
                rel_pose.Rot().Y());
            replyMsg.mutable_link_state()->mutable_relative_pose()->mutable_orientation()->set_z(
                rel_pose.Rot().Z());

            // get self-collide, gravity, kinematic, wind status and canonical
            replyMsg.mutable_link_state()->set_self_collide(_link->GetSelfCollide());
            replyMsg.mutable_link_state()->set_gravity(_link->GetGravityMode());
            replyMsg.mutable_link_state()->set_kinematic(_link->GetKinematic());
            replyMsg.mutable_link_state()->set_enable_wind(_link->WindMode());
            replyMsg.mutable_link_state()->set_canonical(_link->IsCanonicalLink());
        }
    }

    return replyMsg.SerializeAsString(); // returns serialized Link State message
}

uint32_t GetLinkStateMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_GET_LINK_STATE;
}
} // namespace gazebotransport
} // namespace robotics
