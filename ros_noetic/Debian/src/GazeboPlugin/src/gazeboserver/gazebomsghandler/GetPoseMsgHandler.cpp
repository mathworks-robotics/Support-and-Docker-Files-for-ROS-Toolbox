/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/GetPoseMsgHandler.hpp"

namespace robotics {
namespace gazebotransport {

GetPoseMsgHandler::GetPoseMsgHandler(gazebo::physics::WorldPtr ptr)
    : m_ptr(ptr) {
}

GetPoseMsgHandler::~GetPoseMsgHandler() {
}

std::string GetPoseMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    mw::internal::robotics::gazebotransport::Packet replyMsg;
    replyMsg.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_POSE);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds((uint64_t)m_ptr->SimTime().sec);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(
        (uint64_t)m_ptr->SimTime().nsec);

    /// Separate model name and link name from input message
    gazebo::physics::ModelPtr m_model =
        m_ptr->ModelByName(msgContent.get_ground_truth_world_pose()
                               .model_name()); // Retrieve model from Gazebo world pointer

    if (m_model) // Validate input model available
    {
        gazebo::physics::LinkPtr _link =
            m_model->GetLink(msgContent.get_ground_truth_world_pose().link_name()); // Retrieve link

        if (_link) {
            ignition::math::Pose3d pose = _link->WorldPose(); // Get Pose of link

            replyMsg.mutable_pose()->mutable_position()->set_x(pose.Pos()[0]);
            replyMsg.mutable_pose()->mutable_position()->set_y(pose.Pos()[1]);
            replyMsg.mutable_pose()->mutable_position()->set_z(
                pose.Pos()[2]); // Insert pose data into Pose message
            replyMsg.mutable_pose()->mutable_orientation()->set_w(
                pose.Rot().W()); // For Euler : pose.Rot().Euler().X(),
            replyMsg.mutable_pose()->mutable_orientation()->set_x(
                pose.Rot().X()); //         pose.Rot().Euler().Y(), pose.Rot().Euler().Z()
            replyMsg.mutable_pose()->mutable_orientation()->set_y(pose.Rot().Y());
            replyMsg.mutable_pose()->mutable_orientation()->set_z(pose.Rot().Z());
        } else {
            replyMsg.mutable_pose()->mutable_position()->set_x(0.0);
            replyMsg.mutable_pose()->mutable_position()->set_y(0.0);
            replyMsg.mutable_pose()->mutable_position()->set_z(
                0.0); // Insert null data into Pose message if link is not available
            replyMsg.mutable_pose()->mutable_orientation()->set_w(0.0);
            replyMsg.mutable_pose()->mutable_orientation()->set_x(0.0);
            replyMsg.mutable_pose()->mutable_orientation()->set_y(0.0);
            replyMsg.mutable_pose()->mutable_orientation()->set_z(0.0);
        }
    } else {
        replyMsg.mutable_pose()->mutable_position()->set_x(0.0);
        replyMsg.mutable_pose()->mutable_position()->set_y(0.0);
        replyMsg.mutable_pose()->mutable_position()->set_z(
            0.0); // Insert null data into Pose message if model is not available
        replyMsg.mutable_pose()->mutable_orientation()->set_w(0.0);
        replyMsg.mutable_pose()->mutable_orientation()->set_x(0.0);
        replyMsg.mutable_pose()->mutable_orientation()->set_y(0.0);
        replyMsg.mutable_pose()->mutable_orientation()->set_z(0.0);
    }

    return replyMsg.SerializeAsString(); // returns serialized Pose message
}

uint32_t GetPoseMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_GET_GROUND_TRUTH_WORLD_POSE;
}
} // namespace gazebotransport
} // namespace robotics
