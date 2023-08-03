/* Copyright 2020-2021 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/SetJointVelocityMsgHandler.hpp"

namespace robotics {
namespace gazebotransport {

SetJointVelocityMsgHandler::SetJointVelocityMsgHandler(
    gazebo::physics::WorldPtr ptr,
    robotics::gazebotransport::GazeboApplyCommander& commander)
    : m_ptr(ptr)
    , m_commander(commander) {
}
SetJointVelocityMsgHandler::~SetJointVelocityMsgHandler() {
}


std::string SetJointVelocityMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    mw::internal::robotics::gazebotransport::Packet replyMsg;

    replyMsg.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds((uint64_t)m_ptr->SimTime().sec);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(
        (uint64_t)m_ptr->SimTime().nsec);

    gazebo::physics::ModelPtr model =
        m_ptr->ModelByName(msgContent.set_joint_velocity().model_name()); // Retrieve model

    if (model) // Validate model name
    {
        gazebo::physics::JointPtr _Joint =
            model->GetJoint(msgContent.set_joint_velocity().joint_name()); // Retrieve joint

        if (_Joint) // Validate joint name
        {
            // end time for the set joint velocity
            auto const& duration = msgContent.set_joint_velocity().duration();
            ::gazebo::common::Time durationTimeSeconds{static_cast<double>(duration.seconds())};
            ::gazebo::common::Time durationTImeNanoSeconds{
                0, static_cast<int32_t>(duration.nano_seconds())};
            ::gazebo::common::Time endTime =
                m_ptr->SimTime() + durationTimeSeconds + durationTImeNanoSeconds;

            // create and store joint pointer object
            m_commander.insertJointSetVelocityCommand(
                msgContent.set_joint_velocity().model_name() +
                    msgContent.set_joint_velocity().joint_name(),
                std::make_shared<robotics::gazebotransport::JointPtrStorage>(_Joint, endTime,
                                                                             msgContent));

            replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                    Packet_CoSimError_NONE); // Set Joint Velocity - Success
        } else {
            replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                    Packet_CoSimError_JOINT_NAME_INVALID); // Invalid joint name -
                                                                           // Failure
        }
    } else {
        replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                Packet_CoSimError_MODEL_NAME_INVALID); // Invalid model name -
                                                                       // Failure
    }


    return replyMsg.SerializeAsString();
}

uint32_t SetJointVelocityMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_SET_JOINT_VELOCITY; // Message ID from MsgTable
}
} // namespace gazebotransport
} // namespace robotics
