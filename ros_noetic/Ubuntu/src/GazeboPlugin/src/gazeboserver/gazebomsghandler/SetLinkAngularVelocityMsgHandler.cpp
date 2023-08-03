/* Copyright 2020-2021 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/SetLinkAngularVelocityMsgHandler.hpp"

namespace robotics {
namespace gazebotransport {

SetLinkAngularVelocityMsgHandler::SetLinkAngularVelocityMsgHandler(
    gazebo::physics::WorldPtr ptr,
    robotics::gazebotransport::GazeboApplyCommander& commander)
    : m_ptr(ptr)
    , m_commander(commander) {
}
SetLinkAngularVelocityMsgHandler::~SetLinkAngularVelocityMsgHandler() {
}


std::string SetLinkAngularVelocityMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    mw::internal::robotics::gazebotransport::Packet replyMsg;

    replyMsg.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds((uint64_t)m_ptr->SimTime().sec);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(
        (uint64_t)m_ptr->SimTime().nsec);

    gazebo::physics::ModelPtr model =
        m_ptr->ModelByName(msgContent.set_link_angular_velocity().model_name()); // Retrieve model

    if (model) // Validate model name
    {
        gazebo::physics::LinkPtr _Link =
            model->GetLink(msgContent.set_link_angular_velocity().link_name()); // Retrieve link

        if (_Link) // Validate link name
        {
            // end time for the set velocity
            auto const& duration = msgContent.set_link_angular_velocity().duration();
            ::gazebo::common::Time durationTimeSeconds{static_cast<double>(duration.seconds())};
            ::gazebo::common::Time durationTImeNanoSeconds{
                0, static_cast<int32_t>(duration.nano_seconds())};
            ::gazebo::common::Time endTime =
                m_ptr->SimTime() + durationTimeSeconds + durationTImeNanoSeconds;

            // create and store link pointer object
            m_commander.insertSetLinkAngularVelocityCommand(
                msgContent.set_link_angular_velocity().model_name() +
                    msgContent.set_link_angular_velocity().link_name(),
                std::make_shared<robotics::gazebotransport::LinkPtrStorage>(_Link, endTime,
                                                                            msgContent));

            replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                    Packet_CoSimError_NONE); // Set Link Angular Velocity - Success
        } else {
            replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                    Packet_CoSimError_LINK_NAME_INVALID); // Invalid link name -
                                                                          // Failure
        }
    } else {
        replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                Packet_CoSimError_MODEL_NAME_INVALID); // Invalid model name -
                                                                       // Failure
    }


    return replyMsg.SerializeAsString();
}

uint32_t SetLinkAngularVelocityMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_SET_LINK_ANGULAR_VELOCITY; // Message ID from
                                                      // MsgTable
}
} // namespace gazebotransport
} // namespace robotics
