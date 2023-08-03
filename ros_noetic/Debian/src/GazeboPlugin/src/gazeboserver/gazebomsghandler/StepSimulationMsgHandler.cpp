/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/StepSimulationMsgHandler.hpp"

namespace robotics {
namespace gazebotransport {

StepSimulationMsgHandler::StepSimulationMsgHandler(std::shared_ptr<GazeboWorldInterface> ptr)
    : m_ptr(ptr) {
}
StepSimulationMsgHandler::~StepSimulationMsgHandler() {
}

std::string StepSimulationMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {

    if (m_ptr->isPaused()) // Check Gazebo is paused/running
    {
        m_ptr->step(msgContent.step_simulation()
                        .num_steps()); // Step/Simulate Gazebo for input number of steps
    }

    mw::internal::robotics::gazebotransport::Packet replyMsg;
    replyMsg.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_STATUS);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds(0);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError_NONE);

    return replyMsg.SerializeAsString(); // Return serialized data
}

uint32_t StepSimulationMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::
        PacketHeader_MsgID_STEP_SIMULATION; // Message ID from MsgTable
}
} // namespace gazebotransport
} // namespace robotics
