/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/ResetSimulationMsgHandler.hpp"

namespace robotics {
namespace gazebotransport {

ResetSimulationMsgHandler::ResetSimulationMsgHandler(std::shared_ptr<GazeboWorldInterface> ptr)
    : m_ptr(ptr) {
}
ResetSimulationMsgHandler::~ResetSimulationMsgHandler() {
}

std::string ResetSimulationMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    mw::internal::robotics::gazebotransport::Packet replyMsg;

    if (msgContent.reset_simulation().behavior()) {
        m_ptr->reset(); // Reset Gazebo world time and scene
    } else {
        m_ptr->resetTime(); // Reset Gazebo world time only
    }

    replyMsg.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_STATUS);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds(0);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError_NONE);

    return replyMsg.SerializeAsString(); // Return serialized data
}

uint32_t ResetSimulationMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::
        PacketHeader_MsgID_RESET_SIMULATION; // Message ID from MsgTable
}
} // namespace gazebotransport
} // namespace robotics
