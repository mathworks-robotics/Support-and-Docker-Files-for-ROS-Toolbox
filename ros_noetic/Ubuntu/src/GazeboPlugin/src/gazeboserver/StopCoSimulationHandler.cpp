/* Copyright 2019 The MathWorks, Inc. */

#include "gazebotransport/gazeboserver/StopCoSimulationHandler.hpp"
#include "gazebotransport/GazeboServer.hpp"

namespace robotics {
namespace gazebotransport {
StopCoSimulationHandler::StopCoSimulationHandler(std::shared_ptr<GazeboServer> server)
    : m_server(server) {
}

std::string StopCoSimulationHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    bool success = false;

    if (msgContent.has_stop_cosim()) {
        success = m_server->stopCoSimulation(msgContent.stop_cosim().client_id());
    }

    mw::internal::robotics::gazebotransport::Packet replyMsg;
    replyMsg.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds(0);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);

    if (success) {
        replyMsg.set_status(
            mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);
    } else {
        replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                Packet_CoSimError_STOP_COSIM_FAILED);
    }

    return replyMsg.SerializeAsString();
}

uint32_t StopCoSimulationHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_STOP_COSIM;
}
} // namespace gazebotransport
} // namespace robotics
