/* Copyright 2019 The MathWorks, Inc. */

#include "gazebotransport/gazeboserver/RequestCoSimulationHandler.hpp"
#include "gazebotransport/GazeboServer.hpp"

namespace robotics {
namespace gazebotransport {
RequestCoSimulationHandler::RequestCoSimulationHandler(std::shared_ptr<GazeboServer> server,
                                                       std::shared_ptr<GazeboWorldInterface> world)
    : m_server(server)
    , m_world(world) {
}

std::string RequestCoSimulationHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    bool success = false;

    // record server co-simulation status before attempting to start co-simulation
    // if server is not already in co-simulation mode, pause the server if co-simulation
    // is granted
    bool serverCoSimAlready = m_server->getCoSimulationStatus().first;

    if (msgContent.has_request_cosim()) {
        success = m_server->startCoSimulation(msgContent.request_cosim().client_id(),
                                              msgContent.request_cosim().duration());
    }

    mw::internal::robotics::gazebotransport::Packet replyMsg;
    replyMsg.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds(0);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);

    if (success) {
        if (!serverCoSimAlready) {
            m_world->setPaused(true);
        }
        replyMsg.set_status(
            mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);
    } else {
        replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                Packet_CoSimError_COSIM_FAILED);
    }

    return replyMsg.SerializeAsString();
}

uint32_t RequestCoSimulationHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_REQUEST_COSIM;
}
} // namespace gazebotransport
} // namespace robotics
