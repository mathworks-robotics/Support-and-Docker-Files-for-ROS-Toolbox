/* Copyright 2019 The MathWorks, Inc. */
#ifndef STOP_COSIMULATION_MSGHANDLER_HPP
#define STOP_COSIMULATION_MSGHANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"

#include <memory>

namespace robotics {
namespace gazebotransport {
/// forward declare Gazebo server class
class GazeboServer;

/// Handler for request to stop co-simulation message
class StopCoSimulationHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiated message handler to handler "stop co-simulation" message exchange
    explicit StopCoSimulationHandler(std::shared_ptr<GazeboServer> server);

    /**
    @param msgContent      Packet message

    Handles Packet message & construct new Packet message with proper fields (Error/ reply content
    ). Further, serializes Packet message into string and returns.
    */
    std::string handleMessage(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) override;

    /// Returns Message ID
    uint32_t getAcceptID() const override;

  private:
    std::shared_ptr<GazeboServer> m_server;
};
} // namespace gazebotransport
} // namespace robotics

#endif
