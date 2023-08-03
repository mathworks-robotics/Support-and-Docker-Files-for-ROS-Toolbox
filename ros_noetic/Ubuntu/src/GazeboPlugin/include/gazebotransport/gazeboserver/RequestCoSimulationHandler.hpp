/* Copyright 2019 The MathWorks, Inc. */
#ifndef REQUEST_COSIMULATION_MSGHANDLER_HPP
#define REQUEST_COSIMULATION_MSGHANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include "gazebotransport/gazeboserver/GazeboWorldInterface.hpp"

#include <memory>

namespace robotics {
namespace gazebotransport {
/// forward declare Gazebo server class
class GazeboServer;

/// Handler for request co-simulation message
class RequestCoSimulationHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiated message handler to handler co-simulation message exchange
    explicit RequestCoSimulationHandler(std::shared_ptr<GazeboServer> server,
                                        std::shared_ptr<GazeboWorldInterface> world);

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
    std::shared_ptr<GazeboWorldInterface> m_world;
};
} // namespace gazebotransport
} // namespace robotics

#endif
