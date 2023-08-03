/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_RESETSIMULATIONMSGHANDLER_HPP
#define GAZEBOCOSIM_RESETSIMULATIONMSGHANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include "gazebotransport/gazeboserver/GazeboWorldInterface.hpp"

#include <memory>

namespace robotics {
namespace gazebotransport {
class ResetSimulationMsgHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiated message handler to exchange ResetSimulation message
    /*
    @param std::shared_ptr<GazeboWorldInterface>      Pointer of Gazebo World Interface
    */
    explicit ResetSimulationMsgHandler(std::shared_ptr<GazeboWorldInterface> ptr);

    /// Destructor
    ~ResetSimulationMsgHandler() override;

    /**
    @param msgContent      Packet message

    Handles Packet message & construct new Packet message with proper fields (Error/ reply content
    ). Based on input message, either resets scene and time or resets only time.
    Further, serializes Packet ( reply message) into string and returns.
    */
    std::string handleMessage(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) override;

    /// Returns Message ID
    uint32_t getAcceptID() const override;

  private:
    /// Gazebo World Interface pointer
    std::shared_ptr<GazeboWorldInterface> m_ptr;
};
} // namespace gazebotransport
} // namespace robotics
#endif
