/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_MAXSIMULATIONSTEPMSGHANDLER_HPP
#define GAZEBOCOSIM_MAXSIMULATIONSTEPMSGHANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include <gazebo/physics/physics.hh>

namespace robotics {
namespace gazebotransport {
class MaxSimulationStepMsgHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiated message handler to set/get max step size value message
    /*
      @param gazebo::physics::WorldPtr      Pointer of Gazebo World Interface
      */
    explicit MaxSimulationStepMsgHandler(gazebo::physics::WorldPtr ptr);

    /// Destructor
    ~MaxSimulationStepMsgHandler();

    /**
      @param msgContent      Packet message

      Handles Packet message & construct new Packet message with proper fields (Error/ reply content
      ). Based on input message, max step size set or returned
      Further, serializes Packet ( reply message) into string and returns.
      */
    std::string handleMessage(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) override;

    /// Returns Message ID
    uint32_t getAcceptID() const override;

  private:
    bool m_success;
    /// Gazebo world pointer
    gazebo::physics::WorldPtr m_ptr;
};
} // namespace gazebotransport
} // namespace robotics
#endif
