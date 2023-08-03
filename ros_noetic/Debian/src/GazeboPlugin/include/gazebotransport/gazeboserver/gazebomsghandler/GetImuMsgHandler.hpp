/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_GETIMUMSGHANDLER_HPP
#define GAZEBOCOSIM_GETIMUMSGHANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include "gazebo/physics/World.hh"

namespace robotics {
namespace gazebotransport {
class GetImuMsgHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiated message handler to exchange GetImu message
    /*
      @param world           Gazebo simulation world pointer
      */
    explicit GetImuMsgHandler(gazebo::physics::WorldPtr world);
    /// Destructor
    ~GetImuMsgHandler();
    /**
      @param msgContent      Packet message

      Handles Packet message & construct new Packet message with IMU data.
      Based on input message topicname, respective IMU data is retrieved.
      Further, serializes Packet ( reply message) into string and returns.
      */
    std::string handleMessage(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) override;
    /// Returns Message ID
    uint32_t getAcceptID() const override;

  private:
    /// Gazebo world pointer used for getting simulation time
    gazebo::physics::WorldPtr m_world;
};
} // namespace gazebotransport
} // namespace robotics
#endif
