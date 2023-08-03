/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_GETLASERMSGHANDLER_HPP
#define GAZEBOCOSIM_GETLASERMSGHANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include "gazebo/physics/World.hh"

namespace robotics {
namespace gazebotransport {
class GetLaserMsgHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiated message handler to exchange GetLaser message
    /*
      @param world           Gazebo simulation world pointer
      */
    explicit GetLaserMsgHandler(gazebo::physics::WorldPtr world);

    /// Destructor
    ~GetLaserMsgHandler();
    /**
      @param msgContent      Packet message

      Handles Packet message & construct new Packet message with Laser data.
      Based on input message topicname, respective Laser data is retrieved.
      Further, serializes Packet ( reply message) into string and returns.
      */
    std::string handleMessage(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) override;
    /// Returns Message ID
    uint32_t getAcceptID() const override; // Returns Message ID

  private:
    /// Gazebo world pointer used for obtaining simulation time
    gazebo::physics::WorldPtr m_world;
};
} // namespace gazebotransport
} // namespace robotics
#endif
