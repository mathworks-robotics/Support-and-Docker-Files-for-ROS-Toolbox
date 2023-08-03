/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_GETIMAGEMSGHANDLER_HPP
#define GAZEBOCOSIM_GETIMAGEMSGHANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include "gazebo/physics/World.hh"

namespace robotics {
namespace gazebotransport {
class GetImageMsgHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiated message handler to exchange GetImage message
    /*
      @param world           Gazebo simulation world pointer
      */
    explicit GetImageMsgHandler(gazebo::physics::WorldPtr world);
    /// Destructor
    ~GetImageMsgHandler();

    /**
      @param msgContent      Packet message

      Handles Packet message & construct new Packet message with Image data.
      Based on input message topicname, respective camera image is retrieved.
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
