/* Copyright 2020 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_GETLINKSTATEMSGHANDLER_HPP
#define GAZEBOCOSIM_GETLINKSTATEMSGHANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include <gazebo/physics/physics.hh>

namespace robotics {
namespace gazebotransport {
class GetLinkStateMsgHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiated message handler to exchange GetLinkState message
    /*
      @param gazebo::physics::WorldPtr      Pointer of Gazebo World Interface
      */
    explicit GetLinkStateMsgHandler(gazebo::physics::WorldPtr ptr);
    /// Destructor
    ~GetLinkStateMsgHandler();
    /**
      @param msgContent      Packet message

      Handles Packet message & construct new Packet message with LinkState of model.
      Based on input message modelname, respective LinkState data is retrieved.
      Further, it serializes Packet ( reply message) into string and returns.
      */
    std::string handleMessage(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) override;
    /// Returns Message ID
    uint32_t getAcceptID() const override;

  private:
    /// Gazebo world pointer
    gazebo::physics::WorldPtr m_ptr;
};
} // namespace gazebotransport
} // namespace robotics
#endif
