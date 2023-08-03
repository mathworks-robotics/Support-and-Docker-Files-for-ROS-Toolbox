/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_GETPOSEMSGHANDLER_HPP
#define GAZEBOCOSIM_GETPOSEMSGHANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include <gazebo/physics/physics.hh>

namespace robotics {
namespace gazebotransport {
class GetPoseMsgHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiated message handler to exchange GetPose message
    /*
      @param gazebo::physics::WorldPtr      Pointer of Gazebo World Interface
      */
    explicit GetPoseMsgHandler(gazebo::physics::WorldPtr ptr);
    /// Destructor
    ~GetPoseMsgHandler();
    /**
      @param msgContent      Packet message

      Handles Packet message & construct new Packet message with pose of model.
      Based on input message modelname, respective pose data is retrieved.
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
