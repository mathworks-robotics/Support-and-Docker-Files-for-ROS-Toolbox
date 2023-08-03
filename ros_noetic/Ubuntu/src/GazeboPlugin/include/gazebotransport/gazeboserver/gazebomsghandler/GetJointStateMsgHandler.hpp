/* Copyright 2020 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_GETJOINTSTATEMSGHANDLER_HPP
#define GAZEBOCOSIM_GETJOINTSTATEMSGHANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include <gazebo/physics/physics.hh>

namespace robotics {
namespace gazebotransport {
class GetJointStateMsgHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiated message handler to exchange GetJointState message
    /*
      @param gazebo::physics::WorldPtr      Pointer of Gazebo World Interface
      */
    explicit GetJointStateMsgHandler(gazebo::physics::WorldPtr ptr);
    /// Destructor
    ~GetJointStateMsgHandler();
    /**
      @param msgContent      Packet message

      Handles Packet message & construct new Packet message with JointState of model.
      Based on input message modelname, respective JointState data is retrieved.
      Further, it serializes Packet ( reply message) into string and returns.
      */
    std::string handleMessage(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) override;
    /// Returns Message ID
    uint32_t getAcceptID() const override;
    // initialize repeated fields of joint state message
    void initializeRepeatedFields(mw::internal::robotics::gazebotransport::Packet& replyMsg);

  private:
    /// Gazebo world pointer
    gazebo::physics::WorldPtr m_ptr;
};
} // namespace gazebotransport
} // namespace robotics
#endif
