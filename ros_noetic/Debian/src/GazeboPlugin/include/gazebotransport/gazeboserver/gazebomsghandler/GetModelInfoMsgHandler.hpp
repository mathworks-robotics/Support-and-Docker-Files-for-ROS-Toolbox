/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_GETMODELINFOMSGHANDLER_HPP
#define GAZEBOCOSIM_GETMODELINFOMSGHANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include <gazebo/physics/physics.hh>

namespace robotics {
namespace gazebotransport {
class GetModelInfoMsgHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiated message handler to exchange GetModelInfo message
    /*
      @param gazebo::physics::WorldPtr      Pointer of Gazebo World Interface
      */
    explicit GetModelInfoMsgHandler(gazebo::physics::WorldPtr ptr);

    /// Destructor
    ~GetModelInfoMsgHandler();

    /**
      @param msgContent      Packet message

      Handles Packet message & construct new Packet message with all model
      data like models, links, joints names available in Gazebo.
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
