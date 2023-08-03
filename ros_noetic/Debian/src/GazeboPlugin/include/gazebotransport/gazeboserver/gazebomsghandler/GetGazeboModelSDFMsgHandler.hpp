/* Copyright 2021 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_GET_GAZEBO_MODEL_SDFMSGHANDLER_HPP
#define GAZEBOCOSIM_GET_GAZEBO_MODEL_SDFMSGHANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomlsupport/GazeboMLUtils.hpp"
#include <gazebo/physics/physics.hh>

namespace robotics {
namespace gazebotransport {
class GetGazeboModelSDFMsgHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiated message handler to exchange GetGazeboModelSDF message
    /**
     * @param ptr      			Pointer of Gazebo World Interface
     */
    explicit GetGazeboModelSDFMsgHandler(gazebo::physics::WorldPtr ptr);
    /// Destructor
    ~GetGazeboModelSDFMsgHandler();

    /**
     * @param msgContent      Packet message
     *
     * Handles Packet message & construct new Packet message with parameter details of model.
     * Based on requested message model name, respective SDF details are retrieved.
     * Further, it serializes Packet ( reply message) into string and returns.
     */
    std::string handleMessage(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) override;
    /// Returns Message ID
    uint32_t getAcceptID() const override;

  private:
    /// Gazebo world pointer
    gazebo::physics::WorldPtr m_ptr;

    /// Gazebo MATLAB Interface Utils object
    std::shared_ptr<robotics::gazebotransport::GazeboMLUtils> m_utils;
};
} // namespace gazebotransport
} // namespace robotics
#endif
