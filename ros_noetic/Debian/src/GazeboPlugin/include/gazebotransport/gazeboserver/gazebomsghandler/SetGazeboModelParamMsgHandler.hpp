/* Copyright 2020 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_SET_GAZEBO_MODEL_PARAM_MSG_HANDLER_HPP
#define GAZEBOCOSIM_SET_GAZEBO_MODEL_PARAM_MSG_HANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomlsupport/GazeboMLUtils.hpp"
#include <gazebo/physics/physics.hh>

namespace robotics {
namespace gazebotransport {
class SetGazeboModelParamMsgHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiated message handler to exchange SetGazeboModel parameter message
    /**
     * @param ptr                  Pointer of Gazebo World Interface
     */
    explicit SetGazeboModelParamMsgHandler(gazebo::physics::WorldPtr ptr);

    /// Destructor
    ~SetGazeboModelParamMsgHandler();

    /// Set Link Parameter based on Packet message
    /**
     * @param linkMessage     		Gazebo Co-Sim ML_Links message
     * @param link                  Gazebo world link pointer
     */
    void SetLinkParam(mw::internal::robotics::gazebotransport::ML_Links linkMessage,
                      gazebo::physics::LinkPtr link);

    /// Set Joint Parameter based on Packet message
    /**
     * @param linkMessage     		Gazebo Co-Sim ML_Joints message
     * @param link                  Gazebo world link pointer
     */
    std::string SetJointParam(mw::internal::robotics::gazebotransport::ML_Joints jointMessage,
                              gazebo::physics::JointPtr joint);

    /**
     * @param msgContent      Packet message
     *
     * Handles Packet message & construct new Packet message with proper field
     * (Error/reply content). Based on input message, model-link-joint parameter is set.
     * Further, serializes Packet ( reply message) into string and returns.
     */
    std::string handleMessage(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) override;

    /// Returns Message ID
    uint32_t getAcceptID() const override;

  private:
    /// keep original position or orientation values of model-link-joint
    /// if user just setting orientation or position
    std::shared_ptr<mw::internal::robotics::gazebotransport::ML_Pose> retainPoseComponents(
        ignition::math::Pose3d gazebo_pose,
        mw::internal::robotics::gazebotransport::ML_Pose const& poseMsg);

    // return joint type name is string form
    std::string GetJointTypeName(gazebo::msgs::Joint::Type jointType);

    /// Gazebo world pointer
    gazebo::physics::WorldPtr m_ptr;

    /// Gazebo MATLAB Interface Utils object
    std::shared_ptr<robotics::gazebotransport::GazeboMLUtils> m_utils;
};
} // namespace gazebotransport
} // namespace robotics
#endif
