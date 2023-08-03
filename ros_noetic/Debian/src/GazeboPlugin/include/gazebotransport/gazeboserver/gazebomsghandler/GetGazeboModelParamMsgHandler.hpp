/* Copyright 2020 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_GET_GAZEBO_MODEL_PARAMMSGHANDLER_HPP
#define GAZEBOCOSIM_GET_GAZEBO_MODEL_PARAMMSGHANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomlsupport/GazeboMLUtils.hpp"
#include <gazebo/physics/physics.hh>

namespace robotics {
namespace gazebotransport {
class GetGazeboModelParamMsgHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiated message handler to exchange GetGazeboModel message
    /**
     * @param ptr      			Pointer of Gazebo World Interface
     */
    explicit GetGazeboModelParamMsgHandler(gazebo::physics::WorldPtr ptr);
    /// Destructor
    ~GetGazeboModelParamMsgHandler();

    /// Get Link Parameter details and copy to Packet message
    /**
     * @param replyMsg     		Gazebo Co-Sim Packet message
     * @param link     		    Gazebo world link pointer
     */
    void GetLinkParam(mw::internal::robotics::gazebotransport::Packet& replyMsg,
                      gazebo::physics::LinkPtr link);

    /// Get Joint Parameter details and copy to Packet message
    /**
     * @param replyMsg     		Gazebo Co-Sim Packet message
     * @param joint     		Gazebo world joint pointer
     */
    void GetJointParam(mw::internal::robotics::gazebotransport::Packet& replyMsg,
                       gazebo::physics::JointPtr joint);

    /**
     * @param msgContent      Packet message
     *
     * Handles Packet message & construct new Packet message with parameter details of model.
     * Based on requested message model-link-joint name, respective parameter details are retrieved.
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
