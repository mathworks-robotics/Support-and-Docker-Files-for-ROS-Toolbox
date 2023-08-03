/* Copyright 2020 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_GAZEBO_ML_UTILS_HPP
#define GAZEBOCOSIM_GAZEBO_ML_UTILS_HPP

#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include <gazebo/physics/physics.hh>
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

namespace robotics {
namespace gazebotransport {
class GazeboMLUtils {
  public:
    /// Constructor
    explicit GazeboMLUtils();
    /// Destructor
    ~GazeboMLUtils();

    // copy and convert ML_Point message to ignition Vector3d
    ignition::math::Vector3d ConvertIgn(
        const mw::internal::robotics::gazebotransport::ML_Point& msg);

    // copy and convert ML_Cord message to ignition Vector2d
    ignition::math::Vector2d ConvertIgn(
        const mw::internal::robotics::gazebotransport::ML_Cord& msg);

    // copy and convert ML_Quat message to ignition Quaterniond
    ignition::math::Quaterniond ConvertIgn(
        const mw::internal::robotics::gazebotransport::ML_Quat& msg);

    // copy and convert ML_Pose message to ignition Pose3d
    ignition::math::Pose3d ConvertIgn(const mw::internal::robotics::gazebotransport::ML_Pose& msg);

    // copy and convert ignition Vector3d to ML_Point message
    void ConvertMLPose(mw::internal::robotics::gazebotransport::ML_Point* ml_point,
                       const ignition::math::Vector3d& data);

    // copy and convert ignition Quaterniond to ML_Quat message
    void ConvertMLPose(mw::internal::robotics::gazebotransport::ML_Quat* ml_quat,
                       const ignition::math::Quaterniond& data);

    // copy and convert ignition Pose3d to ML_Pose message
    void ConvertMLPose(mw::internal::robotics::gazebotransport::ML_Pose* ml_pose,
                       const ignition::math::Pose3d& data);
};
} // namespace gazebotransport
} // namespace robotics
#endif
