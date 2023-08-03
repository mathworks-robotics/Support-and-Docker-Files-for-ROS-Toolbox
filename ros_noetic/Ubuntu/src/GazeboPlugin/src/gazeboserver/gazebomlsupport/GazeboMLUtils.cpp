/* Copyright 2020 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomlsupport/GazeboMLUtils.hpp"

namespace robotics {
namespace gazebotransport {

GazeboMLUtils::GazeboMLUtils() {
}
GazeboMLUtils::~GazeboMLUtils() {
}

ignition::math::Vector3d GazeboMLUtils::ConvertIgn(
    const mw::internal::robotics::gazebotransport::ML_Point& msg) {
    return ignition::math::Vector3d(msg.x(), msg.y(), msg.z());
}

ignition::math::Vector2d GazeboMLUtils::ConvertIgn(
    const mw::internal::robotics::gazebotransport::ML_Cord& msg) {
    return ignition::math::Vector2d(msg.x(), msg.y());
}

ignition::math::Quaterniond GazeboMLUtils::ConvertIgn(
    const mw::internal::robotics::gazebotransport::ML_Quat& msg) {
    return ignition::math::Quaterniond(msg.w(), msg.x(), msg.y(), msg.z());
}

ignition::math::Pose3d GazeboMLUtils::ConvertIgn(
    const mw::internal::robotics::gazebotransport::ML_Pose& msg) {
    return ignition::math::Pose3d(
        ignition::math::Vector3d(msg.position().x(), msg.position().y(), msg.position().z()),
        ignition::math::Quaterniond(msg.orientation().w(), msg.orientation().x(),
                                    msg.orientation().y(), msg.orientation().z()));
}

void GazeboMLUtils::ConvertMLPose(mw::internal::robotics::gazebotransport::ML_Point* ml_point,
                                  const ignition::math::Vector3d& data) {
    ml_point->set_x(data[0]);
    ml_point->set_y(data[1]);
    ml_point->set_z(data[2]);
}

void GazeboMLUtils::ConvertMLPose(mw::internal::robotics::gazebotransport::ML_Quat* ml_quat,
                                  const ignition::math::Quaterniond& data) {
    ml_quat->set_w(data.W());
    ml_quat->set_x(data.X());
    ml_quat->set_y(data.Y());
    ml_quat->set_z(data.Z());
}

void GazeboMLUtils::ConvertMLPose(mw::internal::robotics::gazebotransport::ML_Pose* ml_pose,
                                  const ignition::math::Pose3d& data) {
    ml_pose->mutable_position()->set_x(data.Pos()[0]);
    ml_pose->mutable_position()->set_y(data.Pos()[1]);
    ml_pose->mutable_position()->set_z(data.Pos()[2]);

    ml_pose->mutable_orientation()->set_w(data.Rot().W());
    ml_pose->mutable_orientation()->set_x(data.Rot().X());
    ml_pose->mutable_orientation()->set_y(data.Rot().Y());
    ml_pose->mutable_orientation()->set_z(data.Rot().Z());
}

} // namespace gazebotransport
} // namespace robotics
