/* Copyright 2020 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/GetJointStateMsgHandler.hpp"

namespace robotics {
namespace gazebotransport {

GetJointStateMsgHandler::GetJointStateMsgHandler(gazebo::physics::WorldPtr ptr)
    : m_ptr(ptr) {
}

GetJointStateMsgHandler::~GetJointStateMsgHandler() {
}

void GetJointStateMsgHandler::initializeRepeatedFields(
    mw::internal::robotics::gazebotransport::Packet& replyMsg) {
    replyMsg.mutable_joint_state()->add_joint_position(0);
    replyMsg.mutable_joint_state()->add_joint_velocity(0);
    auto _axis = replyMsg.mutable_joint_state()->add_axis();
    _axis->mutable_xyz()->set_x(0);
    _axis->mutable_xyz()->set_y(0);
    _axis->mutable_xyz()->set_z(0);
    _axis->set_limit_lower(0);
    _axis->set_limit_upper(0);
    _axis->set_limit_effort(0);
    _axis->set_limit_velocity(0);
    _axis->set_damping(0);
    _axis->set_friction(0);
    _axis->set_use_parent_model_frame(false);
}

std::string GetJointStateMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    mw::internal::robotics::gazebotransport::Packet replyMsg;
    replyMsg.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                          PacketHeader_MsgID_JOINT_STATE);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds((uint64_t)m_ptr->SimTime().sec);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(
        (uint64_t)m_ptr->SimTime().nsec);

    replyMsg.mutable_joint_state()->set_model_name("");
    replyMsg.mutable_joint_state()->set_joint_name("");
    replyMsg.mutable_joint_state()->set_joint_id(0);

    /// Separate model name and joint name from input message
    gazebo::physics::ModelPtr m_model = m_ptr->ModelByName(
        msgContent.get_joint_state().model_name()); // Retrieve model from Gazebo world pointer

    if (m_model) // Validate input model available
    {
        gazebo::physics::JointPtr _joint =
            m_model->GetJoint(msgContent.get_joint_state().joint_name()); // Retrieve joint

        if (_joint) {
            // get model name
            replyMsg.mutable_joint_state()->set_model_name(m_model->GetName());

            // get joint name
            replyMsg.mutable_joint_state()->set_joint_name(_joint->GetName());

            // get joint id
            replyMsg.mutable_joint_state()->set_joint_id(_joint->GetId());

            // get joint type
            mw::internal::robotics::gazebotransport::JointState_Joint_Type _type =
                mw::internal::robotics::gazebotransport::JointState_Joint_Type(
                    _joint->GetMsgType());
            replyMsg.mutable_joint_state()->set_joint_type(_type);

            // get parent details
            gazebo::physics::BasePtr _parent = _joint->GetParent();
            if (_parent) {
                replyMsg.mutable_joint_state()->set_parent_name(_parent->GetName());
                replyMsg.mutable_joint_state()->set_parent_id(_parent->GetId());
            }

            // get child details
            gazebo::physics::BasePtr _child = _joint->GetChild();
            if (_child) {
                replyMsg.mutable_joint_state()->set_child_name(_child->GetName());
                replyMsg.mutable_joint_state()->set_child_id(_child->GetId());
            }

            // get initial anchor pose specified by joint
            ignition::math::Pose3d init_arch_pose =
                _joint->InitialAnchorPose(); // Get Pose of joint ( As per see in Gazebo)
            replyMsg.mutable_joint_state()
                ->mutable_initial_anchor_pose()
                ->mutable_position()
                ->set_x(init_arch_pose.Pos()[0]);
            replyMsg.mutable_joint_state()
                ->mutable_initial_anchor_pose()
                ->mutable_position()
                ->set_y(init_arch_pose.Pos()[1]);
            replyMsg.mutable_joint_state()
                ->mutable_initial_anchor_pose()
                ->mutable_position()
                ->set_z(init_arch_pose.Pos()[2]);
            replyMsg.mutable_joint_state()
                ->mutable_initial_anchor_pose()
                ->mutable_orientation()
                ->set_w(init_arch_pose.Rot().W());
            replyMsg.mutable_joint_state()
                ->mutable_initial_anchor_pose()
                ->mutable_orientation()
                ->set_x(init_arch_pose.Rot().X());
            replyMsg.mutable_joint_state()
                ->mutable_initial_anchor_pose()
                ->mutable_orientation()
                ->set_y(init_arch_pose.Rot().Y());
            replyMsg.mutable_joint_state()
                ->mutable_initial_anchor_pose()
                ->mutable_orientation()
                ->set_z(init_arch_pose.Rot().Z());

            // get joint pose relative to world frame
            ignition::math::Pose3d world_pose = _joint->WorldPose();
            replyMsg.mutable_joint_state()->mutable_world_pose()->mutable_position()->set_x(
                world_pose.Pos()[0]);
            replyMsg.mutable_joint_state()->mutable_world_pose()->mutable_position()->set_y(
                world_pose.Pos()[1]);
            replyMsg.mutable_joint_state()->mutable_world_pose()->mutable_position()->set_z(
                world_pose.Pos()[2]);
            replyMsg.mutable_joint_state()->mutable_world_pose()->mutable_orientation()->set_w(
                world_pose.Rot().W());
            replyMsg.mutable_joint_state()->mutable_world_pose()->mutable_orientation()->set_x(
                world_pose.Rot().X());
            replyMsg.mutable_joint_state()->mutable_world_pose()->mutable_orientation()->set_y(
                world_pose.Rot().Y());
            replyMsg.mutable_joint_state()->mutable_world_pose()->mutable_orientation()->set_z(
                world_pose.Rot().Z());

            // get parent world pose
            ignition::math::Pose3d parent_world_pose = _joint->ParentWorldPose();
            replyMsg.mutable_joint_state()->mutable_parent_world_pose()->mutable_position()->set_x(
                parent_world_pose.Pos()[0]);
            replyMsg.mutable_joint_state()->mutable_parent_world_pose()->mutable_position()->set_y(
                parent_world_pose.Pos()[1]);
            replyMsg.mutable_joint_state()->mutable_parent_world_pose()->mutable_position()->set_z(
                parent_world_pose.Pos()[2]);
            replyMsg.mutable_joint_state()
                ->mutable_parent_world_pose()
                ->mutable_orientation()
                ->set_w(parent_world_pose.Rot().W());
            replyMsg.mutable_joint_state()
                ->mutable_parent_world_pose()
                ->mutable_orientation()
                ->set_x(parent_world_pose.Rot().X());
            replyMsg.mutable_joint_state()
                ->mutable_parent_world_pose()
                ->mutable_orientation()
                ->set_y(parent_world_pose.Rot().Y());
            replyMsg.mutable_joint_state()
                ->mutable_parent_world_pose()
                ->mutable_orientation()
                ->set_z(parent_world_pose.Rot().Z());

            // Gazebo 'JOINT' can have zero, one or two number of axis
            // DOF() returns number axis assigned for joint
            // For '0' axis : All elements of 'axis1' and 'axis2' will be empty
            // For '1' axis : Only 'axis1' fields will be filled
            // For '2' axis : 'axis1' and 'axis2' will be filled
            // get and add position, velocity and axis information for each joint axis
            if (_joint->DOF() == 0) {
                initializeRepeatedFields(replyMsg);
            } else {
                for (unsigned int index = 0; index < _joint->DOF(); index++) {
                    // get and add joint positions(angles)
                    replyMsg.mutable_joint_state()->add_joint_position(_joint->Position(index));

                    // get and add joint velocities
                    replyMsg.mutable_joint_state()->add_joint_velocity(_joint->GetVelocity(index));

                    // get and add 'axis' fields for each joint axis number
                    {
                        ignition::math::Vector3d axis1 = _joint->LocalAxis(index);

                        auto _axis = replyMsg.mutable_joint_state()->add_axis();

                        _axis->mutable_xyz()->set_x(axis1.X());
                        _axis->mutable_xyz()->set_y(axis1.Y());
                        _axis->mutable_xyz()->set_z(axis1.Z());

                        _axis->set_limit_lower(_joint->GetParam(
                            "lo_stop", index)); // can  use (_joint->LowerLimit(index));
                        _axis->set_limit_upper(_joint->GetParam(
                            "hi_stop", index)); // can use (_joint->UpperLimit(index));
                        _axis->set_limit_effort(_joint->GetEffortLimit(index));
                        _axis->set_limit_velocity(_joint->GetVelocityLimit(index));
                        _axis->set_damping(_joint->GetDamping(index));
                        _axis->set_friction(_joint->GetParam("friction", index));

                        ignition::math::Quaterniond jointAFO = _joint->AxisFrameOffset(index);

                        if (jointAFO.X() == 0.0 && jointAFO.Y() == 0.0 && jointAFO.Z() == 0.0 &&
                            jointAFO.W() == 1.0) {
                            _axis->set_use_parent_model_frame(false);
                        } else {
                            _axis->set_use_parent_model_frame(true);
                        }
                    }
                }
            }
        } else {
            // for invalid joint name
            initializeRepeatedFields(replyMsg);
        }
    } else {
        // for invalid model name
        initializeRepeatedFields(replyMsg);
    }

    return replyMsg.SerializeAsString(); // returns serialized Joint State message
}

uint32_t GetJointStateMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_GET_JOINT_STATE;
}
} // namespace gazebotransport
} // namespace robotics
