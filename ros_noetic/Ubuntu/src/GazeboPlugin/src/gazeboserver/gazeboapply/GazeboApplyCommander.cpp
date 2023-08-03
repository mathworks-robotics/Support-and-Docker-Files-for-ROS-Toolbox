/* Copyright 2019-2020 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazeboapply/GazeboApplyCommander.hpp"

namespace robotics {
namespace gazebotransport {

GazeboApplyCommander::GazeboApplyCommander()
    : m_mutex()
    , m_jointCommands()
    , m_jointSetPositionCommands()
    , m_jointSetVelocityCommands()
    , m_linkSetForceCommand()
    , m_linkSetTorqueCommand()
    , m_linkAddForceCommand()
    , m_linkAddTorqueCommand()
    , m_linkSetWorldPose()
    , m_linkSetLinearVelocity()
    , m_linkSetAngularVelocity() {
}

/// insert joint commands, latter overwrites earlier commands
void GazeboApplyCommander::insertJointCommand(
    std::string const& jointName,
    std::shared_ptr<robotics::gazebotransport::JointPtrStorage> jointCommand) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_jointCommands[std::make_pair(
        jointName, jointCommand->m_msgContent.apply_joint_torque().index())] = jointCommand;
}

/// insert joint commands for set joint position, latter overwrites earlier commands
void GazeboApplyCommander::insertJointSetPositionCommand(
    std::string const& jointName,
    std::shared_ptr<robotics::gazebotransport::JointPtrStorage> jointCommand) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_jointSetPositionCommands[std::make_pair(
        jointName, jointCommand->m_msgContent.set_joint_position().index())] = jointCommand;
}

/// insert joint commands for set joint velocity, latter overwrites earlier commands
void GazeboApplyCommander::insertJointSetVelocityCommand(
    std::string const& jointName,
    std::shared_ptr<robotics::gazebotransport::JointPtrStorage> jointCommand) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_jointSetVelocityCommands[std::make_pair(
        jointName, jointCommand->m_msgContent.set_joint_velocity().index())] = jointCommand;
}

/// insert set link force/torque command
void GazeboApplyCommander::insertSetLinkCommand(
    std::string const& linkName,
    std::string const& cmdType,
    std::shared_ptr<robotics::gazebotransport::LinkPtrStorage> linkCommand) {
    if (cmdType == "FORCE") {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_linkSetForceCommand[linkName] = linkCommand;
    } else if (cmdType == "TORQUE") {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_linkSetTorqueCommand[linkName] = linkCommand;
    }
}

/// insert add link force/torque command
void GazeboApplyCommander::insertAddLinkCommand(
    std::string const& cmdType,
    std::shared_ptr<robotics::gazebotransport::LinkPtrStorage> linkCommand) {
    if (cmdType == "FORCE") {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_linkAddForceCommand.push_back(linkCommand);
    } else if (cmdType == "TORQUE") {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_linkAddTorqueCommand.push_back(linkCommand);
    }
}

/// insert set link world pose command
void GazeboApplyCommander::insertSetLinkWorldPoseCommand(
    std::string const& linkName,
    std::shared_ptr<robotics::gazebotransport::LinkPtrStorage> linkCommand) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_linkSetWorldPose[linkName] = linkCommand;
}

/// insert set link linear velocity command
void GazeboApplyCommander::insertSetLinkLinearVelocityCommand(
    std::string const& linkName,
    std::shared_ptr<robotics::gazebotransport::LinkPtrStorage> linkCommand) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_linkSetLinearVelocity[linkName] = linkCommand;
}

/// insert set link angular velocity command
void GazeboApplyCommander::insertSetLinkAngularVelocityCommand(
    std::string const& linkName,
    std::shared_ptr<robotics::gazebotransport::LinkPtrStorage> linkCommand) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_linkSetAngularVelocity[linkName] = linkCommand;
}


/// execute all apply commands
void GazeboApplyCommander::executeApplyCommands(gazebo::common::Time const& currentTime) {
    std::lock_guard<std::mutex> lock(m_mutex);

    /// Checks stored link pointer for ADD force
    {
        auto iter = m_linkAddForceCommand.begin();
        while (iter != m_linkAddForceCommand.end()) {
            if (currentTime <= (*iter)->m_endTime) {
                // Add force if the request is still valid
                ignition::math::Vector3d forceVal(
                    1.0 * (*iter)->m_msgContent.apply_link_wrench().fx(),
                    1.0 * (*iter)->m_msgContent.apply_link_wrench().fy(),
                    1.0 * (*iter)->m_msgContent.apply_link_wrench().fz());
                (*iter)->m_linkPtr->AddForce(forceVal);
                ++iter;
            } else {
                // Remove if the request is out-of-date
                iter = m_linkAddForceCommand.erase(iter);
            }
        }
    }

    /// Checks stored link pointer for ADD torque
    {
        auto iter = m_linkAddTorqueCommand.begin();
        while (iter != m_linkAddTorqueCommand.end()) {
            if (currentTime <= (*iter)->m_endTime) {
                // Add torque if the request is still valid
                ignition::math::Vector3d torqueVal(
                    1.0 * (*iter)->m_msgContent.apply_link_wrench().tx(),
                    1.0 * (*iter)->m_msgContent.apply_link_wrench().ty(),
                    1.0 * (*iter)->m_msgContent.apply_link_wrench().tz());
                (*iter)->m_linkPtr->AddTorque(torqueVal);
                ++iter;
            } else {
                // Remove if the request is out-of-date
                iter = m_linkAddTorqueCommand.erase(iter);
            }
        }
    }


    /// Checks stored link pointer for SET force
    {
        auto iter = m_linkSetForceCommand.begin();
        while (iter != m_linkSetForceCommand.end()) {
            if (currentTime <= iter->second->m_endTime) {
                // Set force if the request is still valid
                ignition::math::Vector3d forceVal(
                    iter->second->m_msgContent.apply_link_wrench().fx(),
                    iter->second->m_msgContent.apply_link_wrench().fy(),
                    iter->second->m_msgContent.apply_link_wrench().fz());

                iter->second->m_linkPtr->SetForce(forceVal);

                ++iter;
            } else {
                // Remove if the request is outdated
                iter = m_linkSetForceCommand.erase(iter);
            }
        }
    }

    /// Checks stored link pointer for SET torque
    {
        auto iter = m_linkSetTorqueCommand.begin();
        while (iter != m_linkSetTorqueCommand.end()) {
            if (currentTime <= iter->second->m_endTime) {
                // Set torque if the request is still valid
                ignition::math::Vector3d torqueVal(
                    iter->second->m_msgContent.apply_link_wrench().tx(),
                    iter->second->m_msgContent.apply_link_wrench().ty(),
                    iter->second->m_msgContent.apply_link_wrench().tz());

                iter->second->m_linkPtr->SetTorque(torqueVal);

                ++iter;
            } else {
                // Remove if the request is outdated
                iter = m_linkSetTorqueCommand.erase(iter);
            }
        }
    }

    {
        /// Checks stored joint pointer and set the effort
        auto iter = m_jointCommands.begin();
        while (iter != m_jointCommands.end()) {
            // if current_time less than apply end time, keep setting effort on joint
            if (currentTime <= iter->second->m_endTime) {
                iter->second->m_jointPtr->SetForce(
                    static_cast<uint32_t>(iter->second->m_msgContent.apply_joint_torque().index()),
                    iter->second->m_msgContent.apply_joint_torque().effort());
                ++iter;
            } else {
                iter = m_jointCommands.erase(iter);
            }
        }
    }

    {
        /// Checks stored joint pointer and set the position
        auto iter = m_jointSetPositionCommands.begin();
        while (iter != m_jointSetPositionCommands.end()) {
            // if current_time less than set end time, keep setting position of joint
            if (currentTime <= iter->second->m_endTime) {
                iter->second->m_jointPtr->SetPosition(
                    static_cast<uint32_t>(iter->second->m_msgContent.set_joint_position().index()),
                    iter->second->m_msgContent.set_joint_position().position());
                ++iter;
            } else {
                iter = m_jointSetPositionCommands.erase(iter);
            }
        }
    }

    {
        /// Checks stored joint pointer and set the velocity
        auto iter = m_jointSetVelocityCommands.begin();
        while (iter != m_jointSetVelocityCommands.end()) {
            // if current_time less than set end time, keep setting velocity of joint
            if (currentTime <= iter->second->m_endTime) {
                iter->second->m_jointPtr->SetVelocity(
                    static_cast<uint32_t>(iter->second->m_msgContent.set_joint_velocity().index()),
                    iter->second->m_msgContent.set_joint_velocity().velocity());
                ++iter;
            } else {
                iter = m_jointSetVelocityCommands.erase(iter);
            }
        }
    }

    {
        // Checks stored model-link pointer and set the world pose of link
        auto iter = m_linkSetWorldPose.begin();
        while (iter != m_linkSetWorldPose.end()) {
            if (currentTime <= iter->second->m_endTime) {
                // Set world pose if the request is still valid
                ignition::math::Pose3d _Pose3d(
                    iter->second->m_msgContent.set_link_world_pose().pose().position().x(),
                    iter->second->m_msgContent.set_link_world_pose().pose().position().y(),
                    iter->second->m_msgContent.set_link_world_pose().pose().position().z(),
                    iter->second->m_msgContent.set_link_world_pose().pose().orientation().w(),
                    iter->second->m_msgContent.set_link_world_pose().pose().orientation().x(),
                    iter->second->m_msgContent.set_link_world_pose().pose().orientation().y(),
                    iter->second->m_msgContent.set_link_world_pose().pose().orientation().z());

                iter->second->m_linkPtr->SetWorldPose(_Pose3d);

                ++iter;
            } else {
                // Remove if the request is outdated
                iter = m_linkSetWorldPose.erase(iter);
            }
        }
    }

    {
        // Checks stored link pointer and set the linear velocity
        auto iter = m_linkSetLinearVelocity.begin();
        while (iter != m_linkSetLinearVelocity.end()) {
            if (currentTime <= iter->second->m_endTime) {
                // Set linear velocity if the request is still valid
                ignition::math::Vector3d velocity(
                    iter->second->m_msgContent.set_link_linear_velocity().velocity().x(),
                    iter->second->m_msgContent.set_link_linear_velocity().velocity().y(),
                    iter->second->m_msgContent.set_link_linear_velocity().velocity().z());

                iter->second->m_linkPtr->SetLinearVel(velocity);

                ++iter;
            } else {
                // Remove if the request is outdated
                iter = m_linkSetLinearVelocity.erase(iter);
            }
        }
    }

    {
        // Checks stored link pointer and set the angular velocity
        auto iter = m_linkSetAngularVelocity.begin();
        while (iter != m_linkSetAngularVelocity.end()) {
            if (currentTime <= iter->second->m_endTime) {
                // Set angular velocity if the request is still valid
                ignition::math::Vector3d velocity(
                    iter->second->m_msgContent.set_link_angular_velocity().velocity().x(),
                    iter->second->m_msgContent.set_link_angular_velocity().velocity().y(),
                    iter->second->m_msgContent.set_link_angular_velocity().velocity().z());

                iter->second->m_linkPtr->SetAngularVel(velocity);

                ++iter;
            } else {
                // Remove if the request is outdated
                iter = m_linkSetAngularVelocity.erase(iter);
            }
        }
    }
}

/// clear all queued apply commands
void GazeboApplyCommander::clearApplyCommands() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_jointCommands.clear();
    m_jointSetPositionCommands.clear();
    m_jointSetVelocityCommands.clear();
    m_linkSetForceCommand.clear();
    m_linkSetTorqueCommand.clear();
    m_linkAddForceCommand.clear();
    m_linkAddTorqueCommand.clear();
    m_linkSetWorldPose.clear();
    m_linkSetLinearVelocity.clear();
    m_linkSetAngularVelocity.clear();
}
} // namespace gazebotransport
} // namespace robotics
