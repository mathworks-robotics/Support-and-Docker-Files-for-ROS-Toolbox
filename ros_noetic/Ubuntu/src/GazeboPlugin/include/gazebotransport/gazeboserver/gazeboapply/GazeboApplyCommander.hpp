/* Copyright 2019-2020 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_APPLY_COMMANDER_HPP
#define GAZEBOCOSIM_APPLY_COMMANDER_HPP

#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/JointPtrStorage.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/LinkPtrStorage.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"
#include <gazebo/physics/physics.hh>

namespace robotics {
namespace gazebotransport {
/*
GazeboApplyCommander keeps track of active Set Link, Add Link, Set Joint commands

Add Link commands are accumulative. Each force/torque command is going to be active
for the user specified duration. If user keep sending ADD command at each time step,
with overlapping duration, the force/torque would be added during the overlapping time.

Set Link commands ignore all the existing ADD command and SET the force for its duration.
If user keep sending SET command at each time step, with overlapping duration, the more
recent SET command would overwrite the old SET command. If user sends multiple SET command
at the same simulation time step, only the last SET command is going to be observed.

Set Joint commands behaves similarly to the Set Link commands, newer commands on the same joint
axis overwrites previous ones

SetJointPosition or SetJointVelocity SET Position or Velocity on specified joint axis (index).
It is active for user specified duration. If user keep sending SET command at each time step,
with overlapping duration, more recent SET Position/Velocity command would overwrite the old
SET command

SetLinkWorldPose or SetLinkLinearVelocity or SetLinkAngularVelocity SET world pose or
linear velocity or angular velocity on specified link of a model. It is active for user
specified duration. If user keep sending SET command at each time step, with overlapping
duration, more recent SET command would overwrite the old SET command

*/
class GazeboApplyCommander {
  public:
    /// constructor
    GazeboApplyCommander();

    /// insert joint commands, latter overwrites earlier commands
    void insertJointCommand(
        std::string const& jointName,
        std::shared_ptr<robotics::gazebotransport::JointPtrStorage> jointCommand);

    /// insert set joint position commands, latter overwrites earlier commands
    void insertJointSetPositionCommand(
        std::string const& jointName,
        std::shared_ptr<robotics::gazebotransport::JointPtrStorage> jointCommand);

    /// insert set joint velocity commands, latter overwrites earlier commands
    void insertJointSetVelocityCommand(
        std::string const& jointName,
        std::shared_ptr<robotics::gazebotransport::JointPtrStorage> jointCommand);

    /// insert "set link force/torque" command
    void insertSetLinkCommand(
        std::string const& linkName,
        std::string const& cmdType,
        std::shared_ptr<robotics::gazebotransport::LinkPtrStorage> linkCommand);

    /// insert "add link force/torque" command
    void insertAddLinkCommand(
        std::string const& cmdType,
        std::shared_ptr<robotics::gazebotransport::LinkPtrStorage> linkCommand);

    /// insert "set link world pose" command
    void insertSetLinkWorldPoseCommand(
        std::string const& linkName,
        std::shared_ptr<robotics::gazebotransport::LinkPtrStorage> linkCommand);

    /// insert "set link linear velocity" command
    void insertSetLinkLinearVelocityCommand(
        std::string const& linkName,
        std::shared_ptr<robotics::gazebotransport::LinkPtrStorage> linkCommand);

    /// insert "set link angular velocity" command
    void insertSetLinkAngularVelocityCommand(
        std::string const& linkName,
        std::shared_ptr<robotics::gazebotransport::LinkPtrStorage> linkCommand);

    /// execute all apply commands
    void executeApplyCommands(gazebo::common::Time const& currentTime);

    /// clear all queued apply commands
    void clearApplyCommands();

  private:
    /// Mutex for sensor message map
    std::mutex m_mutex;

    /// stores joint name-axis pair and its joint storage pointer object
    std::map<std::pair<std::string, uint32_t>,
             std::shared_ptr<robotics::gazebotransport::JointPtrStorage>>
        m_jointCommands;

    /// stores joint name-axis pair and its joint storage pointer object
    std::map<std::pair<std::string, uint32_t>,
             std::shared_ptr<robotics::gazebotransport::JointPtrStorage>>
        m_jointSetPositionCommands;

    /// stores joint name-axis pair and its joint storage pointer object
    std::map<std::pair<std::string, uint32_t>,
             std::shared_ptr<robotics::gazebotransport::JointPtrStorage>>
        m_jointSetVelocityCommands;

    /// stores link name and its link storage pointer object for SET force
    std::map<std::string, std::shared_ptr<robotics::gazebotransport::LinkPtrStorage>>
        m_linkSetForceCommand;

    /// stores link name and its link storage pointer object for SET Torque
    std::map<std::string, std::shared_ptr<robotics::gazebotransport::LinkPtrStorage>>
        m_linkSetTorqueCommand;

    /// stores link name and its link storage pointer object for ADD force
    std::vector<std::shared_ptr<robotics::gazebotransport::LinkPtrStorage>> m_linkAddForceCommand;

    /// stores link name and its link storage pointer object for ADD torque
    std::vector<std::shared_ptr<robotics::gazebotransport::LinkPtrStorage>> m_linkAddTorqueCommand;

    /// stores link name and its link storage pointer object for SET world pose
    std::map<std::string, std::shared_ptr<robotics::gazebotransport::LinkPtrStorage>>
        m_linkSetWorldPose;

    /// stores link name and its link storage pointer object for SET linear velocity
    std::map<std::string, std::shared_ptr<robotics::gazebotransport::LinkPtrStorage>>
        m_linkSetLinearVelocity;

    /// stores link name and its link storage pointer object for SET angular velocity
    std::map<std::string, std::shared_ptr<robotics::gazebotransport::LinkPtrStorage>>
        m_linkSetAngularVelocity;
};
} // namespace gazebotransport
} // namespace robotics
#endif
