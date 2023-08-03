/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_GAZEBOPLUGIN_HPP
#define GAZEBOCOSIM_GAZEBOPLUGIN_HPP

#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/gazeboserver/gazeboapply/GazeboApplyCommander.hpp"
#include "gazebotransport/gazeboserver/GazeboWorldInterface.hpp"

/*
This file belongs to GazeboPlugin Layer of GazeboCoSim
This is World plugin of gazebo which interact with Gazebo environment.
When plugin loads, the Gazebo world pointer, nodes gets initialize.
Further, ServerRun thread started which initializes server connection,
initiates all message handlers and accept client connection.
In addition, gazebo events pauses simulation at start and waits
to trigger from client end.
*/

namespace robotics {
namespace gazebotransport {

class CustomMsgDispatcher;

class GazeboPlugin : public gazebo::WorldPlugin {
  private:
    /// Custom message dispatcher instance
    std::shared_ptr<robotics::gazebotransport::CustomMsgDispatcher> m_customDispatch;

    /// Commander that applies Gazebo force/torques to model links and joints
    robotics::gazebotransport::GazeboApplyCommander m_applyCommander;

    /// Gazebo world pointer
    gazebo::physics::WorldPtr m_world;
    /// Gazebo transport node pointer
    gazebo::transport::NodePtr m_node;

    /// Gazebo server object
    std::shared_ptr<robotics::gazebotransport::GazeboServer> m_server;
    std::shared_ptr<GazeboWorldInterface> m_worldInterface;

    /// Gazebo connects a callback to the world update start signal
    gazebo::event::ConnectionPtr m_worldUpdateStartEventConnection;

    /// Gazebo connects a callback to the world update end signal
    gazebo::event::ConnectionPtr m_worldUpdateEndEventConnection;

    /// Gazebo connects a callback to the time reset signal
    gazebo::event::ConnectionPtr m_timeResetEventConnection;

    /// port number
    uint16_t m_portNumber;

    /// Registers message handlers
    void registerMsgHandler();

    /// Registers custom message handlers
    void registerCustomMsgHandler();

    /// Gazebo function called at plugin loading
    void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    /// Called to monitor end of the world update
    /// The signal is called at the end of Simulation update
    void onWorldUpdateEnd();

    /// Called to monitor start of the world update
    /// The signal is called at the start of Simulation update
    void onWorldUpdateStart();

    /// Called to monitor time reset
    void onTimeReset();
};
} // namespace gazebotransport
} // namespace robotics
#endif
