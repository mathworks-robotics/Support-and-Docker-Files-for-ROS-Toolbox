/* Copyright 2019-2021 The MathWorks, Inc. */

//This function is for internal use only. It may be removed in the future.
// DO NOT EDIT!

#include "gazebotransport/gazeboplugin/GazeboPlugin.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/StepSimulationMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/ResetSimulationMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SubscribeImageMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetImageMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SubscribeLaserMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetLaserMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SubscribeImuMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetImuMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetTopicListMsgHandler.hpp"
#include "gazebotransport/gazeboserver/StopCoSimulationHandler.hpp"
#include "gazebotransport/gazeboserver/RequestCoSimulationHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/ApplyLinkWrenchMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/ApplyJointTorqueMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetPoseMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetModelInfoMsgHandler.hpp"
#include "gazebotransport/gazeboserver/GazeboWorldImpl.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/MaxSimulationStepMsgHandler.hpp"
#include "gazebotransport/gazebocustom/gazebocustommsghandler/PublishCustomMsgHandler.hpp"
#include "gazebotransport/gazebocustom/gazebocustommsghandler/InitPublishCustomMsgHandler.hpp"
#include "gazebotransport/gazebocustom/gazebocustommsghandler/SubscribeCustomMsgHandler.hpp"
#include "gazebotransport/gazebocustom/gazebocustommsghandler/InitSubscribeCustomMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SetLinkWorldPoseMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SetLinkLinearVelocityMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SetLinkAngularVelocityMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SetJointPositionMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SetJointVelocityMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetJointStateMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetLinkStateMsgHandler.hpp"
#include "gazebotransport/gazebocustom/CustomMsgDispatcher.hpp"
// MATLAB Interface
#include "gazebotransport/gazeboserver/gazebomsghandler/SetGazeboModelParamMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetGazeboModelParamMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetGazeboModelSDFMsgHandler.hpp"

// Custom Msg Handler headers


/*
 * This Gazebo Plugin is creating for testing purpose.
 * This file generated on gazebogenmsg() call and
 * it is similar to GazeboPlugin.cpp
 * The objective is to test custom message handlers.
 * Here, user-defined as well as built-in custom message
 * handlers are initialized.
 */

namespace robotics {
    namespace gazebotransport {
        GZ_REGISTER_WORLD_PLUGIN(GazeboPlugin)
        
        void GazeboPlugin::registerCustomMsgHandler() {
            /// Init custom message dispatcher
            this->m_customDispatch = std::make_shared<robotics::gazebotransport::CustomMsgDispatcher>();
            
            /// Initializing custom message handlers
            
            
            // Starts InitPublish custom message handler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::PublishCustomMsgHandler>(
                    this->m_customDispatch));
            
            // Starts Publish custom message handler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::InitPublishCustomMsgHandler>(
                    this->m_customDispatch));
            
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::SubscribeCustomMsgHandler>(
                    this->m_customDispatch));
            
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::InitSubscribeCustomMsgHandler>(
                    this->m_customDispatch));
        }
        
        void GazeboPlugin::registerMsgHandler() {
            this->m_worldInterface = std::make_shared<GazeboWorldImpl>(this->m_world);
            
            // Request co-simulation handler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::RequestCoSimulationHandler>(
                    this->m_server, this->m_worldInterface));
            
            // Stop co-simulation handler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::StopCoSimulationHandler>(this->m_server));
            
            // StepSimulationMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::StepSimulationMsgHandler>(
                    this->m_worldInterface));
            
            // ResetSimulationMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::ResetSimulationMsgHandler>(
                    this->m_worldInterface));
            
            // SubscribeImageMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::SubscribeImageMsgHandler>(this->m_world,
                    this->m_node));
            
            // GetImageMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::GetImageMsgHandler>(this->m_world));
            
            // SubscribeLaserMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::SubscribeLaserMsgHandler>(this->m_world,
                    this->m_node));
            
            // GetLaserMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::GetLaserMsgHandler>(this->m_world));
            
            // SubscribeImuMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::SubscribeImuMsgHandler>(this->m_world,
                    this->m_node));
            
            // GetImuMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::GetImuMsgHandler>(this->m_world));
            
            // GetTopicListMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::GetTopicListMsgHandler>());
            
            // ApplyLinkWrenchMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::ApplyLinkWrenchMsgHandler>(
                    this->m_world, this->m_applyCommander));
            
            // ApplyJointTorqueMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::ApplyJointTorqueMsgHandler>(
                    this->m_world, this->m_applyCommander));
            
            // GetPoseMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::GetPoseMsgHandler>(this->m_world));
            
            // GetModelInfoMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::GetModelInfoMsgHandler>(this->m_world));
            
            // MaxSimulationStepMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::MaxSimulationStepMsgHandler>(this->m_world));
            
            // SetLinkWorldPoseMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::SetLinkWorldPoseMsgHandler>(
                    this->m_world, this->m_applyCommander));
            
            // SetLinkLinearVelocityMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::SetLinkLinearVelocityMsgHandler>(
                    this->m_world, this->m_applyCommander));
            
            // SetLinkAngularVelocityMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::SetLinkAngularVelocityMsgHandler>(
                    this->m_world, this->m_applyCommander));
            
            // SetJointPositionMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::SetJointPositionMsgHandler>(
                    this->m_world, this->m_applyCommander));
            
            // SetJointVelocityMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::SetJointVelocityMsgHandler>(
                    this->m_world, this->m_applyCommander));
            
            // GetJointStateMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::GetJointStateMsgHandler>(this->m_world));
            
            // GetLinkStateMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::GetLinkStateMsgHandler>(this->m_world));
            
            //  SetGazeboModelParamMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::SetGazeboModelParamMsgHandler>(this->m_world));
            
            //  GetGazeboModelParamMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::GetGazeboModelParamMsgHandler>(this->m_world));
					
			//  GetGazeboModelSDFMsgHandler
            this->m_server->registerHandler(
                    std::make_shared<robotics::gazebotransport::GetGazeboModelSDFMsgHandler>(this->m_world));		
        }
        
        void GazeboPlugin::Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
            // Access gazebo world pointer
            this->m_world = _parent;
            
            // load parameters
            if (_sdf->HasElement("portNumber")) {
                this->m_portNumber = static_cast<uint16_t>(_sdf->GetElement("portNumber")->Get<int>());
            } else {
                this->m_portNumber = 15490;
            }
            
            // Create a new transport node
            this->m_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
            
            // Initialize the node with the world name
            this->m_node->Init(_parent->Name());
            
            // Starts thread for server run
            this->m_server = std::make_shared<robotics::gazebotransport::GazeboServer>(
                    this->m_portNumber); // Server initialized
            
            registerMsgHandler();
            
            registerCustomMsgHandler();
            
            this->m_server->run();
            
            // Starts thread for simulation begin update
            this->m_worldUpdateStartEventConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
                    std::bind(&GazeboPlugin::onWorldUpdateStart, this));
            
            // Starts thread for simulation end update
            this->m_worldUpdateEndEventConnection = gazebo::event::Events::ConnectWorldUpdateEnd(
                    std::bind(&GazeboPlugin::onWorldUpdateEnd, this));
            
            // Starts listening to time reset event
            this->m_timeResetEventConnection =
                    gazebo::event::Events::ConnectTimeReset(std::bind(&GazeboPlugin::onTimeReset, this));
        }
        
        void GazeboPlugin::onWorldUpdateEnd() {
            if (this->m_server->getCoSimulationStatus().first) {
                this->m_world->SetPaused(true);
            }
        }
        
        void GazeboPlugin::onTimeReset() {
            // When Gazebo simulation time reset, clear all the queued apply commands
            this->m_applyCommander.clearApplyCommands();
        }
        
        void GazeboPlugin::onWorldUpdateStart() {
            this->m_applyCommander.executeApplyCommands(this->m_world->SimTime());
        }
    } // namespace gazebotransport
} // namespace robotics

