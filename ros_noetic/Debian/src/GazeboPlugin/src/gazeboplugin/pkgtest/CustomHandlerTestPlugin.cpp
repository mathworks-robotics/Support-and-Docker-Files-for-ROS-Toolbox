/* Copyright 2019-2020 The MathWorks, Inc. */
#include <iostream>
#include <math.h>
#include <sdf/sdf.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "CustomHandlerTestPlugin.hpp"

using namespace mw::internal::robotics::gazebotransport;

/*
This is similar to custom/user-defined plugin.
This custom plugin used to test custom message functionalities.
Here, custom message published/subscribe on/from Gazebo
*/

namespace gazebo {
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(CustomHandlerTestPlugin)

void CustomHandlerTestPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {

    this->m_node = transport::NodePtr(new transport::Node());

    this->world = _parent;

    // Initialize the node with the world name
    this->m_node->Init(this->world->Name());

    // Initialize subscribers
    this->customSubsciber0 = this->m_node->Subscribe(
        "~/test_pose_subscriber0", &CustomHandlerTestPlugin::subscriberPose0, this);
    this->customSubsciber1 = this->m_node->Subscribe(
        "~/test_pose_subscriber1", &CustomHandlerTestPlugin::subscriberPose1, this);
    this->customSubsciber2 = this->m_node->Subscribe(
        "~/custom_image_subscribe", &CustomHandlerTestPlugin::subscriberImageMat, this);
    this->customSubsciber3 = this->m_node->Subscribe(
        "~/custom_imu_subscribe", &CustomHandlerTestPlugin::subscriberImuMat, this);

    // Initialize publishers
    this->customPublisher0 = this->m_node->Advertise<TestPose>("~/test_pose_publisher0");
    this->customPublisher1 = this->m_node->Advertise<TestPose>("~/test_pose_publisher1");
    this->customPublisher2 =
        this->m_node->Advertise<robotics::matlab::ImageMat>("/gazebo/default/custom_image_publish");
    this->customPublisher3 =
        this->m_node->Advertise<robotics::matlab::IMUMat>("/gazebo/default/custom_imu_publish");

    this->worldUpdateStartEventConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&CustomHandlerTestPlugin::onWorldUpdateStart, this));

    // Starts listening to time reset event
    this->m_timeResetEventConnection = gazebo::event::Events::ConnectTimeReset(
        std::bind(&CustomHandlerTestPlugin::onTimeReset, this));

    this->m_worldUpdateEndEventConnection = gazebo::event::Events::ConnectWorldUpdateEnd(
        std::bind(&CustomHandlerTestPlugin::onWorldUpdateEnd, this));
}

void CustomHandlerTestPlugin::onWorldUpdateStart() {
    // Creating message which is incremented on each step
    // Further, this message is published on gazebo
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        TestPose m_message0;
        m_message0.set_x(this->itrNum0);
        m_message0.set_y(this->itrNum0);
        m_message0.set_z(this->itrNum0);
        m_message0.set_w(this->itrNum0);

        customPublisher0->Publish(m_message0);

        TestPose m_message1;
        m_message1.set_x(this->itrNum1);
        m_message1.set_y(this->itrNum1);
        m_message1.set_z(this->itrNum1);
        m_message1.set_w(this->itrNum1);

        customPublisher1->Publish(m_message1);

        itrNum0 = itrNum0 + 1.0;
        itrNum1 = itrNum1 + 1.0;

        robotics::matlab::ImageMat ground_pose0;
        ground_pose0.set_m_width(320);
        ground_pose0.set_m_height(240);
        ground_pose0.set_m_pixel_format(3);
        ground_pose0.set_m_step(960);
        ground_pose0.set_m_data("\255\255\255\255\255");
        customPublisher2->Publish(ground_pose0);

        robotics::matlab::IMUMat ground_pose1;
        ground_pose1.set_m_entity_name("IMU");
        ground_pose1.mutable_m_angular_velocity()->set_m_x(0.05);
        ground_pose1.mutable_m_angular_velocity()->set_m_y(0.01);
        ground_pose1.mutable_m_angular_velocity()->set_m_z(0.01);
        ground_pose1.mutable_m_linear_acceleration()->set_m_x(1.00);
        ground_pose1.mutable_m_linear_acceleration()->set_m_y(2.00);
        ground_pose1.mutable_m_linear_acceleration()->set_m_z(3.00);
        ground_pose1.mutable_m_orientation()->set_m_x(0.1);
        ground_pose1.mutable_m_orientation()->set_m_y(0.2);
        ground_pose1.mutable_m_orientation()->set_m_z(0.3);
        ground_pose1.mutable_m_orientation()->set_m_w(0.4);
        ground_pose1.mutable_m_stamp()->set_m_nsec(1000);
        ground_pose1.mutable_m_stamp()->set_m_sec(1);

        customPublisher3->Publish(ground_pose1);
    }
}

void CustomHandlerTestPlugin::onWorldUpdateEnd() {
}

void CustomHandlerTestPlugin::subscriberPose0(TestPosePtr& msg) {
    {
        // Receives the message published
        /*
            std::lock_guard<std::mutex> lock(m_mutex);
        std::cout << "   subscriberPose0 : " << msg->DebugString() << std::endl;
            */
    }
}

void CustomHandlerTestPlugin::subscriberPose1(TestPosePtr& msg) {
    {
        // Receives the message published
        /*
            std::lock_guard<std::mutex> lock(m_mutex);
        std::cout << "  subscriberPose1 : " << msg->DebugString() << std::endl;
            */
    }
}

void CustomHandlerTestPlugin::subscriberImageMat(ImageMatPtr& msg) {
    {
        // Receives the message published

        // std::lock_guard<std::mutex> lock(m_mutex);
        // std::cout << "  subscriberPose1 : " << msg->DebugString() << std::endl;
    }
}

void CustomHandlerTestPlugin::subscriberImuMat(IMUMatPtr& msg) {
    {
        // Receives the message published

        // std::lock_guard<std::mutex> lock(m_mutex);
        // std::cout << "  subscriberPose1 : " << msg->DebugString() << std::endl;
    }
}

void CustomHandlerTestPlugin::onTimeReset() {
    // reset the variables
    this->itrNum0 = 100.0;
    this->itrNum1 = 200.0;
}

} // namespace gazebo
