/* Copyright 2019-2022 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebosensors/GazeboIMUSensor.hpp"
#include "gazebo/sensors/SensorsIface.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/transport/Node.hh"

#include <thread>
#include <chrono>

namespace robotics { namespace gazebotransport {
GazeboIMUSensor::GazeboIMUSensor(gazebo::physics::WorldPtr world, std::string const& topic)
    : GazeboSensorSubscriber(world, topic)
    , m_subscriber(nullptr) {}

void GazeboIMUSensor::initImpl(gazebo::transport::NodePtr node) {
    // connect subscriber callback
    if (!m_subscriber) {
        m_subscriber = node->Subscribe(this->getTopicName(), &GazeboIMUSensor::updateMessage, this);
    }
}

void GazeboIMUSensor::updateMessage(ConstIMUPtr& msg) {
    // update the message buffer with each laser callback
    std::lock_guard<std::mutex> lock(m_mutex);

    // update the update time
    m_lastUpdateTime = ::gazebo::msgs::Convert(msg->stamp());
    m_lastMessage.mutable_header()->mutable_time_stamp()->set_seconds(
        m_lastUpdateTime.sec > 0 ? static_cast<uint64_t>(m_lastUpdateTime.sec) : 0);
    m_lastMessage.mutable_header()->mutable_time_stamp()->set_nano_seconds(
        m_lastUpdateTime.nsec > 0 ? static_cast<uint64_t>(m_lastUpdateTime.nsec) : 0);

    // update the packet body
    // Retrieve gazebo imu message ( angular velocity, linear acceleration and orientation )
    const ::gazebo::msgs::Vector3d& angularVelocity = msg->angular_velocity();
    const ::gazebo::msgs::Vector3d& linearAcceleration = msg->linear_acceleration();
    const ::gazebo::msgs::Quaternion& orientation = msg->orientation();
    m_lastMessage.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_IMU_DATA);
    m_lastMessage.set_is_new_reading(true);
    m_lastMessage.mutable_imu_data()->mutable_linear_acceleration()->set_x(linearAcceleration.x());
    m_lastMessage.mutable_imu_data()->mutable_linear_acceleration()->set_y(linearAcceleration.y());
    m_lastMessage.mutable_imu_data()->mutable_linear_acceleration()->set_z(linearAcceleration.z());
    m_lastMessage.mutable_imu_data()->mutable_angular_velocity()->set_x(angularVelocity.x());
    m_lastMessage.mutable_imu_data()->mutable_angular_velocity()->set_y(angularVelocity.y());
    m_lastMessage.mutable_imu_data()->mutable_angular_velocity()->set_z(angularVelocity.z());
    m_lastMessage.mutable_imu_data()->mutable_orientation()->set_x(orientation.x());
    m_lastMessage.mutable_imu_data()->mutable_orientation()->set_y(orientation.y());
    m_lastMessage.mutable_imu_data()->mutable_orientation()->set_z(orientation.z());
    m_lastMessage.mutable_imu_data()->mutable_orientation()->set_w(orientation.w());

    // formulate empty message
    m_emptyMessage.mutable_header()->mutable_time_stamp()->set_seconds(
        m_lastUpdateTime.sec > 0 ? static_cast<uint64_t>(m_lastUpdateTime.sec) : 0);
    m_emptyMessage.mutable_header()->mutable_time_stamp()->set_nano_seconds(
        m_lastUpdateTime.nsec > 0 ? static_cast<uint64_t>(m_lastUpdateTime.nsec) : 0);
    m_emptyMessage.set_is_new_reading(false);
    m_emptyMessage.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_IMU_DATA);
}

}} // namespace robotics::gazebotransport
