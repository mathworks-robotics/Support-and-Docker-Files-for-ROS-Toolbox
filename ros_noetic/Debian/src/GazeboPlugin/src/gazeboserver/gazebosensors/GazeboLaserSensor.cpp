/* Copyright 2019-2022 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebosensors/GazeboLaserSensor.hpp"
#include "gazebo/sensors/SensorsIface.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/transport/Node.hh"

#include <thread>
#include <chrono>

namespace robotics { namespace gazebotransport {
GazeboLaserSensor::GazeboLaserSensor(gazebo::physics::WorldPtr world, std::string const& topic)
    : GazeboSensorSubscriber(world, topic)
    , m_subscriber(nullptr) {}

void GazeboLaserSensor::initImpl(gazebo::transport::NodePtr node) {
    // connect subscriber callback
    if (!m_subscriber) {
        m_subscriber =
            node->Subscribe(this->getTopicName(), &GazeboLaserSensor::updateMessage, this);
    }
}

void GazeboLaserSensor::updateMessage(ConstLaserScanStampedPtr& msg) {
    // update the message buffer with each laser callback
    std::lock_guard<std::mutex> lock(m_mutex);
    m_lastUpdateTime = ::gazebo::msgs::Convert(msg->time());
    m_lastMessage.mutable_header()->mutable_time_stamp()->set_seconds(
        m_lastUpdateTime.sec > 0 ? static_cast<uint64_t>(m_lastUpdateTime.sec) : 0);
    m_lastMessage.mutable_header()->mutable_time_stamp()->set_nano_seconds(
        m_lastUpdateTime.nsec > 0 ? static_cast<uint64_t>(m_lastUpdateTime.nsec) : 0);
    m_lastMessage.set_is_new_reading(true);
    // fill the laser sensor readings
    m_lastMessage.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_LASER_DATA);
    m_lastMessage.mutable_laser_data()->set_angle_min(msg->scan().angle_min());
    m_lastMessage.mutable_laser_data()->set_angle_max(msg->scan().angle_max());
    m_lastMessage.mutable_laser_data()->set_angle_step(msg->scan().angle_step());
    m_lastMessage.mutable_laser_data()->set_range_min(msg->scan().range_min());
    m_lastMessage.mutable_laser_data()->set_range_max(msg->scan().range_max());
    m_lastMessage.mutable_laser_data()->set_vertical_angle_min(msg->scan().vertical_angle_min());
    m_lastMessage.mutable_laser_data()->set_vertical_angle_max(msg->scan().vertical_angle_max());

    // count field stores number of samples in horizontal and/or vertical direction.
    // 2D lidar provides either horizontal samples or vertical samples
    // 3D lidar provides horizontal as well as vertical samples
    m_lastMessage.mutable_laser_data()->set_count(msg->scan().ranges_size());

    //*** ERROR getting -nan number, which affects deserialization
    double vAngStep = msg->scan().vertical_angle_step();
    // Handling Inf number received for vertical angle step from Gazebo
    if (!std::isfinite(vAngStep)) {
        vAngStep = 0.0;
    }
    m_lastMessage.mutable_laser_data()->set_vertical_angle_step(vAngStep);

    m_lastMessage.mutable_laser_data()->clear_range();
    m_lastMessage.mutable_laser_data()->clear_intensities();

    for (int i = 0; i < static_cast<int>(msg->scan().ranges_size()); i++) {
        // Retrieve range data from gazebo Lidar data message
        double range0 = msg->scan().ranges(i);
        // Handling Inf number received for vertical angle step from Gazebo
        if (!std::isfinite(range0)) {
            range0 = 0.0;
        }
        m_lastMessage.mutable_laser_data()->add_range(range0);
    }

    for (int i = 0; i < static_cast<int>(msg->scan().intensities_size()); i++) {
        double int0 = msg->scan().intensities(i);
        m_lastMessage.mutable_laser_data()->add_intensities(int0);
    }

    // formulate empty message
    m_emptyMessage.mutable_header()->mutable_time_stamp()->set_seconds(
        m_lastUpdateTime.sec > 0 ? static_cast<uint64_t>(m_lastUpdateTime.sec) : 0);
    m_emptyMessage.mutable_header()->mutable_time_stamp()->set_nano_seconds(
        m_lastUpdateTime.nsec > 0 ? static_cast<uint64_t>(m_lastUpdateTime.nsec) : 0);
    m_emptyMessage.set_is_new_reading(false);
    m_emptyMessage.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_LASER_DATA);
}

}} // namespace robotics::gazebotransport
