/* Copyright 2019-2022 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebosensors/GazeboCameraSensor.hpp"
#include "gazebo/sensors/SensorsIface.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/transport/Node.hh"

#include <thread>
#include <chrono>

namespace robotics { namespace gazebotransport {
GazeboCameraSensor::GazeboCameraSensor(gazebo::physics::WorldPtr world, std::string const& topic)
    : GazeboSensorSubscriber(world, topic)
    , m_subscriber(nullptr) {}

void GazeboCameraSensor::initImpl(gazebo::transport::NodePtr node) {
    // connect subscriber callback
    if (!m_subscriber) {
        m_subscriber =
            node->Subscribe(this->getTopicName(), &GazeboCameraSensor::updateMessage, this);
    }
}

void GazeboCameraSensor::updateMessage(ConstImageStampedPtr& msg) {
    // update the message buffer with each laser callback
    std::lock_guard<std::mutex> lock(m_mutex);

    // update the update time
    m_lastUpdateTime = ::gazebo::msgs::Convert(msg->time());
    m_lastMessage.mutable_header()->mutable_time_stamp()->set_seconds(
        m_lastUpdateTime.sec > 0 ? static_cast<uint64_t>(m_lastUpdateTime.sec) : 0);
    m_lastMessage.mutable_header()->mutable_time_stamp()->set_nano_seconds(
        m_lastUpdateTime.nsec > 0 ? static_cast<uint64_t>(m_lastUpdateTime.nsec) : 0);
    m_lastMessage.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_IMAGE);
    m_lastMessage.set_is_new_reading(true);
    // update the packet body
    m_lastMessage.mutable_image()->set_width(msg->image().width());
    m_lastMessage.mutable_image()->set_height(msg->image().height());
    m_lastMessage.mutable_image()->set_data_type(m_msgImgFormatTable[msg->image().pixel_format()]);
    m_lastMessage.mutable_image()->set_data(msg->image().data());

    // formulate empty message
    m_emptyMessage.mutable_header()->mutable_time_stamp()->set_seconds(
        m_lastUpdateTime.sec > 0 ? static_cast<uint64_t>(m_lastUpdateTime.sec) : 0);
    m_emptyMessage.mutable_header()->mutable_time_stamp()->set_nano_seconds(
        m_lastUpdateTime.nsec > 0 ? static_cast<uint64_t>(m_lastUpdateTime.nsec) : 0);
    m_emptyMessage.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_IMAGE);
    m_emptyMessage.set_is_new_reading(false);
}

}} // namespace robotics::gazebotransport
