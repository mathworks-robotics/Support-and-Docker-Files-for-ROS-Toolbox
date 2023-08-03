/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebosensors/SensorContainer.hpp"

namespace robotics {
namespace gazebotransport {
SensorContainer::SensorContainer()
    : m_sensors()
    , m_mutex() {
}

std::pair<std::shared_ptr<GazeboSensorSubscriber>, bool> SensorContainer::insert(
    std::shared_ptr<GazeboSensorSubscriber> sensor) {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto insertResult = m_sensors.insert(std::make_pair(sensor->getTopicName(), sensor));
    return std::make_pair(insertResult.first->second, insertResult.second);
}

std::shared_ptr<GazeboSensorSubscriber> SensorContainer::find(std::string const& topic) {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto iter = m_sensors.find(topic);
    if (iter != m_sensors.end()) {
        return iter->second;
    } else {
        return nullptr;
    }
}

} // namespace gazebotransport
} // namespace robotics
