/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBO_SENSOR_CONTAINER_HPP
#define GAZEBO_SENSOR_CONTAINER_HPP

#include "GazeboSensorSubscriber.hpp"

#include <string>
#include <map>
#include <mutex>
#include <memory>

namespace robotics {
namespace gazebotransport {
/// singleton class that holds all sensors
class SensorContainer {
  public:
    /// get singleton instance
    static SensorContainer& getInstance() {
        static SensorContainer instance{};
        return instance;
    }

    /// insert sensor if the sensor is not already in the container
    /**
     * @param sensor                        Sensor to insert into the container.
                                            Insertion will only happen if sensor for its topic is
     not already in the container
     * @return                              A pair. First points to the sensor that is stored in the
     container
     *                                      Second indicates whether insertion happened or not.
    */
    std::pair<std::shared_ptr<GazeboSensorSubscriber>, bool> insert(
        std::shared_ptr<GazeboSensorSubscriber> sensor);

    /// find sensor based on topic name
    /// @return                             The sensor that is stored in the container and matches
    /// the topic name
    ///                                     Return nullptr if no match is found
    std::shared_ptr<GazeboSensorSubscriber> find(std::string const& topic);

  private:
    /// constructor set to private access
    SensorContainer();

    /// container for all the sensors
    std::map<std::string, std::shared_ptr<GazeboSensorSubscriber>> m_sensors;

    /// guard multi-thread access
    std::mutex m_mutex;
};

} // namespace gazebotransport
} // namespace robotics

#endif
