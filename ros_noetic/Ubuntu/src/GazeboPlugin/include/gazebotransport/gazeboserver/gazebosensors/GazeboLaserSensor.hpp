/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBO_LASER_SENSOR_HPP
#define GAZEBO_LASER_SENSOR_HPP

#include "gazebotransport/gazeboserver/gazebosensors/GazeboSensorSubscriber.hpp"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"

namespace robotics {
namespace gazebotransport {
class GazeboLaserSensor : public GazeboSensorSubscriber {
  public:
    /// constructor
    GazeboLaserSensor(gazebo::physics::WorldPtr world, std::string const& topic);

  private:
    /// initialize
    void initImpl(gazebo::transport::NodePtr node) override;

    /// update message at each new laser message
    void updateMessage(ConstLaserScanStampedPtr& _msg);

    /// Gazebo transport subscriber
    ::gazebo::transport::SubscriberPtr m_subscriber;
};
} // namespace gazebotransport
} // namespace robotics

#endif
