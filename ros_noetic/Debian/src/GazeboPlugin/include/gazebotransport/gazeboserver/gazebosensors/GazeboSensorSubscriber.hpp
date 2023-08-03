/* Copyright 2019-2022 The MathWorks, Inc. */
#ifndef GAZEBO_SENSOR_SUBSCRIBER_HPP
#define GAZEBO_SENSOR_SUBSCRIBER_HPP

#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/physics/PhysicsTypes.hh"

#include <string>

namespace robotics { namespace gazebotransport {
/// GazeboSensorSubscriber interface class for Gazebo sensor subscribers
class GazeboSensorSubscriber {
  public:
    /// constructor
    GazeboSensorSubscriber(gazebo::physics::WorldPtr world, std::string const& topic);

    /// destructor
    virtual ~GazeboSensorSubscriber();

    /// initialize the sensor subscriber to ignition transport layer
    void init(gazebo::transport::NodePtr node);

    /// get latest message from Gazebo
    /**
     * If the current message stored in this object is older than the expected
     * sensor reading time stamp. This method waits for at most 500 ms to retrieve
     * latest sensor message from the Gazebo Sensor and Transport layer.
     */
    mw::internal::robotics::gazebotransport::Packet getLatestMessage();

    /// get the ignition topic name
    std::string getTopicName() const;

    /// get the scoped sensor name based on topic name
    std::string getSensorName() const;

  private:
    virtual void initImpl(gazebo::transport::NodePtr node) = 0;

    void resetMessageBuffer();

    /// stores the world pointer
    gazebo::physics::WorldPtr m_world;

    /// stores the ignition topic name
    std::string m_topicName;

    /// stores the fully scoped sensor name based on topic
    std::string m_sensorName;

    /// stores the sensor type
    std::string m_sensorType;

    /// stores camera update interval
    double m_updateInterval;

  protected:
    /// mutex that guards the internal states of this class
    std::mutex m_mutex;

    /// stores last time the message buffer is updated
    ::gazebo::common::Time m_lastUpdateTime;

    /// stores last sensor update time the message buffer is sent
    ::gazebo::common::Time m_lastSendSensorTime;

    /// stores last simulation time the message buffer is sent
    ::gazebo::common::Time m_lastSendSimTime;

    /// stores the transport message to send back to Simulink
    mw::internal::robotics::gazebotransport::Packet m_lastMessage;

    /// stores the empty transport message to send back to Simulink,
    /// when sensor message is older
    mw::internal::robotics::gazebotransport::Packet m_emptyMessage;
};
}} // namespace robotics::gazebotransport

#endif
