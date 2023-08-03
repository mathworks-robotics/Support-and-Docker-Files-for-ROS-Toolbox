/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBO_CAMERA_SENSOR_HPP
#define GAZEBO_CAMERA_SENSOR_HPP

#include "gazebotransport/gazeboserver/gazebosensors/GazeboSensorSubscriber.hpp"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"

namespace robotics {
namespace gazebotransport {
class GazeboCameraSensor : public GazeboSensorSubscriber {
  public:
    /// constructor
    GazeboCameraSensor(gazebo::physics::WorldPtr world, std::string const& topic);

  private:
    /// initialize
    void initImpl(gazebo::transport::NodePtr node) override;

    /// update message at each new laser message
    void updateMessage(ConstImageStampedPtr& msg);

    /// Gazebo transport subscriber
    ::gazebo::transport::SubscriberPtr m_subscriber;

    /// conversion table between numeric enums and strings
    const std::string m_msgImgFormatTable[19] = {"UNKNOWN_PIXEL_FORMAT",
                                                 "L_INT8",
                                                 "L_INT16",
                                                 "RGB_INT8",
                                                 "RGBA_INT8",
                                                 "BGRA_INT8",
                                                 "RGB_INT16",
                                                 "RGB_INT32",
                                                 "BGR_INT8",
                                                 "BGR_INT16",
                                                 "BGR_INT32",
                                                 "R_FLOAT16",
                                                 "RGB_FLOAT16",
                                                 "R_FLOAT32",
                                                 "RGB_FLOAT32",
                                                 "BAYER_RGGB8",
                                                 "BAYER_RGGR8",
                                                 "BAYER_GBRG8",
                                                 "BAYER_GRBG8"};
};
} // namespace gazebotransport
} // namespace robotics

#endif
