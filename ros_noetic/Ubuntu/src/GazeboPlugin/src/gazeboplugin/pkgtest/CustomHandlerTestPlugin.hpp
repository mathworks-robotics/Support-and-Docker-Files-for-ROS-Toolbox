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
#include "mw.internal.robotics.gazebotransport.TestMsgs.pb.h"
#include "image_mat.pb.h"
#include "imu_mat.pb.h"

using namespace mw::internal::robotics::gazebotransport;

namespace gazebo {

typedef const boost::shared_ptr<const TestPose> TestPosePtr;
typedef const boost::shared_ptr<const robotics::matlab::ImageMat> ImageMatPtr;
typedef const boost::shared_ptr<const robotics::matlab::IMUMat> IMUMatPtr;

class CustomHandlerTestPlugin : public WorldPlugin {
  private:
    gazebo::transport::NodePtr m_node;

    gazebo::transport::SubscriberPtr customSubsciber0, customSubsciber1, customSubsciber2,
        customSubsciber3;
    gazebo::transport::PublisherPtr customPublisher0, customPublisher1, customPublisher2,
        customPublisher3;

    physics::WorldPtr world;
    gazebo::event::ConnectionPtr worldUpdateStartEventConnection;
    gazebo::event::ConnectionPtr m_worldUpdateEndEventConnection;

    /// Gazebo connects a callback to the time reset signal
    gazebo::event::ConnectionPtr m_timeResetEventConnection;

    std::mutex m_mutex;

    double itrNum0 = 0.0;
    double itrNum1 = 0.0;

    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    void onWorldUpdateStart();

    void onWorldUpdateEnd();

    void subscriberPose0(TestPosePtr& msg);

    void subscriberPose1(TestPosePtr& msg);

    void subscriberImageMat(ImageMatPtr& msg);

    void subscriberImuMat(IMUMatPtr& msg);

    void onTimeReset();
};

} // namespace gazebo
