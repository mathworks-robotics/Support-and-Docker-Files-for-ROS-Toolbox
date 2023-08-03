/* Copyright 2019-2021 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/Client.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

#include <mutex>

// used in parameterized test
enum commandSelect {
    // test command type
    SetJointPosition = 0,
    SetJointVelocity = 1,
    SetLinkWorldPose = 2,
    SetLinkLinearVelocity = 3,
    SetLinkAngularVelocity = 4,
    GetJointState = 5,
    GetLinkState = 6
};

enum errorMsgValidation {
    // reply message error type
    Success = 0,
    InvalidModel = 1,
    InvalidLink = 2,
    InvalidJoint = 3
};

class GazeboWorldStat {
  public:
    GazeboWorldStat()
        : m_mutex()
        , m_time()
        , m_worldStat(nullptr) {
    }

    void OnWorldStat(ConstWorldStatisticsPtr& msg) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_time = gazebo::msgs::Convert(msg->sim_time());
    }

    gazebo::common::Time getSimTime() {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_time;
    }

    void setSubscriber(gazebo::transport::SubscriberPtr&& sub) {
        m_worldStat = sub;
    }

  private:
    /// mutex
    std::mutex m_mutex;
    /// Simulation time holder
    gazebo::common::Time m_time;
    /// Gazebo Subscriber pointer to Subscribe world stat data
    gazebo::transport::SubscriberPtr m_worldStat;
};

class pluginTest : public ::testing::TestWithParam<
                       std::tuple<std::string, std::string, commandSelect, errorMsgValidation>> {

  public:
    std::string ipAddress = "127.0.0.1";
    std::string serverPort = "14581";

    /// Session Time-out value
    int time_out = 50000;

    /// Creating Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;

    /// Ignition transport node and subscriber to get simulation time
    gazebo::transport::NodePtr m_node;

    /// Stores world stat and simulation time from Gazebo
    GazeboWorldStat m_worldStat;

    /**
     * It creates and sends reset gazebo scene & time message and receives the reply message from
     * server. Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientResetSceneTime() {


        /// Create Packet message reset simulation time & scene message
        mw::internal::robotics::gazebotransport::Packet m_message;

        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID_RESET_SIMULATION);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_reset_simulation()->set_behavior(
            mw::internal::robotics::gazebotransport::
                ResetSimulation_ResetBehavior_RESET_TIME_AND_SCENE);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;

        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends subscribe image message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientSubscribeImage(
        std::string const& topic_name) {

        /// Create Packet message to subscribe Image
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_SUBSCRIBE_IMAGE);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_subscribe_image()->set_topic_name(topic_name);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        mw::internal::robotics::gazebotransport::Packet reply;

        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends get/request Image message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientGetImage(std::string const& topic_name) {

        /// Create Packet message to get Image
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_REQUEST_IMAGE);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_request_image()->set_topic_name(topic_name);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message

        mw::internal::robotics::gazebotransport::Packet reply;
        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends subscribe imu message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientSubscribeImu(
        std::string const& topic_name) {
        /// Create Packet message to subscribe imu
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_SUBSCRIBE_IMU);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_subscribe_imu()->set_topic_name(topic_name);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;
        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends get/request imu message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientGetImu(std::string const& topic_name) {
        /// Create Packet message to get imu
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::
                                               PacketHeader_MsgID::PacketHeader_MsgID_REQUEST_IMU);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_request_imu()->set_topic_name(topic_name);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;
        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends subscribe laser message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientSubscribeLaser(
        std::string const& topic_name) {
        /// Create Packet message to subscribe laser
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_SUBSCRIBE_LASER);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_subscribe_laser()->set_topic_name(topic_name);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet replyGetLaser;
        if (replyMsg) {
            replyGetLaser.ParseFromString(*replyMsg);
        }

        return replyGetLaser;
    }

    /**
     * It creates and sends get/request laser message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientGetLaser(std::string const& topic_name) {
        /// Create Packet message to get laser
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_REQUEST_LASER);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_request_laser()->set_topic_name(topic_name);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet replyGetLaser;
        if (replyMsg) {
            replyGetLaser.ParseFromString(*replyMsg);
        }

        return replyGetLaser;
    }

    /**
     * It creates and sends step simulation message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientStepSimulation(uint32_t stepSize) {
        /// Create Packet message step simulation message
        mw::internal::robotics::gazebotransport::Packet m_message;

        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID_STEP_SIMULATION);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_step_simulation()->set_num_steps(stepSize);
        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;

        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    mw::internal::robotics::gazebotransport::Packet clientApplyJointTorque(
        std::string const& modelName,
        std::string const& jointName,
        uint32_t indexVal,
        double effortVal,
        uint64_t sec,
        uint64_t nsec) {
        /// Create Packet message to apply torque on a joint
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_APPLY_JOINT_TORQUE);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_apply_joint_torque()->set_model_name(modelName);
        m_message.mutable_apply_joint_torque()->set_joint_name(jointName);
        m_message.mutable_apply_joint_torque()->set_index(indexVal);
        m_message.mutable_apply_joint_torque()->set_effort(effortVal);
        m_message.mutable_apply_joint_torque()->mutable_duration()->set_seconds(sec);
        m_message.mutable_apply_joint_torque()->mutable_duration()->set_nano_seconds(nsec);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;
        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends request co-simulation message and receives the reply message from
     * server. Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientRequestCoSimulation(
        std::string const& clientID) {
        /// Create Packet message request co-simulation message
        mw::internal::robotics::gazebotransport::Packet m_message;

        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID_REQUEST_COSIM);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_request_cosim()->set_client_id(clientID);
        m_message.mutable_request_cosim()->set_duration(std::numeric_limits<double>::infinity());
        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;

        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends stop co-simulation message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientStopCoSimulation(
        std::string const& clientID) {
        /// Create Packet message stop co-simulation message
        mw::internal::robotics::gazebotransport::Packet m_message;

        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID_STOP_COSIM);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_stop_cosim()->set_client_id(clientID);
        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;

        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }
    /**
     * It creates and sends apply link wrench message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientApplyLinkWrench(
        std::string const& modelName,
        std::string const& linkName,
        std::string const& forceType,
        std::string const& torqueType,
        double (&forceVal)[3],
        double (&torqueVal)[3],
        uint64_t sec,
        uint64_t nsec) {

        /// Create Packet message to apply force/torque on a link
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_APPLY_LINK_WRENCH);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_apply_link_wrench()->set_model_name(modelName);
        m_message.mutable_apply_link_wrench()->set_link_name(linkName);
        m_message.mutable_apply_link_wrench()->set_force_type(forceType);
        m_message.mutable_apply_link_wrench()->set_fx(forceVal[0]);
        m_message.mutable_apply_link_wrench()->set_fy(forceVal[1]);
        m_message.mutable_apply_link_wrench()->set_fz(forceVal[2]);
        m_message.mutable_apply_link_wrench()->set_torque_type(torqueType);
        m_message.mutable_apply_link_wrench()->set_tx(torqueVal[0]);
        m_message.mutable_apply_link_wrench()->set_ty(torqueVal[1]);
        m_message.mutable_apply_link_wrench()->set_tz(torqueVal[2]);
        m_message.mutable_apply_link_wrench()->mutable_duration()->set_seconds(sec);
        m_message.mutable_apply_link_wrench()->mutable_duration()->set_nano_seconds(nsec);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;
        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends get ground truth world pose message and receives the reply message from
     * server. Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientGetPose(std::string const& modelName,
                                                                  std::string const& linkName) {
        /// Create Packet message to get pose of a link
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_GET_GROUND_TRUTH_WORLD_POSE);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_get_ground_truth_world_pose()->set_model_name(modelName);
        m_message.mutable_get_ground_truth_world_pose()->set_link_name(linkName);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;
        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends set joint position message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientSetJointPosition(
        std::string const& modelName,
        std::string const& jointName,
        uint32_t indexVal,
        double position,
        uint64_t sec,
        uint64_t nsec) {
        /// Create Packet message to set Joint Position on a joint
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_SET_JOINT_POSITION);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_set_joint_position()->set_model_name(modelName);
        m_message.mutable_set_joint_position()->set_joint_name(jointName);
        m_message.mutable_set_joint_position()->set_index(indexVal);
        m_message.mutable_set_joint_position()->set_position(position);
        m_message.mutable_set_joint_position()->mutable_duration()->set_seconds(sec);
        m_message.mutable_set_joint_position()->mutable_duration()->set_nano_seconds(nsec);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;
        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends set joint velocity message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientSetJointVelocity(
        std::string const& modelName,
        std::string const& jointName,
        uint32_t indexVal,
        double velocity,
        uint64_t sec,
        uint64_t nsec) {
        /// Create Packet message to set Joint velocity on a joint
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_SET_JOINT_VELOCITY);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_set_joint_velocity()->set_model_name(modelName);
        m_message.mutable_set_joint_velocity()->set_joint_name(jointName);
        m_message.mutable_set_joint_velocity()->set_index(indexVal);
        m_message.mutable_set_joint_velocity()->set_velocity(velocity);
        m_message.mutable_set_joint_velocity()->mutable_duration()->set_seconds(sec);
        m_message.mutable_set_joint_velocity()->mutable_duration()->set_nano_seconds(nsec);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;
        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends set link angular velocity message and receives the reply message from
     * server. Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientSetLinkAngularVelocity(
        std::string const& modelName,
        std::string const& linkName,
        double (&velocity)[3],
        uint64_t sec,
        uint64_t nsec) {
        /// Create Packet message to set angular velocity on a link
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_SET_LINK_ANGULAR_VELOCITY);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_set_link_angular_velocity()->set_model_name(modelName);
        m_message.mutable_set_link_angular_velocity()->set_link_name(linkName);
        m_message.mutable_set_link_angular_velocity()->mutable_velocity()->set_x(velocity[0]);
        m_message.mutable_set_link_angular_velocity()->mutable_velocity()->set_y(velocity[1]);
        m_message.mutable_set_link_angular_velocity()->mutable_velocity()->set_z(velocity[2]);
        m_message.mutable_set_link_angular_velocity()->mutable_duration()->set_seconds(sec);
        m_message.mutable_set_link_angular_velocity()->mutable_duration()->set_nano_seconds(nsec);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;
        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends set link linear velocity message and receives the reply message from
     * server. Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientSetLinkLinearVelocity(
        std::string const& modelName,
        std::string const& linkName,
        double (&velocity)[3],
        uint64_t sec,
        uint64_t nsec) {
        /// Create Packet message to set linear velocity on a link
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_SET_LINK_LINEAR_VELOCITY);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_set_link_linear_velocity()->set_model_name(modelName);
        m_message.mutable_set_link_linear_velocity()->set_link_name(linkName);
        m_message.mutable_set_link_linear_velocity()->mutable_velocity()->set_x(velocity[0]);
        m_message.mutable_set_link_linear_velocity()->mutable_velocity()->set_y(velocity[1]);
        m_message.mutable_set_link_linear_velocity()->mutable_velocity()->set_z(velocity[2]);
        m_message.mutable_set_link_linear_velocity()->mutable_duration()->set_seconds(sec);
        m_message.mutable_set_link_linear_velocity()->mutable_duration()->set_nano_seconds(nsec);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;
        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends set link world pose message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientSetLinkWorldPose(
        std::string const& modelName,
        std::string const& linkName,
        double (&pose)[7],
        uint64_t sec,
        uint64_t nsec) {
        /// Create Packet message to set world pose of a link
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_SET_LINK_WORLD_POSE);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_set_link_world_pose()->set_model_name(modelName);
        m_message.mutable_set_link_world_pose()->set_link_name(linkName);
        m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_position()->set_x(pose[0]);
        m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_position()->set_y(pose[1]);
        m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_position()->set_z(pose[2]);
        m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_orientation()->set_x(
            pose[3]);
        m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_orientation()->set_y(
            pose[4]);
        m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_orientation()->set_z(
            pose[5]);
        m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_orientation()->set_w(
            pose[6]);
        m_message.mutable_set_link_world_pose()->mutable_duration()->set_seconds(sec);
        m_message.mutable_set_link_world_pose()->mutable_duration()->set_nano_seconds(nsec);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;
        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends get joint state message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientGetJointState(
        std::string const& modelName,
        std::string const& jointName) {
        /// Create Packet message to get joint state
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_GET_JOINT_STATE);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_get_joint_state()->set_model_name(modelName);
        m_message.mutable_get_joint_state()->set_joint_name(jointName);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;
        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends get link state message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientGetLinkState(
        std::string const& modelName,
        std::string const& linkName) {
        /// Create Packet message to get link state
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_GET_LINK_STATE);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_get_link_state()->set_model_name(modelName);
        m_message.mutable_get_link_state()->set_link_name(linkName);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;

        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * validate received joint state message with ground truth values
     * */
    void validateJointStateResult(mw::internal::robotics::gazebotransport::Packet reply,
                                  const char* modelName,
                                  const char* jointName,
                                  uint32_t jointId,
                                  std::vector<double> const& joint_position,
                                  std::vector<double> const& joint_velocity,
                                  int32_t joint_type,
                                  const char* parentName,
                                  uint32_t parentId,
                                  const char* childName,
                                  uint32_t childId,
                                  std::vector<double> const& initialAnchorPose,
                                  std::vector<double> const& worldPose,
                                  std::vector<double> const& parentWorldPose,
                                  std::vector<double> const& axis1,
                                  std::vector<double> const& axis2) {

        ASSERT_STREQ(reply.joint_state().model_name().c_str(), modelName);
        ASSERT_STREQ(reply.joint_state().joint_name().c_str(), jointName);
        ASSERT_EQ(reply.joint_state().joint_id(), jointId);

        for (int idx = 0; idx < reply.joint_state().joint_position_size(); idx++) {
            ASSERT_DOUBLE_EQ(reply.joint_state().joint_position(idx), joint_position[idx]);
        }

        for (int idx = 0; idx < reply.joint_state().joint_velocity_size(); idx++) {
            ASSERT_DOUBLE_EQ(reply.joint_state().joint_velocity(idx), joint_velocity[idx]);
        }

        // optional fields
        if (reply.joint_state().has_joint_type()) {
            ASSERT_EQ(reply.joint_state().joint_type(),
                      mw::internal::robotics::gazebotransport::JointState_Joint_Type(joint_type));
        }

        if (reply.joint_state().has_parent_name()) {
            ASSERT_STREQ(reply.joint_state().parent_name().c_str(), parentName);
        }

        if (reply.joint_state().has_parent_id()) {
            ASSERT_EQ(reply.joint_state().parent_id(), parentId);
        }

        if (reply.joint_state().has_child_name()) {
            ASSERT_STREQ(reply.joint_state().child_name().c_str(), childName);
        }

        if (reply.joint_state().has_child_id()) {
            ASSERT_EQ(reply.joint_state().child_id(), childId);
        }

        if (reply.joint_state().has_initial_anchor_pose()) {
            ASSERT_DOUBLE_EQ(reply.joint_state().initial_anchor_pose().position().x(),
                             initialAnchorPose[0]);
            ASSERT_DOUBLE_EQ(reply.joint_state().initial_anchor_pose().position().y(),
                             initialAnchorPose[1]);
            ASSERT_DOUBLE_EQ(reply.joint_state().initial_anchor_pose().position().z(),
                             initialAnchorPose[2]);
            ASSERT_DOUBLE_EQ(reply.joint_state().initial_anchor_pose().orientation().x(),
                             initialAnchorPose[3]);
            ASSERT_DOUBLE_EQ(reply.joint_state().initial_anchor_pose().orientation().y(),
                             initialAnchorPose[4]);
            ASSERT_DOUBLE_EQ(reply.joint_state().initial_anchor_pose().orientation().z(),
                             initialAnchorPose[5]);
            ASSERT_DOUBLE_EQ(reply.joint_state().initial_anchor_pose().orientation().w(),
                             initialAnchorPose[6]);
        }

        if (reply.joint_state().has_world_pose()) {
            ASSERT_DOUBLE_EQ(reply.joint_state().world_pose().position().x(), worldPose[0]);
            ASSERT_DOUBLE_EQ(reply.joint_state().world_pose().position().y(), worldPose[1]);
            ASSERT_DOUBLE_EQ(reply.joint_state().world_pose().position().z(), worldPose[2]);
            ASSERT_DOUBLE_EQ(reply.joint_state().world_pose().orientation().x(), worldPose[3]);
            ASSERT_DOUBLE_EQ(reply.joint_state().world_pose().orientation().y(), worldPose[4]);
            ASSERT_DOUBLE_EQ(reply.joint_state().world_pose().orientation().z(), worldPose[5]);
            ASSERT_DOUBLE_EQ(reply.joint_state().world_pose().orientation().w(), worldPose[6]);
        }

        if (reply.joint_state().has_parent_world_pose()) {
            ASSERT_DOUBLE_EQ(reply.joint_state().parent_world_pose().position().x(),
                             parentWorldPose[0]);
            ASSERT_DOUBLE_EQ(reply.joint_state().parent_world_pose().position().y(),
                             parentWorldPose[1]);
            ASSERT_DOUBLE_EQ(reply.joint_state().parent_world_pose().position().z(),
                             parentWorldPose[2]);
            ASSERT_DOUBLE_EQ(reply.joint_state().parent_world_pose().orientation().x(),
                             parentWorldPose[3]);
            ASSERT_DOUBLE_EQ(reply.joint_state().parent_world_pose().orientation().y(),
                             parentWorldPose[4]);
            ASSERT_DOUBLE_EQ(reply.joint_state().parent_world_pose().orientation().z(),
                             parentWorldPose[5]);
            ASSERT_DOUBLE_EQ(reply.joint_state().parent_world_pose().orientation().w(),
                             parentWorldPose[6]);
        }

        // currently in gazebo, joint can have max. two-axis
        for (int idx = 0; idx < reply.joint_state().axis_size(); idx++) {
            if (idx == 0) {
                ASSERT_DOUBLE_EQ(reply.joint_state().axis(idx).xyz().x(), axis1[0]);
                ASSERT_DOUBLE_EQ(reply.joint_state().axis(idx).xyz().y(), axis1[1]);
                ASSERT_DOUBLE_EQ(reply.joint_state().axis(idx).xyz().z(), axis1[2]);
                ASSERT_DOUBLE_EQ(reply.joint_state().axis(idx).limit_lower(), axis1[3]);
                ASSERT_DOUBLE_EQ(reply.joint_state().axis(idx).limit_upper(), axis1[4]);
                ASSERT_DOUBLE_EQ(reply.joint_state().axis(idx).limit_effort(), axis1[5]);
                ASSERT_DOUBLE_EQ(reply.joint_state().axis(idx).limit_velocity(), axis1[6]);
                ASSERT_DOUBLE_EQ(reply.joint_state().axis(idx).damping(), axis1[7]);
                ASSERT_DOUBLE_EQ(reply.joint_state().axis(idx).friction(), axis1[8]);
                ASSERT_EQ(reply.joint_state().axis(idx).use_parent_model_frame(),
                          static_cast<bool>(axis1[9]));
            }

            if (idx == 1) {
                ASSERT_DOUBLE_EQ(reply.joint_state().axis(idx).xyz().x(), axis2[0]);
                ASSERT_DOUBLE_EQ(reply.joint_state().axis(idx).xyz().y(), axis2[1]);
                ASSERT_DOUBLE_EQ(reply.joint_state().axis(idx).xyz().z(), axis2[2]);
                ASSERT_DOUBLE_EQ(reply.joint_state().axis(idx).limit_lower(), axis2[3]);
                ASSERT_DOUBLE_EQ(reply.joint_state().axis(idx).limit_upper(), axis2[4]);
                ASSERT_DOUBLE_EQ(reply.joint_state().axis(idx).limit_effort(), axis2[5]);
                ASSERT_DOUBLE_EQ(reply.joint_state().axis(idx).limit_velocity(), axis2[6]);
                ASSERT_DOUBLE_EQ(reply.joint_state().axis(idx).damping(), axis2[7]);
                ASSERT_DOUBLE_EQ(reply.joint_state().axis(idx).friction(), axis2[8]);
                ASSERT_EQ(reply.joint_state().axis(idx).use_parent_model_frame(),
                          static_cast<bool>(axis2[9]));
            }
        }
    }

    /**
     * validate received link state message with ground truth values
     * */
    void validateLinkStateResult(mw::internal::robotics::gazebotransport::Packet reply,
                                 const char* modelName,
                                 const char* linkName,
                                 uint32_t linkId,
                                 std::vector<double> const& worldLinearVelocity,
                                 std::vector<double> const& worldAngularVelocity,
                                 std::vector<double> const& relativeLinearVelocity,
                                 std::vector<double> const& relativeAngularVelocity,
                                 std::vector<double> const& worldPose,
                                 std::vector<double> const& relativePose,
                                 bool self_collide,
                                 bool gravity,
                                 bool kinematic,
                                 bool enable_wind,
                                 bool canonical) {

        ASSERT_STREQ(reply.link_state().model_name().c_str(), modelName);
        ASSERT_STREQ(reply.link_state().link_name().c_str(), linkName);
        ASSERT_EQ(reply.link_state().link_id(), linkId);

        ASSERT_DOUBLE_EQ(reply.link_state().world_linear_velocity().x(), worldLinearVelocity[0]);
        ASSERT_DOUBLE_EQ(reply.link_state().world_linear_velocity().y(), worldLinearVelocity[1]);
        ASSERT_DOUBLE_EQ(reply.link_state().world_linear_velocity().z(), worldLinearVelocity[2]);

        ASSERT_DOUBLE_EQ(reply.link_state().world_angular_velocity().x(), worldAngularVelocity[0]);
        ASSERT_DOUBLE_EQ(reply.link_state().world_angular_velocity().y(), worldAngularVelocity[1]);
        ASSERT_DOUBLE_EQ(reply.link_state().world_angular_velocity().z(), worldAngularVelocity[2]);

        ASSERT_DOUBLE_EQ(reply.link_state().relative_linear_velocity().x(),
                         relativeLinearVelocity[0]);
        ASSERT_DOUBLE_EQ(reply.link_state().relative_linear_velocity().y(),
                         relativeLinearVelocity[1]);
        ASSERT_DOUBLE_EQ(reply.link_state().relative_linear_velocity().z(),
                         relativeLinearVelocity[2]);

        ASSERT_DOUBLE_EQ(reply.link_state().relative_angular_velocity().x(),
                         relativeAngularVelocity[0]);
        ASSERT_DOUBLE_EQ(reply.link_state().relative_angular_velocity().y(),
                         relativeAngularVelocity[1]);
        ASSERT_DOUBLE_EQ(reply.link_state().relative_angular_velocity().z(),
                         relativeAngularVelocity[2]);

        if (reply.link_state().has_world_pose()) {
            ASSERT_DOUBLE_EQ(reply.link_state().world_pose().position().x(), worldPose[0]);
            ASSERT_DOUBLE_EQ(reply.link_state().world_pose().position().y(), worldPose[1]);
            ASSERT_DOUBLE_EQ(reply.link_state().world_pose().position().z(), worldPose[2]);
            ASSERT_DOUBLE_EQ(reply.link_state().world_pose().orientation().x(), worldPose[3]);
            ASSERT_DOUBLE_EQ(reply.link_state().world_pose().orientation().y(), worldPose[4]);
            ASSERT_DOUBLE_EQ(reply.link_state().world_pose().orientation().z(), worldPose[5]);
            ASSERT_DOUBLE_EQ(reply.link_state().world_pose().orientation().w(), worldPose[6]);
        }

        if (reply.link_state().has_relative_pose()) {
            ASSERT_DOUBLE_EQ(reply.link_state().relative_pose().position().x(), relativePose[0]);
            ASSERT_DOUBLE_EQ(reply.link_state().relative_pose().position().y(), relativePose[1]);
            ASSERT_DOUBLE_EQ(reply.link_state().relative_pose().position().z(), relativePose[2]);
            ASSERT_DOUBLE_EQ(reply.link_state().relative_pose().orientation().x(), relativePose[3]);
            ASSERT_DOUBLE_EQ(reply.link_state().relative_pose().orientation().y(), relativePose[4]);
            ASSERT_DOUBLE_EQ(reply.link_state().relative_pose().orientation().z(), relativePose[5]);
            ASSERT_DOUBLE_EQ(reply.link_state().relative_pose().orientation().w(), relativePose[6]);
        }

        if (reply.link_state().has_self_collide()) {
            ASSERT_DOUBLE_EQ(reply.link_state().self_collide(), self_collide);
        }

        if (reply.link_state().has_gravity()) {
            ASSERT_DOUBLE_EQ(reply.link_state().gravity(), gravity);
        }
        if (reply.link_state().has_kinematic()) {
            ASSERT_DOUBLE_EQ(reply.link_state().kinematic(), kinematic);
        }

        if (reply.link_state().has_enable_wind()) {
            ASSERT_DOUBLE_EQ(reply.link_state().enable_wind(), enable_wind);
        }
        if (reply.link_state().has_canonical()) {
            ASSERT_DOUBLE_EQ(reply.link_state().canonical(), canonical);
        }
    }

    // validate received packet error message
    void validateSetCommandErrorMsg(mw::internal::robotics::gazebotransport::Packet reply,
                                    errorMsgValidation errorType) {
        switch (errorType) {
        case Success: {
            ASSERT_EQ(reply.status(),
                      mw::internal::robotics::gazebotransport::Packet_CoSimError::
                          Packet_CoSimError_NONE); // 0: NO
                                                   // Error
            break;
        }
        case InvalidModel: {
            ASSERT_EQ(reply.status(),
                      mw::internal::robotics::gazebotransport::Packet_CoSimError::
                          Packet_CoSimError_MODEL_NAME_INVALID); // MODEL_NAME_INVALID error
            break;
        }
        case InvalidJoint: {
            ASSERT_EQ(reply.status(),
                      mw::internal::robotics::gazebotransport::Packet_CoSimError::
                          Packet_CoSimError_JOINT_NAME_INVALID); // JOINT_NAME_INVALID error
            break;
        }
        case InvalidLink: {
            ASSERT_EQ(reply.status(),
                      mw::internal::robotics::gazebotransport::Packet_CoSimError::
                          Packet_CoSimError_LINK_NAME_INVALID); // LINK_NAME_INVALID error
            break;
        }
        }
    }

    void SetUp() {
        /// Launch Client
        m_client = std::make_shared<robotics::gazebotransport::Client>(
            ipAddress, serverPort, boost::posix_time::milliseconds(time_out));

        /// Start Gazebo Simulator client
        gazebo::client::setup();

        // setup ignition transport node and subscriber
        m_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
        m_node->Init();
        m_worldStat.setSubscriber(
            m_node->Subscribe("~/world_stats", &GazeboWorldStat::OnWorldStat, &m_worldStat));

        mw::internal::robotics::gazebotransport::Packet reply = clientRequestCoSimulation("TestID");
        bool success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
        }
        /// it should be true as the server would grant request
        ASSERT_TRUE(success);

        std::this_thread::sleep_for(std::chrono::microseconds(100000));
    }

    void TearDown() {
        mw::internal::robotics::gazebotransport::Packet reply = clientStopCoSimulation("TestID");

        bool success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
        } else {
            success = false;
        }
        /// it should be true as the server would stop co-simulation
        ASSERT_TRUE(success);

        m_client->shutdown();

        gazebo::shutdown();
        std::this_thread::sleep_for(std::chrono::microseconds(50000));
    }
};

/*
 * It tests success of the get pose of a link
 * Also, it tests the client successfully gets pose of ground truth model/link
 */
TEST_F(pluginTest, testModelLinkPose) {
    mw::internal::robotics::gazebotransport::Packet reply = clientGetPose("unit_box", "link");

    /// The ground truth position and orientation are defined in world/unit_box.world
    /// which is loaded at the start
    ASSERT_NEAR(reply.pose().position().x(), 0.0, 0.0001);
    ASSERT_NEAR(reply.pose().position().y(), 0.0, 0.0001);
    ASSERT_NEAR(reply.pose().position().z(), 0.5, 0.0001);
    ASSERT_NEAR(reply.pose().orientation().x(), 0.0, 0.0001);
    ASSERT_NEAR(reply.pose().orientation().y(), 0.0, 0.0001);
    ASSERT_NEAR(reply.pose().orientation().z(), 0.0, 0.0001);
    ASSERT_NEAR(reply.pose().orientation().w(), 1.0, 0.0001);
}

/*
 * It tests reply message for invalid model name as input.
 * The client should successfully Pose message with all zero values
 */
TEST_F(pluginTest, testInValidModelName) {
    mw::internal::robotics::gazebotransport::Packet reply = clientGetPose("unit_box0", "link");

    /// The input model is not available in Gazebo
    /// So it returns all zero values for position and orientation
    ASSERT_DOUBLE_EQ(reply.pose().position().x(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().position().y(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().position().z(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().x(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().y(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().z(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().w(), 0);
}

/*
 * It tests reply message for invalid link name as input.
 * The client should successfully Pose message with all zero values
 */
TEST_F(pluginTest, testInValidLinkName) {
    mw::internal::robotics::gazebotransport::Packet reply = clientGetPose("unit_box", "link0");

    /// The input link is not available in Gazebo
    /// So it returns all zero values for position and orientation
    ASSERT_DOUBLE_EQ(reply.pose().position().x(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().position().y(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().position().z(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().x(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().y(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().z(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().w(), 0);
}

/*
 * It tests success of the get all Model information from Gazebo
 * Also, it tests the client successfully receives Model/link/joint names
 * by comparing with default as well as world file Model/link/joint names
 */
TEST_F(pluginTest, testModelNames) {
    /// Create Packet message to get Model info from Gazebo
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_GET_MODEL_INFO);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_get_model_info()->set_topic_name("~/GetModelInfo");

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

    /// Convert reply message to packet message
    mw::internal::robotics::gazebotransport::Packet reply;
    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
    }

    /// The received model info is compared with default gazebo model like "ground_plane" & link
    /// "link" Also, the ground truth model, link and joint names are given with .world file as
    /// "unit_box", "link" & "joint"
    ASSERT_STREQ(reply.model_info().model_data(0).model_name().c_str(), "ground_plane");
    ASSERT_STREQ(reply.model_info().model_data(0).links().link_name(0).c_str(), "link");
    ASSERT_STREQ(reply.model_info().model_data(1).model_name().c_str(), "unit_box");
    ASSERT_STREQ(reply.model_info().model_data(1).links().link_name(0).c_str(), "link");
    ASSERT_STREQ(reply.model_info().model_data(1).joints().joint_name(0).c_str(), "joint");
}

/// Test gazebo simulation reset time and scene action
TEST_F(pluginTest, testResetTimeScene) {
    /// Create Packet message reset simulation time & scene message
    mw::internal::robotics::gazebotransport::Packet m_message;

    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_RESET_SIMULATION);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_reset_simulation()->set_behavior(
        mw::internal::robotics::gazebotransport::
            ResetSimulation_ResetBehavior_RESET_TIME_AND_SCENE);

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));
    bool success;

    /// Convert reply message to packet message
    mw::internal::robotics::gazebotransport::Packet reply;

    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
        success = reply.has_status() && !reply.status();
    } else {
        success = false;
    }

    ASSERT_TRUE(success);
}

/// Test gazebo simulation reset time action
TEST_F(pluginTest, testResetTime) {
    /// Create Packet message reset simulation time message
    mw::internal::robotics::gazebotransport::Packet m_message;

    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_RESET_SIMULATION);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_reset_simulation()->set_behavior(
        mw::internal::robotics::gazebotransport::ResetSimulation_ResetBehavior_RESET_TIME);

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));
    bool success;

    /// Convert reply message to packet message
    mw::internal::robotics::gazebotransport::Packet reply;

    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
        success = reply.has_status() && !reply.status();
    } else {
        success = false;
    }

    ASSERT_TRUE(success);
}

/// Test gazebo image subscribe and get for image sensor 0
TEST_F(pluginTest, imageSensor0SubscribeGetTest) {
    // Reset gazebo scene and time
    mw::internal::robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    // subscribe image sensor 0
    reply = clientSubscribeImage("/gazebo/default/camera0/link/camera/image");

    success = false;

    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    bool subscribeStart = false;

    // Get image sensor based on image sensor update rate with stepping gazebo
    for (int i = 0; i <= 50; i++) {
        // Request image
        reply = clientGetImage("/gazebo/default/camera0/link/camera/image");

        success = false;

        if (reply.has_image()) {
            success = true;
        }

        // check image subscriber started
        if (success) {
            subscribeStart = true;
        } else {
            subscribeStart = false;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }

        // Get image at each step number else no image
        if (subscribeStart) {
            /// It should be true as topic & image is available in gazebo
            ASSERT_TRUE(success);
            /// Verify the received image data with ground truth values.
            auto simTime = m_worldStat.getSimTime();
            EXPECT_NEAR(reply.header().time_stamp().seconds(), simTime.sec, 1);
            EXPECT_NEAR(reply.header().time_stamp().nano_seconds(), simTime.nsec, 1e8);
            ASSERT_EQ(reply.image().width(), 320);
            ASSERT_EQ(reply.image().height(), 240);
            ASSERT_EQ(reply.image().data_type(), "RGB_INT8");
        } else {
            // Should be false as topic is not available / sensor is not publishing data
            ASSERT_FALSE(success);
        }

        // step gazebo by one
        mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(1);
        bool success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
        }
        /// it should be true as topic is available in gazebo
        ASSERT_TRUE(success);
    }
}

/// Test gazebo image subscribe and get for image sensor 1
TEST_F(pluginTest, imageSensor1SubscribeGetTest) {
    // Reset gazebo scene and time
    mw::internal::robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    // subscribe image sensor 1
    reply = clientSubscribeImage("/gazebo/default/camera1/link/camera/image");

    success = false;

    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    bool subscribeStart = false;

    // Get image sensor based on image sensor update rate with stepping gazebo
    for (int i = 0; i <= 50; i++) {
        // Request image
        reply = clientGetImage("/gazebo/default/camera1/link/camera/image");

        success = false;

        if (reply.has_image()) {
            success = true;
        }

        // check image subscriber started
        if (success) {
            subscribeStart = true;
        } else {
            subscribeStart = false;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }

        // Get image at each step number else no image
        if (subscribeStart) {
            /// It should be true as topic & image is available in gazebo
            ASSERT_TRUE(success);
            auto simTime = m_worldStat.getSimTime();
            EXPECT_NEAR(reply.header().time_stamp().seconds(), simTime.sec, 1);
            EXPECT_NEAR(reply.header().time_stamp().nano_seconds(), simTime.nsec, 1e8);
            /// Verify the received image data with ground truth values.
            ASSERT_EQ(reply.image().width(), 320);
            ASSERT_EQ(reply.image().height(), 240);
            ASSERT_EQ(reply.image().data_type(), "RGB_INT8");
        } else {
            // Should be false as topic is not available / sensor is not publishing data
            ASSERT_FALSE(success);
        }

        // step gazebo by one
        mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(1);
        bool success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
        }
        /// it should be true as topic is available in gazebo
        ASSERT_TRUE(success);
    }
}

/// Test gazebo depth image subscribe and get for depth image sensor 0
TEST_F(pluginTest, depthImageSensor0SubscribeGetTest) {
    // Reset gazebo scene and time
    mw::internal::robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    // subscribe depth image sensor 0
    reply = clientSubscribeImage("/gazebo/default/depth_camera0/link/camera/image");

    success = false;

    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    bool subscribeStart = false;

    // Get depth image sensor based on depth image sensor update rate with stepping gazebo
    for (int i = 0; i <= 50; i++) {
        // Request depth image
        reply = clientGetImage("/gazebo/default/depth_camera0/link/camera/image");

        success = false;

        if (reply.has_image()) {
            success = true;
        }

        // check image subscriber started
        if (success) {
            subscribeStart = true;
        } else {
            subscribeStart = false;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }

        // Get depth image at each step number else no depth image
        if (subscribeStart) {
            /// It should be true as topic & depth image is available in gazebo
            ASSERT_TRUE(success);
            auto simTime = m_worldStat.getSimTime();
            EXPECT_NEAR(reply.header().time_stamp().seconds(), simTime.sec, 1);
            EXPECT_NEAR(reply.header().time_stamp().nano_seconds(), simTime.nsec, 1e8);
            /// Verify the received depth image data with ground truth values.
            ASSERT_EQ(reply.image().width(), 320);
            ASSERT_EQ(reply.image().height(), 240);
            ASSERT_EQ(reply.image().data_type(), "R_FLOAT32");
        } else {
            // Should be false as topic is not available / sensor is not publishing data
            ASSERT_FALSE(success);
        }

        // step gazebo by one
        mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(1);
        bool success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
        }
        /// it should be true as topic is available in gazebo
        ASSERT_TRUE(success);
    }
}

/// Test gazebo depth image subscribe and get for depth image sensor 1
TEST_F(pluginTest, depthImageSensor1SubscribeGetTest) {
    // Reset gazebo scene and time
    mw::internal::robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    // subscribe depth image sensor 1
    reply = clientSubscribeImage("/gazebo/default/depth_camera1/link/camera/image");

    success = false;

    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    bool subscribeStart = false;

    // Get depth image sensor based on depth image sensor update rate with stepping gazebo
    for (int i = 0; i <= 50; i++) {
        // Request depth image
        reply = clientGetImage("/gazebo/default/depth_camera1/link/camera/image");

        success = false;

        if (reply.has_image()) {
            success = true;
        }

        // check image subscriber started
        if (success) {
            subscribeStart = true;
        } else {
            subscribeStart = false;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }

        // Get depth image at each step number else no depth image
        if (subscribeStart) {
            /// It should be true as topic & depth image is available in gazebo
            ASSERT_TRUE(success);
            auto simTime = m_worldStat.getSimTime();
            EXPECT_NEAR(reply.header().time_stamp().seconds(), simTime.sec, 1);
            EXPECT_NEAR(reply.header().time_stamp().nano_seconds(), simTime.nsec, 1e8);
            /// Verify the received depth image data with ground truth values.
            ASSERT_EQ(reply.image().width(), 320);
            ASSERT_EQ(reply.image().height(), 240);
            ASSERT_EQ(reply.image().data_type(), "R_FLOAT32");
        } else {
            // Should be false as topic is not available / sensor is not publishing data
            ASSERT_FALSE(success);
        }

        // step gazebo by one
        mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(1);
        bool success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
        }
        /// it should be true as topic is available in gazebo
        ASSERT_TRUE(success);
    }
}

/// Test gazebo 3D lidar subscribe and get lidar data for 3D lidar sensor (velodyne)
TEST_F(pluginTest, laserSensor3DSubscribeTest) {
    // Reset gazebo scene and time
    mw::internal::robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    reply = clientSubscribeLaser("/gazebo/default/velodyne/link/laser/scan");

    success = false;

    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    bool subscribeStart = false;
    // Get laser sensor based on laser sensor update rate with stepping gazebo
    for (int i = 0; i <= 50; i++) {

        reply = clientGetLaser("/gazebo/default/velodyne/link/laser/scan");

        success = false;

        if (reply.has_laser_data()) {
            success = true;
        }

        // check laser subscriber started
        if (success) {
            subscribeStart = true;
        } else {
            subscribeStart = false;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }

        // Get laser data at each step number else not
        if (subscribeStart) {
            /// it should be true as topic & laser data is available in gazebo
            ASSERT_TRUE(success);
            auto simTime = m_worldStat.getSimTime();
            EXPECT_NEAR(reply.header().time_stamp().seconds(), simTime.sec, 1);
            EXPECT_NEAR(reply.header().time_stamp().nano_seconds(), simTime.nsec, 1e8);
            /// Verify the received laser data with ground truth values.
            ASSERT_EQ(reply.laser_data().angle_min(), -2.26889);
            ASSERT_EQ(reply.laser_data().angle_max(), 2.268899);
            ASSERT_EQ(reply.laser_data().count(), 20480);
            ASSERT_EQ(reply.laser_data().range_min(), 0.1);
            ASSERT_EQ(reply.laser_data().range_max(), 10);
            ASSERT_EQ(reply.laser_data().vertical_angle_min(), -0.535);
            ASSERT_EQ(reply.laser_data().vertical_angle_max(), 0.186132);
        } else {
            // Should be false as topic is not available / sensor is not publishing data
            ASSERT_FALSE(success);
        }

        // step gazebo by one
        mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(1);
        bool success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
        }
        /// it should be true as topic is available in gazebo
        ASSERT_TRUE(success);
    }
}

/// Test gazebo laser subscribe and get laser data for laser sensor 0
TEST_F(pluginTest, laserSensor0SubscribeTest) {
    // Reset gazebo scene and time
    mw::internal::robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    reply = clientSubscribeLaser("/gazebo/default/hokuyo0/link/laser/scan");

    success = false;

    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    bool subscribeStart = false;
    // Get laser sensor based on laser sensor update rate with stepping gazebo
    for (int i = 0; i <= 50; i++) {

        reply = clientGetLaser("/gazebo/default/hokuyo0/link/laser/scan");

        success = false;

        if (reply.has_laser_data()) {
            success = true;
        }

        // check laser subscriber started
        if (success) {
            subscribeStart = true;
        } else {
            subscribeStart = false;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }

        // Get laser data at each step number else not
        if (subscribeStart) {
            /// it should be true as topic & laser data is available in gazebo
            ASSERT_TRUE(success);
            auto simTime = m_worldStat.getSimTime();
            EXPECT_NEAR(reply.header().time_stamp().seconds(), simTime.sec, 1);
            EXPECT_NEAR(reply.header().time_stamp().nano_seconds(), simTime.nsec, 1e8);
            /// Verify the received laser data with ground truth values.
            ASSERT_EQ(reply.laser_data().angle_min(), -3.14);
            ASSERT_EQ(reply.laser_data().angle_max(), 3.14);
            ASSERT_EQ(reply.laser_data().count(), 640);
            ASSERT_EQ(reply.laser_data().range_min(), 0.08);
            ASSERT_EQ(reply.laser_data().range_max(), 10);
        } else {
            // Should be false as topic is not available / sensor is not publishing data
            ASSERT_FALSE(success);
        }

        // step gazebo by one
        mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(1);
        bool success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
        }
        /// it should be true as topic is available in gazebo
        ASSERT_TRUE(success);
    }
}

/// Test gazebo laser subscribe and get laser data for laser sensor 1
TEST_F(pluginTest, laserSensor1SubscribeTest) {
    // Reset gazebo scene and time
    mw::internal::robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    reply = clientSubscribeLaser("/gazebo/default/hokuyo1/link/laser/scan");

    success = false;

    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    bool subscribeStart = false;
    // Get laser sensor based on laser sensor update rate with stepping gazebo
    for (int i = 0; i <= 50; i++) {

        reply = clientGetLaser("/gazebo/default/hokuyo1/link/laser/scan");

        success = false;

        if (reply.has_laser_data()) {
            success = true;
        }

        // check laser subscriber started
        if (success) {
            subscribeStart = true;
        } else {
            subscribeStart = false;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }

        // Get laser data at each step number else not
        if (subscribeStart) {
            /// it should be true as topic & laser data is available in gazebo
            ASSERT_TRUE(success);
            auto simTime = m_worldStat.getSimTime();
            EXPECT_NEAR(reply.header().time_stamp().seconds(), simTime.sec, 1);
            EXPECT_NEAR(reply.header().time_stamp().nano_seconds(), simTime.nsec, 1e8);
            /// Verify the received laser data with ground truth values.
            ASSERT_EQ(reply.laser_data().angle_min(), -3.14);
            ASSERT_EQ(reply.laser_data().angle_max(), 3.14);
            ASSERT_EQ(reply.laser_data().count(), 640);
            ASSERT_EQ(reply.laser_data().range_min(), 0.08);
            ASSERT_EQ(reply.laser_data().range_max(), 10);
        } else {
            // Should be false as topic is not available / sensor is not publishing data
            ASSERT_FALSE(success);
        }

        // step gazebo by one
        mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(1);
        bool success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
        }
        /// it should be true as topic is available in gazebo
        ASSERT_TRUE(success);
    }
}

/// Test gazebo subscribe imu sensor and get imu data for imu sensor 0
TEST_F(pluginTest, imuSensor0SubscribeGetTest) {
    // Reset gazebo scene and time
    mw::internal::robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    // subscribe imu sensor 0
    reply = clientSubscribeImu("/gazebo/default/imu0/link/imu/imu");

    success = false;

    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    bool subscribeStart = false;
    // Get laser sensor based on laser sensor update rate with stepping gazebo
    for (int i = 0; i <= 50; i++) {

        reply = clientGetImu("/gazebo/default/imu0/link/imu/imu");

        success = false;

        if (reply.has_imu_data()) {
            success = true;
        }

        // check imu subscriber started
        if (success) {
            subscribeStart = true;
        } else {
            subscribeStart = false;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }

        // Get imu data at each step number else not
        if (subscribeStart) {
            /// it should be true as topic & imu data is available in gazebo
            ASSERT_TRUE(success);
        } else {
            // Should be false as topic is not available / sensor is not publishing data
            ASSERT_FALSE(success);
        }

        // step gazebo by one
        mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(1);
        bool success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
        }
        /// it should be true as topic is available in gazebo
        ASSERT_TRUE(success);
    }
}

/// Test gazebo subscribe imu sensor and get imu data for imu sensor 0
TEST_F(pluginTest, imuSensor1SubscribeGetTest) {
    // Reset gazebo scene and time
    mw::internal::robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    // subscribe imu sensor 1
    reply = clientSubscribeImu("/gazebo/default/imu1/link/imu/imu");

    success = false;

    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    bool subscribeStart = false;
    // Get laser sensor based on laser sensor update rate with stepping gazebo
    for (int i = 0; i <= 50; i++) {

        reply = clientGetImu("/gazebo/default/imu1/link/imu/imu");

        success = false;

        if (reply.has_imu_data()) {
            success = true;
        }

        // check imu subscriber started
        if (success) {
            subscribeStart = true;
        } else {
            subscribeStart = false;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }

        // Get imu data at each step number else not
        if (subscribeStart) {
            /// it should be true as topic & imu data is available in gazebo
            ASSERT_TRUE(success);
        } else {
            // Should be false as topic is not available / sensor is not publishing data
            ASSERT_FALSE(success);
        }

        // step gazebo by one
        mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(1);
        bool success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
        }
        /// it should be true as topic is available in gazebo
        ASSERT_TRUE(success);
    }
}

TEST_F(pluginTest, imageSensorSubscribeTwice) {
    // Reset gazebo scene and time
    mw::internal::robotics::gazebotransport::Packet reply = clientResetSceneTime();

    bool success = false;
    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as reset done
    ASSERT_TRUE(success);

    std::vector<mw::internal::robotics::gazebotransport::Time> imageTime;
    // subscribe image sensor 0 twice and make sure that the client still can get new images
    reply = clientSubscribeImage("/gazebo/default/camera0/link/camera/image");
    reply = clientSubscribeImage("/gazebo/default/camera0/link/camera/image");
    success = false;

    if (reply.has_status() && !reply.status()) {
        success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(success);

    // Collect two images from Gazebo
    size_t loopCount = 0;
    while (imageTime.size() < 2 && loopCount++ < 50) {
        // Request image
        reply = clientGetImage("/gazebo/default/camera0/link/camera/image");

        success = reply.has_image();

        // if image is obtained, log the image time_stamp
        if (success) {
            /// It should be true as topic & image is available in gazebo
            imageTime.push_back(reply.header().time_stamp());
        }

        // step gazebo by 100
        mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(100);
        success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
            std::this_thread::sleep_for(std::chrono::microseconds(200000));
        }
        ASSERT_TRUE(success);
    }

    // verify that two images are not recorded at the same time
    ASSERT_EQ(imageTime.size(), 2);
    EXPECT_FALSE(imageTime[0].seconds() == imageTime[1].seconds() &&
                 imageTime[0].nano_seconds() == imageTime[1].nano_seconds())
        << "Two image should not be obtained at the same time";
}

/// Test gazebo get topic lists
TEST_F(pluginTest, testGetTopicList) {

    mw::internal::robotics::gazebotransport::Packet m_message;
    mw::internal::robotics::gazebotransport::Packet reply;

    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_GET_TOPIC_LIST);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_get_topic_list()->set_topic_name("~/GetTopicList");

    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
    }

    /// Check with default topics available in Gazebo
    // ASSERT_STREQ("/gazebo/default/physics/contacts", reply.topic_list().data(0).name().c_str());
    // // For 'gzserver' launch
    ASSERT_STREQ("/gazebo/default/atmosphere",
                 reply.topic_list().data(0).name().c_str()); // For 'gazebo' launch
}

/*
 * It tests, if the image sensor( topic) is not available in Gazebo
 * & user subscribing that topic. Then, it should return FALSE.
 */
TEST_F(pluginTest, testImageInValidTopic) {
    //*********************************************
    /// TEST Wrong Image Subscriber
    mw::internal::robotics::gazebotransport::Packet reply0 =
        clientSubscribeImage("/gazebo/default/camera/link/camera/image");

    bool success0 = false;

    if (reply0.has_status() && !reply0.status()) {
        success0 = true;
    }
    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(success0);
    ASSERT_EQ(reply0.status(), mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                   Packet_CoSimError_TOPIC_NAME_INVALID);
}

/*
 * It tests, if the image sensor( topic) is not available in Gazebo
 * & user trying to get image data from that topic.
 * Then, all image sensor message fields should zero.
 */
TEST_F(pluginTest, testGetInvalidImage) {
    //*********************************************
    /// TEST Get Invalid Image

    mw::internal::robotics::gazebotransport::Packet replyGetWrongImage =
        clientGetImage("/gazebo/default/camera/link/camera/image");

    bool success0 = false;

    if (replyGetWrongImage.has_status() && !replyGetWrongImage.status()) {
        success0 = true;
    }
    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(success0);
    ASSERT_EQ(replyGetWrongImage.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_TOPIC_NAME_INVALID);
}

/*
 * It tests, if the imu sensor( topic) is not available in Gazebo
 * & user subscribing that topic. Then, it should return FALSE.
 */
TEST_F(pluginTest, testImuInValidTopic) {
    //*********************************************
    /// TEST Wrong Imu Subscriber

    mw::internal::robotics::gazebotransport::Packet reply0 =
        clientSubscribeImu("/gazebo/default/imu/link/imu/imu");

    bool success0 = false;

    if (reply0.has_status() && !reply0.status()) {
        success0 = true;
    }
    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(success0);
    ASSERT_EQ(reply0.status(), mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                   Packet_CoSimError_TOPIC_NAME_INVALID);
}

/*
 * It tests, if the imu sensor( topic) is not available in Gazebo
 * & user trying to get imu data from that topic.
 * Then, all imu sensor message fields should zero.
 */
TEST_F(pluginTest, testGetInValidImu) {
    //*********************************************
    /// TEST Get Invalid Imu

    mw::internal::robotics::gazebotransport::Packet replyGetWrongImu =
        clientGetImu("/gazebo/default/imu/link/imu/imu");

    bool success0 = false;

    if (replyGetWrongImu.has_status() && !replyGetWrongImu.status()) {
        success0 = true;
    }
    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(success0);
    ASSERT_EQ(replyGetWrongImu.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_TOPIC_NAME_INVALID);
}

/*
 * It tests, if the laser sensor( topic) is not available in Gazebo
 * & user subscribing that topic. Then, it should return FALSE.
 */
TEST_F(pluginTest, testLaserInValidTopic) {
    //*********************************************
    /// TEST Wrong Laser Subscriber

    mw::internal::robotics::gazebotransport::Packet reply0 =
        clientSubscribeLaser("/gazebo/default/hokuyo/link/laser/scan");

    bool success0 = false;

    if (reply0.has_status() && !reply0.status()) {
        success0 = true;
    }
    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(success0);
    ASSERT_EQ(reply0.status(), mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                   Packet_CoSimError_TOPIC_NAME_INVALID);
}

/*
 * It tests, if the laser sensor( topic) is not available in Gazebo
 * & user trying to get laser data from that topic.
 * Then, all laser sensor message fields should zero.
 */
TEST_F(pluginTest, testGetInValidLaser) {
    //*********************************************
    /// TEST Get Invalid Laser

    mw::internal::robotics::gazebotransport::Packet replyGetWrongLaser =
        clientGetLaser("/gazebo/default/hokuyo/link/laser/scan");

    bool success0 = false;

    if (replyGetWrongLaser.has_status() && !replyGetWrongLaser.status()) {
        success0 = true;
    }
    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(success0);
    ASSERT_EQ(replyGetWrongLaser.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_TOPIC_NAME_INVALID);
}

/*
 * It tests success of the apply joint torque
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(pluginTest, testApplyJointSuccess) {
    mw::internal::robotics::gazebotransport::Packet reply =

        clientApplyJointTorque("unit_box", "joint", 0, 100000.0, 0, 10000000);

    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error
}

/*
 * It tests success of the apply joint torque for two models with same
 * link and joint names.
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(pluginTest, testApplyJointTwoModelsSuccess) {
    // get ground truth pose of two model before apply command
    mw::internal::robotics::gazebotransport::Packet pose0_before =
        clientGetPose("unit_boxB", "link");
    mw::internal::robotics::gazebotransport::Packet pose1_before =
        clientGetPose("unit_boxA", "link");

    // apply joint torque on two models
    mw::internal::robotics::gazebotransport::Packet reply0 =
        clientApplyJointTorque("unit_boxB", "joint", 0, 100000.0, 10, 10000000);
    ASSERT_EQ(reply0.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error
    mw::internal::robotics::gazebotransport::Packet reply1 =
        clientApplyJointTorque("unit_boxA", "joint", 0, 100000.0, 10, 10000000);
    ASSERT_EQ(reply1.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error

    // simulate for 100 steps
    mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(100);
    bool success = false;
    if (reply.has_status() && !reply.status()) {
        success = true;
        std::this_thread::sleep_for(std::chrono::microseconds(200000));
    }
    ASSERT_TRUE(success);

    // get two model pose after simulation
    mw::internal::robotics::gazebotransport::Packet pose0_after =
        clientGetPose("unit_boxB", "link");
    mw::internal::robotics::gazebotransport::Packet pose1_after =
        clientGetPose("unit_boxA", "link");

    // verify that, there should be change in the pose before and after simulation
    EXPECT_NE(pose0_before.pose().position().x(), pose0_after.pose().position().x());
    EXPECT_NE(pose0_before.pose().position().y(), pose0_after.pose().position().y());
    EXPECT_NE(pose0_before.pose().position().z(), pose0_after.pose().position().z());
    EXPECT_NE(pose1_before.pose().position().x(), pose1_after.pose().position().x());
    EXPECT_NE(pose1_before.pose().position().y(), pose1_after.pose().position().y());
    EXPECT_NE(pose1_before.pose().position().z(), pose1_after.pose().position().z());
}

/*
 * It tests reply message for invalid model name as input.
 * The client should successfully receive the MODEL_NAME_INVALID error message
 */
TEST_F(pluginTest, testApplyJointInValidModelName) {
    mw::internal::robotics::gazebotransport::Packet reply =

        clientApplyJointTorque("unit_box0", "joint", 0, 1000.0, 0, 0);

    ASSERT_EQ(reply.status(), mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                  Packet_CoSimError_MODEL_NAME_INVALID); // MODEL_NAME_INVALID error
}

/*
 * It tests reply message for invalid joint name as input.
 * The client should successfully receive the JOINT_NAME_INVALID error message
 */
TEST_F(pluginTest, testApplyJointInValidJointName) {
    mw::internal::robotics::gazebotransport::Packet reply =

        clientApplyJointTorque("unit_box", "joint0", 0, 1000.0, 0, 0);

    ASSERT_EQ(reply.status(), mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                  Packet_CoSimError_JOINT_NAME_INVALID); // JOINT_NAME_INVALID error
}


/*
 * It tests success of the set joint position for two models with same
 * link and joint names.
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(pluginTest, testSetJointPositionTwoModelsSuccess) {
    // get ground truth pose of two model before apply command
    mw::internal::robotics::gazebotransport::Packet pose0_before =
        clientGetPose("unit_boxB", "link");
    mw::internal::robotics::gazebotransport::Packet pose1_before =
        clientGetPose("unit_boxA", "link");

    // set joint position on two models
    mw::internal::robotics::gazebotransport::Packet reply0 =
        clientSetJointPosition("unit_boxB", "joint", 0, 100000.0, 10, 10000000);
    ASSERT_EQ(reply0.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error
    mw::internal::robotics::gazebotransport::Packet reply1 =
        clientSetJointPosition("unit_boxA", "joint", 0, 100000.0, 10, 10000000);
    ASSERT_EQ(reply1.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error

    // simulate for 100 steps
    mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(100);
    bool success = false;
    if (reply.has_status() && !reply.status()) {
        success = true;
        std::this_thread::sleep_for(std::chrono::microseconds(200000));
    }
    ASSERT_TRUE(success);

    // get two model pose after simulation
    mw::internal::robotics::gazebotransport::Packet pose0_after =
        clientGetPose("unit_boxB", "link");
    mw::internal::robotics::gazebotransport::Packet pose1_after =
        clientGetPose("unit_boxA", "link");

    // verify that, there should be change in the pose before and after simulation
    EXPECT_NE(pose0_before.pose().position().x(), pose0_after.pose().position().x());
    EXPECT_NE(pose0_before.pose().position().y(), pose0_after.pose().position().y());
    EXPECT_NE(pose0_before.pose().position().z(), pose0_after.pose().position().z());
    EXPECT_NE(pose1_before.pose().position().x(), pose1_after.pose().position().x());
    EXPECT_NE(pose1_before.pose().position().y(), pose1_after.pose().position().y());
    EXPECT_NE(pose1_before.pose().position().z(), pose1_after.pose().position().z());
}

/*
 * It tests success of the set joint velocity for two models with same
 * link and joint names.
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(pluginTest, testSetJointVeloityTwoModelsSuccess) {
    // get ground truth pose of two model before apply command
    mw::internal::robotics::gazebotransport::Packet pose0_before =
        clientGetPose("unit_boxB", "link");
    mw::internal::robotics::gazebotransport::Packet pose1_before =
        clientGetPose("unit_boxA", "link");

    // set joint velocity on two models
    mw::internal::robotics::gazebotransport::Packet reply0 =
        clientSetJointVelocity("unit_boxB", "joint", 0, 100000.0, 10, 10000000);
    ASSERT_EQ(reply0.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error
    mw::internal::robotics::gazebotransport::Packet reply1 =
        clientSetJointVelocity("unit_boxA", "joint", 0, 100000.0, 10, 10000000);
    ASSERT_EQ(reply1.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error

    // simulate for 100 steps
    mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(100);
    bool success = false;
    if (reply.has_status() && !reply.status()) {
        success = true;
        std::this_thread::sleep_for(std::chrono::microseconds(200000));
    }
    ASSERT_TRUE(success);

    // get two model pose after simulation
    mw::internal::robotics::gazebotransport::Packet pose0_after =
        clientGetPose("unit_boxB", "link");
    mw::internal::robotics::gazebotransport::Packet pose1_after =
        clientGetPose("unit_boxA", "link");

    // verify that, there should be change in the pose before and after simulation
    EXPECT_NE(pose0_before.pose().position().x(), pose0_after.pose().position().x());
    EXPECT_NE(pose1_before.pose().position().x(), pose1_after.pose().position().x());
}


/*
 * It tests success of the set link linear velocity for two models with same
 * link and joint names.
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(pluginTest, testSetLinkLinearVeloityTwoModelsSuccess) {
    // get ground truth pose of two model before apply command
    mw::internal::robotics::gazebotransport::Packet pose0_before =
        clientGetPose("unit_boxB", "link");
    mw::internal::robotics::gazebotransport::Packet pose1_before =
        clientGetPose("unit_boxA", "link");

    // set link linear velocity on two models
    double velocity[3] = {1000.0, 100.0, 100.0};
    mw::internal::robotics::gazebotransport::Packet reply0 =
        clientSetLinkLinearVelocity("unit_boxB", "link", velocity, 10, 10000000);
    ASSERT_EQ(reply0.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error
    mw::internal::robotics::gazebotransport::Packet reply1 =
        clientSetLinkLinearVelocity("unit_boxA", "link", velocity, 10, 10000000);
    ASSERT_EQ(reply1.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error

    // simulate for 100 steps
    mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(100);
    bool success = false;
    if (reply.has_status() && !reply.status()) {
        success = true;
        std::this_thread::sleep_for(std::chrono::microseconds(200000));
    }
    ASSERT_TRUE(success);

    // get two model pose after simulation
    mw::internal::robotics::gazebotransport::Packet pose0_after =
        clientGetPose("unit_boxB", "link");
    mw::internal::robotics::gazebotransport::Packet pose1_after =
        clientGetPose("unit_boxA", "link");

    // verify that, there should be change in the pose before and after simulation
    EXPECT_NE(pose0_before.pose().position().x(), pose0_after.pose().position().x());
    EXPECT_NE(pose0_before.pose().position().y(), pose0_after.pose().position().y());
    EXPECT_NE(pose0_before.pose().position().z(), pose0_after.pose().position().z());
    EXPECT_NE(pose1_before.pose().position().x(), pose1_after.pose().position().x());
    EXPECT_NE(pose1_before.pose().position().y(), pose1_after.pose().position().y());
    EXPECT_NE(pose1_before.pose().position().z(), pose1_after.pose().position().z());
}

/*
 * It tests success of the set link angular velocity for two models with same
 * link and joint names.
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(pluginTest, testSetLinkAngularVeloityTwoModelsSuccess) {
    // get ground truth pose of two model before apply command
    mw::internal::robotics::gazebotransport::Packet pose0_before =
        clientGetPose("unit_boxB", "link");
    mw::internal::robotics::gazebotransport::Packet pose1_before =
        clientGetPose("unit_boxA", "link");

    // set link angular velocity  on two models
    double velocity[3] = {1000.0, 100.0, 100.0};
    mw::internal::robotics::gazebotransport::Packet reply0 =
        clientSetLinkAngularVelocity("unit_boxB", "link", velocity, 10, 10000000);
    ASSERT_EQ(reply0.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error
    mw::internal::robotics::gazebotransport::Packet reply1 =
        clientSetLinkAngularVelocity("unit_boxA", "link", velocity, 10, 10000000);
    ASSERT_EQ(reply1.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error

    // simulate for 100 steps
    mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(100);
    bool success = false;
    if (reply.has_status() && !reply.status()) {
        success = true;
        std::this_thread::sleep_for(std::chrono::microseconds(200000));
    }
    ASSERT_TRUE(success);

    // get two model pose after simulation
    mw::internal::robotics::gazebotransport::Packet pose0_after =
        clientGetPose("unit_boxB", "link");
    mw::internal::robotics::gazebotransport::Packet pose1_after =
        clientGetPose("unit_boxA", "link");

    // verify that, there should be change in the pose before and after simulation
    EXPECT_NE(pose0_before.pose().position().x(), pose0_after.pose().position().x());
    EXPECT_NE(pose0_before.pose().position().y(), pose0_after.pose().position().y());
    EXPECT_NE(pose0_before.pose().position().z(), pose0_after.pose().position().z());
    EXPECT_NE(pose1_before.pose().position().x(), pose1_after.pose().position().x());
    EXPECT_NE(pose1_before.pose().position().y(), pose1_after.pose().position().y());
    EXPECT_NE(pose1_before.pose().position().z(), pose1_after.pose().position().z());
}

/*
 * It tests success of the set link world pose for two models with same
 * link and joint names.
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(pluginTest, testSetLinkWorldPoseTwoModelsSuccess) {
    // get ground truth pose of two model before apply command
    mw::internal::robotics::gazebotransport::Packet pose0_before =
        clientGetPose("unit_boxB", "link");
    mw::internal::robotics::gazebotransport::Packet pose1_before =
        clientGetPose("unit_boxA", "link");

    // set link world pose on two models
    double pose[7] = {3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

    mw::internal::robotics::gazebotransport::Packet reply0 =
        clientSetLinkWorldPose("unit_boxB", "link", pose, 10, 10000000);
    ASSERT_EQ(reply0.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error

    mw::internal::robotics::gazebotransport::Packet reply1 =
        clientSetLinkWorldPose("unit_boxA", "link", pose, 10, 10000000);
    ASSERT_EQ(reply1.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error

    // simulate for 100 steps
    mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(100);
    bool success = false;
    if (reply.has_status() && !reply.status()) {
        success = true;
        std::this_thread::sleep_for(std::chrono::microseconds(200000));
    }
    ASSERT_TRUE(success);

    // get two model pose after simulation
    mw::internal::robotics::gazebotransport::Packet pose0_after =
        clientGetPose("unit_boxB", "link");
    mw::internal::robotics::gazebotransport::Packet pose1_after =
        clientGetPose("unit_boxA", "link");

    // verify that, there should be change in the pose before and after simulation
    EXPECT_NE(pose0_before.pose().position().x(), pose0_after.pose().position().x());
    EXPECT_NE(pose0_before.pose().position().y(), pose0_after.pose().position().y());
    EXPECT_NE(pose0_before.pose().position().z(), pose0_after.pose().position().z());
    EXPECT_NE(pose1_before.pose().position().x(), pose1_after.pose().position().x());
    EXPECT_NE(pose1_before.pose().position().y(), pose1_after.pose().position().y());
    EXPECT_NE(pose1_before.pose().position().z(), pose1_after.pose().position().z());
}


/*
 * It tests success of the apply link force/torque
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(pluginTest, testApplyLinkWrenchSuccess) {
    double force_values[3] = {15000.0, 0.0, 0.0};
    double torque_values[3] = {15000.0, 0.0, 0.0};

    mw::internal::robotics::gazebotransport::Packet reply = clientApplyLinkWrench(
        "unit_box", "link", "ADD", "ADD", force_values, torque_values, 0, 100000000);

    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // NO Error
}

/*
 * It tests success of the apply link wrench for two models with same
 * link and joint names.
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(pluginTest, testApplyLinkWrenchTwoModelsSuccess) {
    // get ground truth pose of two model before apply command
    mw::internal::robotics::gazebotransport::Packet pose0_before =
        clientGetPose("unit_boxB", "link");
    mw::internal::robotics::gazebotransport::Packet pose1_before =
        clientGetPose("unit_boxA", "link");

    // apply link wrench on two models
    double force_values[3] = {15000.0, 0.0, 0.0};
    double torque_values[3] = {15000.0, 0.0, 0.0};

    mw::internal::robotics::gazebotransport::Packet reply0 = clientApplyLinkWrench(
        "unit_boxB", "link", "SET", "SET", force_values, torque_values, 10, 100000000);
    ASSERT_EQ(reply0.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error
    mw::internal::robotics::gazebotransport::Packet reply1 = clientApplyLinkWrench(
        "unit_boxA", "link", "SET", "SET", force_values, torque_values, 10, 100000000);

    ASSERT_EQ(reply1.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error

    // simulate for 100 steps
    mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(100);
    bool success = false;
    if (reply.has_status() && !reply.status()) {
        success = true;
        std::this_thread::sleep_for(std::chrono::microseconds(200000));
    }
    ASSERT_TRUE(success);

    // get two model pose after simulation
    mw::internal::robotics::gazebotransport::Packet pose0_after =
        clientGetPose("unit_boxB", "link");
    mw::internal::robotics::gazebotransport::Packet pose1_after =
        clientGetPose("unit_boxA", "link");

    // verify that, there should be change in the pose before and after simulation
    EXPECT_NE(pose0_before.pose().position().x(), pose0_after.pose().position().x());
    EXPECT_NE(pose0_before.pose().position().y(), pose0_after.pose().position().y());
    EXPECT_NE(pose0_before.pose().position().z(), pose0_after.pose().position().z());
    EXPECT_NE(pose1_before.pose().position().x(), pose1_after.pose().position().x());
    EXPECT_NE(pose1_before.pose().position().y(), pose1_after.pose().position().y());
    EXPECT_NE(pose1_before.pose().position().z(), pose1_after.pose().position().z());
}

/*
 * It tests success of the apply link wrench for two models with same
 * link and joint names.
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(pluginTest, testApplyLinkWrenchADDTwoModelsSuccess) {
    // get ground truth pose of two model before apply command
    mw::internal::robotics::gazebotransport::Packet pose0_before =
        clientGetPose("unit_boxB", "link");
    mw::internal::robotics::gazebotransport::Packet pose1_before =
        clientGetPose("unit_boxA", "link");

    // apply link wrench on two models
    double force_values[3] = {15000.0, 0.0, 0.0};
    double torque_values[3] = {15000.0, 0.0, 0.0};

    mw::internal::robotics::gazebotransport::Packet reply0 = clientApplyLinkWrench(
        "unit_boxB", "link", "ADD", "ADD", force_values, torque_values, 10, 100000000);
    ASSERT_EQ(reply0.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error
    mw::internal::robotics::gazebotransport::Packet reply1 = clientApplyLinkWrench(
        "unit_boxA", "link", "ADD", "ADD", force_values, torque_values, 10, 100000000);

    ASSERT_EQ(reply1.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error

    // simulate for 100 steps
    mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(100);
    bool success = false;
    if (reply.has_status() && !reply.status()) {
        success = true;
        std::this_thread::sleep_for(std::chrono::microseconds(200000));
    }
    ASSERT_TRUE(success);

    // get two model pose after simulation
    mw::internal::robotics::gazebotransport::Packet pose0_after =
        clientGetPose("unit_boxB", "link");
    mw::internal::robotics::gazebotransport::Packet pose1_after =
        clientGetPose("unit_boxA", "link");

    // verify that, there should be change in the pose before and after simulation
    EXPECT_NE(pose0_before.pose().position().x(), pose0_after.pose().position().x());
    EXPECT_NE(pose0_before.pose().position().y(), pose0_after.pose().position().y());
    EXPECT_NE(pose0_before.pose().position().z(), pose0_after.pose().position().z());
    EXPECT_NE(pose1_before.pose().position().x(), pose1_after.pose().position().x());
    EXPECT_NE(pose1_before.pose().position().y(), pose1_after.pose().position().y());
    EXPECT_NE(pose1_before.pose().position().z(), pose1_after.pose().position().z());
}

/*
 * It tests reply message for invalid model name as input.
 * The client should successfully receive the MODEL_NAME_INVALID error message
 */
TEST_F(pluginTest, testApplyLinkInValidModelName) {
    double force_values[3] = {3000.0, 0.0, 0.0};
    double torque_values[3] = {3000.0, 0.0, 0.0};

    mw::internal::robotics::gazebotransport::Packet reply =
        clientApplyLinkWrench("unt_box", "link", "ADD", "ADD", force_values, torque_values, 0, 0);

    ASSERT_EQ(reply.status(), mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                  Packet_CoSimError_MODEL_NAME_INVALID); // MODEL_NAME_INVALID error
}

/*
 * It tests reply message for invalid link name as input.
 * The client should successfully receive the LINK_NAME_INVALID error message
 */
TEST_F(pluginTest, testApplyLinkInValidLinkName) {
    double force_values[3] = {3000.0, 0.0, 0.0};
    double torque_values[3] = {3000.0, 0.0, 0.0};

    mw::internal::robotics::gazebotransport::Packet reply =
        clientApplyLinkWrench("unit_box", "lik", "ADD", "ADD", force_values, torque_values, 0, 0);

    ASSERT_EQ(reply.status(), mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                  Packet_CoSimError_LINK_NAME_INVALID); // LINK_NAME_INVALID error
}

/*
 * It tests reply message for invalid force type name as input.
 * The client should successfully receive the FORCE_TYPE_INVALID error message
 */
TEST_F(pluginTest, testApplyLinkInValidForceType) {
    double force_values[3] = {3000.0, 0.0, 0.0};
    double torque_values[3] = {3000.0, 0.0, 0.0};

    mw::internal::robotics::gazebotransport::Packet reply =
        clientApplyLinkWrench("unit_box", "link", "AD", "ADD", force_values, torque_values, 0, 0);

    ASSERT_EQ(reply.status(), mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                  Packet_CoSimError_FORCE_TYPE_INVALID); // FORCE_TYPE_INVALID error
}

/*
 * It tests reply message for invalid torque type name as input.
 * The client should successfully receive the TORQUE_TYPE_INVALID error message
 */
TEST_F(pluginTest, testApplyLinkInValidTorqueType) {
    double force_values[3] = {3000.0, 0.0, 0.0};
    double torque_values[3] = {3000.0, 0.0, 0.0};

    mw::internal::robotics::gazebotransport::Packet reply =
        clientApplyLinkWrench("unit_box", "link", "ADD", "AD", force_values, torque_values, 0, 0);

    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_TORQUE_TYPE_INVALID); // TORQUE_TYPE_INVALID error
}
/*
 * It tests success of the get max step size value from Gazebo
 * Also, it tests the client successfully gets the default step size (0.001)
 */
TEST_F(pluginTest, testGetStepSize) {
    /// Create Packet message to get max step size from Gazebo

    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_MAX_STEP_SIZE);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_max_step_size()->set_type(
        mw::internal::robotics::gazebotransport::MaxStepSize_TYPE::MaxStepSize_TYPE_GET_STEP_SIZE);
    m_message.mutable_max_step_size()->set_size(0);

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

    /// Convert reply message to packet message
    mw::internal::robotics::gazebotransport::Packet reply;

    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
    }

    ASSERT_EQ(reply.max_step_size().size(), 0.001); // default value 0.001
}

/*
 * It tests success of the set max step size value to Gazebo
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(pluginTest, testSetStepSize) {
    /// Create Packet message to set max step size of Gazebo
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_MAX_STEP_SIZE);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_max_step_size()->set_type(
        mw::internal::robotics::gazebotransport::MaxStepSize_TYPE::MaxStepSize_TYPE_SET_STEP_SIZE);
    m_message.mutable_max_step_size()->set_size(0.01);

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

    /// Convert reply message to packet message
    mw::internal::robotics::gazebotransport::Packet reply;

    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
    }

    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error
}

// parameterized test for set joint-link as well as
// read joint-link state commands
TEST_P(pluginTest, testLinkJointStateFunctionalities) {
    // get parameter for test
    auto model_jointLink = GetParam();
    // type of error message
    errorMsgValidation error_type = std::get<3>(model_jointLink);
    // type of test command
    commandSelect testCommand = std::get<2>(model_jointLink);

    switch (testCommand) {
    case SetJointPosition: {
        // It tests success of the set joint position by
        // reply message with NO Error.
        // Further, It tests reply message for invalid model name
        // and invalid joint name as input with respective
        // error message
        mw::internal::robotics::gazebotransport::Packet reply = clientSetJointPosition(
            std::get<0>(model_jointLink), std::get<1>(model_jointLink), 0, 1.0, 0, 0);

        validateSetCommandErrorMsg(reply, error_type);

        break;
    }

    case SetJointVelocity: {
        // It tests success of the set joint velocity by
        // reply message with NO Error.
        // Further, It tests reply message for invalid model name
        // and invalid joint name as input with respective
        // error message
        mw::internal::robotics::gazebotransport::Packet reply = clientSetJointVelocity(
            std::get<0>(model_jointLink), std::get<1>(model_jointLink), 0, 1.0, 0, 0);

        validateSetCommandErrorMsg(reply, error_type);

        break;
    }

    case SetLinkWorldPose: {
        // It tests success of the set link world pose by
        // reply message with NO Error.
        // Further, It tests reply message for invalid model name
        // and invalid link name as input with respective
        // error message
        double pose[7] = {3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

        mw::internal::robotics::gazebotransport::Packet reply = clientSetLinkWorldPose(
            std::get<0>(model_jointLink), std::get<1>(model_jointLink), pose, 0, 10000000);

        validateSetCommandErrorMsg(reply, error_type);

        break;
    }

    case SetLinkLinearVelocity: {
        // It tests success of the set link linear velocity by
        // reply message with NO Error.
        // Further, It tests reply message for invalid model name
        // and invalid link name as input with respective
        // error message
        double velocity[3] = {3.0, 0.0, 0.0};

        mw::internal::robotics::gazebotransport::Packet reply = clientSetLinkLinearVelocity(
            std::get<0>(model_jointLink), std::get<1>(model_jointLink), velocity, 0, 10000000);

        validateSetCommandErrorMsg(reply, error_type);

        break;
    }

    case SetLinkAngularVelocity: {
        // It tests success of the set link angular velocity by
        // reply message with NO Error.
        // Further, It tests reply message for invalid model name
        // and invalid link name as input with respective
        // error message
        double velocity[3] = {3.0, 0.0, 0.0};

        mw::internal::robotics::gazebotransport::Packet reply = clientSetLinkAngularVelocity(
            std::get<0>(model_jointLink), std::get<1>(model_jointLink), velocity, 0, 10000000);

        validateSetCommandErrorMsg(reply, error_type);

        break;
    }

    case GetJointState: {
        // It tests success of the get joint state by
        // reply message with respective joint state details.
        // Further, It tests reply message for invalid model name
        // and invalid joint name as input with all zero values of
        // joint state.

        // Reset gazebo scene and time
        mw::internal::robotics::gazebotransport::Packet reply = clientResetSceneTime();

        bool success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
        }
        /// it should be true as reset done
        ASSERT_TRUE(success);

        reply = clientGetJointState(std::get<0>(model_jointLink), std::get<1>(model_jointLink));

        if (error_type == Success) {
            // ground truth values
            std::string model_name = "unit_box";
            std::string joint_name = "joint";
            uint32_t joint_id = 65375;
            std::vector<double> joint_position = {0.0};
            std::vector<double> joint_velocity = {0.0};
            int32_t joint_type = 1;
            std::string const parent_name = "link";
            uint32_t parent_id = 10;
            std::string const child_name = "link";
            uint32_t child_id = 10;
            std::vector<double> initial_anchor_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
            std::vector<double> world_pose = {0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0};
            std::vector<double> parent_world_pose = {0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0};
            std::vector<double> axis1 = {0.0, 0.0, 0.0, -1e+16, 1e+16, -1.0, -1.0, 0.0, 0.0, 0.0};
            std::vector<double> axis2 = {0.0};

            // validation
            validateJointStateResult(
                reply, model_name.c_str(), joint_name.c_str(), joint_id, joint_position,
                joint_velocity, joint_type, parent_name.c_str(), parent_id, child_name.c_str(),
                child_id, initial_anchor_pose, world_pose, parent_world_pose, axis1, axis2);
        } else {
            // ground truth values
            std::string model_name = "";
            std::string joint_name = "";
            uint32_t joint_id = 0;
            std::vector<double> joint_position = {0.0};
            std::vector<double> joint_velocity = {0.0};
            int32_t joint_type = 0;
            std::string const parent_name = "";
            uint32_t parent_id = 0;
            std::string const child_name = "";
            uint32_t child_id = 0;
            std::vector<double> initial_anchor_pose = {0.0};
            std::vector<double> world_pose = {0.0};
            std::vector<double> parent_world_pose = {0.0};
            std::vector<double> axis1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            std::vector<double> axis2 = {0.0};

            // validation
            validateJointStateResult(
                reply, model_name.c_str(), joint_name.c_str(), joint_id, joint_position,
                joint_velocity, joint_type, parent_name.c_str(), parent_id, child_name.c_str(),
                child_id, initial_anchor_pose, world_pose, parent_world_pose, axis1, axis2);
        }
        break;
    }

    case GetLinkState: {
        // It tests success of the get link state by
        // reply message with respective link state details.
        // Further, It tests reply message for invalid model name
        // and invalid link name as input with all zero values of
        // link state.

        // Reset gazebo scene and time
        mw::internal::robotics::gazebotransport::Packet reply = clientResetSceneTime();

        bool success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
        }
        /// it should be true as reset done
        ASSERT_TRUE(success);

        reply = clientGetLinkState(std::get<0>(model_jointLink), std::get<1>(model_jointLink));

        if (error_type == Success) {
            // ground truth values
            std::string model_name = "unit_box";
            std::string link_name = "link";
            uint32_t link_id = 10;
            std::vector<double> world_linear_velocity = {0.0, 0.0, 0.0};
            std::vector<double> world_angular_velocity = {0.0, 0.0, 0.0};
            std::vector<double> relative_linear_velocity = {0.0, 0.0, 0.0};
            std::vector<double> relative_angular_velocity = {0.0, 0.0, 0.0};
            std::vector<double> world_pose = {0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0};
            std::vector<double> relative_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
            bool self_collide = false;
            bool gravity = true;
            bool kinematic = false;
            bool enable_wind = false;
            bool canonical = true;
            // validation
            validateLinkStateResult(reply, model_name.c_str(), link_name.c_str(), link_id,
                                    world_linear_velocity, world_angular_velocity,
                                    relative_linear_velocity, relative_angular_velocity, world_pose,
                                    relative_pose, self_collide, gravity, kinematic, enable_wind,
                                    canonical);
        } else {
            // ground truth values
            std::string model_name = "";
            std::string link_name = "";
            uint32_t link_id = 0;
            std::vector<double> world_linear_velocity = {0.0, 0.0, 0.0};
            std::vector<double> world_angular_velocity = {0.0, 0.0, 0.0};
            std::vector<double> relative_linear_velocity = {0.0, 0.0, 0.0};
            std::vector<double> relative_angular_velocity = {0.0, 0.0, 0.0};
            std::vector<double> world_pose = {0.0};
            std::vector<double> relative_pose = {0.0};
            bool self_collide = false;
            bool gravity = false;
            bool kinematic = false;
            bool enable_wind = false;
            bool canonical = false;
            // validation
            validateLinkStateResult(reply, model_name.c_str(), link_name.c_str(), link_id,
                                    world_linear_velocity, world_angular_velocity,
                                    relative_linear_velocity, relative_angular_velocity, world_pose,
                                    relative_pose, self_collide, gravity, kinematic, enable_wind,
                                    canonical);
        }
        break;
    }
    }
}

// test parameter initialization for joint-link state functionalities
INSTANTIATE_TEST_CASE_P(
    pluginTestForJointLinkState,
    pluginTest,
    ::testing::Values(std::make_tuple("unit_box", "joint", SetJointPosition, Success),
                      std::make_tuple("unit_box", "joint0", SetJointPosition, InvalidJoint),
                      std::make_tuple("unit_box0", "joint", SetJointPosition, InvalidModel),
                      std::make_tuple("unit_box", "joint", SetJointVelocity, Success),
                      std::make_tuple("unit_box", "joint0", SetJointVelocity, InvalidJoint),
                      std::make_tuple("unit_box0", "joint", SetJointVelocity, InvalidModel),
                      std::make_tuple("unit_box", "link", SetLinkWorldPose, Success),
                      std::make_tuple("unit_box", "link0", SetLinkWorldPose, InvalidLink),
                      std::make_tuple("unit_box0", "link", SetLinkWorldPose, InvalidModel),
                      std::make_tuple("unit_box", "link", SetLinkLinearVelocity, Success),
                      std::make_tuple("unit_box", "link0", SetLinkLinearVelocity, InvalidLink),
                      std::make_tuple("unit_box0", "link", SetLinkLinearVelocity, InvalidModel),
                      std::make_tuple("unit_box", "link", SetLinkAngularVelocity, Success),
                      std::make_tuple("unit_box", "link0", SetLinkAngularVelocity, InvalidLink),
                      std::make_tuple("unit_box0", "link", SetLinkAngularVelocity, InvalidModel),
                      std::make_tuple("unit_box", "joint", GetJointState, Success),
                      std::make_tuple("unit_box", "joint0", GetJointState, InvalidJoint),
                      std::make_tuple("unit_box0", "joint", GetJointState, InvalidModel),
                      std::make_tuple("unit_box", "link", GetLinkState, Success),
                      std::make_tuple("unit_box", "link0", GetLinkState, InvalidLink),
                      std::make_tuple("unit_box0", "link", GetLinkState, InvalidModel)));

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
