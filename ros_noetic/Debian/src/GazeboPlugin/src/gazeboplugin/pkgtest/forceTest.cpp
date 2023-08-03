/* Copyright 2019 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/Client.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

#include <mutex>

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

class forceTest : public testing::Test {
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

        std::this_thread::sleep_for(std::chrono::microseconds(100000));
    }

    void TearDown() {
        m_client->shutdown();

        gazebo::shutdown();
        std::this_thread::sleep_for(std::chrono::microseconds(50000));
    }
};


/*
 * It tests success of the apply joint torque
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(forceTest, testApplyJointSuccess) {
    mw::internal::robotics::gazebotransport::Packet reply =
        clientApplyJointTorque("unit_box", "joint", 0, 100000.0, 0, 10000000);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error
}

/*
 * It tests success of the apply link force/torque
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(forceTest, testApplyLinkWrenchSuccess) {
    double force_values[3] = {0.0, 0.0, 25.0};
    double torque_values[3] = {0.0, 0.0, 0.0};

    mw::internal::robotics::gazebotransport::Packet reply =
        clientApplyLinkWrench("unit_box", "link", "SET", "SET", force_values, torque_values, 1, 0);

    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // NO Error

    // step gazebo by one
    for (int i = 0; i < 5000; i++) {

        reply = clientStepSimulation(1);
    }

    bool m_success = false;
    if (reply.has_status() && !reply.status()) {
        m_success = true;
    }
    /// it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
