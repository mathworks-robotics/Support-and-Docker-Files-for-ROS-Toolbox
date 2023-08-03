/* Copyright 2020 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetLinkStateMsgHandler.hpp"

class getLinkStateTest : public testing::Test {

  public:
    std::string ipAddress = "127.0.0.1";

    /// Session Time-out value
    int time_out = 1000;

    /// Creating GazeboServer and Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    std::shared_ptr<robotics::gazebotransport::GazeboServer> m_server;

    /// Gazebo world pointer
    gazebo::physics::WorldPtr m_world;

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
     * validate received link state message with ground truth values
     * */
    void validateResult(mw::internal::robotics::gazebotransport::Packet reply,
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

    void SetUp() {
        /// Launch & Run GazeboServer
        m_server = std::make_shared<robotics::gazebotransport::GazeboServer>(
            static_cast<uint16_t>(std::stoi("0")));
        m_server->run();

        /// Launch Client
        m_client = std::make_shared<robotics::gazebotransport::Client>(
            ipAddress, m_server->getPortName(), boost::posix_time::milliseconds(time_out));

        // Start Gazebo Simulator Server
        gazebo::setupServer();

        /// Load unit-box world file
        m_world = gazebo::loadWorld("world/applyForceTorqueNoPlugin.world");
        ASSERT_TRUE(m_world != nullptr) << "Fail to load the test scenario.";

        /// Launch GetLinkState message handler
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::GetLinkStateMsgHandler>(m_world));

        m_world->Run();
        m_world->SetPaused(true);
        m_world->Reset();
    }

    void TearDown() {
        /// Closes server and client
        m_client->shutdown();
        m_server->stop();
        gazebo::shutdown();
    }
};

/*
 * It tests success of the get link state of input link of model
 * Also, it tests the client successfully gets link state of model-link
 */
TEST_F(getLinkStateTest, testFirstLink) {
    std::string modelName = "2_body";
    std::string linkName = "base";

    mw::internal::robotics::gazebotransport::Packet reply = clientGetLinkState(modelName, linkName);

    m_world->Step(1);

    // ground truth
    std::string model_name = "2_body";
    std::string link_name = "base";
    uint32_t link_id = 16;
    std::vector<double> world_linear_velocity = {0.0, 0.0, 0.0};
    std::vector<double> world_angular_velocity = {0.0, 0.0, 0.0};
    std::vector<double> relative_linear_velocity = {0.0, 0.0, 0.0};
    std::vector<double> relative_angular_velocity = {0.0, 0.0, 0.0};
    std::vector<double> world_pose = {0.0, 2.0, 0.1, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> relative_pose = {0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0};
    bool self_collide = false;
    bool gravity = true;
    bool kinematic = false;
    bool enable_wind = false;
    bool canonical = true;

    // validation
    validateResult(reply, model_name.c_str(), link_name.c_str(), link_id, world_linear_velocity,
                   world_angular_velocity, relative_linear_velocity, relative_angular_velocity,
                   world_pose, relative_pose, self_collide, gravity, kinematic, enable_wind,
                   canonical);
}

/*
 * It tests success of the get link state of input link of model
 * Also, it tests the client successfully gets link state of model-link
 */
TEST_F(getLinkStateTest, testSecondLink) {
    std::string modelName = "2_body";
    std::string linkName = "follower";

    mw::internal::robotics::gazebotransport::Packet reply = clientGetLinkState(modelName, linkName);

    m_world->Step(1);

    // ground truth
    std::string model_name = "2_body";
    std::string link_name = "follower";
    uint32_t link_id = 47;
    std::vector<double> world_linear_velocity = {0.0, 0.0, 0.0};
    std::vector<double> world_angular_velocity = {0.0, 0.0, 0.0};
    std::vector<double> relative_linear_velocity = {0.0, 0.0, 0.0};
    std::vector<double> relative_angular_velocity = {0.0, 0.0, 0.0};
    std::vector<double> world_pose = {0.0, 2.0, 0.2, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> relative_pose = {0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0};
    bool self_collide = false;
    bool gravity = true;
    bool kinematic = false;
    bool enable_wind = false;
    bool canonical = false;

    // validation
    validateResult(reply, model_name.c_str(), link_name.c_str(), link_id, world_linear_velocity,
                   world_angular_velocity, relative_linear_velocity, relative_angular_velocity,
                   world_pose, relative_pose, self_collide, gravity, kinematic, enable_wind,
                   canonical);
}

/*
 * It tests reply message for invalid model name as input.
 * The client should successfully receive link state message
 * with all required and repeated fields with zero values
 */
TEST_F(getLinkStateTest, testInvalidModelName) {
    mw::internal::robotics::gazebotransport::Packet reply =
        clientGetLinkState("4_body", "follower");

    m_world->Step(1);

    // ground truth
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
    validateResult(reply, model_name.c_str(), link_name.c_str(), link_id, world_linear_velocity,
                   world_angular_velocity, relative_linear_velocity, relative_angular_velocity,
                   world_pose, relative_pose, self_collide, gravity, kinematic, enable_wind,
                   canonical);
}

/*
 * It tests reply message for invalid link name as input.
 * The client should successfully receive link state message
 * with all required and repeated fields with zero values
 */
TEST_F(getLinkStateTest, testInvalidLinkName) {
    mw::internal::robotics::gazebotransport::Packet reply =
        clientGetLinkState("4_body", "follower0");

    m_world->Step(1);

    // ground truth
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
    validateResult(reply, model_name.c_str(), link_name.c_str(), link_id, world_linear_velocity,
                   world_angular_velocity, relative_linear_velocity, relative_angular_velocity,
                   world_pose, relative_pose, self_collide, gravity, kinematic, enable_wind,
                   canonical);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
