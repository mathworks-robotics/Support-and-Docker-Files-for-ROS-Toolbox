/* Copyright 2020 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetJointStateMsgHandler.hpp"

class getJointStateTest : public testing::Test {

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
     * validate received joint state message with ground truth values
     * */
    void validateResult(mw::internal::robotics::gazebotransport::Packet reply,
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

        /// Launch GetJointState message handler
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::GetJointStateMsgHandler>(m_world));

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
 * It tests success of the get joint state of joint containing one axis
 * Also, it tests the client successfully gets joint state of model-joint
 */
TEST_F(getJointStateTest, testOneAxisJoint) {
    std::string modelName = "2_body";
    std::string jointName = "follower_joint";

    mw::internal::robotics::gazebotransport::Packet reply =
        clientGetJointState(modelName, jointName);

    m_world->Step(1);

    // ground truth
    std::string model_name = "2_body";
    std::string joint_name = "follower_joint";
    uint32_t joint_id = 26;
    std::vector<double> joint_position0 = {0.0};
    std::vector<double> joint_velocity0 = {0.0};
    int32_t joint_type = 1;
    std::string const parent_name = "base";
    uint32_t parent_id = 16;
    std::string const child_name = "follower";
    uint32_t child_id = 18;
    std::vector<double> initial_anchor_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> world_pose = {0.0, 2.0, 0.2, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> parent_world_pose = {0.0, 2.0, 0.2, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> axis1 = {0.0, 0.0, 1.0, -1e+16, 1e+16, -1, -1, 0.0, 0.0, 0.0};
    std::vector<double> axis2 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // validation
    validateResult(reply, model_name.c_str(), joint_name.c_str(), joint_id, joint_position0,
                   joint_velocity0, joint_type, parent_name.c_str(), parent_id, child_name.c_str(),
                   child_id, initial_anchor_pose, world_pose, parent_world_pose, axis1, axis2);
}

/*
 * It tests success of the get joint state of joint containing two axis
 * Also, it tests the client successfully gets joint state of model-joint
 */
TEST_F(getJointStateTest, testTwoAxisJoint) {
    std::string modelName = "3_body";
    std::string jointName = "follower_joint";

    mw::internal::robotics::gazebotransport::Packet reply =
        clientGetJointState(modelName, jointName);

    m_world->Step(1);

    // ground truth
    std::string model_name = "3_body";
    std::string joint_name = "follower_joint";
    uint32_t joint_id = 57;
    std::vector<double> joint_position0 = {-0.0, -0.0};
    std::vector<double> joint_velocity0 = {0.0, 0.0};
    int32_t joint_type = 2;
    std::string const parent_name = "base";
    uint32_t parent_id = 50;
    std::string const child_name = "follower";
    uint32_t child_id = 52;
    std::vector<double> initial_anchor_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> world_pose = {2.0, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> parent_world_pose = {2.0, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> axis1 = {0.0, 1.0, 0.0, -1e+16, 1e+16, -1.0, -1.0, 0.0, 0.0, 0.0};
    std::vector<double> axis2;

    if (std::stod(GAZEBO_VERSION) >= 11.0 )
    {
		// default axis limits are -1e+16 to +1e+16 for Gazebo 11
        axis2 = {0.0, 0.0, 1.0, -1e+16, 1e+16, -1.0, -1.0, 0.0, 0.0, 0.0};
    }
    else
    {
        axis2 = {0.0, 0.0, 1.0, -INFINITY, INFINITY, -1.0, -1.0, 0.0, 0.0, 0.0};
    }

    // validation
    validateResult(reply, model_name.c_str(), joint_name.c_str(), joint_id, joint_position0,
                   joint_velocity0, joint_type, parent_name.c_str(), parent_id, child_name.c_str(),
                   child_id, initial_anchor_pose, world_pose, parent_world_pose, axis1, axis2);
}

/*
 * It tests reply message for invalid model name as input.
 * The client should successfully receive joint state message
 * with all required and repeated fields with zero values
 */
TEST_F(getJointStateTest, testInvalidModelName) {
    mw::internal::robotics::gazebotransport::Packet reply =
        clientGetJointState("4_body", "follower_joint");

    m_world->Step(1);

    // ground truth
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
    validateResult(reply, model_name.c_str(), joint_name.c_str(), joint_id, joint_position,
                   joint_velocity, joint_type, parent_name.c_str(), parent_id, child_name.c_str(),
                   child_id, initial_anchor_pose, world_pose, parent_world_pose, axis1, axis2);
}

/*
 * It tests reply message for invalid joint name as input.
 * The client should successfully receive joint state message
 * with all required and repeated fields with zero values
 */
TEST_F(getJointStateTest, testInvalidJointName) {
    mw::internal::robotics::gazebotransport::Packet reply =
        clientGetJointState("4_body", "follower_joint0");

    m_world->Step(1);

    // ground truth
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
    validateResult(reply, model_name.c_str(), joint_name.c_str(), joint_id, joint_position,
                   joint_velocity, joint_type, parent_name.c_str(), parent_id, child_name.c_str(),
                   child_id, initial_anchor_pose, world_pose, parent_world_pose, axis1, axis2);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
