/* Copyright 2019-2020 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SetJointPositionMsgHandler.hpp"

class setJointPositionTest : public testing::Test {
  public:
    std::string ipAddress = "127.0.0.1";

    /// Session Time-out value
    int time_out = 1000;

    /// Creating GazeboServer and Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    std::shared_ptr<robotics::gazebotransport::GazeboServer> m_server;

    /// Gazebo message repository reference
    robotics::gazebotransport::GazeboApplyCommander m_commander;

    /// Gazebo world pointer
    gazebo::physics::WorldPtr m_world;

    /// Gazebo connect a callback to the world update start signal
    gazebo::event::ConnectionPtr m_worldUpdateStartEventConnection;

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

        std::this_thread::sleep_for(std::chrono::microseconds(100000));

        /// Launch setJointPosition message handler
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::SetJointPositionMsgHandler>(m_world,
                                                                                    m_commander));

        // Starts thread for simulation begin update
        m_worldUpdateStartEventConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&setJointPositionTest::onWorldUpdateStart, this));

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    void onWorldUpdateStart() {
        m_commander.executeApplyCommands(m_world->SimTime());
    }

    void TearDown() {
        /// Closes server and client
        m_client->shutdown();
        m_server->stop();
        gazebo::shutdown();
    }
};

/*
 * It tests success of the set joint position
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(setJointPositionTest, testJointPositionSuccess) {
    // pause the world and JointPosition should be zero
    m_world->Run();
    m_world->SetPaused(true);
    m_world->Reset();

    // ground truth
    std::string modelName = "2_body";
    std::string jointName = "follower_joint";

    // retrieve model and joint
    auto model = m_world->ModelByName(modelName);
    auto joint = model->GetJoint(jointName);
    uint32_t axis = 0;

    // set JointPosition for 1 second and check for the next iteration
    double position = 1;
    uint64_t time = 1;
    mw::internal::robotics::gazebotransport::Packet reply =
        clientSetJointPosition(modelName, jointName, axis, position, time, 0);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);

    // first check for 1 msec i.e. single step and check then simulator for 1 second
    // in both cases, position should be same as set
    m_world->Step(1);
    EXPECT_DOUBLE_EQ(joint->Position(axis), position);

    m_world->Step(1000);
    EXPECT_DOUBLE_EQ(joint->Position(axis), position);
}

/*
 * It tests success of the set joint position with overlapping time frames
 */
TEST_F(setJointPositionTest, testJointPositionOverlap) {
    // pause the world and JointPosition should be zero
    m_world->Run();
    m_world->SetPaused(true);
    m_world->Reset();

    // ground truth
    std::string modelName = "2_body";
    std::string jointName = "follower_joint";

    // retrieve model and joint
    auto model = m_world->ModelByName(modelName);
    auto joint = model->GetJoint(jointName);
    uint32_t axis = 0;

    // set JointPosition for 1 second
    double position = 1;
    mw::internal::robotics::gazebotransport::Packet reply =
        clientSetJointPosition(modelName, jointName, axis, position, 1, 0);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error

    // step simulator for 0.5 second and check for terminal joint position
    m_world->Step(500);
    EXPECT_DOUBLE_EQ(joint->Position(axis), position);

    // change the position to 0.5
    position = 0.5;

    // set JointPosition for 1 second
    reply = clientSetJointPosition(modelName, jointName, axis, position, 1, 0);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error

    // step and check
    // position should be changed to new location
    m_world->Step(1);
    EXPECT_DOUBLE_EQ(joint->Position(axis), position);

    m_world->Step(1000);
    EXPECT_DOUBLE_EQ(joint->Position(axis), position);
}

/*
 * It tests success of the set joint position on two axis at the same time
 */
TEST_F(setJointPositionTest, testJointPositionTwoAxis) {
    // pause the world and JointPosition should be zero
    m_world->Run();
    m_world->SetPaused(true);
    m_world->Reset();

    // ground truth
    std::string modelName = "2_body";
    std::string jointName = "follower_joint";
    uint32_t axisA = 0;
    uint32_t axisB = 1;
    double position = 1;

    // retrieve model and joint
    auto model = m_world->ModelByName(modelName);
    auto joint = model->GetJoint(jointName);

    // set JointPosition of axis 0 for 1 second
    mw::internal::robotics::gazebotransport::Packet reply =
        clientSetJointPosition(modelName, jointName, axisA, position, 1, 0);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error

    // set JointPosition of axis 1 for 1 second
    reply = clientSetJointPosition(modelName, jointName, axisB, position, 1, 0);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error

    // single step and check for both axis
    m_world->Step(1);
    EXPECT_DOUBLE_EQ(joint->Position(axisA), position);
    EXPECT_DOUBLE_EQ(joint->Position(axisB), position);

    // multiple step, till end time and check for both axis
    m_world->Step(1000);
    EXPECT_DOUBLE_EQ(joint->Position(axisA), position);
    EXPECT_DOUBLE_EQ(joint->Position(axisB), position);
}

/*
 * It tests reply message for invalid model name as input.
 * The client should successfully receive the MODEL_NAME_INVALID error message
 */
TEST_F(setJointPositionTest, testInValidModelName) {
    mw::internal::robotics::gazebotransport::Packet reply =
        clientSetJointPosition("unit_box0", "joint", 0, 1000.0, 0, 0);

    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_MODEL_NAME_INVALID); // MODEL_NAME_INVALID
                                                         // error
}

/*
 * It tests reply message for invalid joint name as input.
 * The client should successfully receive the JOINT_NAME_INVALID error message
 */
TEST_F(setJointPositionTest, testInValidJointName) {
    mw::internal::robotics::gazebotransport::Packet reply =
        clientSetJointPosition("unit_box", "joint0", 0, 1000.0, 0, 0);

    ASSERT_EQ(reply.status(), mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                  Packet_CoSimError_JOINT_NAME_INVALID); // JOINT_NAME_INVALID error
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
