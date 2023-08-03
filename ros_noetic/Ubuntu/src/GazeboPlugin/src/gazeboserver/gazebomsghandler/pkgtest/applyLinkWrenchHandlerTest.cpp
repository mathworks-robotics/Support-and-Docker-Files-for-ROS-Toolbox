/* Copyright 2019 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/ApplyLinkWrenchMsgHandler.hpp"

class applyLinkWrenchTest : public testing::Test {
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

        /// Launch ApplyLinkWrenchMsgHandler message handler
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::ApplyLinkWrenchMsgHandler>(m_world,
                                                                                   m_commander));

        // Starts thread for simulation begin update
        m_worldUpdateStartEventConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&applyLinkWrenchTest::onWorldUpdateStart, this));

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
 It tests success of the apply link force/torque
 Also, it tests the client successfully gets the NO Error message
*/
TEST_F(applyLinkWrenchTest, testForceTorqueAddSuccess) {
    m_world->Run();
    m_world->SetPaused(true);
    m_world->ResetTime();

    // apply force and torque on z axis
    double force_values[3] = {0.0, 0.0, 1.0};
    double torque_values[3] = {0.0, 0.0, 1.0};

    std::string modelName = "unit_box";
    std::string linkName = "link";
    auto model = m_world->ModelByName(modelName);
    auto link = model->GetLink(linkName);

    // apply force and torque for 0.001 second and check for the next iteration
    mw::internal::robotics::gazebotransport::Packet reply = clientApplyLinkWrench(
        modelName, linkName, "ADD", "ADD", force_values, torque_values, 0, 1000000);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);

    // step simulator for 1 second and check for terminal velocity
    m_world->Step(1);
    EXPECT_DOUBLE_EQ(link->WorldForce().X(), force_values[0]);
    EXPECT_DOUBLE_EQ(link->WorldForce().Y(), force_values[1]);
    EXPECT_DOUBLE_EQ(link->WorldForce().Z(), force_values[2]);
    EXPECT_DOUBLE_EQ(link->WorldTorque().X(), torque_values[0]);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Y(), torque_values[1]);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Z(), torque_values[2]);

    // check for next step and force/torque should be 0
    m_world->Step(1);
    EXPECT_DOUBLE_EQ(link->WorldForce().X(), 0);
    EXPECT_DOUBLE_EQ(link->WorldForce().Y(), 0);
    EXPECT_DOUBLE_EQ(link->WorldForce().Z(), 0);
    EXPECT_DOUBLE_EQ(link->WorldTorque().X(), 0);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Y(), 0);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Z(), 0);
}

/*
 It tests success of the apply link force/torque
 Also, it tests the client successfully gets the NO Error message
*/
TEST_F(applyLinkWrenchTest, testForceTorqueSetSuccess) {
    m_world->Run();
    m_world->SetPaused(true);
    m_world->ResetTime();

    // apply force and torque on z axis
    double force_values[3] = {0.0, 0.0, 1.0};
    double torque_values[3] = {0.0, 0.0, 1.0};

    std::string modelName = "unit_box";
    std::string linkName = "link";
    auto model = m_world->ModelByName(modelName);
    auto link = model->GetLink(linkName);

    // apply force and torque for 0.001 second and check for the next iteration
    mw::internal::robotics::gazebotransport::Packet reply = clientApplyLinkWrench(
        modelName, linkName, "SET", "SET", force_values, torque_values, 0, 1000000);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);

    // step simulator for 1 second and check for terminal velocity
    m_world->Step(1);
    EXPECT_DOUBLE_EQ(link->WorldForce().X(), force_values[0]);
    EXPECT_DOUBLE_EQ(link->WorldForce().Y(), force_values[1]);
    EXPECT_DOUBLE_EQ(link->WorldForce().Z(), force_values[2]);
    EXPECT_DOUBLE_EQ(link->WorldTorque().X(), torque_values[0]);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Y(), torque_values[1]);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Z(), torque_values[2]);

    // check for next step and force/torque should be 0
    m_world->Step(1);
    EXPECT_DOUBLE_EQ(link->WorldForce().X(), 0);
    EXPECT_DOUBLE_EQ(link->WorldForce().Y(), 0);
    EXPECT_DOUBLE_EQ(link->WorldForce().Z(), 0);
    EXPECT_DOUBLE_EQ(link->WorldTorque().X(), 0);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Y(), 0);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Z(), 0);
}

/*
 It tests success of the add link force/torque twice
 Also, it tests the client successfully gets the NO Error message
*/
TEST_F(applyLinkWrenchTest, testForceTorqueAddTwiceSuccess) {
    m_world->Run();
    m_world->SetPaused(true);
    m_world->ResetTime();

    // apply force and torque on z axis
    double force_values[3] = {0.0, 0.0, 1.0};
    double torque_values[3] = {0.0, 0.0, 1.0};

    std::string modelName = "unit_box";
    std::string linkName = "link";
    auto model = m_world->ModelByName(modelName);
    auto link = model->GetLink(linkName);

    // apply force and torque for 0.001 second and check for the next iteration
    mw::internal::robotics::gazebotransport::Packet reply = clientApplyLinkWrench(
        modelName, linkName, "ADD", "ADD", force_values, torque_values, 0, 1000000);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);
    reply = clientApplyLinkWrench(modelName, linkName, "ADD", "ADD", force_values, torque_values, 0,
                                  1000000);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);

    // step simulator for 1 second and check for terminal velocity
    m_world->Step(1);
    EXPECT_DOUBLE_EQ(link->WorldForce().X(), 2 * force_values[0]);
    EXPECT_DOUBLE_EQ(link->WorldForce().Y(), 2 * force_values[1]);
    EXPECT_DOUBLE_EQ(link->WorldForce().Z(), 2 * force_values[2]);
    EXPECT_DOUBLE_EQ(link->WorldTorque().X(), 2 * torque_values[0]);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Y(), 2 * torque_values[1]);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Z(), 2 * torque_values[2]);

    // check for next step and force/torque should be 0
    m_world->Step(1);
    EXPECT_DOUBLE_EQ(link->WorldForce().X(), 0);
    EXPECT_DOUBLE_EQ(link->WorldForce().Y(), 0);
    EXPECT_DOUBLE_EQ(link->WorldForce().Z(), 0);
    EXPECT_DOUBLE_EQ(link->WorldTorque().X(), 0);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Y(), 0);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Z(), 0);
}

/*
 It tests success of the set link force/torque twice
 Also, it tests the client successfully gets the NO Error message
*/
TEST_F(applyLinkWrenchTest, testForceTorqueSetTwiceSuccess) {
    m_world->Run();
    m_world->SetPaused(true);
    m_world->ResetTime();

    // apply force and torque on z axis
    double force_values[3] = {0.0, 0.0, 1.0};
    double torque_values[3] = {0.0, 0.0, 1.0};

    std::string modelName = "unit_box";
    std::string linkName = "link";
    auto model = m_world->ModelByName(modelName);
    auto link = model->GetLink(linkName);

    // apply force and torque for 0.001 second and check for the next iteration
    mw::internal::robotics::gazebotransport::Packet reply = clientApplyLinkWrench(
        modelName, linkName, "SET", "SET", force_values, torque_values, 0, 1000000);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);

    force_values[0] = 1;
    reply = clientApplyLinkWrench(modelName, linkName, "SET", "SET", force_values, torque_values, 0,
                                  1000000);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);

    // step simulator for 1 second and check for terminal velocity
    m_world->Step(1);
    EXPECT_DOUBLE_EQ(link->WorldForce().X(), force_values[0]);
    EXPECT_DOUBLE_EQ(link->WorldForce().Y(), force_values[1]);
    EXPECT_DOUBLE_EQ(link->WorldForce().Z(), force_values[2]);
    EXPECT_DOUBLE_EQ(link->WorldTorque().X(), torque_values[0]);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Y(), torque_values[1]);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Z(), torque_values[2]);

    // check for next step and force/torque should be 0
    m_world->Step(1);
    EXPECT_DOUBLE_EQ(link->WorldForce().X(), 0);
    EXPECT_DOUBLE_EQ(link->WorldForce().Y(), 0);
    EXPECT_DOUBLE_EQ(link->WorldForce().Z(), 0);
    EXPECT_DOUBLE_EQ(link->WorldTorque().X(), 0);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Y(), 0);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Z(), 0);
}

/*
 It tests success of the add and set link force/torque
 Also, it tests the client successfully gets the NO Error message
*/
TEST_F(applyLinkWrenchTest, testForceTorqueAddSetSuccess) {
    m_world->Run();
    m_world->SetPaused(true);
    m_world->ResetTime();

    // apply force and torque on z axis
    double force_values[3] = {0.0, 0.0, 1.0};
    double torque_values[3] = {0.0, 0.0, 1.0};

    std::string modelName = "unit_box";
    std::string linkName = "link";
    auto model = m_world->ModelByName(modelName);
    auto link = model->GetLink(linkName);

    // apply force and torque for 0.001 second and check for the next iteration
    mw::internal::robotics::gazebotransport::Packet reply = clientApplyLinkWrench(
        modelName, linkName, "ADD", "ADD", force_values, torque_values, 0, 1000000);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);
    reply = clientApplyLinkWrench(modelName, linkName, "SET", "SET", force_values, torque_values, 0,
                                  1000000);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);

    // step simulator for 1 second and check for terminal velocity
    m_world->Step(1);
    EXPECT_DOUBLE_EQ(link->WorldForce().X(), force_values[0]);
    EXPECT_DOUBLE_EQ(link->WorldForce().Y(), force_values[1]);
    EXPECT_DOUBLE_EQ(link->WorldForce().Z(), force_values[2]);
    EXPECT_DOUBLE_EQ(link->WorldTorque().X(), torque_values[0]);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Y(), torque_values[1]);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Z(), torque_values[2]);

    // check for next step and force/torque should be 0
    m_world->Step(1);
    EXPECT_DOUBLE_EQ(link->WorldForce().X(), 0);
    EXPECT_DOUBLE_EQ(link->WorldForce().Y(), 0);
    EXPECT_DOUBLE_EQ(link->WorldForce().Z(), 0);
    EXPECT_DOUBLE_EQ(link->WorldTorque().X(), 0);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Y(), 0);
    EXPECT_DOUBLE_EQ(link->WorldTorque().Z(), 0);
}

/*
 * It tests reply message for invalid model name as input.
 * The client should successfully receive the MODEL_NAME_INVALID error message
 */
TEST_F(applyLinkWrenchTest, testInValidModelName) {
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
TEST_F(applyLinkWrenchTest, testInValidLinkName) {
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
TEST_F(applyLinkWrenchTest, testInValidForceType) {
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
TEST_F(applyLinkWrenchTest, testInValidTorqueType) {
    double force_values[3] = {3000.0, 0.0, 0.0};
    double torque_values[3] = {3000.0, 0.0, 0.0};

    mw::internal::robotics::gazebotransport::Packet reply =
        clientApplyLinkWrench("unit_box", "link", "ADD", "AD", force_values, torque_values, 0, 0);

    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_TORQUE_TYPE_INVALID); // TORQUE_TYPE_INVALID error
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
