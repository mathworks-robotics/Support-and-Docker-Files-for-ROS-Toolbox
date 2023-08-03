/* Copyright 2019 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetTopicListMsgHandler.hpp"
#include "gazebo/gazebo_client.hh"
#include "gazebo/gazebo.hh"

class getTopicList : public testing::Test {
  public:
    std::string ipAddress = "127.0.0.1";
    /// Creating GazeboServer and Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    std::shared_ptr<robotics::gazebotransport::GazeboServer> m_server;

    void SetUp() {
        /// Launch & Run GazeboServer
        m_server = std::make_shared<robotics::gazebotransport::GazeboServer>(
            static_cast<uint16_t>(std::stoi("0")));
        m_server->run();

        /// Launch Client
        m_client = std::make_shared<robotics::gazebotransport::Client>(
            ipAddress, m_server->getPortName(), boost::posix_time::milliseconds(10000));

        /// Start Gazebo Simulator Client
        gazebo::client::setup();

        /// Launch GetTopicList message handler
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::GetTopicListMsgHandler>());

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    void TearDown() {
        /// Closes server and client
        m_client->shutdown();
        m_server->stop();
        gazebo::shutdown();
    }
};

/*
 * This tests request topic list by sending GET_TOPIC_LIST message.
 * Further, it receives topic lists from server.
 * Few default topic lists are compared with the received topic list.
 */
TEST_F(getTopicList, testTopicList) {
    /// Create topic list request message
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_GET_TOPIC_LIST);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_get_topic_list()->set_topic_name("~/GetTopicList");

    /// Client sends topic request message & receives the list of topic from server
    mw::internal::robotics::gazebotransport::Packet reply;
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));
    if (replyMsg) {
        /// Converts reply message into Packet message
        reply.ParseFromString(*replyMsg);
    }

    /// Check with default topics available in Gazebo
    /// These are few default topic lists, which should be same in the received data.
    // ASSERT_STREQ("/gazebo/default/atmosphere", reply.topic_list().data(0).name().c_str());
    // ///Gecko #g1990794
    ASSERT_STREQ("/gazebo/default/physics/contacts", reply.topic_list().data(0).name().c_str());
    // ASSERT_STREQ("/gazebo/default/diagnostics", reply.topic_list().data(2).name().c_str());
    // ///Gecko #g1990794 ASSERT_STREQ("/gazebo/default/factory",
    // reply.topic_list().data(3).name().c_str()); ASSERT_STREQ("/gazebo/default/gui",
    // reply.topic_list().data(4).name().c_str());         ///Gecko #g1990794
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
