/* Copyright 2019-2022 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetImageMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SubscribeImageMsgHandler.hpp"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/gazebo.hh"

class imagePublish : public testing::Test {

  public:
    std::string ipAddress = "127.0.0.1";

    /// Small image size parameters
    int imgSmallWidth = 100 * 3;
    int imgSmallHeight = 100;
    uint8_t smallImagePixel = 128;

    /// Large image size parameters
    int imgLargeWidth = 1200 * 3;
    int imgLargeHeight = 1200;
    uint8_t largeImagePixel = 132;

    /// Session Time-out value
    int time_out = 5000;

    /// Creating GazeboServer and Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    std::shared_ptr<robotics::gazebotransport::GazeboServer> m_server;

    // Gazebo Node pointer
    gazebo::transport::NodePtr nodeSub;

    /// Gazebo world pointer
    gazebo::physics::WorldPtr world;

    /// Threads to publish image sensor 0 and image sensor 1 data
    std::shared_ptr<std::thread> th0;
    std::shared_ptr<std::thread> th1;

    /// Keep publisher threads (th0 & th1 ) alive
    int clientEnd = 1;

    // Indicates publishing of image sensor 0 and image sensor 1 is started
    bool startedPubImage0 = false;
    bool startedPubImage1 = false;

    /**
     * @param width          Image width value
     * @param height         Image height value
     * @param ImageDepth     Image pixel bits value
     * @param typeString     Image pixel data value
     *
     * This function creates ImageStamped data, which is used by image sensor publisher
     */
    gazebo::msgs::ImageStamped getImageMessage(int& width,
                                               int& height,
                                               int& ImageDepth,
                                               std::string const& dataString) {
        // creates image message
        gazebo::msgs::ImageStamped imgMsg;
        gazebo::common::Time timestamp = world->SimTime();
        gazebo::msgs::Set(imgMsg.mutable_time(), timestamp);

        imgMsg.mutable_image()->set_width(width);
        imgMsg.mutable_image()->set_height(height);
        imgMsg.mutable_image()->set_pixel_format(gazebo::common::Image::L_INT8);
        imgMsg.mutable_image()->set_step(width * ImageDepth);
        imgMsg.mutable_image()->set_data(dataString);

        return imgMsg;
    }

    /**
     * It publishes image with small size. The topic name is initialized inside this function.
     */
    void smallImagePublisher() {
        /// Initialize gazebo node
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();

        /// Start gazebo publisher
        gazebo::transport::PublisherPtr pub =
            node->Advertise<gazebo::msgs::ImageStamped>("/gazebo/default/camera/link/camera/image");

        int ImageDepth = 8;

        /// Initialize string with smallImagePixel pixel value
        std::string imageString;
        for (int i = 0; i < imgSmallWidth * imgSmallHeight; ++i) {
            imageString += (char)smallImagePixel;
        }
        /// Publish small image data in a loop
        while (this->clientEnd) {
            // Get & publish ImageStamped data
            gazebo::msgs::ImageStamped imgMsg =
                getImageMessage(imgSmallWidth, imgSmallHeight, ImageDepth, imageString);
            pub->Publish(imgMsg);
            this->startedPubImage0 = true;
            std::this_thread::sleep_for(std::chrono::microseconds(10000));
        }
    }

    /**
     * It publishes image with large size. The topic name is initialized inside this function.
     */
    void largeImagePublisher() {
        /// Initialize gazebo node
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();

        /// Start gazebo publisher
        gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::ImageStamped>(
            "/gazebo/default/camera1/link/camera/image");

        int ImageDepth = 8;
        /// Initialize string with largeImagePixel pixel value
        std::string imageString;
        for (int i = 0; i < imgLargeWidth * imgLargeHeight; ++i) {
            imageString += (char)largeImagePixel;
        }
        /// Publish large image data in a loop
        while (this->clientEnd) {
            // Get & publish ImageStamped data
            gazebo::msgs::ImageStamped imgMsg =
                getImageMessage(imgLargeWidth, imgLargeHeight, ImageDepth, imageString);
            pub->Publish(imgMsg);
            this->startedPubImage1 = true;
            std::this_thread::sleep_for(std::chrono::microseconds(10000));
        }
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

        /// Convert reply message to packet message
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
        world = gazebo::loadWorld("world/unitBoxHandlerTest.world");
        ASSERT_TRUE(world != nullptr) << "Failed to load world";
        world->SetPaused(true);

        /// Launch IMAGE data publisher
        th0 = std::make_shared<std::thread>(&imagePublish::smallImagePublisher, this);
        th1 = std::make_shared<std::thread>(&imagePublish::largeImagePublisher, this);

        // Ensure the IMAGE pub is started
        while (!this->startedPubImage0 && !this->startedPubImage1) {
            std::this_thread::sleep_for(std::chrono::microseconds(50000));
        }

        /// Init Node, SubPtr and GazeboMsgRepo for msgHandler
        nodeSub = gazebo::transport::NodePtr(new gazebo::transport::Node());
        nodeSub->Init();

        /// Message handler starts
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::SubscribeImageMsgHandler>(world, nodeSub));
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::GetImageMsgHandler>(world));

        std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }

    void TearDown() {
        //*********************************************
        /// Close threads
        this->clientEnd = 0;
        th0->join();
        th1->join();
        std::this_thread::sleep_for(std::chrono::microseconds(10000));
        /// Closes server and client
        m_client->shutdown();
        m_server->stop();
        world->Stop();
        gazebo::shutdown();
        std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }
};

/*
 * Subscribe and Get small Image sensor data
 * It tests the small Image sensor is successfully subscribed or not.
 * Also, it tests the client successfully gets the Image data or not.
 * The verifications are done with ground truth values.
 */
TEST_F(imagePublish, testSmallImageSensor) {
    world->Run();
    world->SetPaused(true);
    world->ResetTime();

    //*********************************************
    /// TEST Small Image Subscriber
    mw::internal::robotics::gazebotransport::Packet reply0 =
        clientSubscribeImage("/gazebo/default/camera/link/camera/image");

    bool m_success0 = false;

    if (reply0.has_status() && !reply0.status()) {
        m_success0 = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success0);

    std::this_thread::sleep_for(std::chrono::microseconds(100000));

    //*********************************************
    /// TEST Get Small Image first time

    mw::internal::robotics::gazebotransport::Packet replyGetImage =
        clientGetImage("/gazebo/default/camera/link/camera/image");

    // Verify the received image data with ground truth values.
    ASSERT_EQ(replyGetImage.image().width(), imgSmallWidth);
    ASSERT_EQ(replyGetImage.image().height(), imgSmallHeight);
    ASSERT_EQ(replyGetImage.image().data_type(), "L_INT8");


    std::string buffSmall = replyGetImage.image().data();
    std::vector<unsigned char> imageSmalldata(buffSmall.begin(), buffSmall.end());

    for (int i = 0; i < imgSmallWidth * imgSmallHeight; i++) {
        ASSERT_EQ((int)imageSmalldata[i], smallImagePixel);
    }

    //*********************************************
    /// TEST Get Small Image again without stepping
    /// i.e. image message contains old update
    replyGetImage = clientGetImage("/gazebo/default/camera/link/camera/image");

    // The valid message send by Camera Sensor handler,
    // as there is no stepping and same topic called again.
    ASSERT_TRUE(replyGetImage.has_image());

    // Verify the received image data with ground truth values.
    ASSERT_EQ(replyGetImage.image().width(), imgSmallWidth);
    ASSERT_EQ(replyGetImage.image().height(), imgSmallHeight);
    ASSERT_EQ(replyGetImage.image().data_type(), "L_INT8");

    //*********************************************
    /// TEST Get Small Image again with stepping
    /// i.e. image message contains new update

    // step and get image
    world->Step(100);
    replyGetImage = clientGetImage("/gazebo/default/camera/link/camera/image");

    // Verify the received image data with ground truth values.
    ASSERT_EQ(replyGetImage.image().width(), imgSmallWidth);
    ASSERT_EQ(replyGetImage.image().height(), imgSmallHeight);
    ASSERT_EQ(replyGetImage.image().data_type(), "L_INT8");

    buffSmall = replyGetImage.image().data();
    std::vector<unsigned char> imageSmalldata1(buffSmall.begin(), buffSmall.end());

    for (int i = 0; i < imgSmallWidth * imgSmallHeight; i++) {
        ASSERT_EQ((int)imageSmalldata1[i], smallImagePixel);
    }

    std::this_thread::sleep_for(std::chrono::microseconds(100000));
}

/*
 * It tests, if the image sensor( topic) is not available in Gazebo
 * & user subscribing that topic. Then, it should return FALSE.
 */
TEST_F(imagePublish, testInValidTopic) {
    //*********************************************
    /// TEST Wrong Image Subscriber
    mw::internal::robotics::gazebotransport::Packet reply0 =
        clientSubscribeImage("/gazebo/default/camera0/link/camera/image");

    bool m_success0 = false;

    if (reply0.has_status() && !reply0.status()) {
        m_success0 = true;
    }
    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(m_success0);
    ASSERT_EQ(reply0.status(), mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                   Packet_CoSimError_TOPIC_NAME_INVALID);

    std::this_thread::sleep_for(std::chrono::microseconds(100000));
}

/*
 * It tests, if the image sensor( topic) is not available in Gazebo
 * & user trying to get image data from that topic.
 * Then, all image sensor message fields should zero.
 */
TEST_F(imagePublish, testGetInvalidImage) {
    //*********************************************
    /// TEST Get Invalid Image

    mw::internal::robotics::gazebotransport::Packet replyGetWrongImage =
        clientGetImage("/gazebo/default/camera0/link/camera/image");

    bool m_success0 = false;

    if (replyGetWrongImage.has_status() && !replyGetWrongImage.status()) {
        m_success0 = true;
    }
    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(m_success0);
    ASSERT_EQ(replyGetWrongImage.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_TOPIC_NAME_INVALID);

    std::this_thread::sleep_for(std::chrono::microseconds(100000));
}

/*
 * Subscribe and Get Large Image Sensor data
 * It tests the Large Image Sensor is successfully subscribed or not.
 * Also, it tests the client successfully gets the Large Image Sensor data or not.
 * The verifications are done with ground truth values.
 */
TEST_F(imagePublish, testLargeImage) {

    world->Run();
    world->SetPaused(true);
    world->ResetTime();

    //*********************************************
    /// TEST Large Image Subscriber
    mw::internal::robotics::gazebotransport::Packet reply0 =
        clientSubscribeImage("/gazebo/default/camera1/link/camera/image");

    bool m_success1 = false;

    if (reply0.has_status() && !reply0.status()) {
        m_success1 = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success1);

    std::this_thread::sleep_for(std::chrono::microseconds(100000));

    //*********************************************
    /// TEST Get Large Image first time

    mw::internal::robotics::gazebotransport::Packet replyGetLargeImg =
        clientGetImage("/gazebo/default/camera1/link/camera/image");

    /// Verify the received image data with ground truth values.
    ASSERT_EQ(replyGetLargeImg.image().width(), imgLargeWidth);
    ASSERT_EQ(replyGetLargeImg.image().height(), imgLargeHeight);
    ASSERT_EQ(replyGetLargeImg.image().data_type(), "L_INT8");

    std::string buffLarge = replyGetLargeImg.image().data();
    std::vector<unsigned char> image_dataLarge(buffLarge.begin(), buffLarge.end());

    for (int i = 0; i < imgLargeWidth * imgLargeHeight; i++) {
        ASSERT_EQ((int)image_dataLarge[i], largeImagePixel);
    }

    //*********************************************
    /// TEST Get Large Image again without stepping
    /// i.e. image message contains old update
    replyGetLargeImg = clientGetImage("/gazebo/default/camera1/link/camera/image");

    // The valid message send by Camera Sensor handler,
    // as there is no stepping and same topic called again.
    ASSERT_TRUE(replyGetLargeImg.has_image());

    /// Verify the received image data with ground truth values.
    ASSERT_EQ(replyGetLargeImg.image().width(), imgLargeWidth);
    ASSERT_EQ(replyGetLargeImg.image().height(), imgLargeHeight);
    ASSERT_EQ(replyGetLargeImg.image().data_type(), "L_INT8");

    //*********************************************
    /// TEST Get Large Image again with stepping
    /// i.e. image message contains new update

    // step and get image
    world->Step(100);

    replyGetLargeImg = clientGetImage("/gazebo/default/camera1/link/camera/image");

    /// Verify the received image data with ground truth values.
    ASSERT_EQ(replyGetLargeImg.image().width(), imgLargeWidth);
    ASSERT_EQ(replyGetLargeImg.image().height(), imgLargeHeight);
    ASSERT_EQ(replyGetLargeImg.image().data_type(), "L_INT8");

    buffLarge = replyGetLargeImg.image().data();
    std::vector<unsigned char> image_dataLarge1(buffLarge.begin(), buffLarge.end());

    for (int i = 0; i < imgLargeWidth * imgLargeHeight; i++) {
        ASSERT_EQ((int)image_dataLarge1[i], largeImagePixel);
    }

    std::this_thread::sleep_for(std::chrono::microseconds(100000));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
