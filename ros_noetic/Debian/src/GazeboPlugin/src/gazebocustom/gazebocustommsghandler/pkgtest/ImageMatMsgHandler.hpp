/* Copyright 2019-2020 The MathWorks, Inc. */
// This function is for internal use only. It may be removed in the future.
// DO NOT EDIT!

#ifndef IMAGEMATMSGHANDLER_HPP
#define IMAGEMATMSGHANDLER_HPP
#include <memory>
#include <cstdint>
#include <string>
#include "image_mat.pb.h"
#include "gazebotransport/gazebocustom/CustomMsgHandler.hpp"
#include "gazebotransport/gazebocustom/CustomMsgDispatcher.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include <map>
#include <vector>
#include <mutex>

namespace robotics {
namespace gazebotransport {
typedef const boost::shared_ptr<const robotics::matlab::ImageMat> ImageMatCustomPtr;

/**
 * This class initiates Gazebo Subscriber for each topic name.
 * Further, it start callback, which subscribe custom message
 * and stores in the Custom Message Container.
 * */
class ImageMatSubscriber {
  public:
    /// Constructor
    /*
    @param topicName                Topic name for custom message subscriber
    @param node                     Gazebo transport node pointer
    @param customContainer          Custom message storage instance
    */
    explicit ImageMatSubscriber(std::string const& topicName)
        : m_topicName(topicName)
        , m_subscriberPtr(nullptr)
        , m_messageString("")
        , m_isNew(false)
        , m_mutex() {
    }

    /// Destructor
    virtual ~ImageMatSubscriber() {
    }

    // subscriber callback which subscribe custom message from specific topic
    // and stores in the custom message container
    void OnUpdate(robotics::gazebotransport::ImageMatCustomPtr& _msg) {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_messageString = _msg->SerializeAsString();
            m_isNew = true;
        }
    }

    // initialize gazebo subscriber pointer
    void InitImpl(gazebo::transport::NodePtr node) {
        if (!m_subscriberPtr) {
            m_subscriberPtr = node->Subscribe(m_topicName, &ImageMatSubscriber::OnUpdate, this);
        }
    }

    // set isNew field to false
    void resetMessageStatus() {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_isNew = false;
    }

    // return current status of message ( isNew )
    bool getMessageStatus() {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_isNew;
    }

    // return received message string
    std::string getMessage() {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            return m_messageString;
        }
    }

  private:
    // topic name of subscriber
    std::string m_topicName;

    // gazebo subscriber pointer
    gazebo::transport::SubscriberPtr m_subscriberPtr;

    // received message string
    std::string m_messageString;

    // status of message
    bool m_isNew;

    std::mutex m_mutex;
};

/**
 * This class is created to test custom message handler functionality.
 * The custom message handlers are created on gazebogenmsg() call
 * These handlers will be same as this.
 * The ImageMatMsgHandler initialize gazebo publisher, subscriber.
 * Further, it publish and subscribe custom message on/from gazebo.
 * It can support multiple topics of same message type.
 * */
class ImageMatMsgHandler : public CustomMsgHandler {
  public:
    /// Constructor
    /*
    @param node                     Gazebo transport node pointer
    @param customContainer          Custom message storage instance
    */
    explicit ImageMatMsgHandler(gazebo::transport::NodePtr node);

    /// Destructor
    ~ImageMatMsgHandler();

    // initialize custom message publisher
    std::string initPublisher(mw::internal::robotics::gazebotransport::Packet const& msgContent);

    // initialize custom message subscriber
    std::string initSubscriber(mw::internal::robotics::gazebotransport::Packet const& msgContent);

    // publish custom message
    std::string publishCustomMsg(mw::internal::robotics::gazebotransport::Packet const& msgContent);

    // subscribe custom message
    std::pair<bool, std::string> subscribeCustomMsg(
        mw::internal::robotics::gazebotransport::Packet const& msgContent);

    // store subscriber instance into subscriber map based on topic name
    std::pair<std::shared_ptr<ImageMatSubscriber>, bool> insert(
        std::string const& topic_name,
        std::shared_ptr<ImageMatSubscriber> subsciberPointer);

    // return subscriber instance stored in map based on topic name
    std::shared_ptr<ImageMatSubscriber> find(std::string const& topic);

    // store gazebo publisher pointer in the map based on topic name
    void setCustomPub(gazebo::transport::PublisherPtr customPubPtr, std::string const& topicName);

    // get publisher pointer from map based on topic name
    gazebo::transport::PublisherPtr getCustomPub(std::string const& topicName);

  private:
    // gazebo transport node
    gazebo::transport::NodePtr m_node;

    // Stores custom message subscriber instance based on topic name
    std::map<std::string, std::shared_ptr<ImageMatSubscriber>> m_subscriberPtrMap;

    // Storage map of custom message publisher pointer
    std::map<std::string, gazebo::transport::PublisherPtr> m_customPublisherMap;

    std::mutex m_mutex;
};

} // namespace gazebotransport
} // namespace robotics

#endif
