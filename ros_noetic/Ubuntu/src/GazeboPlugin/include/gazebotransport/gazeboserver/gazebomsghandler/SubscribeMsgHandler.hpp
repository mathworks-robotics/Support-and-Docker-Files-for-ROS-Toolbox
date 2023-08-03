/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_SUBSCRIBE_MSG_HANDLER_HPP
#define GAZEBOCOSIM_SUBSCRIBE_MSG_HANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetTopicListMsgHandler.hpp"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace robotics {
namespace gazebotransport {
class SubscribeMsgHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class is the abstract parent for all the subscribe messages, such as SubscribeImage
    /*
      @param world              Pointer to Gazebo world for query simulation time
      @param node               Pointer to Gazebo transport node
    */
    explicit SubscribeMsgHandler(gazebo::physics::WorldPtr world, gazebo::transport::NodePtr node);

    /// Destructor
    virtual ~SubscribeMsgHandler();

    /*
    @param msgContent      Packet message

    Handles Packet message & construct new Packet message with proper fields (Error/ reply content).
    Based on input message topicname, corresponding camera is subscribed and OnImage callback is
    called. Further, it serializes Packet ( reply message) into string and returns.
    */
    std::string handleMessage(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) override;

  private:
    /// Derived subscriber classes must return the Gazebo transport topic name it subscribes to
    virtual std::string getTopicName(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) = 0;

    /// Derived subscriber class must perform subscribe for each sensor
    virtual void doSubscribe(gazebo::physics::WorldPtr world,
                             gazebo::transport::NodePtr node,
                             mw::internal::robotics::gazebotransport::Packet const& msgContent) = 0;

  private:
    // Gazebo world pointer for getting simulation time
    gazebo::physics::WorldPtr m_world;
    // Transport node
    gazebo::transport::NodePtr m_node;
    // TopicList message handler instance
    std::shared_ptr<GetTopicListMsgHandler> m_topicList;
};
} // namespace gazebotransport
} // namespace robotics
#endif
