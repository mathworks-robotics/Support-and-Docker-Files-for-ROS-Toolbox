/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/SubscribeImageMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebosensors/SensorContainer.hpp"
#include "gazebotransport/gazeboserver/gazebosensors/GazeboCameraSensor.hpp"

namespace robotics {
namespace gazebotransport {
SubscribeImageMsgHandler::SubscribeImageMsgHandler(gazebo::physics::WorldPtr world,
                                                   gazebo::transport::NodePtr node)
    : SubscribeMsgHandler(world, node) {
}

SubscribeImageMsgHandler::~SubscribeImageMsgHandler() {
}

void SubscribeImageMsgHandler::doSubscribe(
    gazebo::physics::WorldPtr world,
    gazebo::transport::NodePtr node,
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    auto insertResult = SensorContainer::getInstance().insert(
        std::make_shared<GazeboCameraSensor>(world, getTopicName(msgContent)));
    insertResult.first->init(node);
}

std::string SubscribeImageMsgHandler::getTopicName(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    return msgContent.subscribe_image().topic_name();
}

uint32_t SubscribeImageMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_SUBSCRIBE_IMAGE;
}
} // namespace gazebotransport
} // namespace robotics
