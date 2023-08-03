/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/SubscribeLaserMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebosensors/SensorContainer.hpp"
#include "gazebotransport/gazeboserver/gazebosensors/GazeboLaserSensor.hpp"

namespace robotics {
namespace gazebotransport {
SubscribeLaserMsgHandler::SubscribeLaserMsgHandler(gazebo::physics::WorldPtr world,
                                                   gazebo::transport::NodePtr node)
    : SubscribeMsgHandler(world, node) {
}

SubscribeLaserMsgHandler::~SubscribeLaserMsgHandler() {
}

void SubscribeLaserMsgHandler::doSubscribe(
    gazebo::physics::WorldPtr world,
    gazebo::transport::NodePtr node,
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    auto insertResult = SensorContainer::getInstance().insert(
        std::make_shared<GazeboLaserSensor>(world, getTopicName(msgContent)));
    insertResult.first->init(node);
}

std::string SubscribeLaserMsgHandler::getTopicName(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    return msgContent.subscribe_laser().topic_name();
}

uint32_t SubscribeLaserMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_SUBSCRIBE_LASER;
}
} // namespace gazebotransport
} // namespace robotics
