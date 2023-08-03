/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/SubscribeImuMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebosensors/SensorContainer.hpp"
#include "gazebotransport/gazeboserver/gazebosensors/GazeboIMUSensor.hpp"

namespace robotics {
namespace gazebotransport {
SubscribeImuMsgHandler::SubscribeImuMsgHandler(gazebo::physics::WorldPtr world,
                                               gazebo::transport::NodePtr node)
    : SubscribeMsgHandler(world, node) {
}

SubscribeImuMsgHandler::~SubscribeImuMsgHandler() {
}

void SubscribeImuMsgHandler::doSubscribe(
    gazebo::physics::WorldPtr world,
    gazebo::transport::NodePtr node,
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    auto insertResult = SensorContainer::getInstance().insert(
        std::make_shared<GazeboIMUSensor>(world, getTopicName(msgContent)));
    insertResult.first->init(node);
}

std::string SubscribeImuMsgHandler::getTopicName(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    return msgContent.subscribe_imu().topic_name();
}

uint32_t SubscribeImuMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_SUBSCRIBE_IMU;
}
} // namespace gazebotransport
} // namespace robotics
