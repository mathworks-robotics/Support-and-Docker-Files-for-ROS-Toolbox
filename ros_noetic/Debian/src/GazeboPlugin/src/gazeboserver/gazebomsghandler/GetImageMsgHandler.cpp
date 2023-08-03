/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/GetImageMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebosensors/SensorContainer.hpp"

namespace robotics {
namespace gazebotransport {

GetImageMsgHandler::GetImageMsgHandler(gazebo::physics::WorldPtr world)
    : m_world(world) {
}
GetImageMsgHandler::~GetImageMsgHandler() {
}

std::string GetImageMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    auto sensor = SensorContainer::getInstance().find(msgContent.request_image().topic_name());

    if (sensor) {
        // returns serialized new Image message
        return sensor->getLatestMessage().SerializeAsString();
    } else {
        mw::internal::robotics::gazebotransport::Packet message;
        auto simTime = m_world->SimTime();
        message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
        message.mutable_header()->mutable_time_stamp()->set_seconds(
            static_cast<uint64_t>(simTime.sec));
        message.mutable_header()->mutable_time_stamp()->set_nano_seconds(
            static_cast<uint64_t>(simTime.nsec));
        message.set_status(
            mw::internal::robotics::gazebotransport::Packet_CoSimError_TOPIC_NAME_INVALID);

        return message.SerializeAsString(); // returns serialized new Image message
    }
}

uint32_t GetImageMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_REQUEST_IMAGE;
}
} // namespace gazebotransport
} // namespace robotics
