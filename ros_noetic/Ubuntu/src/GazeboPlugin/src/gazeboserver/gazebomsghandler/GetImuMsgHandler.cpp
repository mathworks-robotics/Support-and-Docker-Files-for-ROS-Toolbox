/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/GetImuMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebosensors/SensorContainer.hpp"

namespace robotics {
namespace gazebotransport {

GetImuMsgHandler::GetImuMsgHandler(gazebo::physics::WorldPtr world)
    : m_world(world) {
}
GetImuMsgHandler::~GetImuMsgHandler() {
}

std::string GetImuMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    mw::internal::robotics::gazebotransport::Packet new_message;

    {
        auto sensor = SensorContainer::getInstance().find(msgContent.request_imu().topic_name());
        if (sensor) {
            new_message = sensor->getLatestMessage();
        } else {
            gazebo::common::Time timestamp = m_world->SimTime();
            new_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::
                                                     PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
            new_message.mutable_header()->mutable_time_stamp()->set_seconds(
                (uint64_t)timestamp.sec);
            new_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(
                (uint64_t)timestamp.nsec);
            new_message.set_status(
                mw::internal::robotics::gazebotransport::Packet_CoSimError_TOPIC_NAME_INVALID);
        }
    }

    return new_message.SerializeAsString(); //  returns serialized new Lidar message
}

uint32_t GetImuMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_REQUEST_IMU;
}
} // namespace gazebotransport
} // namespace robotics
