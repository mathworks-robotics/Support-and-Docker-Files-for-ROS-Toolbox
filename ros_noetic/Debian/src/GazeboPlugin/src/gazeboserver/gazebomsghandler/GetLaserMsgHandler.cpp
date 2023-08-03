/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/GetLaserMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebosensors/SensorContainer.hpp"

namespace robotics {
namespace gazebotransport {

GetLaserMsgHandler::GetLaserMsgHandler(gazebo::physics::WorldPtr world)
    : m_world(world) {
}

GetLaserMsgHandler::~GetLaserMsgHandler() {
}

std::string GetLaserMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    mw::internal::robotics::gazebotransport::Packet m_message;

    {
        auto sensor = SensorContainer::getInstance().find(msgContent.request_laser().topic_name());

        if (sensor) {
            m_message = sensor->getLatestMessage();
        } else {
            gazebo::common::Time timestamp = m_world->SimTime();
            m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::
                                                   PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
            m_message.mutable_header()->mutable_time_stamp()->set_seconds((uint64_t)timestamp.sec);
            m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(
                (uint64_t)timestamp.nsec);
            m_message.set_status(
                mw::internal::robotics::gazebotransport::Packet_CoSimError_TOPIC_NAME_INVALID);
        }
    }

    return m_message.SerializeAsString(); //  returns serialized new Lidar message
}

uint32_t GetLaserMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_REQUEST_LASER;
}
} // namespace gazebotransport
} // namespace robotics
