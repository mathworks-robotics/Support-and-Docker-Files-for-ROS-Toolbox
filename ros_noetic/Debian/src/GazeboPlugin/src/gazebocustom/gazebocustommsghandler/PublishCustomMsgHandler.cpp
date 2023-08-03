/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazebocustom/gazebocustommsghandler/PublishCustomMsgHandler.hpp"
#include "iostream"

namespace robotics {
namespace gazebotransport {
PublishCustomMsgHandler::PublishCustomMsgHandler(
    std::shared_ptr<CustomMsgDispatcher> customDispatcher)
    : m_customDispatcher(customDispatcher) {
}

PublishCustomMsgHandler::~PublishCustomMsgHandler() {
}

std::string PublishCustomMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    std::string replyMsg = this->m_customDispatcher->publishCustomMsg(
        msgContent.custom_message_support().message_type(), msgContent);

    if (replyMsg == "") {
        mw::internal::robotics::gazebotransport::Packet message;
        message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
        message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        message.set_status(
            mw::internal::robotics::gazebotransport::Packet_CoSimError_MESSAGE_TYPE_INVALID);
        return message.SerializeAsString(); // returns serialized error message
    } else {
        return replyMsg;
    }
}

uint32_t PublishCustomMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_CUSTOM_MESSAGE_PUBLISHER;
}
} // namespace gazebotransport
} // namespace robotics
