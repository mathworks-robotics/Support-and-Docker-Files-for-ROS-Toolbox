/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazebocustom/gazebocustommsghandler/SubscribeCustomMsgHandler.hpp"
#include "iostream"

namespace robotics {
namespace gazebotransport {


SubscribeCustomMsgHandler::SubscribeCustomMsgHandler(
    std::shared_ptr<CustomMsgDispatcher> customDispatcher)
    : m_customDispatcher(customDispatcher) {
}

SubscribeCustomMsgHandler::~SubscribeCustomMsgHandler() {
}

std::string SubscribeCustomMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    auto replyMsg = this->m_customDispatcher->subscribeCustomMsg(
        msgContent.request_custom_message_support().message_type(), msgContent);

    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_CUSTOM_MESSAGE_SUBSCRIBER);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_custom_message_support()->set_topic_name(
        msgContent.request_custom_message_support().topic_name());
    m_message.mutable_custom_message_support()->set_message_type(
        msgContent.request_custom_message_support().message_type());
    m_message.mutable_custom_message_support()->set_data(replyMsg.second);
    m_message.mutable_custom_message_support()->set_is_new(replyMsg.first);

    return m_message.SerializeAsString(); // returns serialized error message
}

uint32_t SubscribeCustomMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_CUSTOM_MESSAGE_SUBSCRIBER;
}
} // namespace gazebotransport
} // namespace robotics
