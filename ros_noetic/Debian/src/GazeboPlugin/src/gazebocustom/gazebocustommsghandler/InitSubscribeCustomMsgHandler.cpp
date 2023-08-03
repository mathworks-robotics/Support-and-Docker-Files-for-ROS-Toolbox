/* Copyright 2019-2020 The MathWorks, Inc. */
#include "gazebotransport/gazebocustom/gazebocustommsghandler/InitSubscribeCustomMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetTopicListMsgHandler.hpp"
#include <boost/algorithm/string.hpp>
#include <iostream>

namespace robotics {
namespace gazebotransport {
InitSubscribeCustomMsgHandler::InitSubscribeCustomMsgHandler(
    std::shared_ptr<CustomMsgDispatcher> customDispatcher)
    : m_customDispatcher(customDispatcher) {
}

InitSubscribeCustomMsgHandler::~InitSubscribeCustomMsgHandler() {
}

std::string InitSubscribeCustomMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    // check topic is published by custom message
    // topic should be available in gazebo
    bool validTopic = false;

    mw::internal::robotics::gazebotransport::Packet newTopicList = m_topicList->GetAllTopicList();
    for (int i = 0; i < newTopicList.topic_list().data_size(); i++) {
        if (newTopicList.topic_list().data(i).name() ==
            msgContent.init_custom_subscriber().topic_name()) {
            validTopic = true;
        }
    }

    if (validTopic) {
        std::string replyMsg = this->m_customDispatcher->initSubscriber(
            msgContent.init_custom_subscriber().message_type(), msgContent);

        if (replyMsg == "") {
            mw::internal::robotics::gazebotransport::Packet message;
            message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::
                                                 PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
            message.mutable_header()->mutable_time_stamp()->set_seconds(0);
            message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
            message.set_status(
                mw::internal::robotics::gazebotransport::Packet_CoSimError_MESSAGE_TYPE_INVALID);
            return message.SerializeAsString(); // returns serialized error message
        } else {
            // return message received in the reply message
            return replyMsg;
        }
    } else {
        // return error message for topic name invalid
        mw::internal::robotics::gazebotransport::Packet message;
        message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
        message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);

        if (!validTopic) {
            message.set_status(
                mw::internal::robotics::gazebotransport::Packet_CoSimError_TOPIC_NAME_INVALID);
        } else {
            message.set_status(
                mw::internal::robotics::gazebotransport::Packet_CoSimError_MESSAGE_TYPE_INVALID);
        }

        return message.SerializeAsString();
    }
}

uint32_t InitSubscribeCustomMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_INIT_CUSTOM_MESSAGE_SUBSCRIBER;
}
} // namespace gazebotransport
} // namespace robotics
