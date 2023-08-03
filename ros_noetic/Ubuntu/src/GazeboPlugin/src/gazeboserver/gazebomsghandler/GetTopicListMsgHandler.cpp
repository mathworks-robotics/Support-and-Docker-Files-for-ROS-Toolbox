/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/GetTopicListMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"

namespace robotics {
namespace gazebotransport {

GetTopicListMsgHandler::GetTopicListMsgHandler() {
}
GetTopicListMsgHandler::~GetTopicListMsgHandler() {
}

mw::internal::robotics::gazebotransport::Packet GetTopicListMsgHandler::GetAllTopicList() {
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_TOPIC_LIST);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);

    auto topics = gazebo::transport::getAdvertisedTopics();

    size_t msgCount = 0;
    for (auto iterType = topics.begin(); iterType != topics.end(); ++iterType) {
        for (auto iterName = iterType->second.begin(); iterName != iterType->second.end();
             ++iterName) {
            auto* msgTopicInfo = m_message.mutable_topic_list()->add_data();
            msgTopicInfo->set_name(*iterName);
            msgTopicInfo->set_type(iterType->first);
            ++msgCount;
        }
    }

    return m_message;
}

std::string GetTopicListMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {

    mw::internal::robotics::gazebotransport::Packet newTopicList = GetAllTopicList();

    return newTopicList.SerializeAsString(); // returns serialized TopicList message
}

uint32_t GetTopicListMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_GET_TOPIC_LIST;
}
} // namespace gazebotransport
} // namespace robotics
