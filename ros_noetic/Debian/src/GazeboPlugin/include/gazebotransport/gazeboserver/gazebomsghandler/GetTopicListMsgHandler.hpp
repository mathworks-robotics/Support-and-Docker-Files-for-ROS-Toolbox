/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_GETTOPICLISTMSGHANDLER_HPP
#define GAZEBOCOSIM_GETTOPICLISTMSGHANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"

namespace robotics {
namespace gazebotransport {
class GetTopicListMsgHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiated message handler to exchange TopicList message
    explicit GetTopicListMsgHandler();
    ~GetTopicListMsgHandler();

    // Retrieve Topic list available in Gazebo
    mw::internal::robotics::gazebotransport::Packet GetAllTopicList();
    /**
      @param msgContent      Packet message
      Calls GetAllTopicList and serializes Packet ( reply message) into string and returns.
      */
    std::string handleMessage(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) override;
    /// Returns Message ID
    uint32_t getAcceptID() const override;
};
} // namespace gazebotransport
} // namespace robotics
#endif
