/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_INITSUBSCRIBECUSTOMMSGHANDLER_HPP
#define GAZEBOCOSIM_INITSUBSCRIBECUSTOMMSGHANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include "gazebotransport/gazebocustom/CustomMsgDispatcher.hpp"

namespace robotics {
namespace gazebotransport {

class GetTopicListMsgHandler;

class InitSubscribeCustomMsgHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiates message handler to exchange InitSubscribeCustom message
    /*
    @param customDispatcher         CustomMsgDispatcher object
    */
    explicit InitSubscribeCustomMsgHandler(std::shared_ptr<CustomMsgDispatcher> customDispatcher);

    /// Destructor
    ~InitSubscribeCustomMsgHandler() override;

    /**
    @param msgContent      Packet message

    Handles Packet message & construct new Packet message with proper fields
    (Error/ reply content). Based on input message, initializes custom message subscriber on Gazebo.
    Further, serializes Packet ( reply message) into string and returns.
    */
    std::string handleMessage(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) override;

    /// Returns Message ID
    uint32_t getAcceptID() const override;

  private:
    /// Custom Message Dispatcher pointer
    std::shared_ptr<CustomMsgDispatcher> m_customDispatcher;

    // Get topic list pointer
    std::shared_ptr<GetTopicListMsgHandler> m_topicList;
};
} // namespace gazebotransport
} // namespace robotics
#endif
