/* Copyright 2019-2022 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_INITPUBLISHCUSTOMMSGHANDLER_HPP
#define GAZEBOCOSIM_INITPUBLISHCUSTOMMSGHANDLER_HPP

#include "gazebotransport/MsgHandler.hpp"
#include "gazebotransport/CustomMsgDispatcher.hpp"

namespace robotics {
namespace gazebotransport {
class GAZEBOTRANSPORT_EXPORT_CLASS InitPublishCustomMsgHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiates message handler to exchange InitPublishCustom message
    /*
    @param customDispatcher             CustomMsgDispatcher object
    */
    explicit InitPublishCustomMsgHandler(std::shared_ptr<CustomMsgDispatcher> customDispatcher);

    /// Destructor
    ~InitPublishCustomMsgHandler() override;

    /**
    @param msgContent      Packet message

    Handles Packet message & construct new Packet message with proper fields
    (Error/ reply content). Based on input message, initializes custom message publisher on Gazebo.
    Further, serializes Packet ( reply message) into string and returns.
    */
    std::string handleMessage(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) override;

    /// Returns Message ID
    uint32_t getAcceptID() const override;

  private:
    /// Custom Message Dispatcher pointer
    std::shared_ptr<CustomMsgDispatcher> m_customDispatcher;
};
} // namespace gazebotransport
} // namespace robotics
#endif
