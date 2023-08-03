/* Copyright 2019-2022 The MathWorks, Inc. */
#ifndef CUSTOMMSGHANDLER_HPP
#define CUSTOMMSGHANDLER_HPP
#include <memory>
#include <cstdint>
#include <string>
#include "gazebotransport/mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

namespace robotics {
namespace gazebotransport {
class CustomMsgHandler {
  public:
    /// Parent class of all CustomMsgHandler class
    /// Virtual function to initialize publisher on gazebo
    virtual std::string initPublisher(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) = 0;

    /// Virtual function to initialize subscriber on gazebo
    virtual std::string initSubscriber(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) = 0;

    /// Destructor
    virtual ~CustomMsgHandler() = default;

    /// Virtual function to publish custom message on gazebo
    virtual std::string publishCustomMsg(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) = 0;

    /// Virtual function to subscribe custom message from gazebo
    virtual std::pair<bool, std::string> subscribeCustomMsg(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) = 0;
};
typedef std::shared_ptr<CustomMsgHandler> CustomMsgHandlerPtr;
} // namespace gazebotransport
} // namespace robotics

#endif
