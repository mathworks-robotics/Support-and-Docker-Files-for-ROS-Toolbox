/* Copyright 2019-2022 The MathWorks, Inc. */
#ifndef MSGHANDLER_HPP
#define MSGHANDLER_HPP
#include <memory>
#include <cstdint>
#include <string>
#include "gazebotransport/mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

namespace robotics {
namespace gazebotransport {
class MsgHandler {
  public:
    /// Parent class of all msgHandler class
    /// Virtual function to Handles & process message
    virtual std::string handleMessage(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) = 0;

    /// Destructor
    virtual ~MsgHandler() = default;

    /// Virtual function to return Message ID
    virtual uint32_t getAcceptID() const = 0;
};
// MessageHandler shared instances
typedef std::shared_ptr<MsgHandler> MsgHandlerPtr;

} // namespace gazebotransport
} // namespace robotics

#endif
