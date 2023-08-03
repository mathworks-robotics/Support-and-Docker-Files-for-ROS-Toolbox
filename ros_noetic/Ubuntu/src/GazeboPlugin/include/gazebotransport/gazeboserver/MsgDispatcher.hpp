/* Copyright 2019 The MathWorks, Inc. */
#ifndef MSGDISPATCHER_HPP
#define MSGDISPATCHER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include <map>

namespace robotics {
namespace gazebotransport {
class MsgDispatcher {
  public:
    /**
    @param handler      Message handler

    Stores Initiated message handler in map based on MessageID
    */
    void registerHandler(MsgHandlerPtr handler); // Initiates message respective handler

    /**
    @param msgid    Message ID
    @param msg      Packet message

    The msgHandler map contains number of msgHandler Initiated at server. Retrieves msgHandler based
    on msgid and calls corresponding handleMessage.
    */
    std::string handleMessage(uint32_t msgid,
                              mw::internal::robotics::gazebotransport::Packet const& msg) const;

    /// Clears MsgHandler from map
    void reset();

  private:
    /// Stores multiple MsgHandler initiated
    std::map<uint32_t, MsgHandlerPtr> m_handlers;
};
typedef std::shared_ptr<MsgDispatcher> MsgDispatcherPtr;
} // namespace gazebotransport
} // namespace robotics

#endif
