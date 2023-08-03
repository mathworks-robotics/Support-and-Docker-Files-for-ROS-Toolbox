/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/MsgDispatcher.hpp"

namespace robotics {
namespace gazebotransport {
void MsgDispatcher::registerHandler(MsgHandlerPtr handler) {
    auto iter = m_handlers.find(handler->getAcceptID());

    if (iter == m_handlers.end()) {
        m_handlers[handler->getAcceptID()] = handler;
    }
}
std::string MsgDispatcher::handleMessage(
    uint32_t msgid,
    mw::internal::robotics::gazebotransport::Packet const& msg) const {
    auto iter = m_handlers.find(msgid);
    if (iter != m_handlers.end()) {
        return iter->second->handleMessage(msg);
    } else {
        return "";
    }
}

void MsgDispatcher::reset() {
    m_handlers.clear();
}
} // namespace gazebotransport
} // namespace robotics
