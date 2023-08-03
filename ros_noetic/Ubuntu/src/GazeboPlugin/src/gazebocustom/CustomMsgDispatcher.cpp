/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazebocustom/CustomMsgDispatcher.hpp"

namespace robotics {
namespace gazebotransport {
void CustomMsgDispatcher::registerCustomHandler(CustomMsgHandlerPtr handler,
                                                std::string const& msgName) {
    auto iter = m_handlers.find(msgName);
    if (iter == m_handlers.end()) {
        m_handlers[msgName] = handler;
    }
}

std::string CustomMsgDispatcher::initPublisher(
    std::string const& msgName,
    mw::internal::robotics::gazebotransport::Packet const& msg) const {
    auto iter = m_handlers.find(msgName);
    if (iter != m_handlers.end()) {
        return iter->second->initPublisher(msg);
    } else {
        return "";
    }
}

std::string CustomMsgDispatcher::publishCustomMsg(
    std::string const& msgName,
    mw::internal::robotics::gazebotransport::Packet const& msg) const {
    auto iter = m_handlers.find(msgName);
    if (iter != m_handlers.end()) {
        return iter->second->publishCustomMsg(msg);
    } else {
        return "";
    }
}

std::string CustomMsgDispatcher::initSubscriber(
    std::string const& msgName,
    mw::internal::robotics::gazebotransport::Packet const& msg) const {
    auto iter = m_handlers.find(msgName);
    if (iter != m_handlers.end()) {
        return iter->second->initSubscriber(msg);
    } else {
        return "";
    }
}

std::pair<bool, std::string> CustomMsgDispatcher::subscribeCustomMsg(
    std::string const& msgName,
    mw::internal::robotics::gazebotransport::Packet const& msg) const {
    auto iter = m_handlers.find(msgName);
    if (iter != m_handlers.end()) {
        return iter->second->subscribeCustomMsg(msg);
    } else {
        return std::make_pair(false, "");
    }
}

void CustomMsgDispatcher::reset() {
    m_handlers.clear();
}
} // namespace gazebotransport
} // namespace robotics
