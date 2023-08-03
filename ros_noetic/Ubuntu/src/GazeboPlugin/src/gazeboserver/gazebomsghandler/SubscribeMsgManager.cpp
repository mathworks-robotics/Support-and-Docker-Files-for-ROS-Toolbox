/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/SubscribeMsgManager.hpp"

namespace robotics {
namespace gazebotransport {

SubscribeMsgManager::SubscribeMsgManager()
    : m_update() {
}
SubscribeMsgManager::~SubscribeMsgManager() {
}

void SubscribeMsgManager::registerTopic(std::string const& topic) {
    {
        std::lock_guard<std::mutex> lock(m_lock);
        m_update[topic] = true;
    }
}

bool SubscribeMsgManager::doUpdate(std::string const& topic) {
    {
        std::lock_guard<std::mutex> lock(m_lock);
        auto iter = m_update.find(topic);
        if (iter != m_update.end()) {
            bool ret = iter->second;
            iter->second = false;
            return ret;
        } else {
            return false;
        }
    }
}

void SubscribeMsgManager::enableUpdate() {
    {
        std::lock_guard<std::mutex> lock(m_lock);
        for (auto iter = m_update.begin(); iter != m_update.end(); ++iter) {
            iter->second = true;
        }
    }
}

} // namespace gazebotransport
} // namespace robotics
