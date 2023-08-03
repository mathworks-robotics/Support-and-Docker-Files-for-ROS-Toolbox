/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/transport/ConnectionManager.hpp"

namespace robotics {
namespace gazebotransport {
ConnectionManager::ConnectionManager()
    : m_connections()
    , m_mutex() {
}

ConnectionManager::~ConnectionManager() {
    stopAll();
}

void ConnectionManager::start(std::shared_ptr<Connection> const& c,
                              boost::asio::deadline_timer::duration_type const& timeout) {
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_connections.insert(c);
    }
    c->start(timeout);
}

void ConnectionManager::stop(std::shared_ptr<Connection> c) {
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto iter = m_connections.find(c);
        if (iter != m_connections.end()) {
            m_connections.erase(iter);
        }
    }
    c->stop();
}

void ConnectionManager::stopAll() {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto c : m_connections) {
        c->stop();
    }
    m_connections.clear();
}

size_t ConnectionManager::getNumActiveConnections() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_connections.size();
}
} // namespace gazebotransport
} // namespace robotics
