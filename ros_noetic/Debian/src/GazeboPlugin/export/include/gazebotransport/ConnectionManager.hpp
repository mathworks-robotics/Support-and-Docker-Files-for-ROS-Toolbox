/* Copyright 2019-2020 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_CONNECTION_MANAGER_HPP
#define GAZEBOCOSIM_CONNECTION_MANAGER_HPP

#include "gazebotransport/Connection.hpp"

#include <set>
#include <mutex>

namespace robotics {
namespace gazebotransport {

/// ConnectionManager holds all active connections to a Server
class GAZEBOTRANSPORT_EXPORT_CLASS ConnectionManager {
  public:
    /// Constructor
    ConnectionManager();

    /// Destructor
    ~ConnectionManager();

    /// Start a new connection and take it into collection
    void start(std::shared_ptr<Connection> const& c,
               boost::asio::deadline_timer::duration_type const& timeout);

    /// Stop a connection and remove it from collection
    void stop(std::shared_ptr<Connection> c);

    /// Stop all connections and clear the collection
    void stopAll();

    /// Get number of active connections hosted by manager
    size_t getNumActiveConnections();

  private:
    /// All active connections
    std::set<std::shared_ptr<Connection>> m_connections;

    /// mutex for multi-threading operations
    std::mutex m_mutex;
};
} // namespace gazebotransport
} // namespace robotics

#endif /*GAZEBOCOSIM_CONNECTION_MANAGER_HPP*/
