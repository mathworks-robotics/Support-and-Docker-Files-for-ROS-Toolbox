/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_SERVER_HPP
#define GAZEBOCOSIM_SERVER_HPP

#include "gazebotransport/gazebotransport_util.hpp"
#include "gazebotransport/Callback.hpp"

#include <cstdint>  // for uint16_t
#include <stddef.h> // for size_t
#include <memory>
#include <future>
#include <atomic>
#include <mutex>

#include "gazebotransport/BoostVersionSupport.hpp"

namespace robotics {
namespace gazebotransport {
/// ConnectionManager holds all active connections to a Server
class ConnectionManager;

/// Server accepts connection on given port number
class GAZEBOTRANSPORT_EXPORT_CLASS Server : public std::enable_shared_from_this<Server> {
  public:
    /// Constructor
    /**
    @param portNumber    The TCP/IP port server monitors
    @param callback      The function to be called when new messages were received on the portNumber
    @exception           May throw if the port is already occupied

    Use 0 as port number would bind the server to a random open port
    Default handshake wait time is 5 seconds
    */
    Server(uint16_t portNumber, CallbackPtr callback);

    /// Destructor
    ~Server();

    /// Start server
    void run();

    /// Shutdown the server
    void shutdown();

    /// Check whether server is running
    bool isRunning() const;

    /// Get server port number
    uint16_t getPortNumber() const;

    /// Change server handshake timeout
    /**
    @param timeout                 Amount of time to wait for handshake completion

    Default timeout is 5 seconds
    */
    void setTimeout(boost::asio::deadline_timer::duration_type const& timeout);

    /// Get number of active connections hosted by server
    size_t getNumActiveConnections();

  private:
    /// Keep running waitAccept function in the io_context queue to handle new client connections
    void waitAccept();

    /// io_context queue
    boost::asio::io_context m_io;

    /// TCP/IP connection acceptor
    boost::asio::ip::tcp::acceptor m_acceptor;

    /// Server port number
    uint16_t m_port;

    /// Manages all active connections to the server
    std::unique_ptr<ConnectionManager> m_connections;

    /// Holds a socket to be passed to new connections
    boost::asio::ip::tcp::socket m_socket;

    /// Amount of time to wait for handshake completion
    boost::asio::deadline_timer::duration_type m_timeout;

    /// Callback to be executed when new messages were received on connections
    CallbackPtr m_callback;

    /// Boost io service run result
    std::future<size_t> m_runResult;

    /// Run status
    std::atomic<bool> m_running;

    /// Mutex for multi-thread guard
    std::mutex m_mutex;
};
} // namespace gazebotransport
} // namespace robotics

#endif /*GAZEBOCOSIM_SERVER_HPP*/
