/* Copyright 2019-2020 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_CONNECTION_HPP
#define GAZEBOCOSIM_CONNECTION_HPP


#include "gazebotransport/Callback.hpp"

#include <memory>
#include <mutex>

#include "boost/asio.hpp"

namespace robotics {
namespace gazebotransport {
/// ConnectionManager that owns all connections
class ConnectionManager;

/// Connection maintains service for each Client that is connected to Server
/**
The Connection object enables "shared from this" to maintain the object lifetime
across multiple threads
*/
class GAZEBOTRANSPORT_EXPORT_CLASS Connection : public std::enable_shared_from_this<Connection> {
  public:
    /// Constructor
    /**
    @param socket                  TCP/IP connection socket opened by server
    @param callback                Functor to call when new message were received on the socket
    @param connections             The connection manager that keep track of this connection
    */
    Connection(boost::asio::ip::tcp::socket socket,
               Callback& callback,
               ConnectionManager& connections);

    /// Start reading from socket
    /**
    @param timeout                 Amount of time to wait for handshake completion
    */
    void start(boost::asio::deadline_timer::duration_type const& timeout);

    /// Stop reading from socket and close it
    void stop();

    /// Check whether connection is open
    bool isOpen() const;

  private:
    /// Perform one-time handshake
    /**
    @param timeout                 Amount of time to wait for handshake completion

    Handshake verifies that the server and client are both working on Gazebo co-simulation
    This avoids maintaining connections to unspecified clients
    */
    void doHandshake(boost::asio::deadline_timer::duration_type const& timeout);

    /// Keep reading from socket
    void doRead();

    /// Send reply to client
    void doWrite();

    /// TCPIP connection socket
    boost::asio::ip::tcp::socket m_socket;

    /// Deadline timer for handshake
    boost::asio::deadline_timer m_handshakeTimer;

    /// Callback functor to be triggered with new message received on socket
    Callback& m_callback;

    /// Connection manager that owns this connection
    ConnectionManager& m_connections;

    /// Buffer that stores incoming message
    boost::asio::streambuf m_buffer;

    /// Mutex lock that guard the stream buffer
    mutable std::mutex m_bufferMutex;

    /// String content queue to send
    std::string m_reply;
};
} // namespace gazebotransport
} // namespace robotics

#endif /*GAZEBOCOSIM_CONNECTION_HPP*/
