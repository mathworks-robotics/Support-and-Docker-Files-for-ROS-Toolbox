/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_CLIENT_HPP
#define GAZEBOCOSIM_CLIENT_HPP

#include "gazebotransport/gazebotransport_util.hpp"

#include <cstdint>  // for uint16_t
#include <stddef.h> // for size_t
#include <memory>
#include <string>

#include "gazebotransport/BoostVersionSupport.hpp"

namespace robotics {
namespace gazebotransport {
/// Client creates a TCP/IP client that connects to Server
class GAZEBOTRANSPORT_EXPORT_CLASS Client {
  public:
    /// Client constructor
    /**
    @param ipaddress       server ip address
    @param port            server port number
    @param timeout         amount of time to wait for handshake completion
    @exception             May throw if cannot connect to Server or handshake timeout
    */
    Client(std::string const& ipaddress,
           std::string const& port,
           boost::asio::deadline_timer::duration_type const& timeout);

    /// Client destructor
    ~Client();

    /// write a message to Server and wait for response
    /**
    @param message         message to sent to Server
    @param timeout         timeout for this write operation
    @return                an optional message returned from Server

    This is a synchronized operation

    If server replied within timeout, the optional output would be
    filled with value. If server didn't reply within timeout, the
    optional output would be filled with boost::none
    */
    boost::optional<std::string> write(std::string const& message,
                                       boost::asio::deadline_timer::duration_type const& timeout);

    /// shutdown client
    void shutdown();

  private:
    /// boost io_context queue
    boost::asio::io_context m_io;
    /// TCP/IP connection socket
    boost::asio::ip::tcp::socket m_socket;
};
} // namespace gazebotransport
} // namespace robotics

#endif /*GAZEBOCOSIM_CLIENT_HPP*/
