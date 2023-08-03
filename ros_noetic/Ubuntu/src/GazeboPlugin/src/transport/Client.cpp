/* Copyright 2019-2021 The MathWorks, Inc. */
#include "gazebotransport/Client.hpp"
#include "gazebotransport/PacketEnding.hpp"

#include "gazebotransport/transport/readWithTimeout.hpp"
#include "gazebotransport/MismatchPluginVersionException.hpp"
#include <iostream>
#include <exception>
#include <array>
#include <string>

namespace robotics {
namespace gazebotransport {
Client::Client(std::string const& ipaddress,
               std::string const& port,
               boost::asio::deadline_timer::duration_type const& timeout)
    : m_io()
    , m_socket(m_io) {
    // resolve the Server ip address and port number and connect to Server
    boost::asio::ip::tcp::resolver resolver{m_io};
    boost::asio::connect(m_socket, resolver.resolve({ipaddress, port}));

    // synchronized send handshake to Server and wait for handshake reply
    boost::system::error_code ignored;
    auto handshake = HandshakeMessage::getStr() + EndOfMessage::getStr();
    auto sendBuffer = boost::asio::buffer(handshake.data(), handshake.size());
    boost::asio::write(m_socket, sendBuffer, ignored);
    auto reply = readWithTimeOut(m_io, m_socket, timeout);

    // if handshake failed, throw an internal exception
    // this exception would eventually be caught and processed in GazeboClientMI built-in class
    if (!reply || !(*reply == HandshakeMessage::getStr())) {
        throw MismatchPluginVersionException();
    }
}

Client::~Client() {
    // close the socket when destruct
    shutdown();
}

void Client::shutdown() {
    if (m_socket.is_open()) {
        // send a shutdown message to server
        boost::system::error_code ignored;
        auto sumMsg = EndOfSession::getStr() + EndOfMessage::getStr();
        auto sendBuffer = boost::asio::buffer(sumMsg.data(), sumMsg.size());
        boost::asio::write(m_socket, sendBuffer, ignored);

        // close socket, ignoring any error
        m_socket.shutdown(boost::asio::socket_base::shutdown_both, ignored);
        m_socket.close(ignored);
        m_io.reset();
    }
}

boost::optional<std::string> Client::write(
    std::string const& message,
    boost::asio::deadline_timer::duration_type const& timeout) {
    // clear the socket buffer before writing new request
    while (m_socket.available()) {
        boost::system::error_code ignored;
        boost::asio::streambuf ignoredBuffer;
        boost::asio::read_until(m_socket, ignoredBuffer, EndOfMessage::getStr(), ignored);
    }

    // synchronized send message to Server and wait for response
    boost::system::error_code ignored;
    auto sumMsg = message + EndOfMessage::getStr();
    auto sendBuffer = boost::asio::buffer(sumMsg.data(), sumMsg.size());
    boost::asio::write(m_socket, sendBuffer, ignored);

    // synchronized read from Server with timeout
    return readWithTimeOut(m_io, m_socket, timeout);
}

} // namespace gazebotransport
} // namespace robotics
