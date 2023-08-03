/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/transport/Connection.hpp"
#include "gazebotransport/transport/ConnectionManager.hpp"
#include "gazebotransport/PacketEnding.hpp"

#include "boost/date_time.hpp"
#include "boost/version.hpp"

namespace robotics {
namespace gazebotransport {
Connection::Connection(boost::asio::ip::tcp::socket socket,
                       Callback& callback,
                       ConnectionManager& connections)
    : m_socket(std::move(socket))
#if BOOST_VERSION <= 106501
    , m_handshakeTimer(m_socket.get_io_service())
#else
    , m_handshakeTimer(m_socket.get_executor())
#endif
    , m_callback(callback)
    , m_connections(connections)
    , m_buffer()
    , m_bufferMutex()
    , m_reply() {
}

void Connection::start(boost::asio::deadline_timer::duration_type const& timeout) {
    doHandshake(timeout);
}

void Connection::stop() {
    std::lock_guard<std::mutex> lock(m_bufferMutex);
    if (m_socket.is_open()) {
        boost::system::error_code ignored;
        m_socket.shutdown(boost::asio::socket_base::shutdown_both, ignored);
        m_socket.close(ignored);
    }
}

void Connection::doHandshake(boost::asio::deadline_timer::duration_type const& timeout) {
    auto sthis = shared_from_this();

    // setup handshake deadline in 5 seconds
    m_handshakeTimer.expires_from_now(timeout);
    m_handshakeTimer.async_wait([sthis](const boost::system::error_code& ec) {
        if (ec != boost::asio::error::operation_aborted) {
            sthis->m_connections.stop(sthis);
        }
    });

    // read first message and check for handshake
    boost::asio::async_read_until(
        m_socket, m_buffer, EndOfMessage::getStr(),
        [sthis](const boost::system::error_code& ec, std::size_t /*bytes_transferred*/) {
            bool stopConnection = false;

            if (!ec) {
                std::lock_guard<std::mutex> lock(sthis->m_bufferMutex);

                // check handshake message
                sthis->m_reply = std::string(boost::asio::buffers_begin(sthis->m_buffer.data()),
                                             boost::asio::buffers_end(sthis->m_buffer.data()));
                if (sthis->m_reply == HandshakeMessage::getStr() + EndOfMessage::getStr()) {
                    // reply handshake back to client
                    sthis->doWrite();
                } else {
                    // stop the connection due to failed handshake
                    stopConnection = true;
                }

                // cancel the handshake timer
                boost::system::error_code ignored;
                sthis->m_handshakeTimer.cancel(ignored);

                // clear the buffer
                sthis->m_buffer.consume(sthis->m_buffer.size());
            }

            if (stopConnection) {
                sthis->m_connections.stop(sthis);
            }
        });
}

void Connection::doRead() {
    auto sthis = shared_from_this();
    boost::asio::async_read_until(
        m_socket, m_buffer, EndOfMessage::getStr(),
        [sthis](const boost::system::error_code& ec, std::size_t bytes_transferred) {
            // if there is any error, exit
            if (ec) {
                return;
            }

            bool closeConnection = false;

            // critical section that ensures only one read call can modify the read buffer at any
            // given time
            {
                std::lock_guard<std::mutex> lock(sthis->m_bufferMutex);
                // first check whether this is to end session
                if (bytes_transferred ==
                    EndOfSession::getStr().size() + EndOfMessage::getStr().size()) {
                    std::string message{
                        boost::asio::buffers_begin(sthis->m_buffer.data()),
                        boost::asio::buffers_begin(sthis->m_buffer.data()) +
                            static_cast<std::ptrdiff_t>(EndOfSession::getStr().size())};
                    closeConnection = (message == EndOfSession::getStr());
                }

                if (!closeConnection) {
                    // trigger callback with incoming message
                    sthis->m_reply = sthis->m_callback(sthis->m_buffer, ec, bytes_transferred) +
                                     EndOfMessage::getStr();
                }

                // clear message that is processed
                sthis->m_buffer.consume(sthis->m_buffer.size());
            }

            if (closeConnection) {
                sthis->m_connections.stop(sthis);
            } else {
                // write reply message to client
                sthis->doWrite();
            }
        });
}

void Connection::doWrite() {
    auto sthis = shared_from_this();
    boost::asio::async_write(m_socket, boost::asio::buffer(m_reply.data(), m_reply.size()),
                             [sthis](boost::system::error_code ec, std::size_t /*len*/) {
                                 if (!ec) {
                                     sthis->doRead();
                                 }
                             });
}

bool Connection::isOpen() const {
    std::lock_guard<std::mutex> lock(m_bufferMutex);
    return m_socket.is_open();
}

} // namespace gazebotransport
} // namespace robotics
