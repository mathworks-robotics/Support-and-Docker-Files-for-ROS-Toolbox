/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/Server.hpp"
#include "gazebotransport/transport/ConnectionManager.hpp"

#include "boost/date_time.hpp"

#include <iostream>
#include <exception>
#include <string>

namespace robotics {
namespace gazebotransport {
Server::Server(uint16_t portNumber, CallbackPtr callback)
    : m_io()
    , m_acceptor(m_io)
    , m_port(0)
    , m_connections(new ConnectionManager())
    , m_socket(m_io)
    , m_timeout(boost::posix_time::seconds(5))
    , m_callback(callback)
    , m_running(false)
    , m_mutex() {
    m_acceptor.open(boost::asio::ip::tcp::v4());
    m_acceptor.bind({boost::asio::ip::tcp::v4(), portNumber});
    m_acceptor.listen();
    m_port = m_acceptor.local_endpoint().port();
}

void Server::run() {
    if (!isRunning()) {
        m_io.reset();

        // schedule first wait for accept work
        waitAccept();

        // start running m_io in a separate thread
        auto sthis = shared_from_this();
        m_runResult = std::async(std::launch::async, [sthis]() {
            for (;;) {
                try {
                    return sthis->m_io.run();
                } catch (std::exception const& /*ex*/) {
                    // keep the io service running until server is shutdown
                }
            }
        });

        m_running = true;
    }
}

void Server::waitAccept() {
    auto sthis = shared_from_this();
    m_acceptor.async_accept(m_socket, [sthis](boost::system::error_code ec) {
        {
            std::lock_guard<std::mutex> lock(sthis->m_mutex);

            if (!sthis->m_acceptor.is_open()) {
                return;
            }

            if (!ec) {
                sthis->m_connections->start(
                    std::make_shared<Connection>(std::move(sthis->m_socket), *sthis->m_callback,
                                                 *sthis->m_connections),
                    sthis->m_timeout);
            }
        }

        sthis->waitAccept();
    });
}

Server::~Server() {
    // shutdown server when it is destructed
    shutdown();
}

bool Server::isRunning() const {
    return m_running.load();
}

void Server::shutdown() {
    if (isRunning()) {
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            // The server is stopped by canceling all outstanding asynchronous
            // operations. Once all operations have finished the io_service::run()
            // call will exit.
            boost::system::error_code ignored;
            m_acceptor.close(ignored);
            m_connections->stopAll();
        }

        // Once acceptor, connections and socket are closed
        // Run result should be ready to return
        // Clean up the run result
        m_runResult.get();

        m_running = false;
    }
}

uint16_t Server::getPortNumber() const {
    return m_port;
}

void Server::setTimeout(boost::asio::deadline_timer::duration_type const& timeout) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_timeout = timeout;
}

size_t Server::getNumActiveConnections() {
    return m_connections->getNumActiveConnections();
}

} // namespace gazebotransport
} // namespace robotics
