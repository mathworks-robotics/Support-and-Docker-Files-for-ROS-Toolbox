/* Copyright 2019 The MathWorks, Inc. */

#include "gazebotransport/transport/readWithTimeout.hpp"
#include "gazebotransport/PacketEnding.hpp"

namespace robotics {
namespace gazebotransport {
boost::optional<std::string> readWithTimeOut(
    boost::asio::io_context& io,
    boost::asio::ip::tcp::socket& socket,
    boost::asio::deadline_timer::duration_type const& timeout) {
    // reset io
    io.reset();

    auto reply = std::make_shared<boost::optional<std::string>>(boost::none);

    // synchronized read from Server with timeout
    auto readTimer = std::make_shared<boost::asio::deadline_timer>(io);
    readTimer->expires_from_now(timeout);
    readTimer->async_wait([&socket](const boost::system::error_code& ec) {
        if (ec != boost::asio::error::operation_aborted) {
            // once timeout is reached, cancel all pending operations
            socket.cancel();
        }
    });

    auto buffer = std::make_shared<boost::asio::streambuf>();
    boost::asio::async_read_until(
        socket, *buffer, EndOfMessage::getStr(),
        [readTimer, reply, buffer](const boost::system::error_code& ec, size_t len) {
            // fill-in the error if read is not canceled
            // check whether error happened and make sure the received buffer length 
            // (len) is greater than the end-of-message length
            if (!ec && len >= EndOfMessage::getStr().size()) {
                // once read completes successfully, cancel the timer
                readTimer->cancel();

                // fill-in the message if read operation is completed
                // remove the end-of-message separator from the reply
                *reply = std::string(boost::asio::buffers_begin(buffer->data()),
                                     boost::asio::buffers_end(buffer->data()) -
                                        static_cast<ptrdiff_t>(EndOfMessage::getStr().size()));
            }
        });

    // start running the two async operation
    io.run();

    return *reply;
}
} // namespace gazebotransport
} // namespace robotics
