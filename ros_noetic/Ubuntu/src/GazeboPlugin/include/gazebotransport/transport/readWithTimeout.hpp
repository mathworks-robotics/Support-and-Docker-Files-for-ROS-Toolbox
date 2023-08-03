/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_READ_WITH_TIMEOUT_HPP
#define GAZEBOCOSIM_READ_WITH_TIMEOUT_HPP

#include <string>

#include "boost/optional.hpp"
#include "boost/asio.hpp"
#include "boost/date_time.hpp"

#include "gazebotransport/BoostVersionSupport.hpp"

namespace robotics {
namespace gazebotransport {
/// Synchronized (blocking) read from TCP/IP socket with time out
/**
@param io                        ASIO service queue
@param socket                    TCP/IP socket, must be open when called
@param timeout                   Maximum wait time allowed for the read operation

@return                          Returns the received message if it is received
                                 within timeout. Otherwise return boost::none
*/
boost::optional<std::string> readWithTimeOut(
    boost::asio::io_context& io,
    boost::asio::ip::tcp::socket& socket,
    boost::asio::deadline_timer::duration_type const& timeout);
} // namespace gazebotransport
} // namespace robotics

#endif
