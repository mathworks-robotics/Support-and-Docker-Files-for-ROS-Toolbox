/* Copyright 2019 The MathWorks, Inc. */
#ifndef BOOST_VERSION_SUPPORT_HPP
#define BOOST_VERSION_SUPPORT_HPP
#include <boost/version.hpp>

#include "boost/optional.hpp"
#include "boost/asio.hpp"

#if BOOST_VERSION <= 106501
namespace boost {
namespace asio {
typedef io_service io_context;
}
} // namespace boost
#endif
#endif
