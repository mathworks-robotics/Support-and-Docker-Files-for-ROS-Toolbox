/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_CALLBACK_HPP
#define GAZEBOCOSIM_CALLBACK_HPP

#include "gazebotransport/gazebotransport_util.hpp"

#include "boost/asio.hpp"
#include <memory>
#include <stddef.h> // for size_t

namespace robotics {
namespace gazebotransport {

/// Callback functor interface to be called when new message is received on the Server
class GAZEBOTRANSPORT_EXPORT_CLASS Callback {
  public:
    /// Functor to be called
    /**
    @param buffer                boost buffer that contains the message as byte array
    @param ec                    error code in the boost asio read operation
    @param len                   number of bytes of the message stored in buffer
    */
    virtual std::string operator()(boost::asio::streambuf const& buffer,
                                   boost::system::error_code const& ec,
                                   size_t len) = 0;

    /// Destructor
    virtual ~Callback() = default;
};

/// Smart pointer wrapper for Callback class
typedef std::shared_ptr<Callback> CallbackPtr;
} // namespace gazebotransport
} // namespace robotics

#endif
