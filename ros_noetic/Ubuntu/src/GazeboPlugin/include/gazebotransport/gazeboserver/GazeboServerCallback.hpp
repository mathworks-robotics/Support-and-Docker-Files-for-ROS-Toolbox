/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOSERVERCALLBACK_HPP
#define GAZEBOSERVERCALLBACK_HPP

#include "gazebotransport/Callback.hpp"
#include "gazebotransport/gazeboserver/MsgDispatcher.hpp"

namespace robotics {
namespace gazebotransport {
class GazeboServerCallback : public Callback {
  public:
    /// constructor
    /**
    @param dispatcher       server ip address

    Called at the time of GazeboServer initialization. This class initiates MsgDispatcher and
    handles messages.
    */
    GazeboServerCallback(std::shared_ptr<MsgDispatcher> dispatcher);

    /// Handles input String
    /**
    @param buffer   received string
    @param ec       boost error code
    @param len      size of received string

    This function deserialize input string (string received from client) into Packet message.The
    Packet message contains MsgID and Content. Then, MsgID and Content are passed to handleMessage
    of MsgDispatcher, where based on MsgID, respective handleMessage is called. Further, a reply
    message will be returned from handleMessage, which is serialized and returned as string.
    */
    std::string operator()(boost::asio::streambuf const& buffer,
                           boost::system::error_code const& ec,
                           size_t len) override; //

  private:
    /// Message dispatcher object
    std::shared_ptr<MsgDispatcher> m_dispatcher;
};
} // namespace gazebotransport
} // namespace robotics
#endif
