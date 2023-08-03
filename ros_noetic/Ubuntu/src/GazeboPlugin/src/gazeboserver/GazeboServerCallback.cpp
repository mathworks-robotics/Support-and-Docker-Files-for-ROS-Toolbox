/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/GazeboServerCallback.hpp"
#include "gazebotransport/PacketEnding.hpp"
#include "gazebotransport/gazeboserver/MsgDispatcher.hpp"
#include <boost/asio.hpp>

namespace robotics {
namespace gazebotransport {

GazeboServerCallback::GazeboServerCallback(std::shared_ptr<MsgDispatcher> dispatcher)
    : m_dispatcher(dispatcher) {
}

std::string GazeboServerCallback::operator()(boost::asio::streambuf const& buffer,
                                             boost::system::error_code const& ec,
                                             size_t len) {
    std::string replyString;

    // check whether error happened and make sure the received buffer length (len)
    // is greater than the end-of-message length
    if (!ec && len >= EndOfMessage::getStr().size()) {
        std::string ss{boost::asio::buffers_begin(buffer.data()),
                       boost::asio::buffers_end(buffer.data()) -
                           static_cast<std::ptrdiff_t>(
                               EndOfMessage::getStr().size())}; // Separate delimiter from buffer
        mw::internal::robotics::gazebotransport::Packet request;
        bool ret = request.ParseFromString(ss); // De-Serialize into Packet message
        if (ret) {
            replyString = m_dispatcher->handleMessage(request.header().id(), request);
        } else {
            mw::internal::robotics::gazebotransport::Packet replyMsg;
            replyMsg.mutable_header()->set_id(mw::internal::robotics::gazebotransport::
                                                  PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
            replyMsg.mutable_header()->mutable_time_stamp()->set_seconds(0);
            replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
            replyMsg.set_status(
                mw::internal::robotics::gazebotransport::Packet_CoSimError_MSG_INVALID);
            replyString = replyMsg.SerializeAsString();
        }

    } else {
        mw::internal::robotics::gazebotransport::Packet replyMsg;
        replyMsg.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
        replyMsg.mutable_header()->mutable_time_stamp()->set_seconds(0);
        replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        replyMsg.set_status(
            mw::internal::robotics::gazebotransport::Packet_CoSimError_SOCKET_FAILED);
        replyString = replyMsg.SerializeAsString();
    }

    return replyString;
}
} // namespace gazebotransport
} // namespace robotics
