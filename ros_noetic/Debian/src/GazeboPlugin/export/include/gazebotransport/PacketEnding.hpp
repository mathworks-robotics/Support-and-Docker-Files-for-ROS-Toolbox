/* Copyright 2019-2021 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_PACKET_ENDING_HPP
#define GAZEBOCOSIM_PACKET_ENDING_HPP

#include "gazebotransport/gazebotransport_util.hpp"
#include "gazebotransport/mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"
#include <string>

namespace robotics {
namespace gazebotransport {
/// EndOfMessage identifies the end of each message
class GAZEBOTRANSPORT_EXPORT_CLASS EndOfMessage {
  public:
    static std::string const& getStr() {
        static std::string const eom = "\r\n\r\n";
        return eom;
    }
};

/// EndOfSession identifies a session ending message
class GAZEBOTRANSPORT_EXPORT_CLASS EndOfSession {
  public:
    static std::string const& getStr() {
        static std::string const eos = "EndSession";
        return eos;
    }
};

/// HandshakeMessage this message is exchanged when a client connects a server
/// This message is required to check plugin version at client and server end are same
class GAZEBOTRANSPORT_EXPORT_CLASS HandshakeMessage {
  public:
    static std::string const& getStr() {
        mw::internal::robotics::gazebotransport::PluginVersion
            versionMsg; // get MATLAB supported version string
        static std::string const handshake = "HandShakeGazeboCoSimPlugin" + versionMsg.version();
        return handshake;
    }
};

} // namespace gazebotransport
} // namespace robotics

#endif /*GAZEBOCOSIM_PACKET_ENDING_HPP*/
