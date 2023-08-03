/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_LINKPTRSTORAGE_HPP
#define GAZEBOCOSIM_LINKPTRSTORAGE_HPP

#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

namespace robotics {
namespace gazebotransport {

class LinkPtrStorage {
  public:
    /// Constructor
    /// This class stores Gazebo link pointer for apply force/torque on link functionality
    /// It is used further to reset/subtract force/torque based on user input duration value
    /*
      @param gazebo::physics::LinkPtr       Gazebo link pointer
      @param endTime                        End time for the link storage
      @param Packet                         Packet message for apply wrench command
      */
    LinkPtrStorage(gazebo::physics::LinkPtr linkPtr,
                   ::gazebo::common::Time endTime,
                   mw::internal::robotics::gazebotransport::Packet const& msgContent)
        : m_linkPtr(linkPtr)
        , m_endTime(endTime)
        , m_msgContent(msgContent) {
    }

    // Gazebo link pointer
    ::gazebo::physics::LinkPtr m_linkPtr;
    // End time
    ::gazebo::common::Time m_endTime;
    // Packet message for apply wrench command
    mw::internal::robotics::gazebotransport::Packet m_msgContent;
};

} // namespace gazebotransport
} // namespace robotics
#endif
