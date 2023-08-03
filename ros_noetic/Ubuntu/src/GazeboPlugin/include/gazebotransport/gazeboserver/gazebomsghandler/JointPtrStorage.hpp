/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_JOINTPTRSTORAGE_HPP
#define GAZEBOCOSIM_JOINTPTRSTORAGE_HPP

#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

namespace robotics {
namespace gazebotransport {

class JointPtrStorage {
  public:
    /// Constructor
    /// This class stores Gazebo joint pointer for apply torque on joint functionality
    /// It is used further to reset torque based on user input duration value
    /*
      @param gazebo::physics::JointPtr      Gazebo joint pointer
      @param endTime                        Simulation end time
      @param Packet                         Packet message for apply torque command
      */
    JointPtrStorage(gazebo::physics::JointPtr jointPtr,
                    ::gazebo::common::Time endTime,
                    mw::internal::robotics::gazebotransport::Packet const& msgContent)
        : m_jointPtr(jointPtr)
        , m_endTime(endTime)
        , m_msgContent(msgContent) {
    }

    // Gazebo joint pointer
    ::gazebo::physics::JointPtr m_jointPtr;
    // End time
    ::gazebo::common::Time m_endTime;
    // Packet message for apply torque command
    mw::internal::robotics::gazebotransport::Packet m_msgContent;
};

} // namespace gazebotransport
} // namespace robotics
#endif
