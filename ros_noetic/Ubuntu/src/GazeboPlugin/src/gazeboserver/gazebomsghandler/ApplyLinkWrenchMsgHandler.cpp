/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/ApplyLinkWrenchMsgHandler.hpp"

namespace robotics {
namespace gazebotransport {

ApplyLinkWrenchMsgHandler::ApplyLinkWrenchMsgHandler(
    gazebo::physics::WorldPtr ptr,
    robotics::gazebotransport::GazeboApplyCommander& commander)
    : m_ptr(ptr)
    , m_commander(commander) {
}
ApplyLinkWrenchMsgHandler::~ApplyLinkWrenchMsgHandler() {
}

std::string ApplyLinkWrenchMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {

    mw::internal::robotics::gazebotransport::Packet replyMsg;
    replyMsg.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds((uint64_t)m_ptr->SimTime().sec);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(
        (uint64_t)m_ptr->SimTime().nsec);

    gazebo::physics::ModelPtr m_model =
        m_ptr->ModelByName(msgContent.apply_link_wrench().model_name());

    std::string forceType = msgContent.apply_link_wrench().force_type();
    std::string torqueType = msgContent.apply_link_wrench().torque_type();
    gazebo::physics::LinkPtr _link;

    if (m_model) {
        _link = m_model->GetLink(msgContent.apply_link_wrench().link_name()); // Retrieve link

        if (_link) {
            if (forceType == "SET" || forceType == "ADD") {
                if (torqueType == "SET" || torqueType == "ADD") {
                    // end time for the apply
                    auto const& duration = msgContent.apply_link_wrench().duration();
                    ::gazebo::common::Time durationTimeSeconds{
                        static_cast<double>(duration.seconds())};
                    ::gazebo::common::Time durationTImeNanoSeconds{
                        0, static_cast<int32_t>(duration.nano_seconds())};
                    ::gazebo::common::Time endTime =
                        m_ptr->SimTime() + durationTimeSeconds + durationTImeNanoSeconds;

                    auto command = std::make_shared<robotics::gazebotransport::LinkPtrStorage>(
                        _link, endTime, msgContent);

                    if (forceType == "SET") {
                        // adding link pointer storage for SET force map ( overwriting on same link
                        // name )
                        m_commander.insertSetLinkCommand(
                            msgContent.apply_link_wrench().model_name() +
                                msgContent.apply_link_wrench().link_name(),
                            "FORCE", command);
                    } else if (forceType == "ADD") {
                        // adding link pointer storage for ADD force map ( creating new for same
                        // link name )
                        m_commander.insertAddLinkCommand("FORCE", command);
                    }

                    if (torqueType == "SET") {
                        // adding link pointer storage for SET torque map ( overwriting on same link
                        // name )
                        m_commander.insertSetLinkCommand(
                            msgContent.apply_link_wrench().model_name() +
                                msgContent.apply_link_wrench().link_name(),
                            "TORQUE", command);
                    } else if (torqueType == "ADD") {
                        // adding link pointer storage for ADD torque map ( creating new for same
                        // link name )
                        m_commander.insertAddLinkCommand("TORQUE", command);
                    }

                    replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                            Packet_CoSimError_NONE); // NO ERROR
                } else {
                    replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                            Packet_CoSimError_TORQUE_TYPE_INVALID); // TORQUE TYPE
                                                                                    // is incorrect
                }
            } else {
                replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                        Packet_CoSimError_FORCE_TYPE_INVALID); // FORCE TYPE is
                                                                               // incorrect
            }
        } else {
            replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                    Packet_CoSimError_LINK_NAME_INVALID); // LINK not available
        }
    } else {
        replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                Packet_CoSimError_MODEL_NAME_INVALID); // MODEL not available
    }

    return replyMsg.SerializeAsString();
}

uint32_t ApplyLinkWrenchMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_APPLY_LINK_WRENCH; // Message ID from MsgTable
}
} // namespace gazebotransport
} // namespace robotics
