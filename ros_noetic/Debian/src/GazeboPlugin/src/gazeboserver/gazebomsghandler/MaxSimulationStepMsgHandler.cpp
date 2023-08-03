/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/MaxSimulationStepMsgHandler.hpp"

namespace robotics {
namespace gazebotransport {

MaxSimulationStepMsgHandler::MaxSimulationStepMsgHandler(gazebo::physics::WorldPtr ptr)
    : m_ptr(ptr) {
}
MaxSimulationStepMsgHandler::~MaxSimulationStepMsgHandler() {
}

std::string MaxSimulationStepMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    mw::internal::robotics::gazebotransport::Packet replyMsg;

    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds((uint64_t)m_ptr->SimTime().sec);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(
        (uint64_t)m_ptr->SimTime().nsec);

    if (msgContent.max_step_size().type()) {
        replyMsg.mutable_header()->set_id(mw::internal::robotics::gazebotransport::
                                              PacketHeader_MsgID::PacketHeader_MsgID_MAX_STEP_SIZE);
        replyMsg.mutable_max_step_size()->set_type(
            mw::internal::robotics::gazebotransport::MaxStepSize_TYPE::
                MaxStepSize_TYPE_GET_STEP_SIZE);
        replyMsg.mutable_max_step_size()->set_size(m_ptr->Physics()->GetMaxStepSize());
    } else {
        if (msgContent.max_step_size().type() == 0) {
            m_ptr->Physics()->SetMaxStepSize(msgContent.max_step_size().size());
            replyMsg.mutable_header()->set_id(mw::internal::robotics::gazebotransport::
                                                  PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
            replyMsg.set_status(
                mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);
        } else {
            replyMsg.mutable_header()->set_id(mw::internal::robotics::gazebotransport::
                                                  PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
            replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                    Packet_CoSimError_MAX_STEP_SIZE_ERROR);
        }
    }

    return replyMsg.SerializeAsString();
}

uint32_t MaxSimulationStepMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_MAX_STEP_SIZE; // Message ID from MsgTable
}
} // namespace gazebotransport
} // namespace robotics
