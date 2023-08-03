/* Copyright 2021 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/GetGazeboModelSDFMsgHandler.hpp"

namespace robotics {
namespace gazebotransport {

GetGazeboModelSDFMsgHandler::GetGazeboModelSDFMsgHandler(gazebo::physics::WorldPtr ptr)
    : m_ptr(ptr)
    , m_utils(std::make_shared<GazeboMLUtils>()) {
}
GetGazeboModelSDFMsgHandler::~GetGazeboModelSDFMsgHandler() {
}

std::string GetGazeboModelSDFMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    mw::internal::robotics::gazebotransport::Packet replyMsg;

    replyMsg.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                          PacketHeader_MsgID_GAZEBO_MODEL_SDF);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds((uint64_t)m_ptr->SimTime().sec);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(
        (uint64_t)m_ptr->SimTime().nsec);

    std::string model_name = msgContent.get_gazebo_model_sdf().model_name();

    gazebo::physics::ModelPtr model = m_ptr->ModelByName(model_name); // Retrieve model

    if (model) {
        // get and add model name from Gazebo
        replyMsg.mutable_gazebo_model_sdf()->set_model_name(model_name);
        // get requested model SDF
        auto modelSDF = model->GetSDF();
        std::shared_ptr<sdf::SDF> gazeboModel;
        gazeboModel = std::make_shared<sdf::SDF>();
        sdf::init(gazeboModel);
        gazeboModel->Root()->InsertElement(modelSDF);
        // get SDF string and add to reply message
        replyMsg.mutable_gazebo_model_sdf()->set_sdf_string(gazeboModel->ToString());
    } else {
        // if model does not exist then add empty string in the
        // model name and SDF string fields
        replyMsg.mutable_gazebo_model_sdf()->set_model_name("");
        replyMsg.mutable_gazebo_model_sdf()->set_sdf_string("");
    }

    return replyMsg.SerializeAsString();
}

uint32_t GetGazeboModelSDFMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_GET_GAZEBO_MODEL_SDF; // Message ID from MsgTable
}
} // namespace gazebotransport
} // namespace robotics
