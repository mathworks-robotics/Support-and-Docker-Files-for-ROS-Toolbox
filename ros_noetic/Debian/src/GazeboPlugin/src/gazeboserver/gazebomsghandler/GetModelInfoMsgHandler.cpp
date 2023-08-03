/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/GetModelInfoMsgHandler.hpp"

namespace robotics {
namespace gazebotransport {

GetModelInfoMsgHandler::GetModelInfoMsgHandler(gazebo::physics::WorldPtr ptr)
    : m_ptr(ptr) {
}
GetModelInfoMsgHandler::~GetModelInfoMsgHandler() {
}

std::string GetModelInfoMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    mw::internal::robotics::gazebotransport::Packet replyMsg;
    replyMsg.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_MODEL_INFO);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds((uint64_t)m_ptr->SimTime().sec);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(
        (uint64_t)m_ptr->SimTime().nsec);

    std::vector<gazebo::physics::ModelPtr> models =
        m_ptr->Models(); // Retrieve all Model pointer from Gazebo world pointer

    if (models.size() > 0) {
        for (size_t i = 0; i < models.size(); ++i) // Loop for number of models available in Gazebo
        {
            gazebo::physics::ModelPtr _model = models[i]; // Retrieves i Th Model pointer
            replyMsg.mutable_model_info()->add_model_data()->set_model_name(
                _model->GetName()); // Get i Th Model name and store in Model message

            std::vector<gazebo::physics::LinkPtr> links =
                _model->GetLinks(); // Get all links of i Th Model
            for (size_t j = 0; j < links.size(); ++j) {
                gazebo::physics::LinkPtr link0 = links[j]; // Get j Th link of i Th Model
                replyMsg.mutable_model_info()
                    ->mutable_model_data(i)
                    ->mutable_links()
                    ->add_link_name(link0->GetName());
            }
            std::vector<gazebo::physics::JointPtr> joints =
                _model->GetJoints(); // Get all joints of i Th Model
            for (size_t j = 0; j < joints.size(); ++j) {
                gazebo::physics::JointPtr joint0 = joints[j]; // Get j Th joint of i Th Model
                replyMsg.mutable_model_info()
                    ->mutable_model_data(i)
                    ->mutable_joints()
                    ->add_joint_name(joint0->GetName());
            }
        }
    } else {
        replyMsg.mutable_model_info()->add_model_data()->set_model_name("");
    }

    return replyMsg.SerializeAsString(); // returns serialized new ModelInfo message
}

uint32_t GetModelInfoMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_GET_MODEL_INFO;
}
} // namespace gazebotransport
} // namespace robotics
