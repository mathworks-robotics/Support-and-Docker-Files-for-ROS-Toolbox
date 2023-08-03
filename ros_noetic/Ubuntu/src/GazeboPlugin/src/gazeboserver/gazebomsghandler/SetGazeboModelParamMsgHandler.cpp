/* Copyright 2020 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebomsghandler/SetGazeboModelParamMsgHandler.hpp"

namespace robotics {
namespace gazebotransport {

SetGazeboModelParamMsgHandler::SetGazeboModelParamMsgHandler(gazebo::physics::WorldPtr ptr)
    : m_ptr(ptr)
    , m_utils(std::make_shared<GazeboMLUtils>()) {
}

SetGazeboModelParamMsgHandler::~SetGazeboModelParamMsgHandler() {
}

std::shared_ptr<mw::internal::robotics::gazebotransport::ML_Pose>
SetGazeboModelParamMsgHandler::retainPoseComponents(
    ignition::math::Pose3d gazebo_pose,
    mw::internal::robotics::gazebotransport::ML_Pose const& poseMsg) {
    std::shared_ptr<mw::internal::robotics::gazebotransport::ML_Pose> model_pose;
    model_pose = std::make_shared<mw::internal::robotics::gazebotransport::ML_Pose>();
    m_utils->ConvertMLPose(model_pose.get(), gazebo_pose);
    // add user input position values if present
    if (poseMsg.has_position()) {
        mw::internal::robotics::gazebotransport::ML_Point ml_position = poseMsg.position();
        model_pose->mutable_position()->CopyFrom(ml_position);
    }
    // add user input orientation values if present
    if (poseMsg.has_orientation()) {
        mw::internal::robotics::gazebotransport::ML_Quat ml_orientation = poseMsg.orientation();
        model_pose->mutable_orientation()->CopyFrom(ml_orientation);
    }
    return model_pose;
}

void SetGazeboModelParamMsgHandler::SetLinkParam(
    mw::internal::robotics::gazebotransport::ML_Links linkMessage,
    gazebo::physics::LinkPtr link) {
    // set Model-Link 'Gravity' parameter
    if (linkMessage.has_gravity()) {
        link->SetGravityMode(linkMessage.gravity());
    }
    // set Model-Link 'Self-Collide' parameter
    if (linkMessage.has_self_collide()) {
        link->SetSelfCollide(linkMessage.self_collide());
    }
    // set Model-Link 'Kinematic' parameter
    if (linkMessage.has_kinematic()) {
        link->SetKinematic(linkMessage.kinematic());
    }
    // set Model-Link 'EnableWind' parameter
    if (linkMessage.has_enabled_wind()) {
        link->SetWindMode(linkMessage.enabled_wind());
    }
    // set Model-Link 'Canonical' parameter
    if (linkMessage.has_canonical()) {
        link->SetCanonicalLink(linkMessage.canonical());
    }
    // set Model-Link 'IsStatic' parameter
    if (linkMessage.has_is_static()) {
        link->SetStatic(linkMessage.is_static());
    }
    // set Model-Link 'Pose' parameter
    if (linkMessage.has_pose()) {
        // get previous pose details
        ignition::math::Pose3d world_pose = link->WorldPose();
        auto linkPose = retainPoseComponents(world_pose, linkMessage.pose());

        link->SetWorldPose(m_utils->ConvertIgn(*linkPose.get()));
    }
    // set Model-Link 'Inertial' parameters
    if (linkMessage.has_inertial()) {
        gazebo::physics::InertialPtr inertial_ptr = link->GetInertial();

        if (linkMessage.inertial().has_mass()) {
            inertial_ptr->SetMass(linkMessage.inertial().mass());
        }
        if (linkMessage.inertial().has_ixx()) {
            inertial_ptr->SetIXX(linkMessage.inertial().ixx());
        }
        if (linkMessage.inertial().has_ixy()) {
            inertial_ptr->SetIXY(linkMessage.inertial().ixy());
        }
        if (linkMessage.inertial().has_ixz()) {
            inertial_ptr->SetIXZ(linkMessage.inertial().ixz());
        }
        if (linkMessage.inertial().has_iyy()) {
            inertial_ptr->SetIYY(linkMessage.inertial().iyy());
        }
        if (linkMessage.inertial().has_iyz()) {
            inertial_ptr->SetIYZ(linkMessage.inertial().iyz());
        }
        if (linkMessage.inertial().has_izz()) {
            inertial_ptr->SetIZZ(linkMessage.inertial().izz());
        }

        link->SetInertial(inertial_ptr);
    }
}

std::string SetGazeboModelParamMsgHandler::GetJointTypeName(gazebo::msgs::Joint::Type jointType) {
    std::string jointTypeString = "";
    switch (jointType) {
    case gazebo::msgs::Joint_Type_REVOLUTE:
        jointTypeString = "REVOLUTE";
        break;
    case gazebo::msgs::Joint_Type_REVOLUTE2:
        jointTypeString = "REVOLUTE2";
        break;
    case gazebo::msgs::Joint_Type_PRISMATIC:
        jointTypeString = "PRISMATIC";
        break;
    case gazebo::msgs::Joint_Type_UNIVERSAL:
        jointTypeString = "UNIVERSAL";
        break;
    case gazebo::msgs::Joint_Type_BALL:
        jointTypeString = "BALL";
        break;
    case gazebo::msgs::Joint_Type_SCREW:
        jointTypeString = "SCREW";
        break;
    case gazebo::msgs::Joint_Type_GEARBOX:
        jointTypeString = "GEARBOX";
        break;
    case gazebo::msgs::Joint_Type_FIXED:
        jointTypeString = "FIXED";
        break;
    }
    return jointTypeString;
}

std::string SetGazeboModelParamMsgHandler::SetJointParam(
    mw::internal::robotics::gazebotransport::ML_Joints jointMessage,
    gazebo::physics::JointPtr joint) {

    // get joint type name
    std::string jointTypeName = GetJointTypeName(joint->GetMsgType());
    // add joint type name is error message which is used on client side
    std::string errorMessage = jointTypeName + ":";

    // set Model-Joint 'Pose' parameter
    if (jointMessage.has_pose()) {
        // get previous pose details
        ignition::math::Pose3d init_anchor_pose = joint->InitialAnchorPose();
        auto jointPose = retainPoseComponents(init_anchor_pose, jointMessage.pose());

        gazebo::physics::LinkPtr childLink = joint->GetChild();
        gazebo::physics::LinkPtr parentLink = joint->GetParent();
        joint->Load(childLink, parentLink, m_utils->ConvertIgn(*jointPose.get()));
    }
    // set Model-Joint parameters
    if (joint->DOF() == 1) {
        if (jointMessage.has_fudge_factor()) {
            joint->SetParam("fudge_factor", 0, jointMessage.fudge_factor());
        }
        if (jointMessage.has_cfm()) {
            joint->SetParam("cfm", 0, jointMessage.cfm());
        }
        if (jointMessage.has_suspension_erp()) {
            joint->SetParam("suspension_erp", 0, jointMessage.suspension_erp());
            if (joint->GetParam("suspension_erp", 0) != jointMessage.suspension_erp()) {
                errorMessage += "SuspensionERP;";
            }
        }
        if (jointMessage.has_suspension_cfm()) {
            joint->SetParam("suspension_cfm", 0, jointMessage.suspension_cfm());
            if (joint->GetParam("suspension_cfm", 0) != jointMessage.suspension_cfm()) {
                errorMessage += "SuspensionCFM;";
            }
        }
    }
    // set Model-Joint parameters
    if (joint->DOF() == 2) {
        if (jointMessage.has_fudge_factor()) {
            joint->SetParam("fudge_factor", 1, jointMessage.fudge_factor());
        }
        if (jointMessage.has_cfm()) {
            joint->SetParam("cfm", 1, jointMessage.cfm());
        }
        if (jointMessage.has_suspension_erp()) {
            joint->SetParam("suspension_erp", 1, jointMessage.suspension_erp());
            if (joint->GetParam("suspension_erp", 1) != jointMessage.suspension_erp()) {
                errorMessage += "SuspensionERP;";
            }
        }
        if (jointMessage.has_suspension_cfm()) {
            joint->SetParam("suspension_cfm", 1, jointMessage.suspension_cfm());
            if (joint->GetParam("suspension_cfm", 1) != jointMessage.suspension_cfm()) {
                errorMessage += "SuspensionCFM;";
            }
        }
    }
    // set Model-Joint axis 0 parameters
    if (jointMessage.has_axis1()) {


        if (jointMessage.axis1().has_xyz()) {
            joint->SetAxis(0, m_utils->ConvertIgn(jointMessage.axis1().xyz()));
        }

        if (jointMessage.axis1().has_friction()) {
            joint->SetParam("friction", 0, jointMessage.axis1().friction());
        }

        if (jointMessage.axis1().has_damping()) {
            joint->SetDamping(0, jointMessage.axis1().damping());
        }

        if (jointMessage.axis1().has_angle()) {
            if (!joint->SetPosition(0, jointMessage.axis1().angle())) {
                errorMessage += "Angle;";
            }
        }
    }
    // set Model-Joint axis 1 parameters
    if (jointMessage.has_axis2()) {

        if (jointMessage.axis2().has_xyz()) {
            joint->SetAxis(1, m_utils->ConvertIgn(jointMessage.axis2().xyz()));
        }

        if (jointMessage.axis2().has_friction()) {
            joint->SetParam("friction", 1, jointMessage.axis2().friction());
        }

        if (jointMessage.axis2().has_damping()) {
            joint->SetDamping(1, jointMessage.axis2().damping());
        }

        if (jointMessage.axis2().has_angle()) {
            if (!joint->SetPosition(1, jointMessage.axis2().angle())) {
                errorMessage += "Angle;";
            }
        }
    }

    return errorMessage;
}


std::string SetGazeboModelParamMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {

    mw::internal::robotics::gazebotransport::Packet replyMsg;

    replyMsg.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds((uint64_t)m_ptr->SimTime().sec);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(
        (uint64_t)m_ptr->SimTime().nsec);

    std::string model_name = msgContent.gazebo_model().name();

    gazebo::physics::ModelPtr model = m_ptr->ModelByName(model_name); // Retrieve model

    if (model) {
        // set Model parameters
        if (msgContent.gazebo_model().has_pose()) {
            // get previous pose details
            ignition::math::Pose3d world_pose = model->WorldPose();
            auto modelPose = retainPoseComponents(world_pose, msgContent.gazebo_model().pose());

            model->SetWorldPose(m_utils->ConvertIgn(*modelPose.get()));
        }

        if (msgContent.gazebo_model().has_enable_wind()) {
            model->SetWindMode(msgContent.gazebo_model().enable_wind());
        }

        if (msgContent.gazebo_model().has_self_collide()) {
            model->SetSelfCollide(msgContent.gazebo_model().self_collide());
        }

        if (msgContent.gazebo_model().has_is_static()) {
            model->SetStatic(msgContent.gazebo_model().is_static());
        }
        // set Model-Link parameters
        bool linkExists = true; // status of model-link existence
        for (int linkId = 0; linkId < msgContent.gazebo_model().links_size(); linkId++) {
            gazebo::physics::LinkPtr link =
                model->GetLink(msgContent.gazebo_model().links(linkId).name()); // Retrieve link

            if (link) {
                SetLinkParam(msgContent.gazebo_model().links(linkId), link);
            } else {
                linkExists = false;
            }
        }
        // set Model-Joint parameters
        bool jointExists = true;       // status of model-joint existence
        bool jointAxisNone = false;    // status of joint with no axis
        bool jointAxisInvalid = false; // status of joint axis invalid
        for (int jointId = 0; jointId < msgContent.gazebo_model().joints_size(); jointId++) {
            gazebo::physics::JointPtr joint =
                model->GetJoint(msgContent.gazebo_model().joints(jointId).name()); // Retrieve joint

            if (joint) {
                mw::internal::robotics::gazebotransport::ML_Joints jointMessage =
                    msgContent.gazebo_model().joints(jointId);
                // check joint axis existence
                if ((joint->DOF() == 0 && jointMessage.has_pose()) || joint->DOF() > 0) {
                    if (jointMessage.has_axis2() && joint->DOF() < 2) {
                        jointAxisInvalid = true;
                    } else {

                        replyMsg.set_error_message(SetJointParam(jointMessage, joint));
                    }

                } else {
                    jointAxisNone = true;
                }
            } else {
                jointExists = false;
            }
        }
        // add error message based on model-link-joint-axis existence
        if (jointExists && linkExists && !jointAxisNone && !jointAxisInvalid) {
            replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                    Packet_CoSimError_NONE); //  Success
        } else {
            if (!jointExists) {
                replyMsg.set_status(
                    mw::internal::robotics::gazebotransport::Packet_CoSimError::
                        Packet_CoSimError_JOINT_NAME_INVALID); //  joint name invalid
            }
            if (!linkExists) {
                replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                        Packet_CoSimError_LINK_NAME_INVALID); //  link name invalid
            }
            if (jointAxisNone) {
                replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                        Packet_CoSimError_JOINT_AXIS_NONE); //  joint axis none
            }
            if (jointAxisInvalid) {
                replyMsg.set_status(
                    mw::internal::robotics::gazebotransport::Packet_CoSimError::
                        Packet_CoSimError_INVALID_JOINT_AXIS); //  invalid joint axis
            }
        }
    } else {
        replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                Packet_CoSimError_MODEL_NAME_INVALID); // Failed
    }

    return replyMsg.SerializeAsString();
}

uint32_t SetGazeboModelParamMsgHandler::getAcceptID() const {
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
        PacketHeader_MsgID_SET_GAZEBO_MODEL_PARAM; // Message ID from MsgTable
}
} // namespace gazebotransport
} // namespace robotics
