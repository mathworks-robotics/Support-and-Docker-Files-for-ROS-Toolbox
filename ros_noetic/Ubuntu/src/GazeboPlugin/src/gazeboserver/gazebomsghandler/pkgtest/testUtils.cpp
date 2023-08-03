/* Copyright 2020-2021 The MathWorks, Inc. */
#include "testUtils.hpp"

// validate received packet error message
void testUtils::validateSetCommandErrorMsg(mw::internal::robotics::gazebotransport::Packet reply,
                                           errorMsgValidation errorType) {
    switch (errorType) {
    case Success: {
        ASSERT_EQ(reply.status(), mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                      Packet_CoSimError_NONE); // 0: No Error
        break;
    }
    case InvalidModel: {
        ASSERT_EQ(reply.status(),
                  mw::internal::robotics::gazebotransport::Packet_CoSimError::
                      Packet_CoSimError_MODEL_NAME_INVALID); // MODEL_NAME_INVALID error
        break;
    }
    case InvalidJoint: {
        ASSERT_EQ(reply.status(),
                  mw::internal::robotics::gazebotransport::Packet_CoSimError::
                      Packet_CoSimError_JOINT_NAME_INVALID); // JOINT_NAME_INVALID error
        break;
    }
    case InvalidLink: {
        ASSERT_EQ(reply.status(),
                  mw::internal::robotics::gazebotransport::Packet_CoSimError::
                      Packet_CoSimError_LINK_NAME_INVALID); // LINK_NAME_INVALID error
        break;
    }
    case InvalidAxis: {
        ASSERT_EQ(reply.status(),
                  mw::internal::robotics::gazebotransport::Packet_CoSimError::
                      Packet_CoSimError_INVALID_JOINT_AXIS); // INVALID_JOINT_Axis error
        break;
    }
    case NoneAxis: {
        ASSERT_EQ(reply.status(), mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                      Packet_CoSimError_JOINT_AXIS_NONE); // NONE_JOINT_Axis error
        break;
    }
    default:
        break;
    }
}

// validate Gazebo model parameters with ground truth values,
// which are SET with set model param functionality
void testUtils::validateModelResults(gazebo::physics::WorldPtr m_world,
                                     std::string const& modelName,
                                     double (&position)[3],
                                     double (&orientation)[4],
                                     bool paramStatus,
                                     modelParamName paramName) {
    auto model = m_world->ModelByName(modelName);
    // validate for various parameter combinations
    switch (paramName) {
    case Pose: {
        ignition::math::Pose3d world_pose = model->WorldPose();

        EXPECT_DOUBLE_EQ(world_pose.Pos()[0], position[0]);
        EXPECT_DOUBLE_EQ(world_pose.Pos()[1], position[1]);
        EXPECT_DOUBLE_EQ(world_pose.Pos()[2], position[2]);
        EXPECT_DOUBLE_EQ(world_pose.Rot().W(), orientation[0]);
        EXPECT_DOUBLE_EQ(world_pose.Rot().X(), orientation[1]);
        EXPECT_DOUBLE_EQ(world_pose.Rot().Y(), orientation[2]);
        EXPECT_DOUBLE_EQ(world_pose.Rot().Z(), orientation[3]);
    } break;
    case Wind: {
        EXPECT_EQ(model->WindMode(), paramStatus);
    } break;
    case SelfCollide: {
        EXPECT_EQ(model->GetSelfCollide(), paramStatus);
    } break;
    case IsStatic: {
        EXPECT_EQ(model->IsStatic(), paramStatus);
    } break;
    default:
        break;
    }
}

// validate Gazebo model-link parameters with ground truth values,
// which are SET with set model-link param functionality
void testUtils::validateModelLinkResults(gazebo::physics::WorldPtr m_world,
                                         std::string const& modelName,
                                         std::string const& linkName,
                                         double (&position)[3],
                                         double (&orientation)[4],
                                         double (&productOfInertia)[3],
                                         double (&principalMoments)[3],
                                         double mass,
                                         bool paramStatus,
                                         modelLinkParamName paramName) {
    auto model = m_world->ModelByName(modelName);
    auto link = model->GetLink(linkName);

    // validate for various parameter combinations
    switch (paramName) {
    case linkPose: {
        ignition::math::Pose3d world_pose = link->WorldPose();

        EXPECT_DOUBLE_EQ(world_pose.Pos()[0], position[0]);
        EXPECT_DOUBLE_EQ(world_pose.Pos()[1], position[1]);
        EXPECT_DOUBLE_EQ(world_pose.Pos()[2], position[2]);
        EXPECT_DOUBLE_EQ(world_pose.Rot().W(), orientation[0]);
        EXPECT_DOUBLE_EQ(world_pose.Rot().X(), orientation[1]);
        EXPECT_DOUBLE_EQ(world_pose.Rot().Y(), orientation[2]);
        EXPECT_DOUBLE_EQ(world_pose.Rot().Z(), orientation[3]);
    } break;
    case linkWind: {
        EXPECT_EQ(link->WindMode(), paramStatus);
    } break;
    case linkSelfCollide: {
        EXPECT_EQ(link->GetSelfCollide(), paramStatus);
    } break;
    case linkIsStatic: {
        EXPECT_EQ(link->IsStatic(), paramStatus);
    } break;
    case linkCanonical: {
        EXPECT_EQ(link->IsCanonicalLink(), paramStatus);
    } break;
    case linkGravity: {
        EXPECT_EQ(link->GetGravityMode(), paramStatus);
    } break;
    case linkKinematics: {
        EXPECT_EQ(link->GetKinematic(), paramStatus);
    } break;
    case linkMass: {
        gazebo::physics::InertialPtr intertial_ptr = link->GetInertial();
        EXPECT_EQ(intertial_ptr->Mass(), mass);
    } break;
    case linkProdOfInertia: {
        gazebo::physics::InertialPtr intertial_ptr = link->GetInertial();
        EXPECT_EQ(intertial_ptr->IXY(), productOfInertia[0]);
        EXPECT_EQ(intertial_ptr->IXZ(), productOfInertia[1]);
        EXPECT_EQ(intertial_ptr->IYZ(), productOfInertia[2]);
    } break;
    case linkPrincipalMom: {
        gazebo::physics::InertialPtr intertial_ptr = link->GetInertial();
        EXPECT_EQ(intertial_ptr->IXX(), principalMoments[0]);
        EXPECT_EQ(intertial_ptr->IYY(), principalMoments[1]);
        EXPECT_EQ(intertial_ptr->IZZ(), principalMoments[2]);
    } break;
    default:
        break;
    }
}

// validate Gazebo model parameters with ground truth values,
// which are SET with set model param functionality
void testUtils::validateModelJointResults(gazebo::physics::WorldPtr m_world,
                                          std::string const& modelName,
                                          std::string const& jointName,
                                          double (&position)[3],
                                          double (&orientation)[4],
                                          double (&xyz)[3],
                                          double fudgeFactor,
                                          double cfm,
                                          double suspensionCfm,
                                          double suspensionErp,
                                          uint32_t axis,
                                          double angle,
                                          double damping,
                                          double friction,
                                          modelJointParamName paramName) {
    auto model = m_world->ModelByName(modelName);
    auto joint = model->GetJoint(jointName);

    // validate for various parameter combinations
    switch (paramName) {
    case jointPose: {
        ignition::math::Pose3d initial_anchor_pose = joint->InitialAnchorPose();

        EXPECT_DOUBLE_EQ(initial_anchor_pose.Pos()[0], position[0]);
        EXPECT_DOUBLE_EQ(initial_anchor_pose.Pos()[1], position[1]);
        EXPECT_DOUBLE_EQ(initial_anchor_pose.Pos()[2], position[2]);
        EXPECT_DOUBLE_EQ(initial_anchor_pose.Rot().W(), orientation[0]);
        EXPECT_DOUBLE_EQ(initial_anchor_pose.Rot().X(), orientation[1]);
        EXPECT_DOUBLE_EQ(initial_anchor_pose.Rot().Y(), orientation[2]);
        EXPECT_DOUBLE_EQ(initial_anchor_pose.Rot().Z(), orientation[3]);
    } break;
    case jointFudgeFac: {
        EXPECT_EQ(joint->GetParam("fudge_factor", 0), fudgeFactor);
    } break;
    case jointCfm: {
        EXPECT_EQ(joint->GetParam("cfm", 0), cfm);
    } break;
    case jointSupCfm: {
        EXPECT_EQ(joint->GetParam("suspension_cfm", 0), suspensionCfm);
    } break;
    case jointSupErp: {
        EXPECT_EQ(joint->GetParam("suspension_erp", 0), suspensionErp);
    } break;
    case jointAngle: {
        EXPECT_NEAR(joint->Position(axis), angle, 0.001);
    } break;
    case jointXYZ: {

        ignition::math::Vector3d axis_xyz = joint->GlobalAxis(axis);
        EXPECT_EQ(axis_xyz[0], xyz[0]);
        EXPECT_EQ(axis_xyz[1], xyz[1]);
        EXPECT_EQ(axis_xyz[2], xyz[2]);
    } break;
    case jointDamping: {
        EXPECT_EQ(joint->GetDamping(axis), damping);
    } break;
    case jointFriction: {
        EXPECT_EQ(joint->GetParam("friction", axis), friction);
    } break;
    default:
        break;
    }
}

// SET Gazebo model parameters with ground truth values
void testUtils::setGazeboModelParam(gazebo::physics::WorldPtr m_world,
                                    std::string const& modelName,
                                    double (&position)[3],
                                    double (&orientation)[4],
                                    bool paramStatus) {
    gazebo::physics::ModelPtr _Model = m_world->ModelByName(modelName);

    ignition::math::Pose3d pose(position[0], position[1], position[2], orientation[0],
                                orientation[1], orientation[2], orientation[3]);
    // SET Gazebo model parameters using Gazebo C++ API
    _Model->SetWorldPose(pose);
    _Model->SetWindMode(paramStatus);
    _Model->SetSelfCollide(paramStatus);
    _Model->SetStatic(paramStatus);
}

// SET Gazebo model-link parameters with ground truth values
void testUtils::setGazeboModelLinkParam(gazebo::physics::WorldPtr m_world,
                                        std::string const& modelName,
                                        std::string const& linkName,
                                        double (&position)[3],
                                        double (&orientation)[4],
                                        double (&productOfInertia)[3],
                                        double (&principalMoments)[3],
                                        double mass,
                                        bool paramStatus) {
    gazebo::physics::ModelPtr _Model = m_world->ModelByName(modelName);
    gazebo::physics::LinkPtr _link = _Model->GetLink(linkName);

    // SET Gazebo model-link parameters using Gazebo C++ API
    _link->SetGravityMode(paramStatus);
    _link->SetSelfCollide(paramStatus);
    _link->SetKinematic(paramStatus);
    _link->SetWindMode(paramStatus);
    _link->SetCanonicalLink(paramStatus);
    _link->SetStatic(paramStatus);

    ignition::math::Pose3d pose(position[0], position[1], position[2], orientation[0],
                                orientation[1], orientation[2], orientation[3]);
    _link->SetWorldPose(pose);

    gazebo::physics::InertialPtr inertial_ptr = _link->GetInertial();
    inertial_ptr->SetMass(mass);
    inertial_ptr->SetIXY(productOfInertia[0]);
    inertial_ptr->SetIXZ(productOfInertia[1]);
    inertial_ptr->SetIYZ(productOfInertia[2]);

    inertial_ptr->SetIXX(principalMoments[0]);
    inertial_ptr->SetIYY(principalMoments[1]);
    inertial_ptr->SetIZZ(principalMoments[2]);
    _link->SetInertial(inertial_ptr);
}

// SET Gazebo model-joint parameters with ground truth values
void testUtils::setGazeboModelJointParam(gazebo::physics::WorldPtr m_world,
                                         std::string const& modelName,
                                         std::string const& jointName,
                                         double (&position)[3],
                                         double (&orientation)[4],
                                         double (&xyz)[3],
                                         double fudgeFactor,
                                         double cfm,
                                         double suspensionCfm,
                                         double suspensionErp,
                                         uint32_t axis,
                                         double angle,
                                         double damping,
                                         double friction,
                                         modelJointParamName setParamName) {
    gazebo::physics::ModelPtr _Model = m_world->ModelByName(modelName);
    gazebo::physics::JointPtr _joint = _Model->GetJoint(jointName);

    // SET Gazebo model-joint parameters using Gazebo C++ API
    gazebo::physics::LinkPtr childLink = _joint->GetChild();
    gazebo::physics::LinkPtr parentLink = _joint->GetParent();

    ignition::math::Pose3d pose(position[0], position[1], position[2], orientation[0],
                                orientation[1], orientation[2], orientation[3]);

    // SET parameters one at a time using Gazebo C++ API
    switch (setParamName) {
    case jointPose: {
        _joint->Load(childLink, parentLink, pose);
    } break;
    case jointAngle: {
        _joint->SetPosition(axis, angle);
    } break;
    case jointXYZ: {
        ignition::math::Vector3d xyz_val(xyz[0], xyz[1], xyz[2]);
        _joint->SetAxis(axis, xyz_val);
    } break;
    case jointFudgeFac: {
        _joint->SetParam("fudge_factor", axis, fudgeFactor);
        _joint->SetParam("cfm", axis, cfm);
        _joint->SetParam("suspension_erp", axis, suspensionErp);
        _joint->SetParam("suspension_cfm", axis, suspensionCfm);
        _joint->SetParam("friction", axis, friction);
        _joint->SetDamping(axis, damping);
    } break;
    default:
        break;
    }
}

// validate Gazebo model parameters with ground truth values,
// which are received after calling GET gazebo model functionality
void testUtils::validateGetGazeboModelParam(mw::internal::robotics::gazebotransport::Packet reply,
                                            std::string const& modelName,
                                            double (&position)[3],
                                            double (&orientation)[4],
                                            bool paramStatus,
                                            errorMsgValidation errorType) {
    // perform validation based on error type
    switch (errorType) {
    case Success: {
        // validate with ground truth values
        EXPECT_STREQ(reply.gazebo_model().name().c_str(), modelName.c_str());

        mw::internal::robotics::gazebotransport::ML_Pose pose = reply.gazebo_model().pose();
        EXPECT_NEAR(pose.position().x(), position[0], 0.0001);
        EXPECT_NEAR(pose.position().y(), position[1], 0.0001);
        EXPECT_NEAR(pose.position().z(), position[2], 0.0001);

        EXPECT_NEAR(pose.orientation().w(), orientation[0], 0.0001);
        EXPECT_NEAR(pose.orientation().x(), orientation[1], 0.0001);
        EXPECT_NEAR(pose.orientation().y(), orientation[2], 0.0001);
        EXPECT_NEAR(pose.orientation().z(), orientation[3], 0.0001);

        EXPECT_EQ(reply.gazebo_model().enable_wind(), paramStatus);
        EXPECT_EQ(reply.gazebo_model().self_collide(), paramStatus);
        EXPECT_EQ(reply.gazebo_model().is_static(), paramStatus);
    } break;

    case InvalidModel: {
        // model name field should be empty
        EXPECT_STREQ(reply.gazebo_model().name().c_str(), "");
    } break;
    default:
        break;
    }
}

// validate Gazebo model-link parameters with ground truth values,
// which are received after calling GET gazebo model-link functionality
void testUtils::validateGetGazeboModelLinkParam(
    mw::internal::robotics::gazebotransport::Packet reply,
    std::string const& modelName,
    std::string const& linkName,
    double (&position)[3],
    double (&orientation)[4],
    double (&productOfInertia)[3],
    double (&principalMoments)[3],
    double mass,
    bool windStatus,
    bool selfCollideStatus,
    bool isStaticStatus,
    bool canonicalStatus,
    bool gravityStatus,
    bool kinematicStatus,
    errorMsgValidation errorType) {
    // perform validation based on error type
    switch (errorType) {
    case Success: {
        // validate with ground truth values
        EXPECT_STREQ(reply.gazebo_model().name().c_str(), modelName.c_str());
        EXPECT_NE(reply.gazebo_model().links_size(), 0);
        EXPECT_EQ(reply.gazebo_model().joints_size(), 0);
        EXPECT_STREQ(reply.gazebo_model().links(0).name().c_str(), linkName.c_str());

        mw::internal::robotics::gazebotransport::ML_Pose pose =
            reply.gazebo_model().links(0).pose();

        EXPECT_NEAR(pose.position().x(), position[0], 0.0001);
        EXPECT_NEAR(pose.position().y(), position[1], 0.0001);
        EXPECT_NEAR(pose.position().z(), position[2], 0.0001);

        EXPECT_NEAR(pose.orientation().w(), orientation[0], 0.0001);
        EXPECT_NEAR(pose.orientation().x(), orientation[1], 0.0001);
        EXPECT_NEAR(pose.orientation().y(), orientation[2], 0.0001);
        EXPECT_NEAR(pose.orientation().z(), orientation[3], 0.0001);

        mw::internal::robotics::gazebotransport::ML_Inertial inertial =
            reply.gazebo_model().links(0).inertial();

        EXPECT_EQ(inertial.mass(), mass);
        EXPECT_EQ(inertial.ixy(), productOfInertia[0]);
        EXPECT_EQ(inertial.ixz(), productOfInertia[1]);
        EXPECT_EQ(inertial.iyz(), productOfInertia[2]);
        EXPECT_EQ(inertial.ixx(), principalMoments[0]);
        EXPECT_EQ(inertial.iyy(), principalMoments[1]);
        EXPECT_EQ(inertial.izz(), principalMoments[2]);

        EXPECT_EQ(reply.gazebo_model().links(0).enabled_wind(), windStatus);
        EXPECT_EQ(reply.gazebo_model().links(0).self_collide(), selfCollideStatus);
        EXPECT_EQ(reply.gazebo_model().links(0).is_static(), isStaticStatus);
        EXPECT_EQ(reply.gazebo_model().links(0).canonical(), canonicalStatus);
        EXPECT_EQ(reply.gazebo_model().links(0).gravity(), gravityStatus);
        EXPECT_EQ(reply.gazebo_model().links(0).kinematic(), kinematicStatus);
    } break;

    case InvalidModel: {
        // model name field should be empty
        EXPECT_STREQ(reply.gazebo_model().name().c_str(), "");
    } break;

    case InvalidLink: {
        // model name field should be valid
        EXPECT_STREQ(reply.gazebo_model().name().c_str(), modelName.c_str());
        // links and joints field should be empty
        EXPECT_EQ(reply.gazebo_model().links_size(), 0);
        EXPECT_EQ(reply.gazebo_model().joints_size(), 0);
    } break;
    default:
        break;
    }
}

// validate Gazebo model-joint parameters with ground truth values,
// which are received after calling GET gazebo model-joint functionality
void testUtils::validateGetGazeboModelJointParam(
    mw::internal::robotics::gazebotransport::Packet reply,
    std::string const& modelName,
    std::string const& jointName,
    double (&position)[3],
    double (&orientation)[4],
    double (&xyz)[3],
    double fudgeFactor,
    double cfm,
    double suspensionCfm,
    double suspensionErp,
    uint32_t axis,
    double angle,
    double damping,
    double friction,
    errorMsgValidation errorType,
    modelJointParamName setParamName) {
    // perform validation based on error type
    switch (errorType) {
    case Success: {
        EXPECT_STREQ(reply.gazebo_model().name().c_str(), modelName.c_str());
        EXPECT_EQ(reply.gazebo_model().links_size(), 0);
        EXPECT_NE(reply.gazebo_model().joints_size(), 0);
        EXPECT_STREQ(reply.gazebo_model().joints(0).name().c_str(), jointName.c_str());

        if (setParamName == jointPose) {
            mw::internal::robotics::gazebotransport::ML_Pose pose =
                reply.gazebo_model().joints(0).pose();

            EXPECT_NEAR(pose.position().x(), position[0], 0.0001);
            EXPECT_NEAR(pose.position().y(), position[1], 0.0001);
            EXPECT_NEAR(pose.position().z(), position[2], 0.0001);
            EXPECT_NEAR(pose.orientation().w(), orientation[0], 0.0001);
            EXPECT_NEAR(pose.orientation().x(), orientation[1], 0.0001);
            EXPECT_NEAR(pose.orientation().y(), orientation[2], 0.0001);
            EXPECT_NEAR(pose.orientation().z(), orientation[3], 0.0001);
        } else {
            if (setParamName == jointFudgeFac) {
                EXPECT_EQ(reply.gazebo_model().joints(0).cfm(), cfm);
                EXPECT_EQ(reply.gazebo_model().joints(0).fudge_factor(), fudgeFactor);
                EXPECT_EQ(reply.gazebo_model().joints(0).suspension_cfm(), suspensionCfm);
                EXPECT_EQ(reply.gazebo_model().joints(0).suspension_erp(), suspensionErp);
            }
        }

        mw::internal::robotics::gazebotransport::ML_Axis mlAxis;
        if (axis == 0) {
            mlAxis = reply.gazebo_model().joints(0).axis1();
        }
        if (axis == 1) {
            mlAxis = reply.gazebo_model().joints(0).axis2();
        }

        if (axis == 0 || axis == 1) {
            switch (setParamName) {
            case jointXYZ: {
                EXPECT_NEAR(mlAxis.xyz().x(), xyz[0], 0.001);
                EXPECT_NEAR(mlAxis.xyz().y(), xyz[1], 0.001);
                EXPECT_NEAR(mlAxis.xyz().z(), xyz[2], 0.001);
            } break;
            case jointFudgeFac: {
                EXPECT_EQ(mlAxis.damping(), damping);
                EXPECT_EQ(mlAxis.friction(), friction);
            } break;
            case jointAngle: {
                EXPECT_NEAR(mlAxis.angle(), angle, 0.001);
            } break;
            default:
                break;
            }
        }
    } break;

    case InvalidModel: {
        // model name field should be empty
        EXPECT_STREQ(reply.gazebo_model().name().c_str(), "");
    } break;

    case InvalidJoint: {
        // model name field should be valid
        EXPECT_STREQ(reply.gazebo_model().name().c_str(), modelName.c_str());
        // links and joints field should be empty
        EXPECT_EQ(reply.gazebo_model().links_size(), 0);
        EXPECT_EQ(reply.gazebo_model().joints_size(), 0);
    } break;
    default:
        break;
    }
}

/*
 * It creates and sends Get Gazebo model parameters and receives the reply message from server.
 * Further, it creates and returns Packet message from reply message.
 */
mw::internal::robotics::gazebotransport::Packet testUtils::clientGetModelParam(
    std::string const& modelName,
    std::shared_ptr<robotics::gazebotransport::Client> m_client,
    int time_out) {
    /// Create Packet message to Get Model Parameters
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_GET_GAZEBO_MODEL_PARAM);

    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_get_gazebo_model_param()->set_model_name(modelName);

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

    /// Convert reply message to packet message
    mw::internal::robotics::gazebotransport::Packet reply;
    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
    }

    return reply;
}

/*
 * It creates and sends Get Gazebo model-link parameters and receives the reply message from server.
 * Further, it creates and returns Packet message from reply message.
 */
mw::internal::robotics::gazebotransport::Packet testUtils::clientGetModelLinkParam(
    std::string const& modelName,
    std::string const& linkName,
    std::shared_ptr<robotics::gazebotransport::Client> m_client,
    int time_out) {
    /// Create Packet message to Get Model Parameters
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_GET_GAZEBO_MODEL_PARAM);

    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_get_gazebo_model_param()->set_model_name(modelName);
    m_message.mutable_get_gazebo_model_param()->set_is_link(true);
    m_message.mutable_get_gazebo_model_param()->set_link_joint_name(linkName);

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

    /// Convert reply message to packet message
    mw::internal::robotics::gazebotransport::Packet reply;
    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
    }

    return reply;
}

/*
 * It creates and sends Get Gazebo model-joint parameters and receives the reply message from
 * server. Further, it creates and returns Packet message from reply message.
 */
mw::internal::robotics::gazebotransport::Packet testUtils::clientGetModelJointParam(
    std::string const& modelName,
    std::string const& jointName,
    std::shared_ptr<robotics::gazebotransport::Client> m_client,
    int time_out) {
    /// Create Packet message to Get Model Parameters
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_GET_GAZEBO_MODEL_PARAM);

    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_get_gazebo_model_param()->set_model_name(modelName);
    m_message.mutable_get_gazebo_model_param()->set_is_link(false);
    m_message.mutable_get_gazebo_model_param()->set_link_joint_name(jointName);

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

    /// Convert reply message to packet message
    mw::internal::robotics::gazebotransport::Packet reply;
    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
    }

    return reply;
}

/**
 * It creates and sends reset gazebo scene & time message and receives the reply message from
 * server. Further, it creates and returns Packet message from reply message.
 */
mw::internal::robotics::gazebotransport::Packet testUtils::clientResetSceneTime(
    std::shared_ptr<robotics::gazebotransport::Client> m_client,
    int time_out) {
    /// Create Packet message reset simulation time & scene message
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_RESET_SIMULATION);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_reset_simulation()->set_behavior(
        mw::internal::robotics::gazebotransport::
            ResetSimulation_ResetBehavior_RESET_TIME_AND_SCENE);

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

    /// Convert reply message to packet message
    mw::internal::robotics::gazebotransport::Packet reply;

    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
    }

    return reply;
}

/*
 * It creates and sends Set Gazebo model-link parameters and receives the reply message from server.
 * Further, it creates and returns Packet message from reply message.
 */
mw::internal::robotics::gazebotransport::Packet testUtils::clientSetModelLinkParam(
    std::string const& modelName,
    std::string const& linkName,
    double (&position)[3],
    double (&orientation)[4],
    double (&productOfInertia)[3],
    double (&principalMoments)[3],
    double mass,
    bool paramStatus,
    modelLinkParamName paramName,
    std::shared_ptr<robotics::gazebotransport::Client> m_client,
    int time_out) {

    /// Create Packet message to Set Model Parameters
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_SET_GAZEBO_MODEL_PARAM);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_gazebo_model()->set_name(modelName);

    mw::internal::robotics::gazebotransport::ML_Links* link_message =
        m_message.mutable_gazebo_model()->add_links();

    link_message->set_name(linkName);

    // initialize packet message fields based on required parameter names
    switch (paramName) {
    case linkPose: {
        mw::internal::robotics::gazebotransport::ML_Pose* ml_pose = link_message->mutable_pose();

        ml_pose->mutable_position()->set_x(position[0]);
        ml_pose->mutable_position()->set_y(position[1]);
        ml_pose->mutable_position()->set_z(position[2]);
        ml_pose->mutable_orientation()->set_w(orientation[0]);
        ml_pose->mutable_orientation()->set_x(orientation[1]);
        ml_pose->mutable_orientation()->set_y(orientation[2]);
        ml_pose->mutable_orientation()->set_z(orientation[3]);
    } break;

    case linkWind: {
        link_message->set_enabled_wind(paramStatus);
    } break;

    case linkSelfCollide: {
        link_message->set_self_collide(paramStatus);
    } break;

    case linkIsStatic: {
        link_message->set_is_static(paramStatus);
    } break;

    case linkCanonical: {
        link_message->set_canonical(paramStatus);
    } break;

    case linkGravity: {
        link_message->set_gravity(paramStatus);
    } break;

    case linkKinematics: {
        link_message->set_kinematic(paramStatus);
    } break;

    case linkMass: {
        mw::internal::robotics::gazebotransport::ML_Inertial* ml_inertial =
            link_message->mutable_inertial();
        ml_inertial->set_mass(mass);
    } break;

    case linkProdOfInertia: {
        mw::internal::robotics::gazebotransport::ML_Inertial* ml_inertial =
            link_message->mutable_inertial();
        ml_inertial->set_ixy(productOfInertia[0]);
        ml_inertial->set_ixz(productOfInertia[1]);
        ml_inertial->set_iyz(productOfInertia[2]);
    } break;

    case linkPrincipalMom: {
        mw::internal::robotics::gazebotransport::ML_Inertial* ml_inertial =
            link_message->mutable_inertial();
        ml_inertial->set_ixx(principalMoments[0]);
        ml_inertial->set_iyy(principalMoments[1]);
        ml_inertial->set_izz(principalMoments[2]);
    } break;
    default:
        break;
    }

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

    /// Convert reply message to packet message
    mw::internal::robotics::gazebotransport::Packet reply;
    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
    }

    return reply;
}

/*
 * It creates and sends Set Gazebo model parameters and receives the reply message from server.
 * Further, it creates and returns Packet message from reply message.
 */
mw::internal::robotics::gazebotransport::Packet testUtils::clientSetModelParam(
    std::string const& modelName,
    double (&position)[3],
    double (&orientation)[4],
    bool paramStatus,
    modelParamName paramName,
    std::shared_ptr<robotics::gazebotransport::Client> m_client,
    int time_out) {
    /// Create Packet message to Set Model Parameters
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_SET_GAZEBO_MODEL_PARAM);

    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_gazebo_model()->set_name(modelName);

    // initialize packet message fields based on required parameter names
    switch (paramName) {
    case Pose: {
        m_message.mutable_gazebo_model()->mutable_pose()->mutable_position()->set_x(position[0]);
        m_message.mutable_gazebo_model()->mutable_pose()->mutable_position()->set_y(position[1]);
        m_message.mutable_gazebo_model()->mutable_pose()->mutable_position()->set_z(position[2]);
        m_message.mutable_gazebo_model()->mutable_pose()->mutable_orientation()->set_w(
            orientation[0]);
        m_message.mutable_gazebo_model()->mutable_pose()->mutable_orientation()->set_x(
            orientation[1]);
        m_message.mutable_gazebo_model()->mutable_pose()->mutable_orientation()->set_y(
            orientation[2]);
        m_message.mutable_gazebo_model()->mutable_pose()->mutable_orientation()->set_z(
            orientation[3]);
    } break;

    case Wind: {
        m_message.mutable_gazebo_model()->set_enable_wind(paramStatus);
    } break;

    case SelfCollide: {
        m_message.mutable_gazebo_model()->set_self_collide(paramStatus);
    } break;

    case IsStatic: {
        m_message.mutable_gazebo_model()->set_is_static(paramStatus);
    } break;
    default:
        break;
    }

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

    /// Convert reply message to packet message
    mw::internal::robotics::gazebotransport::Packet reply;
    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
    }

    return reply;
}

/*
 * It creates and sends Get Gazebo model-joint parameters and receives the reply message from
 * server. Further, it creates and returns Packet message from reply message.
 */
mw::internal::robotics::gazebotransport::Packet testUtils::clientSetModelJointParam(
    std::string const& modelName,
    std::string const& jointName,
    double (&position)[3],
    double (&orientation)[4],
    double (&xyz)[3],
    double fudgeFactor,
    double cfm,
    double suspensionCfm,
    double suspensionErp,
    uint32_t axis,
    double angle,
    double damping,
    double friction,
    modelJointParamName paramName,
    std::shared_ptr<robotics::gazebotransport::Client> m_client,
    int time_out) {
    /// Create Packet message to Set Model Parameters
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_SET_GAZEBO_MODEL_PARAM);

    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_gazebo_model()->set_name(modelName);

    mw::internal::robotics::gazebotransport::ML_Joints* joint_message =
        m_message.mutable_gazebo_model()->add_joints();

    joint_message->set_name(jointName);

    // initialize packet message fields based on required parameter names
    switch (paramName) {
    case jointPose: {
        mw::internal::robotics::gazebotransport::ML_Pose* ml_pose = joint_message->mutable_pose();

        ml_pose->mutable_position()->set_x(position[0]);
        ml_pose->mutable_position()->set_y(position[1]);
        ml_pose->mutable_position()->set_z(position[2]);
        ml_pose->mutable_orientation()->set_w(orientation[0]);
        ml_pose->mutable_orientation()->set_x(orientation[1]);
        ml_pose->mutable_orientation()->set_y(orientation[2]);
        ml_pose->mutable_orientation()->set_z(orientation[3]);
    } break;

    case jointFudgeFac: {
        joint_message->set_fudge_factor(fudgeFactor);
    } break;
    case jointCfm: {
        joint_message->set_cfm(cfm);
    } break;
    case jointSupCfm: {
        joint_message->set_suspension_cfm(suspensionCfm);
    } break;
    case jointSupErp: {
        joint_message->set_suspension_erp(suspensionErp);
    } break;

    case jointXYZ: {
        if (axis == 0) {
            mw::internal::robotics::gazebotransport::ML_Axis* ml_axis1 =
                joint_message->mutable_axis1();
            mw::internal::robotics::gazebotransport::ML_Point* ml_axis1_xyz =
                ml_axis1->mutable_xyz();
            ml_axis1_xyz->set_x(xyz[0]);
            ml_axis1_xyz->set_y(xyz[1]);
            ml_axis1_xyz->set_z(xyz[2]);
        }
        if (axis == 1) {
            mw::internal::robotics::gazebotransport::ML_Axis* ml_axis2 =
                joint_message->mutable_axis2();
            mw::internal::robotics::gazebotransport::ML_Point* ml_axis2_xyz =
                ml_axis2->mutable_xyz();
            ml_axis2_xyz->set_x(xyz[0]);
            ml_axis2_xyz->set_y(xyz[1]);
            ml_axis2_xyz->set_z(xyz[2]);
        }
    } break;

    case jointDamping: {
        if (axis == 0) {
            mw::internal::robotics::gazebotransport::ML_Axis* ml_axis1 =
                joint_message->mutable_axis1();
            ml_axis1->set_damping(damping);
        }
        if (axis == 1) {
            mw::internal::robotics::gazebotransport::ML_Axis* ml_axis2 =
                joint_message->mutable_axis2();
            ml_axis2->set_damping(damping);
        }
    } break;

    case jointFriction: {
        if (axis == 0) {
            mw::internal::robotics::gazebotransport::ML_Axis* ml_axis1 =
                joint_message->mutable_axis1();
            ml_axis1->set_friction(friction);
        }
        if (axis == 1) {
            mw::internal::robotics::gazebotransport::ML_Axis* ml_axis2 =
                joint_message->mutable_axis2();
            ml_axis2->set_friction(friction);
        }
    } break;

    case jointAngle: {
        if (axis == 0) {
            mw::internal::robotics::gazebotransport::ML_Axis* ml_axis1 =
                joint_message->mutable_axis1();
            ml_axis1->set_angle(angle);
        }
        if (axis == 1) {
            mw::internal::robotics::gazebotransport::ML_Axis* ml_axis2 =
                joint_message->mutable_axis2();
            ml_axis2->set_angle(angle);
        }
    } break;
    default:
        break;
    }

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

    /// Convert reply message to packet message
    mw::internal::robotics::gazebotransport::Packet reply;
    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
    }

    return reply;
}

/*
 * It verifies the extracted SDF string contains same name which
 * is requested by user
 */
void testUtils::verifyExtractedSDF(mw::internal::robotics::gazebotransport::Packet reply,
                                   std::string expectedModelName,
                                   bool isValid) {
    std::string sdfString = reply.gazebo_model_sdf().sdf_string();
    std::string modelname = reply.gazebo_model_sdf().model_name();
    if (isValid) {
        /// get model name from SDF string
        sdf::SDF insertSDF;
        insertSDF.SetFromString(sdfString);
        sdf::ElementPtr model = insertSDF.Root()->GetElement("model");
        auto modelParam = model->GetAttribute("name");
        std::string sdfModelName = modelParam->GetAsString();
        /// verify SDF model with reply message model name
        ASSERT_STREQ(modelname.c_str(), sdfModelName.c_str());
        /// verify SDF model with reply message model name
        ASSERT_STREQ(expectedModelName.c_str(), sdfModelName.c_str());
    } else {
        /// reply message SDF string should be empty
        ASSERT_STREQ(sdfString.c_str(), "");
        /// reply message model name should be empty
        ASSERT_STREQ(modelname.c_str(), "");
        ASSERT_STREQ(modelname.c_str(), expectedModelName.c_str());
    }
}

/*
 * It creates and sends Get Gazebo model SDF and receives the reply message from server.
 * Further, it creates and returns Packet message from reply message.
 */
mw::internal::robotics::gazebotransport::Packet testUtils::clientGetModelSDF(
    std::string const& modelName,
    std::shared_ptr<robotics::gazebotransport::Client> m_client,
    int time_out) {
    /// Create Packet message to Get Model SDF
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_GET_GAZEBO_MODEL_SDF);

    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_get_gazebo_model_sdf()->set_model_name(modelName);

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

    /// Convert reply message to packet message
    mw::internal::robotics::gazebotransport::Packet reply;
    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
    }

    return reply;
}
