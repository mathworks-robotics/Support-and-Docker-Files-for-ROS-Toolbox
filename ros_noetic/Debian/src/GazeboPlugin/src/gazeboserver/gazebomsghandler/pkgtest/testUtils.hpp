/* Copyright 2020-2021 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include "gazebotransport/gazeboserver/gazebomlsupport/GazeboMLUtils.hpp"
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"

// utility functions used for Gazebo MATLAB Interface handler and plugin testing

// Get Model parameter operation types
enum getModelParamOperation { GetModel = 0, GetLink = 1, GetJoint = 2 };

// Model parameter names
enum modelParamName { Pose = 0, Wind = 1, SelfCollide = 2, IsStatic = 3 };

// Model-link parameter names
enum modelLinkParamName {
    linkPose = 0,
    linkWind = 1,
    linkSelfCollide = 2,
    linkIsStatic = 3,
    linkCanonical = 4,
    linkGravity = 5,
    linkKinematics = 6,
    linkMass = 7,
    linkProdOfInertia = 8,
    linkPrincipalMom = 9
};

// Model-joint parameter names
enum modelJointParamName {
    jointPose = 0,
    jointFudgeFac = 1,
    jointCfm = 2,
    jointSupCfm = 3,
    jointSupErp = 4,
    jointAngle = 5,
    jointXYZ = 6,
    jointDamping = 7,
    jointFriction = 8
};

// reply message error type
enum errorMsgValidation {
    Success = 0,
    InvalidModel = 1,
    InvalidLink = 2,
    InvalidJoint = 3,
    NoneAxis = 4,
    InvalidAxis = 5
};

// Class includes required functions for testing of Gazebo MATLAB Interface
class testUtils {
  public:
    // validate received packet error message
    void validateSetCommandErrorMsg(mw::internal::robotics::gazebotransport::Packet reply,
                                    errorMsgValidation errorType);

    // validate Gazebo model parameters with ground truth values,
    // which are SET with set model param functionality
    void validateModelResults(gazebo::physics::WorldPtr m_world,
                              std::string const& modelName,
                              double (&position)[3],
                              double (&orientation)[4],
                              bool paramStatus,
                              modelParamName paramName);

    // validate Gazebo model-link parameters with ground truth values,
    // which are SET with set model-link param functionality
    void validateModelLinkResults(gazebo::physics::WorldPtr m_world,
                                  std::string const& modelName,
                                  std::string const& linkName,
                                  double (&position)[3],
                                  double (&orientation)[4],
                                  double (&productOfInertia)[3],
                                  double (&principalMoments)[3],
                                  double mass,
                                  bool paramStatus,
                                  modelLinkParamName paramName);

    // validate Gazebo model parameters with ground truth values,
    // which are SET with set model param functionality
    void validateModelJointResults(gazebo::physics::WorldPtr m_world,
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
                                   modelJointParamName paramName);

    // SET Gazebo model parameters with ground truth values
    void setGazeboModelParam(gazebo::physics::WorldPtr m_world,
                             std::string const& modelName,
                             double (&position)[3],
                             double (&orientation)[4],
                             bool paramStatus);

    // SET Gazebo model-link parameters with ground truth values
    void setGazeboModelLinkParam(gazebo::physics::WorldPtr m_world,
                                 std::string const& modelName,
                                 std::string const& linkName,
                                 double (&position)[3],
                                 double (&orientation)[4],
                                 double (&productOfInertia)[3],
                                 double (&principalMoments)[3],
                                 double mass,
                                 bool paramStatus);

    // SET Gazebo model-joint parameters with ground truth values
    void setGazeboModelJointParam(gazebo::physics::WorldPtr m_world,
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
                                  modelJointParamName setParamName);

    // validate Gazebo model parameters with ground truth values,
    // which are received after calling GET gazebo model functionality
    void validateGetGazeboModelParam(mw::internal::robotics::gazebotransport::Packet reply,
                                     std::string const& modelName,
                                     double (&position)[3],
                                     double (&orientation)[4],
                                     bool paramStatus,
                                     errorMsgValidation errorType);

    // validate Gazebo model-link parameters with ground truth values,
    // which are received after calling GET gazebo model-link functionality
    void validateGetGazeboModelLinkParam(mw::internal::robotics::gazebotransport::Packet reply,
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
                                         errorMsgValidation errorType);

    // validate Gazebo model-joint parameters with ground truth values,
    // which are received after calling GET gazebo model-joint functionality
    void validateGetGazeboModelJointParam(mw::internal::robotics::gazebotransport::Packet reply,
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
                                          modelJointParamName setParamName);

    // create, send and receive Get model param message
    mw::internal::robotics::gazebotransport::Packet clientGetModelParam(
        std::string const& modelName,
        std::shared_ptr<robotics::gazebotransport::Client> m_client,
        int time_out);

    // create, send and receive Get model-link param message
    mw::internal::robotics::gazebotransport::Packet clientGetModelLinkParam(
        std::string const& modelName,
        std::string const& linkName,
        std::shared_ptr<robotics::gazebotransport::Client> m_client,
        int time_out);

    // create, send and receive Get model-joint param message
    mw::internal::robotics::gazebotransport::Packet clientGetModelJointParam(
        std::string const& modelName,
        std::string const& jointName,
        std::shared_ptr<robotics::gazebotransport::Client> m_client,
        int time_out);

    // create, send and receive reset gazebo world message
    mw::internal::robotics::gazebotransport::Packet clientResetSceneTime(
        std::shared_ptr<robotics::gazebotransport::Client> m_client,
        int time_out);

    // create, send and receive Set model-link param message
    mw::internal::robotics::gazebotransport::Packet clientSetModelLinkParam(
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
        int time_out);

    // create, send and receive Set model param message
    mw::internal::robotics::gazebotransport::Packet clientSetModelParam(
        std::string const& modelName,
        double (&position)[3],
        double (&orientation)[4],
        bool paramStatus,
        modelParamName paramName,
        std::shared_ptr<robotics::gazebotransport::Client> m_client,
        int time_out);

    // create, send and receive Set model-joint param message
    mw::internal::robotics::gazebotransport::Packet clientSetModelJointParam(
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
        int time_out);

    void verifyExtractedSDF(mw::internal::robotics::gazebotransport::Packet reply,
                            std::string expectedModelName,
                            bool isValid);

    mw::internal::robotics::gazebotransport::Packet clientGetModelSDF(
        std::string const& modelName,
        std::shared_ptr<robotics::gazebotransport::Client> m_client,
        int time_out);
};
