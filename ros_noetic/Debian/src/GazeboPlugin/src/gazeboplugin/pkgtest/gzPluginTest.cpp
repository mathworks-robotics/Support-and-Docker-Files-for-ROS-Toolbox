/* Copyright 2020-2021 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/Client.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"
#include "gazeboserver/gazebomsghandler/pkgtest/testUtils.hpp"
#include <mutex>

// Plugin TEST for Gazebo MATLAB Interface

// test functionalities used in test SetUp()
static void setUpTest(std::shared_ptr<robotics::gazebotransport::Client>& m_client,
                      std::shared_ptr<testUtils>& m_testUtils,
                      std::string const& ipAddress,
                      std::string const& serverPort,
                      int time_out) {
    /// Launch Client
    m_client = std::make_shared<robotics::gazebotransport::Client>(
        ipAddress, serverPort, boost::posix_time::milliseconds(time_out));

    m_testUtils = std::make_shared<testUtils>();

    /// Start Gazebo Simulator client
    gazebo::client::setup();
    std::this_thread::sleep_for(std::chrono::microseconds(100000));
}

// test functionalities used in test TearDown()
static void tearDownTest(std::shared_ptr<robotics::gazebotransport::Client>& m_client) {
    m_client->shutdown();
    gazebo::shutdown();
    std::this_thread::sleep_for(std::chrono::microseconds(50000));
}

// Class to test get Gazebo model, link, joint parameter
class getGazeboModelParamPluginTest
    : public ::testing::TestWithParam<std::tuple<std::string,
                                                 std::string,
                                                 getModelParamOperation,
                                                 errorMsgValidation,
                                                 bool,
                                                 modelJointParamName>> {
  public:
    std::string ipAddress = "127.0.0.1";
    std::string serverPort = "14581";
    /// Session Time-out value
    int time_out = 50000;

    /// Creating Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    /// Test Utils object
    std::shared_ptr<testUtils> m_testUtils;

    void SetUp() {
        /// Launch Client
        setUpTest(m_client, m_testUtils, ipAddress, serverPort, time_out);
    }

    void TearDown() {
        tearDownTest(m_client);
    }
};

// Class to test set Gazebo model parameter
class setGazeboModelLinkParamPluginTest
    : public ::testing::TestWithParam<
          std::tuple<std::string, std::string, modelLinkParamName, errorMsgValidation, bool>> {
  public:
    std::string ipAddress = "127.0.0.1";
    std::string serverPort = "14581";
    /// Session Time-out value
    int time_out = 50000;

    /// Creating Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    /// Test Utils object
    std::shared_ptr<testUtils> m_testUtils;

    void SetUp() {
        /// Launch Client
        setUpTest(m_client, m_testUtils, ipAddress, serverPort, time_out);
    }

    void TearDown() {
        tearDownTest(m_client);
    }
};

// Class to test get Gazebo model-link parameter
class setGazeboModelParamPluginTest
    : public ::testing::TestWithParam<
          std::tuple<std::string, modelParamName, errorMsgValidation, bool>> {
  public:
    std::string ipAddress = "127.0.0.1";
    std::string serverPort = "14581";
    /// Session Time-out value
    int time_out = 50000;

    /// Creating Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    /// Test Utils object
    std::shared_ptr<testUtils> m_testUtils;

    void SetUp() {
        /// Launch Client
        setUpTest(m_client, m_testUtils, ipAddress, serverPort, time_out);
    }

    void TearDown() {
        tearDownTest(m_client);
    }
};

// Class to test get Gazebo model-joint parameter
class setGazeboModelJointParamPluginTest
    : public ::testing::TestWithParam<
          std::tuple<std::string, std::string, modelJointParamName, errorMsgValidation, uint32_t>> {
  public:
    std::string ipAddress = "127.0.0.1";
    std::string serverPort = "14581";
    /// Session Time-out value
    int time_out = 50000;

    /// Creating Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    /// Test Utils object
    std::shared_ptr<testUtils> m_testUtils;

    void SetUp() {
        /// Launch Client
        setUpTest(m_client, m_testUtils, ipAddress, serverPort, time_out);
    }

    void TearDown() {
        tearDownTest(m_client);
    }
};

/*
 * It tests success of the get model, link and joint parameter.
 * Also, it tests the client successfully receives without any Error message
 * Parameterized testing is used to test various combinations.
 */
TEST_P(getGazeboModelParamPluginTest, testGetModelParameters) {
    mw::internal::robotics::gazebotransport::Packet resetReply =
        m_testUtils->clientResetSceneTime(this->m_client, this->time_out);

    // pause the world and reset model
    auto modelParam = GetParam();

    std::string modelName = std::get<0>(modelParam);
    std::string linkJointName = std::get<1>(modelParam);
    // type of test command
    getModelParamOperation testParam = std::get<2>(modelParam);
    // type of error message
    errorMsgValidation error_type = std::get<3>(modelParam);
    // parameter status
    bool paramStatus = std::get<4>(modelParam);

    switch (testParam) {
    case GetModel: {
        // ground truth
        double position_values[3] = {0.0, 0.0, 0.5};
        double orientation_values[4] = {1.0, 0.0, 0.0, 0.0};
        // send and receives reply for get model parameters
        mw::internal::robotics::gazebotransport::Packet reply =
            m_testUtils->clientGetModelParam(modelName, this->m_client, this->time_out);
        // validate received reply with ground truth
        // function arguments are as follows,
        // { ReplyMessage(Packet), modelname, Position, Orientation, ParameterStatus , ErrorType}
        this->m_testUtils->validateGetGazeboModelParam(reply, modelName, position_values,
                                                       orientation_values, paramStatus, error_type);
    } break;

    case GetLink: {
        // ground truth
        double position_values[3] = {0.0, 0.0, 0.5};
        double orientation_values[4] = {1.0, 0.0, 0.0, 0.0};
        double prod_inertia_values[3] = {0.0, 0.0, 0.0};
        double pincipal_mom_values[3] = {1.0, 1.0, 1.0};
        double mass = 1.0;
        bool windStatus = false;
        bool selfCollideStatus = false;
        bool isStaticStatus = false;
        bool canonicalStatus = true;
        bool gravityStatus = true;
        bool kinematicStatus = false;


        // send and receives reply for get model-link parameters
        mw::internal::robotics::gazebotransport::Packet getReply =
            m_testUtils->clientGetModelLinkParam(modelName, linkJointName, this->m_client,
                                                 this->time_out);
        // validate received reply with ground truth
        // function arguments are as follows,
        // { ReplyMessage(Packet), modelname, linkJointName, Position, Orientation,
        // ProductOfInertia,
        //   principalMoments, Mass, Wind, SelfCollide, IsStatic, Canonical, Gravity, Kinematic,
        //   ErrorType}
        this->m_testUtils->validateGetGazeboModelLinkParam(
            getReply, modelName, linkJointName, position_values, orientation_values,
            prod_inertia_values, pincipal_mom_values, mass, windStatus, selfCollideStatus,
            isStaticStatus, canonicalStatus, gravityStatus, kinematicStatus, error_type);
    } break;

    case GetJoint: {
        // ground truth
        double position_values[3] = {0.0, 0.0, 0.0};
        double orientation_values[4] = {1.0, 0.0, 0.0, 0.0};
        double xyz_values[3] = {1.0, 0.0, 0.0};
        double fudgeFactor_values = 1.0;
        double cfm_values = -1.0;
        double suspensionCfm_values = 0.0;
        double suspensionErp_values = 0.0;
        double angle_values = 0.0;
        double damping_values = 0.0;
        double friction_values = 0.0;
        uint32_t axis_index = static_cast<uint32_t>(std::get<4>(modelParam));
        modelJointParamName setParamName = std::get<5>(modelParam);

        // send and receives reply for get model-joint parameters
        mw::internal::robotics::gazebotransport::Packet reply =
            m_testUtils->clientGetModelJointParam(modelName, linkJointName, this->m_client,
                                                  this->time_out);
        // validate received reply with ground truth
        // function arguments are as follows,
        // { ReplyMessage(Packet), modelname, linkJointName, Position, Orientation, XYZ,
        //   FudgeFactor, CFM, SuspensionCFM, SuspensionERP, AxisIndex, Angle, Damping,
        //   Friction, ErrorType, ParameterName}
        this->m_testUtils->validateGetGazeboModelJointParam(
            reply, modelName, linkJointName, position_values, orientation_values, xyz_values,
            fudgeFactor_values, cfm_values, suspensionCfm_values, suspensionErp_values, axis_index,
            angle_values, damping_values, friction_values, error_type, setParamName);
    } break;
    default:
        break;
    }
}

// test parameter initialization for get Gazebo model, link and joint parameters
// test parameters are in following sequence,
// 		("modelname","linkJointName",OperationType,ErrorType,ParameterStatus,ParameterNameForGetJoint)
//
// Details about OperationType, ErrorType and ParameterName are as follows,
//      OperationType = {GetModel, GetLink, GetJoint}
// 		ErrorType = {Success, InvalidModel, InvalidLink, InvalidJoint, InvalidAxis, NoneAxis
// } 		ParameterName(used only for GetJoint and NOT used for GetModel, GetLink )
//                = {jointPose,jointFudgeFac,jointCfm,jointSupCfm,
//                  jointSupErp,jointXYZ,jointDamping,jointFriction,
//                  jointAngle}
//
INSTANTIATE_TEST_CASE_P(
    testingGetModelParameters,
    getGazeboModelParamPluginTest,
    ::testing::Values(
        // Get model param
        std::make_tuple("unit_box", "", GetModel, Success, false, jointPose),
        std::make_tuple("unit_box_invalid", "", GetModel, InvalidModel, false, jointPose),
        // Get model-link param
        std::make_tuple("unit_box", "link", GetLink, Success, false, jointPose),
        std::make_tuple("unit_box_invalid", "link", GetLink, InvalidModel, false, jointPose),
        std::make_tuple("unit_box", "link_invalid", GetLink, InvalidLink, false, jointPose),
        // Get model-joint param
        std::make_tuple("unit_box", "joint", GetJoint, Success, false, jointPose),
        std::make_tuple("unit_box", "joint", GetJoint, Success, false, jointAngle),
        std::make_tuple("unit_box", "joint", GetJoint, Success, false, jointXYZ),
        std::make_tuple("unit_box", "joint", GetJoint, Success, false, jointFudgeFac),
        std::make_tuple("unit_box_invalid", "joint", GetJoint, InvalidModel, false, jointPose),
        std::make_tuple("unit_box_invalid", "joint", GetJoint, InvalidModel, false, jointAngle),
        std::make_tuple("unit_box_invalid", "joint", GetJoint, InvalidModel, false, jointXYZ),
        std::make_tuple("unit_box_invalid", "joint", GetJoint, InvalidModel, false, jointFudgeFac),
        std::make_tuple("unit_box", "joint_invalid", GetJoint, InvalidJoint, false, jointPose),
        std::make_tuple("unit_box", "joint_invalid", GetJoint, InvalidJoint, false, jointAngle),
        std::make_tuple("unit_box", "joint_invalid", GetJoint, InvalidJoint, false, jointXYZ),
        std::make_tuple("unit_box",
                        "joint_invalid",
                        GetJoint,
                        InvalidJoint,
                        false,
                        jointFudgeFac)));

/*
 * It tests success of the set model parameter.
 * Also, it tests the client successfully receives without any Error message
 * Parameterized testing is used to test various combinations.
 */
TEST_P(setGazeboModelParamPluginTest, testSetModelParameters) {

    auto modelParam = GetParam();
    // model name
    std::string modelName = std::get<0>(modelParam);
    // type of test command
    modelParamName testParam = std::get<1>(modelParam);
    // type of error message
    errorMsgValidation error_type = std::get<2>(modelParam);
    bool paramStatus = std::get<3>(modelParam);

    // ground truth
    double position_values[3] = {1.0, 2.0, 0.5};
    double orientation_values[4] = {1.0, 0.0, 0.0, 0.0};

    // send and receives reply for set model parameters
    mw::internal::robotics::gazebotransport::Packet reply =
        m_testUtils->clientSetModelParam(modelName, position_values, orientation_values,
                                         paramStatus, testParam, this->m_client, this->time_out);

    // validate received reply with ground truth
    m_testUtils->validateSetCommandErrorMsg(reply, error_type);
}

// test parameter initialization for set Gazebo model parameters
// test parameters are in following sequence,
// 		("modelname",ParameterName,ErrorType,ParameterStatus)
//
// Details about ParameterName, ErrorType and ParameterStatus are as follows,
// 		ParameterName = {Pose,Wind,SelfCollide,IsStatic}
// 		ErrorType = {Success, InvalidModel }
// 		ParameterStatus = { true, false}
//
INSTANTIATE_TEST_CASE_P(
    testingSetModelParameters,
    setGazeboModelParamPluginTest,
    ::testing::Values(std::make_tuple("unit_box", Pose, Success, true),
                      std::make_tuple("unit_box_invalid", Pose, InvalidModel, true),
                      std::make_tuple("unit_box", Wind, Success, true),
                      std::make_tuple("unit_box", Wind, Success, false),
                      std::make_tuple("unit_box_invalid", Wind, InvalidModel, true),
                      std::make_tuple("unit_box", SelfCollide, Success, true),
                      std::make_tuple("unit_box", SelfCollide, Success, false),
                      std::make_tuple("unit_box_invalid", SelfCollide, InvalidModel, true),
                      std::make_tuple("unit_box", IsStatic, Success, true),
                      std::make_tuple("unit_box", IsStatic, Success, false),
                      std::make_tuple("unit_box_invalid", IsStatic, InvalidModel, true)));

/*
 * It tests success of the set model-link parameter.
 * Also, it tests the client successfully receives without any Error message
 * Parameterized testing is used to test various combinations.
 */
TEST_P(setGazeboModelLinkParamPluginTest, testSetModelLinkParameters) {

    // get testing parameters
    auto modelParam = GetParam();
    std::string modelName = std::get<0>(modelParam);
    std::string linkName = std::get<1>(modelParam);
    // type of test command
    modelLinkParamName testParam = std::get<2>(modelParam);
    // type of error message
    errorMsgValidation error_type = std::get<3>(modelParam);
    // parameter status
    bool paramStatus = std::get<4>(modelParam);

    // ground truth
    double position_values[3] = {1.0, 2.0, 0.5};
    double orientation_values[4] = {1.0, 0.0, 0.0, 0.0};
    double prod_inertia_values[3] = {1.0, 2.0, 0.5};
    double pincipal_mom_values[3] = {1.0, 2.0, 0.5};
    double mass = 2.0;

    // send and receives reply for set model-link parameters
    mw::internal::robotics::gazebotransport::Packet reply = m_testUtils->clientSetModelLinkParam(
        modelName, linkName, position_values, orientation_values, prod_inertia_values,
        pincipal_mom_values, mass, paramStatus, testParam, this->m_client, this->time_out);

    // validate received reply with ground truth
    m_testUtils->validateSetCommandErrorMsg(reply, error_type);
}

// test parameter initialization for set Gazebo model-link parameters
// test parameters are in following sequence,
// 		("modelname","linkname",ParameterName,ErrorType,ParameterStatus)
//
// Details about ParameterName, ErrorType and ParameterStatus are as follows,
// 		ParameterName = {linkPose,linkWind,linkSelfCollide,linkIsStatic,
//                  linkCanonical,linkGravity,linkKinematics,linkMass,
//                  linkProdOfInertia,linkPrincipalMom}
// 		ErrorType = {Success, InvalidModel, InvalidLink, InvalidAxis, NoneAxis }
// 		ParameterStatus = { true, false}
//
INSTANTIATE_TEST_CASE_P(
    testingSetModelLinkParameters,
    setGazeboModelLinkParamPluginTest,
    ::testing::Values( // test model link pose
        std::make_tuple("unit_box", "link", linkPose, Success, true),
        std::make_tuple("unit_box_invalid", "link", linkPose, InvalidModel, true),
        std::make_tuple("unit_box", "link_invalid", linkPose, InvalidLink, true),
        // test model link wind status
        std::make_tuple("unit_box", "link", linkWind, Success, true),
        std::make_tuple("unit_box", "link", linkWind, Success, false),
        std::make_tuple("unit_box_invalid", "link", linkWind, InvalidModel, true),
        std::make_tuple("unit_box", "link_invalid", linkWind, InvalidLink, true),
        // test model link self collide status
        std::make_tuple("unit_box", "link", linkSelfCollide, Success, true),
        std::make_tuple("unit_box", "link", linkSelfCollide, Success, false),
        std::make_tuple("unit_box_invalid", "link", linkSelfCollide, InvalidModel, true),
        std::make_tuple("unit_box", "link_invalid", linkSelfCollide, InvalidLink, true),
        // test model link isstatic status
        std::make_tuple("unit_box", "link", linkIsStatic, Success, true),
        std::make_tuple("unit_box", "link", linkIsStatic, Success, false),
        std::make_tuple("unit_box_invalid", "link", linkIsStatic, InvalidModel, true),
        std::make_tuple("unit_box", "link_invalid", linkIsStatic, InvalidLink, true),
        // test model link kinematics status
        std::make_tuple("unit_box", "link", linkKinematics, Success, true),
        std::make_tuple("unit_box", "link", linkKinematics, Success, false),
        std::make_tuple("unit_box_invalid", "link", linkKinematics, InvalidModel, true),
        std::make_tuple("unit_box", "link_invalid", linkKinematics, InvalidLink, true),
        // test model link gravity status
        std::make_tuple("unit_box", "link", linkGravity, Success, true),
        std::make_tuple("unit_box", "link", linkGravity, Success, false),
        std::make_tuple("unit_box_invalid", "link", linkGravity, InvalidModel, true),
        std::make_tuple("unit_box", "link_invalid", linkGravity, InvalidLink, true),
        // test model link canonical status
        std::make_tuple("unit_box", "link", linkCanonical, Success, true),
        std::make_tuple("unit_box", "link", linkCanonical, Success, false),
        std::make_tuple("unit_box_invalid", "link", linkCanonical, InvalidModel, true),
        std::make_tuple("unit_box", "link_invalid", linkCanonical, InvalidLink, true),
        // test model link mass
        std::make_tuple("unit_box", "link", linkMass, Success, true),
        std::make_tuple("unit_box_invalid", "link", linkMass, InvalidModel, true),
        std::make_tuple("unit_box", "link_invalid", linkMass, InvalidLink, true),
        // test model link prod of inertia
        std::make_tuple("unit_box", "link", linkProdOfInertia, Success, true),
        std::make_tuple("unit_box_invalid", "link", linkProdOfInertia, InvalidModel, true),
        std::make_tuple("unit_box", "link_invalid", linkProdOfInertia, InvalidLink, true),
        // test model link principal moment
        std::make_tuple("unit_box", "link", linkPrincipalMom, Success, true),
        std::make_tuple("unit_box_invalid", "link", linkPrincipalMom, InvalidModel, true),
        std::make_tuple("unit_box", "link_invalid", linkPrincipalMom, InvalidLink, true)));

/*
 * It tests success of the set model-joint parameter.
 * Also, it tests the client successfully receives without any Error message
 * Parameterized testing is used to test various combinations.
 */
TEST_P(setGazeboModelJointParamPluginTest, testSetModelJointParameters) {

    // get testing parameters
    auto modelParam = GetParam();
    std::string modelName = std::get<0>(modelParam);
    std::string jointName = std::get<1>(modelParam);
    // type of test command
    modelJointParamName testParam = std::get<2>(modelParam);
    // type of error message
    errorMsgValidation error_type = std::get<3>(modelParam);
    uint32_t axis_index = std::get<4>(modelParam);

    // ground truth
    double position_values[3] = {2.0, 2.0, 0.5};
    double orientation_values[4] = {1.0, 0.0, 0.0, 0.0};
    double xyz_values[3] = {0.0, 1.0, 0.0};
    double fudgeFactor_values = 1.0;
    double cfm_values = 1.0;
    double suspensionCfm_values = 0.5;
    double suspensionErp_values = 0.5;
    double angle_values = 1.0;
    double damping_values = 1.0;
    double friction_values = 1.0;

    // send and receives reply for set model-joint parameters
    mw::internal::robotics::gazebotransport::Packet reply =
        this->m_testUtils->clientSetModelJointParam(
            modelName, jointName, position_values, orientation_values, xyz_values,
            fudgeFactor_values, cfm_values, suspensionCfm_values, suspensionErp_values, axis_index,
            angle_values, damping_values, friction_values, testParam, this->m_client,
            this->time_out);

    // validate received reply with ground truth
    m_testUtils->validateSetCommandErrorMsg(reply, error_type);
}

// test parameter initialization for set Gazebo model-joint parameters
// test parameters are in following sequence,
// 		("modelname","jointname",ParameterName,ErrorType,AxisIndex)
//
// Details about ParameterName and ErrorType are as follows,
// 		ParameterName = {jointPose,jointFudgeFac,jointCfm,jointSupCfm,
//                  jointSupErp,jointXYZ,jointDamping,jointFriction,
//                  jointAngle}
// 		ErrorType = {Success, InvalidModel, InvalidJoint, InvalidAxis, NoneAxis }
//
INSTANTIATE_TEST_CASE_P(
    testingSetModelJointParameters,
    setGazeboModelJointParamPluginTest,
    ::testing::Values(                                               // test model joint pose
        std::make_tuple("unit_box", "joint", jointPose, Success, 0), // One axis
        std::make_tuple("unit_box_invalid", "joint", jointPose, InvalidModel, 0), // One axis
        std::make_tuple("unit_box", "joint_invalid", jointPose, InvalidJoint, 0), // One axis
        // test model joint cfm
        std::make_tuple("unit_box", "joint", jointCfm, Success, 0),              // One axis
        std::make_tuple("unit_box_invalid", "joint", jointCfm, InvalidModel, 0), // One axis
        std::make_tuple("unit_box", "joint_invalid", jointCfm, InvalidJoint, 0), // One axis
        // test model joint fudge factor
        std::make_tuple("unit_box", "joint", jointFudgeFac, Success, 0),              // One axis
        std::make_tuple("unit_box_invalid", "joint", jointFudgeFac, InvalidModel, 0), // One axis
        std::make_tuple("unit_box", "joint_invalid", jointFudgeFac, InvalidJoint, 0), // One axis
        // test model joint suspension cfm
        // std::make_tuple("unit_box", "joint", jointSupCfm, Success, 0),  // One axis
        std::make_tuple("unit_box_invalid", "joint", jointSupCfm, InvalidModel, 0), // One axis
        std::make_tuple("unit_box", "joint_invalid", jointSupCfm, InvalidJoint, 0), // One axis
        // test model joint suspension erp
        // std::make_tuple("unit_box", "joint", jointSupErp, Success, 0),  // One axis
        std::make_tuple("unit_box_invalid", "joint", jointSupErp, InvalidModel, 0), // One axis
        std::make_tuple("unit_box", "joint_invalid", jointSupErp, InvalidJoint, 0), // One axis
        // test model joint axis damping
        std::make_tuple("unit_box", "joint", jointDamping, Success, 0),              // One axis
        std::make_tuple("unit_box_invalid", "joint", jointDamping, InvalidModel, 0), // One axis
        std::make_tuple("unit_box", "joint_invalid", jointDamping, InvalidJoint, 0), // One axis
        // test model joint axis friction
        std::make_tuple("unit_box", "joint", jointFriction, Success, 0),              // One axis
        std::make_tuple("unit_box_invalid", "joint", jointFriction, InvalidModel, 0), // One axis
        std::make_tuple("unit_box", "joint_invalid", jointFriction, InvalidJoint, 0), // One axis
        // test model joint axis angle
        std::make_tuple("unit_box", "joint", jointAngle, Success, 0),              // One axis
        std::make_tuple("unit_box_invalid", "joint", jointAngle, InvalidModel, 0), // One axis
        std::make_tuple("unit_box", "joint_invalid", jointAngle, InvalidJoint, 0), // One axis
        // test model joint axis xyz
        std::make_tuple("unit_box", "joint", jointXYZ, Success, 0),              // One axis
        std::make_tuple("unit_box_invalid", "joint", jointXYZ, InvalidModel, 0), // One axis
        std::make_tuple("unit_box", "joint_invalid", jointXYZ, InvalidJoint, 0)  // One axis
        ));

/*
 * It tests success of the get model SDF details.
 * Also, it tests the client successfully receives without any Error message
 */
TEST_F(getGazeboModelParamPluginTest, testGetModelSDFRead) {

    // get testing parameters
    std::string modelName = "unit_box";
    std::string expectedModelName = modelName;
    bool isValid = true;
    // send and receives reply for get model SDF
    mw::internal::robotics::gazebotransport::Packet reply =
        this->m_testUtils->clientGetModelSDF(modelName, this->m_client, this->time_out);

    this->m_testUtils->verifyExtractedSDF(reply, expectedModelName, isValid);
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
