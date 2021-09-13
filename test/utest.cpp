/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include <gtest/gtest.h>
#include <agribot_common/CanBus/test/CanMock.h>
#include <hal_mobile_app/MobileAppHandler.h>
#include <agribot_common/CanBus/CanHelperFunctions.h>
#include <agribot_msgs/ChangeDeltasGeometryResponse.h>
#include <hal_mobile_app/MobileAppConstants.h>
#include <linux/can.h>

int16_t getAngleSign(const int16_t preprocessed_angle)
{
  return ((preprocessed_angle & TplConfig::negative_sign_mask) == TplConfig::negative_sign_mask ? -1 : 1);
}

double decodePto(std::vector<std::uint8_t> pto)
{
  return static_cast<double>(CanHelperFunctions::getI16Msbf(MobileAppTpl::feedback_byte::first_vector, pto));
}

double decodeFlow(std::vector<std::uint8_t> qcf)
{
  return static_cast<double>(CanHelperFunctions::getI16Msbf(MobileAppTpl::feedback_byte::first_vector, qcf));
}

double decodeTplAngle(std::vector<std::uint8_t> angle)
{
  std::int16_t preprocessed_angle = CanHelperFunctions::getI16Msbf(MobileAppTpl::feedback_byte::first_vector, angle);
  std::int16_t sign = getAngleSign(preprocessed_angle);
  preprocessed_angle &= ~(TplConfig::negative_sign_mask);

  return static_cast<double>(preprocessed_angle) * static_cast<double>(sign) /
         static_cast<double>(TplConfig::encode_constant);
}

double decodeTplHeight(std::vector<std::uint8_t> height)
{
  int16_t preprocessed_height = CanHelperFunctions::getI16Msbf(MobileAppTpl::feedback_byte::first_vector, height);
  return static_cast<double>(preprocessed_height) / static_cast<double>(TplConfig::encode_constant);
}

class TestFrameParsingMethods : public ::testing::Test
{
protected:
  can_frame received_frame;
  std::unique_ptr<CanMock> can_test_;
  MobileAppHandler mobile_app_comm_handler;

public:
  void SetUpCanMock()
  {
    can_test_ = std::make_unique<CanMock>();
    EXPECT_CALL(*can_test_, iInitialize("can1")).WillOnce(::testing::Return(true));
    can_test_->iInitialize("can1");
  };

  void SetUpMockFunctionCalls()
  {
    EXPECT_CALL(*can_test_, bReceiveFrame(testing::_))
        .WillOnce(testing::DoAll(::testing::SetArgPointee<0>(received_frame), ::testing::Return(true)))
        .WillOnce(::testing::Return(false));
  };

  void SetUpMobileAppHandler()
  {
    mobile_app_comm_handler.initializeCanHandler(std::move(can_test_));
    mobile_app_comm_handler.vReceiveCanFrames();
  };
};

class TestFrameSendingMethods : public ::testing::Test
{
protected:
  can_frame expected_frame;
  std::unique_ptr<CanMock> can_device_;
  can_frame* sent_frame = new can_frame;
  MobileAppHandler mobile_app_comm_handler;

public:
  void SetUpCanMock()
  {
    can_device_ = std::make_unique<CanMock>();
    EXPECT_CALL(*can_device_, iInitialize("can1")).WillOnce(::testing::Return(true));
    can_device_->iInitialize("can1");
  };
  void SetUpMockFunctionCalls()
  {
    EXPECT_CALL(*can_device_, iSendFrame(testing::_, false)).WillOnce(testing::SaveArgPointee<0>(sent_frame));
  };
  void SetUpMobileAppHandler()
  {
    mobile_app_comm_handler.initializeCanHandler(std::move(can_device_));
  };
  ~TestFrameSendingMethods()
  {
    delete sent_frame;
  };
};

class TestFrameParsingMethodsTpl : public ::testing::Test
{
protected:
  can_frame request_frame;
  std::unique_ptr<CanMock> can_device_;
  can_frame* response_frame = new can_frame;
  MobileAppHandler mobile_app_comm_handler;

public:
  void SetUpTplRequestFrame(uint8_t config_id_byte_value, uint8_t data_1_byte_value, uint8_t data_2_byte_value)
  {
    request_frame.can_id = MobileAppCanFrameId::tpl_config;
    request_frame.can_dlc = 3;
    request_frame.data[MobileAppTpl::config_id_byte] = config_id_byte_value;
    request_frame.data[1] = data_1_byte_value;
    request_frame.data[2] = data_2_byte_value;
  };
  void SetUpCanMock()
  {
    can_device_ = std::make_unique<CanMock>();
    EXPECT_CALL(*can_device_, iInitialize("can1")).WillOnce(::testing::Return(true));
    can_device_->iInitialize("can1");
  };
  void SetUpMockFunctionCalls()
  {
    EXPECT_CALL(*can_device_, bReceiveFrame(testing::_))
        .WillOnce(testing::DoAll(::testing::SetArgPointee<0>(request_frame), ::testing::Return(true)))
        .WillOnce(::testing::Return(false));
    EXPECT_CALL(*can_device_, iSendFrame(testing::_, false)).WillOnce(testing::SaveArgPointee<0>(response_frame));
  };
  void SetUpMobileAppHandler()
  {
    mobile_app_comm_handler.initializeCanHandler(std::move(can_device_));
    mobile_app_comm_handler.vReceiveCanFrames();
  };
  ~TestFrameParsingMethodsTpl()
  {
    delete response_frame;
  };
};

TEST(TestTplFeedbackFrame, preparePtoData__When_prepare_pto_frame_data__Return_correctly_encoded_vector)
{
  // setup structure:
  tool_manager::TplConfig feedback;
  feedback.pto_rpm = 30.0;

  // Test setup:
  std::unique_ptr<CanMock> can_test_;
  can_test_ = std::make_unique<CanMock>();
  EXPECT_CALL(*can_test_, iInitialize("can1")).WillOnce(::testing::Return(true));
  can_test_->iInitialize("can1");
  MobileAppHandler mobile_app_comm_handler;
  mobile_app_comm_handler.initializeCanHandler(std::move(can_test_));

  // Test:
  std::vector<std::uint8_t> encoded_pto = mobile_app_comm_handler.preparePtoData(feedback);
  double decoded_pto = decodePto(encoded_pto);
  ASSERT_EQ(decoded_pto, feedback.pto_rpm);
}

TEST(TestTplFeedbackFrame, prepareRedQcfData__When_prepare_red_qcf_frame_data__Return_correctly_encoded_vector)
{
  // setup structure:
  tool_manager::TplConfig feedback;
  feedback.red_valve_flow = 12.0;

  // Test setup:
  std::unique_ptr<CanMock> can_test_;
  can_test_ = std::make_unique<CanMock>();
  EXPECT_CALL(*can_test_, iInitialize("can1")).WillOnce(::testing::Return(true));
  can_test_->iInitialize("can1");
  MobileAppHandler mobile_app_comm_handler;
  mobile_app_comm_handler.initializeCanHandler(std::move(can_test_));

  // Test:
  std::vector<std::uint8_t> encoded_red_qcf =
      mobile_app_comm_handler.prepareQcfData(feedback, TplConfig::ValveColor::red);
  double decoded_red_valve_flow = decodeFlow(encoded_red_qcf);
  ASSERT_EQ(decoded_red_valve_flow, feedback.red_valve_flow);
}

TEST(TestTplFeedbackFrame, prepareGreenQcfData__When_prepare_green_qcf_frame_data__Return_correctly_encoded_vector)
{
  // setup structure:
  tool_manager::TplConfig feedback;
  feedback.green_valve_flow = 67.0;

  // Test setup:
  std::unique_ptr<CanMock> can_test_;
  can_test_ = std::make_unique<CanMock>();
  EXPECT_CALL(*can_test_, iInitialize("can1")).WillOnce(::testing::Return(true));
  can_test_->iInitialize("can1");
  MobileAppHandler mobile_app_comm_handler;
  mobile_app_comm_handler.initializeCanHandler(std::move(can_test_));

  // Test:
  std::vector<std::uint8_t> encoded_green_qcf =
      mobile_app_comm_handler.prepareQcfData(feedback, TplConfig::ValveColor::green);
  double decoded_green_valve_flow = decodeFlow(encoded_green_qcf);
  ASSERT_EQ(decoded_green_valve_flow, feedback.green_valve_flow);
}

TEST(TestTplFeedbackFrame, prepareBlueQcfData__When_prepare_blue_qcf_frame_data__Return_correctly_encoded_vector)
{
  // setup structure:
  tool_manager::TplConfig feedback;
  feedback.blue_valve_flow = 100.0;

  // Test setup:
  std::unique_ptr<CanMock> can_test_;
  can_test_ = std::make_unique<CanMock>();
  EXPECT_CALL(*can_test_, iInitialize("can1")).WillOnce(::testing::Return(true));
  can_test_->iInitialize("can1");
  MobileAppHandler mobile_app_comm_handler;
  mobile_app_comm_handler.initializeCanHandler(std::move(can_test_));

  // Test:
  std::vector<std::uint8_t> encoded_blue_qcf =
      mobile_app_comm_handler.prepareQcfData(feedback, TplConfig::ValveColor::blue);
  double decoded_blue_valve_flow = decodeFlow(encoded_blue_qcf);
  ASSERT_EQ(decoded_blue_valve_flow, feedback.blue_valve_flow);
}

TEST(TestTplFeedbackFrame, prepareTplAngleData__When_prepare_tpl_angle_frame_data__Return_correctly_encoded_vector)
{
  // setup structure:
  tool_manager::TplConfig feedback;
  feedback.tpl_angle = -0.13;

  // Test setup:
  std::unique_ptr<CanMock> can_test_;
  can_test_ = std::make_unique<CanMock>();
  EXPECT_CALL(*can_test_, iInitialize("can1")).WillOnce(::testing::Return(true));
  can_test_->iInitialize("can1");
  MobileAppHandler mobile_app_comm_handler;
  mobile_app_comm_handler.initializeCanHandler(std::move(can_test_));

  // Test:
  std::vector<std::uint8_t> encoded_tpl_angle = mobile_app_comm_handler.prepareAngleData(feedback);
  double decoded_tpl_angle = decodeTplAngle(encoded_tpl_angle);
  ASSERT_EQ(decoded_tpl_angle, feedback.tpl_angle);
}

TEST(TestTplFeedbackFrame, prepareTplHeightData__When_prepare_tpl_height_frame_data__Return_correctly_encoded_vector)
{
  // setup structure:
  tool_manager::TplConfig feedback;
  feedback.tpl_height = 0.37;

  // Test setup:
  std::unique_ptr<CanMock> can_test_;
  can_test_ = std::make_unique<CanMock>();
  EXPECT_CALL(*can_test_, iInitialize("can1")).WillOnce(::testing::Return(true));
  can_test_->iInitialize("can1");
  MobileAppHandler mobile_app_comm_handler;
  mobile_app_comm_handler.initializeCanHandler(std::move(can_test_));

  // Test:
  std::vector<std::uint8_t> encoded_tpl_height = mobile_app_comm_handler.prepareHeightData(feedback);
  double decoded_tpl_height = decodeTplAngle(encoded_tpl_height);
  ASSERT_EQ(decoded_tpl_height, feedback.tpl_height);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_delta_geometry_front_can_frame_Received_and_parsed_correctly)
{
  received_frame.can_id = MobileAppCanFrameId::change_delta_geometry_front;
  received_frame.can_dlc = 4;
  received_frame.data[MobileAppDeltaGeometry::request_response_byte] = MobileAppDeltaGeometry::request_byte_value;
  received_frame.data[MobileAppDeltaGeometry::caterpillar_state_byte] =
      MobileAppDeltaGeometry::normal_caterpillar_state;
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, RECEIVED_CHANGE_DELTA_GEOMETRY_FRONT);
  ASSERT_EQ(mobile_app_comm_handler.bGetChangeDeltaGeometryUpFront(), MobileAppDeltaGeometry::normal_caterpillar_state);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_delta_geometry_back_can_frame_Received_and_parsed_correctly)
{
  received_frame.can_id = MobileAppCanFrameId::change_delta_geometry_back;
  received_frame.can_dlc = 4;
  received_frame.data[MobileAppDeltaGeometry::request_response_byte] = MobileAppDeltaGeometry::request_byte_value;
  received_frame.data[MobileAppDeltaGeometry::caterpillar_state_byte] =
      MobileAppDeltaGeometry::normal_caterpillar_state;
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, RECEIVED_CHANGE_DELTA_GEOMETRY_BACK);
  ASSERT_EQ(mobile_app_comm_handler.bGetChangeDeltaGeometryUpBack(), MobileAppDeltaGeometry::normal_caterpillar_state);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_unknown_can_frames_Handled_correctly)
{
  received_frame.can_id = 0xAAA;
  received_frame.can_dlc = 4;
  received_frame.data[MobileAppDeltaGeometry::request_response_byte] = MobileAppDeltaGeometry::request_byte_value;
  received_frame.data[MobileAppDeltaGeometry::caterpillar_state_byte] =
      MobileAppDeltaGeometry::normal_caterpillar_state;
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, 0x00);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_wrong_response_request_byte_value_Handled_correctly)
{
  received_frame.can_id = 0x16C;
  received_frame.can_dlc = 4;
  received_frame.data[MobileAppSteeringMode::request_response_byte] = 2U;
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, 0x00);
}

TEST_F(TestFrameSendingMethods, sendDeltasGeometryResponseFrameFront_Assembled_corectly_before_sending)
{
  expected_frame.can_id = MobileAppCanFrameId::change_delta_geometry_front;
  expected_frame.can_dlc = MobileAppDeltaGeometry::delta_geometry_response_frame_length;
  expected_frame.data[MobileAppDeltaGeometry::request_response_byte] = MobileAppDeltaGeometry::response_byte_value;
  expected_frame.data[MobileAppDeltaGeometry::caterpillar_state_byte] =
      static_cast<uint8_t>(agribot_msgs::ChangeDeltasGeometryResponse::ELEVATED_CATERPILLAR_GEOMETRY);
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  mobile_app_comm_handler.sendDeltasGeometryResponseFrameFront(
      agribot_msgs::ChangeDeltasGeometryResponse::ELEVATED_CATERPILLAR_GEOMETRY);
  ASSERT_EQ(expected_frame.data[MobileAppDeltaGeometry::caterpillar_state_byte],
            sent_frame->data[MobileAppDeltaGeometry::caterpillar_state_byte]);
  ASSERT_EQ(expected_frame.data[MobileAppDeltaGeometry::request_response_byte],
            sent_frame->data[MobileAppDeltaGeometry::request_response_byte]);
  ASSERT_EQ(expected_frame.can_id, sent_frame->can_id);
}

TEST_F(TestFrameSendingMethods, sendDeltasGeometryResponseFrameBack_Assembled_corectly_before_sending)
{
  expected_frame.can_id = MobileAppCanFrameId::change_delta_geometry_back;
  expected_frame.can_dlc = MobileAppDeltaGeometry::delta_geometry_response_frame_length;
  expected_frame.data[MobileAppDeltaGeometry::request_response_byte] = MobileAppDeltaGeometry::response_byte_value;
  expected_frame.data[MobileAppDeltaGeometry::caterpillar_state_byte] =
      static_cast<uint8_t>(agribot_msgs::ChangeDeltasGeometryResponse::ELEVATED_CATERPILLAR_GEOMETRY);
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  mobile_app_comm_handler.sendDeltasGeometryResponseFrameBack(
      agribot_msgs::ChangeDeltasGeometryResponse::ELEVATED_CATERPILLAR_GEOMETRY);
  ASSERT_EQ(expected_frame.data[MobileAppDeltaGeometry::caterpillar_state_byte],
            sent_frame->data[MobileAppDeltaGeometry::caterpillar_state_byte]);
  ASSERT_EQ(expected_frame.data[MobileAppDeltaGeometry::request_response_byte],
            sent_frame->data[MobileAppDeltaGeometry::request_response_byte]);
  ASSERT_EQ(expected_frame.can_id, sent_frame->can_id);
}

TEST_F(TestFrameSendingMethods, sendSteeringModeResponseFrame_Assembled_corectly_before_sending)
{
  expected_frame.can_id = MobileAppCanFrameId::change_steering_mode;
  expected_frame.can_dlc = MobileAppSteeringMode::response_frame_length;
  expected_frame.data[MobileAppSteeringMode::request_response_byte] = MobileAppSteeringMode::response_byte_value;
  expected_frame.data[MobileAppSteeringMode::return_code_byte] =
      static_cast<uint8_t>(MobileAppSteeringMode::SteeringMode::crab_mode);
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  mobile_app_comm_handler.sendSteeringModeResponseFrame(
      static_cast<std::uint8_t>(MobileAppSteeringMode::SteeringMode::crab_mode));
  ASSERT_EQ(expected_frame.can_id, sent_frame->can_id);
  ASSERT_EQ(expected_frame.can_dlc, sent_frame->can_dlc);
  ASSERT_EQ(expected_frame.data[MobileAppSteeringMode::request_response_byte],
            sent_frame->data[MobileAppSteeringMode::request_response_byte]);
  ASSERT_EQ(expected_frame.data[MobileAppSteeringMode::return_code_byte],
            sent_frame->data[MobileAppSteeringMode::return_code_byte]);
}

TEST_F(TestFrameSendingMethods, sendTplResponseFrame_Assembled_corectly_before_sending)
{
  expected_frame.can_id = MobileAppCanFrameId::tpl_response;
  expected_frame.can_dlc = MobileAppTpl::tpl_resp_frame_len;
  expected_frame.data[MobileAppTpl::config_id_byte] = 1U;
  expected_frame.data[MobileAppTpl::ret_code_byte] = 0U;
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  mobile_app_comm_handler.sendTplResponseFrame(0U, 1U);
  ASSERT_EQ(expected_frame.can_id, sent_frame->can_id);
  ASSERT_EQ(expected_frame.can_dlc, sent_frame->can_dlc);
  ASSERT_EQ(expected_frame.data[MobileAppTpl::config_id_byte], sent_frame->data[MobileAppTpl::config_id_byte]);
  ASSERT_EQ(expected_frame.data[MobileAppTpl::ret_code_byte], sent_frame->data[MobileAppTpl::ret_code_byte]);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_manual_steering_frame_backward_right_Parsed_correctly)
{
  received_frame.can_id = MobileAppCanFrameId::protocol_ark;
  received_frame.can_dlc = 6;
  received_frame.data[LIN_VELO_SIGN_BYTE] = 0;   // 0 - BACKWARD, 1 - FORWARD
  received_frame.data[DIRECTION_SIGN_BYTE] = 0;  // 0 - RIGHT_ARCH, 1 - LEFT_ARCH
  received_frame.data[LIN_VELO_BYTE] = 1;
  received_frame.data[RADIUS_BYTE] = 1;
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, RECEIVED_MANUAL_STEERING);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_manual_steering_frame_forward_left_Parsed_correctly)
{
  received_frame.can_id = MobileAppCanFrameId::protocol_ark;
  received_frame.can_dlc = 6;
  received_frame.data[LIN_VELO_SIGN_BYTE] = 1;   // 0 - BACKWARD, 1 - FORWARD
  received_frame.data[DIRECTION_SIGN_BYTE] = 1;  // 0 - RIGHT_ARCH, 1 - LEFT_ARCH
  received_frame.data[LIN_VELO_BYTE] = 1;
  received_frame.data[RADIUS_BYTE] = 1;
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, RECEIVED_MANUAL_STEERING);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_deficient_manual_steering_frame_Parsed_correctly)
{
  received_frame.can_id = MobileAppCanFrameId::protocol_ark;
  received_frame.can_dlc = 2;
  received_frame.data[LIN_VELO_SIGN_BYTE] = 0;   // 0 - BACKWARD, else - FORWARD
  received_frame.data[DIRECTION_SIGN_BYTE] = 0;  // 0 - RIGHT_ARCH, else - LEFT_ARCH
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, 0x00);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_incorrect_manual_steering_frame_Parsed_correctly)
{
  received_frame.can_id = MobileAppCanFrameId::protocol_ark;
  received_frame.can_dlc = 6;
  received_frame.data[LIN_VELO_SIGN_BYTE] = 2;   // 0 - BACKWARD, 1 - FORWARD
  received_frame.data[DIRECTION_SIGN_BYTE] = 2;  // 0 - RIGHT_ARCH, 1 - LEFT_ARCH
  received_frame.data[LIN_VELO_BYTE] = 1;
  received_frame.data[RADIUS_BYTE] = 1;
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, 0x00);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_protocol_handshake_frame_Parsed_correctly)
{
  received_frame.can_id = MobileAppCanFrameId::protocol_handshake;
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, RECEIVED_HANDSHAKE);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_protocol_heartbeat_frame_Parsed_correctly)
{
  received_frame.can_id = MobileAppCanFrameId::protocol_hb;
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, RECEIVED_HEARTBEAT);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_protocol_manual_steering_frame_Parsed_correctly)
{
  received_frame.can_id = MobileAppCanFrameId::protocol_manual_steering;
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, RECEIVED_MANUAL_MODE_COMF);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_protocol_automatic_steering_frame_Parsed_correctly)
{
  received_frame.can_id = MobileAppCanFrameId::protocol_automatic_steering;
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, RECEIVED_AUTO_STEERING);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_protocol_error_frame_Parsed_correctly)
{
  received_frame.can_id = MobileAppCanFrameId::protocol_error;
  received_frame.can_dlc = 2;
  received_frame.data[ERROR_TYPE_BYTE] = 1U;
  received_frame.data[ERROR_SPECIFIC_BYTE] = 1U;
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, RECEIVED_PROTOCOL_ERROR);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_protocol_man_error_frame_Parsed_correctly)
{
  received_frame.can_id = MobileAppCanFrameId::protocol_man_error;
  received_frame.can_dlc = 2;
  received_frame.data[ERROR_TYPE_BYTE] = 1U;
  received_frame.data[ERROR_SPECIFIC_BYTE] = 1U;
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, RECEIVED_MANUALSTEERING_ERROR);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_protocol_eng_error_start_frame_Parsed_correctly)
{
  received_frame.can_id = MobileAppCanFrameId::protocol_eng_error_start;
  received_frame.can_dlc = 2;
  received_frame.data[ENG_ERR_NUM_TOTAL_BYTE] = 1U;
  received_frame.data[LAMP_STATE_BYTE] = 1U;
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, RECEIVED_ENG_ERROR_START);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_protocol_eng_error_frame_Parsed_correctly)
{
  received_frame.can_id = MobileAppCanFrameId::protocol_eng_error;
  received_frame.can_dlc = 7;
  received_frame.data[ENG_ERROR_NUM_BYTE] = 1U;
  received_frame.data[SPN_FIRST_BYTE] = 1U;
  received_frame.data[FMI_BYTE] = 1U;
  received_frame.data[OCC_BYTE] = 1U;
  received_frame.data[ENG_ERR_STATE_BYTE] = 1u;
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, RECEIVED_ENG_ERROR);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_steering_mode_frame_crab_mode_Parsed_correctly)
{
  received_frame.can_id = MobileAppCanFrameId::change_steering_mode;
  received_frame.can_dlc = 2;
  received_frame.data[MobileAppSteeringMode::request_response_byte] = MobileAppSteeringMode::request_byte_value;
  received_frame.data[MobileAppSteeringMode::steering_mode_byte] =
      static_cast<uint8_t>(MobileAppSteeringMode::SteeringMode::crab_mode);
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, RECEIVED_CHANGE_STEERING_MODE);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_steering_mode_frame_four_wheel_mode_Parsed_correctly)
{
  received_frame.can_id = MobileAppCanFrameId::change_steering_mode;
  received_frame.can_dlc = 2;
  received_frame.data[MobileAppSteeringMode::request_response_byte] = MobileAppSteeringMode::request_byte_value;
  received_frame.data[MobileAppSteeringMode::steering_mode_byte] =
      static_cast<uint8_t>(MobileAppSteeringMode::SteeringMode::four_wheel_mode);
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, RECEIVED_CHANGE_STEERING_MODE);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_steering_mode_frame_front_wheel_mode_Parsed_correctly)
{
  received_frame.can_id = MobileAppCanFrameId::change_steering_mode;
  received_frame.can_dlc = 2;
  received_frame.data[MobileAppSteeringMode::request_response_byte] = MobileAppSteeringMode::request_byte_value;
  received_frame.data[MobileAppSteeringMode::steering_mode_byte] =
      static_cast<uint8_t>(MobileAppSteeringMode::SteeringMode::front_wheel_mode);

  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, RECEIVED_CHANGE_STEERING_MODE);
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_steering_mode_frame_wrong_value_Parsed_correctly)
{
  received_frame.can_id = MobileAppCanFrameId::change_steering_mode;
  received_frame.can_dlc = 2;
  received_frame.data[MobileAppSteeringMode::request_response_byte] = MobileAppSteeringMode::request_byte_value;
  received_frame.data[MobileAppSteeringMode::steering_mode_byte] =
      static_cast<uint8_t>(MobileAppSteeringMode::SteeringMode::wrong_value);
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  u_int16_t status = mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(status, RECEIVED_CHANGE_STEERING_MODE);
}

TEST(TestFrameSendingMethodsTpl, prepareTplFeedbackFrames_Prepared_correctly)
{
  // Test setup:
  MobileAppHandler mobile_app_comm_handler;
  std::vector<std::vector<std::uint8_t>> filled_out_received_frame_data;
  tool_manager::TplConfig test_config;
  test_config.pto_rpm = 3;
  filled_out_received_frame_data.push_back(mobile_app_comm_handler.preparePtoData(test_config));
  filled_out_received_frame_data.push_back(
      mobile_app_comm_handler.prepareQcfData(test_config, TplConfig::ValveColor::red));
  filled_out_received_frame_data.push_back(
      mobile_app_comm_handler.prepareQcfData(test_config, TplConfig::ValveColor::green));
  filled_out_received_frame_data.push_back(
      mobile_app_comm_handler.prepareQcfData(test_config, TplConfig::ValveColor::blue));
  filled_out_received_frame_data.push_back(mobile_app_comm_handler.prepareAngleData(test_config));
  filled_out_received_frame_data.push_back(mobile_app_comm_handler.prepareHeightData(test_config));

  // Test:
  std::vector<std::vector<std::uint8_t>> tested_method_frame_data =
      mobile_app_comm_handler.prepareTplFeedbackFrames(test_config);
  ASSERT_EQ(filled_out_received_frame_data, tested_method_frame_data);
}

TEST(TestFrameSendingMethodsTpl, sendTplFeedbackFrames_Assembled_correctly)
{
  // Test setup:
  std::unique_ptr<CanMock> can_device_;
  can_device_ = std::make_unique<CanMock>();
  EXPECT_CALL(*can_device_, iInitialize("can1")).WillOnce(::testing::Return(true));
  can_device_->iInitialize("can1");
  MobileAppHandler mobile_app_comm_handler;
  tool_manager::TplConfig test_config;
  test_config.pto_rpm = 3;
  std::vector<std::vector<std::uint8_t>> expected_frame_data =
      mobile_app_comm_handler.prepareTplFeedbackFrames(test_config);

  std::vector<std::shared_ptr<can_frame>> received_frame;
  for (std::uint8_t i = 0; i < 6; i++)
  {
    received_frame.push_back(std::make_shared<can_frame>());
  }
  can_frame expected_frame;
  std::uint8_t config_id = TplConfig::front_config_start_id;
  expected_frame.can_id = MobileAppCanFrameId::tpl_feedback;
  expected_frame.can_dlc = TplConfig::feedback_frame_size;
  expected_frame.data[MobileAppTpl::config_id_byte] = config_id;
  expected_frame.data[MobileAppTpl::feedback_byte::first_frame] =
      expected_frame_data[0][MobileAppTpl::feedback_byte::first_vector];
  expected_frame.data[MobileAppTpl::feedback_byte::zero_frame] =
      expected_frame_data[0][MobileAppTpl::feedback_byte::zero_vector];

  EXPECT_CALL(*can_device_, iSendFrame(testing::_, false))
      .WillOnce(testing::SaveArgPointee<0>(received_frame[0]))
      .WillOnce(testing::SaveArgPointee<0>(received_frame[1]))
      .WillOnce(testing::SaveArgPointee<0>(received_frame[2]))
      .WillOnce(testing::SaveArgPointee<0>(received_frame[3]))
      .WillOnce(testing::SaveArgPointee<0>(received_frame[4]))
      .WillOnce(testing::SaveArgPointee<0>(received_frame[5]));

  mobile_app_comm_handler.initializeCanHandler(std::move(can_device_));

  // Test:
  mobile_app_comm_handler.sendTplFeedbackFrames(TplConfig::SelectedTpl::front, expected_frame_data);
  for (std::uint8_t i; i < 6; i++)
  {
    expected_frame.data[MobileAppTpl::config_id_byte] = static_cast<std::uint8_t>(i + 1);
    expected_frame.data[MobileAppTpl::feedback_byte::first_frame] =
        expected_frame_data[i][MobileAppTpl::feedback_byte::first_vector];
    expected_frame.data[MobileAppTpl::feedback_byte::zero_frame] =
        expected_frame_data[i][MobileAppTpl::feedback_byte::zero_vector];
    ASSERT_EQ(expected_frame.can_id, received_frame[i]->can_id);
    ASSERT_EQ(expected_frame.can_dlc, received_frame[i]->can_dlc);
    ASSERT_EQ(expected_frame.data[MobileAppTpl::config_id_byte], received_frame[i]->data[MobileAppTpl::config_id_byte]);
    ASSERT_EQ(expected_frame.data[MobileAppTpl::feedback_byte::first_frame],
              received_frame[i]->data[MobileAppTpl::feedback_byte::first_frame]);
    ASSERT_EQ(expected_frame.data[MobileAppTpl::feedback_byte::zero_frame],
              received_frame[i]->data[MobileAppTpl::feedback_byte::zero_frame]);
  }
}

TEST(TestMobileAppHandlerOneLiners, bCheckByte_Correct_data)
{
  uint8_t frame_byte;
  uint8_t expected_data;
  MobileAppHandler mobile_app_comm_handler;

  frame_byte = 1;
  expected_data = 1;

  ASSERT_TRUE(mobile_app_comm_handler.bCheckByte(frame_byte, expected_data));

  frame_byte = 2;

  ASSERT_FALSE(mobile_app_comm_handler.bCheckByte(frame_byte, expected_data));
}

TEST(TestMobileAppHandlerOneLiners, bRfidIsAntSelected_Correct_data)
{
  can_frame test_frame;
  test_frame.can_id = MobileAppCanFrameId::rfid;
  test_frame.can_dlc = 2U;
  test_frame.data[MobileAppRfid::com_byte] = MobileAppRfid::scan_trigger_frame;
  test_frame.data[MobileAppRfid::selected_antenna_byte] = MobileAppRfid::back_ant_selected;
  MobileAppHandler mobile_app_comm_handler;

  ASSERT_TRUE(mobile_app_comm_handler.bRfidIsAntSelected(test_frame, MobileAppRfid::back_ant_selected));
}

TEST(TestMobileAppHandlerOneLiners, bRfidIsAntSelected_Wrong_antenna_byte_value)
{
  can_frame test_frame;
  test_frame.can_id = MobileAppCanFrameId::rfid;
  test_frame.can_dlc = 2U;
  test_frame.data[MobileAppRfid::com_byte] = MobileAppRfid::scan_trigger_frame;
  test_frame.data[MobileAppRfid::selected_antenna_byte] = 3U;
  MobileAppHandler mobile_app_comm_handler;

  ASSERT_FALSE(mobile_app_comm_handler.bRfidIsAntSelected(test_frame, MobileAppRfid::front_ant_selected));
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_rfid_frame_Scan_front)
{
  received_frame.can_id = MobileAppCanFrameId::rfid;
  received_frame.can_dlc = 2;
  received_frame.data[MobileAppRfid::com_byte] = MobileAppRfid::scan_trigger_frame;
  received_frame.data[MobileAppRfid::selected_antenna_byte] = MobileAppRfid::front_ant_selected;
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_TRUE(mobile_app_comm_handler.bScanFrontTriggered());
  ASSERT_FALSE(mobile_app_comm_handler.bScanBackTriggered());
}

TEST_F(TestFrameParsingMethods, u16ParseCanFrames_rfid_frame_Scan_back)
{
  received_frame.can_id = MobileAppCanFrameId::rfid;
  received_frame.can_dlc = 2;
  received_frame.data[MobileAppRfid::com_byte] = MobileAppRfid::scan_trigger_frame;
  received_frame.data[MobileAppRfid::selected_antenna_byte] = MobileAppRfid::back_ant_selected;
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();

  // Test:
  mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_FALSE(mobile_app_comm_handler.bScanFrontTriggered());
  ASSERT_TRUE(mobile_app_comm_handler.bScanBackTriggered());
}

TEST_F(TestFrameParsingMethodsTpl, u16ParseCanFrames_tpl_frame_Front_tpl_Pto_success)
{
  uint8_t tpl_request_config_id = 1U;
  uint8_t tpl_request_frame_first_byte = 3;
  uint8_t tpl_request_frame_second_byte = 232;
  SetUpTplRequestFrame(tpl_request_config_id, tpl_request_frame_first_byte, tpl_request_frame_second_byte);
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();
  // Test:
  mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(MobileAppCanFrameId::tpl_response, response_frame->can_id);
  ASSERT_EQ(request_frame.data[MobileAppTpl::config_id_byte], response_frame->data[MobileAppTpl::config_id_byte]);
}

TEST_F(TestFrameParsingMethodsTpl, u16ParseCanFrames_tpl_frame_Front_tpl_Red_valve_success)
{
  uint8_t tpl_request_config_id = 2U;
  uint8_t tpl_request_frame_first_byte = 0;
  uint8_t tpl_request_frame_second_byte = 100;
  SetUpTplRequestFrame(tpl_request_config_id, tpl_request_frame_first_byte, tpl_request_frame_second_byte);
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();
  // Test:
  mobile_app_comm_handler.u16ParseCanFrames();
  ASSERT_EQ(MobileAppCanFrameId::tpl_response, response_frame->can_id);
  ASSERT_EQ(request_frame.data[MobileAppTpl::config_id_byte], response_frame->data[MobileAppTpl::config_id_byte]);
}

TEST_F(TestFrameParsingMethodsTpl, u16ParseCanFrames_tpl_frame_Front_tpl_Green_valve_success)
{
  uint8_t tpl_request_config_id = 3U;
  uint8_t tpl_request_frame_first_byte = 0;
  uint8_t tpl_request_frame_second_byte = 100;
  SetUpTplRequestFrame(tpl_request_config_id, tpl_request_frame_first_byte, tpl_request_frame_second_byte);
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();
  // Test:
  mobile_app_comm_handler.u16ParseCanFrames();

  ASSERT_EQ(MobileAppCanFrameId::tpl_response, response_frame->can_id);
  ASSERT_EQ(request_frame.data[MobileAppTpl::config_id_byte], response_frame->data[MobileAppTpl::config_id_byte]);
}

TEST_F(TestFrameParsingMethodsTpl, u16ParseCanFrames_tpl_frame_Front_tpl_Blue_valve_success)
{
  uint8_t tpl_request_config_id = 4U;
  uint8_t tpl_request_frame_first_byte = 0;
  uint8_t tpl_request_frame_second_byte = 100;
  SetUpTplRequestFrame(tpl_request_config_id, tpl_request_frame_first_byte, tpl_request_frame_second_byte);
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();
  // Test:
  mobile_app_comm_handler.u16ParseCanFrames();

  ASSERT_EQ(MobileAppCanFrameId::tpl_response, response_frame->can_id);
  ASSERT_EQ(request_frame.data[MobileAppTpl::config_id_byte], response_frame->data[MobileAppTpl::config_id_byte]);
}

TEST_F(TestFrameParsingMethodsTpl, u16ParseCanFrames_tpl_frame_Front_tpl_Angle_success)
{
  uint8_t tpl_request_config_id = 5U;
  uint8_t tpl_request_frame_first_byte = 0;
  uint8_t tpl_request_frame_second_byte = 200;
  SetUpTplRequestFrame(tpl_request_config_id, tpl_request_frame_first_byte, tpl_request_frame_second_byte);
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();
  // Test:
  mobile_app_comm_handler.u16ParseCanFrames();

  ASSERT_EQ(MobileAppCanFrameId::tpl_response, response_frame->can_id);
  ASSERT_EQ(request_frame.data[MobileAppTpl::config_id_byte], response_frame->data[MobileAppTpl::config_id_byte]);
}

TEST_F(TestFrameParsingMethodsTpl, u16ParseCanFrames_tpl_frame_Front_tpl_Height_success)
{
  uint8_t tpl_request_config_id = 6U;
  uint8_t tpl_request_frame_first_byte = 0;
  uint8_t tpl_request_frame_second_byte = 200;
  SetUpTplRequestFrame(tpl_request_config_id, tpl_request_frame_first_byte, tpl_request_frame_second_byte);
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();
  mobile_app_comm_handler.u16ParseCanFrames();

  ASSERT_EQ(MobileAppCanFrameId::tpl_response, response_frame->can_id);
  ASSERT_EQ(request_frame.data[MobileAppTpl::config_id_byte], response_frame->data[MobileAppTpl::config_id_byte]);
}

TEST_F(TestFrameParsingMethodsTpl, u16ParseCanFrames_tpl_frame_Back_tpl_Pto_success)
{
  uint8_t tpl_request_config_id = 7U;
  uint8_t tpl_request_frame_first_byte = 0;
  uint8_t tpl_request_frame_second_byte = 100;
  SetUpTplRequestFrame(tpl_request_config_id, tpl_request_frame_first_byte, tpl_request_frame_second_byte);
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();
  mobile_app_comm_handler.u16ParseCanFrames();

  ASSERT_EQ(MobileAppCanFrameId::tpl_response, response_frame->can_id);
  ASSERT_EQ(request_frame.data[MobileAppTpl::config_id_byte], response_frame->data[MobileAppTpl::config_id_byte]);
}

TEST_F(TestFrameParsingMethodsTpl, u16ParseCanFrames_tpl_frame_Back_tpl_Red_valve_success)
{
  uint8_t tpl_request_config_id = 8U;
  uint8_t tpl_request_frame_first_byte = 0;
  uint8_t tpl_request_frame_second_byte = 100;
  SetUpTplRequestFrame(tpl_request_config_id, tpl_request_frame_first_byte, tpl_request_frame_second_byte);
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();
  mobile_app_comm_handler.u16ParseCanFrames();

  ASSERT_EQ(MobileAppCanFrameId::tpl_response, response_frame->can_id);
  ASSERT_EQ(request_frame.data[MobileAppTpl::config_id_byte], response_frame->data[MobileAppTpl::config_id_byte]);
}

TEST_F(TestFrameParsingMethodsTpl, u16ParseCanFrames_tpl_frame_Back_tpl_Green_valve_success)
{
  uint8_t tpl_request_config_id = 9U;
  uint8_t tpl_request_frame_first_byte = 0;
  uint8_t tpl_request_frame_second_byte = 100;
  SetUpTplRequestFrame(tpl_request_config_id, tpl_request_frame_first_byte, tpl_request_frame_second_byte);
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();
  mobile_app_comm_handler.u16ParseCanFrames();

  ASSERT_EQ(MobileAppCanFrameId::tpl_response, response_frame->can_id);
  ASSERT_EQ(request_frame.data[MobileAppTpl::config_id_byte], response_frame->data[MobileAppTpl::config_id_byte]);
}

TEST_F(TestFrameParsingMethodsTpl, u16ParseCanFrames_tpl_frame_Back_tpl_Blue_valve_success)
{
  uint8_t tpl_request_config_id = 10U;
  uint8_t tpl_request_frame_first_byte = 0;
  uint8_t tpl_request_frame_second_byte = 100;
  SetUpTplRequestFrame(tpl_request_config_id, tpl_request_frame_first_byte, tpl_request_frame_second_byte);
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();
  mobile_app_comm_handler.u16ParseCanFrames();

  ASSERT_EQ(MobileAppCanFrameId::tpl_response, response_frame->can_id);
  ASSERT_EQ(request_frame.data[MobileAppTpl::config_id_byte], response_frame->data[MobileAppTpl::config_id_byte]);
}

TEST_F(TestFrameParsingMethodsTpl, u16ParseCanFrames_tpl_frame_Back_tpl_Angle_success_Max)
{
  uint8_t tpl_request_config_id = 11U;
  uint8_t tpl_request_frame_first_byte = 0;
  uint8_t tpl_request_frame_second_byte = 100;
  SetUpTplRequestFrame(tpl_request_config_id, tpl_request_frame_first_byte, tpl_request_frame_second_byte);
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();
  mobile_app_comm_handler.u16ParseCanFrames();

  ASSERT_EQ(MobileAppCanFrameId::tpl_response, response_frame->can_id);
  ASSERT_EQ(request_frame.data[MobileAppTpl::config_id_byte], response_frame->data[MobileAppTpl::config_id_byte]);
}

TEST_F(TestFrameParsingMethodsTpl, u16ParseCanFrames_tpl_frame_Back_tpl_Height_success)
{
  uint8_t tpl_request_config_id = 12U;
  uint8_t tpl_request_frame_first_byte = 0;
  uint8_t tpl_request_frame_second_byte = 100;
  SetUpTplRequestFrame(tpl_request_config_id, tpl_request_frame_first_byte, tpl_request_frame_second_byte);
  SetUpCanMock();
  SetUpMockFunctionCalls();
  SetUpMobileAppHandler();
  mobile_app_comm_handler.u16ParseCanFrames();

  ASSERT_EQ(MobileAppCanFrameId::tpl_response, response_frame->can_id);
  ASSERT_EQ(request_frame.data[MobileAppTpl::config_id_byte], response_frame->data[MobileAppTpl::config_id_byte]);
}

TEST_F(TestFrameSendingMethods, vSendRfidTagsIds_Assembled_correctly_before_sending)
{
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
