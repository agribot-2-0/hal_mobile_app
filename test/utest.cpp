/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include <gtest/gtest.h>
#include <agribot_common/CanBus/test/CanMock.h>
#include <hal_mobile_app/MobileAppHandler.h>
#include <agribot_common/CanBus/CanHelperFunctions.h>
#include <hal_mobile_app/MobileAppConstants.h>

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

  return static_cast<double>(preprocessed_angle) * sign / TplConfig::encode_constant;
}

double decodeTplHeight(std::vector<std::uint8_t> height)
{
  int16_t preprocessed_height = CanHelperFunctions::getI16Msbf(MobileAppTpl::feedback_byte::first_vector, height);
  return static_cast<double>(preprocessed_height) / TplConfig::encode_constant;
}

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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
