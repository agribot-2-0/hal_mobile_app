/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include <gtest/gtest.h>
#include <agribot_common/CanBus/test/CanMock.h>

TEST(TestTplMobileAppFrame, sendTplStatusViaCan__When_prepare_tpl_status_frame__Return_correct_tpl_status_frame)
{
  /*
  // Test structure setup:
  TplControl control_values_to_send;
  control_values_to_send.pto_valve = -12;
  control_values_to_send.red_qcf_valve = -83;
  control_values_to_send.green_qcf_valve = 97;
  control_values_to_send.blue_qcf_valve = -100;
  control_values_to_send.angle_actuator = 100;
  control_values_to_send.height_actuator = 34;

  // Test setup:
  std::unique_ptr<CanMock> can_test_;
  can_frame frame;
  can_test_ = std::make_unique<CanMock>();
  EXPECT_CALL(*can_test_, iInitialize("can0")).WillOnce(::testing::Return(true));
  can_test_->iInitialize("can0");
  TTCommControl ttcontrol_communication_handler;
  ttcontrol_communication_handler.initializeCanHandler(std::move(can_test_));
  SelectedTpl selection = SelectedTpl::front;

  // Test:
  ttcontrol_communication_handler.prepareTplControlFrame(selection, control_values_to_send, frame);
  TplControl control_values_to_received = decodeTplControlFrame(frame);

  ASSERT_EQ(control_values_to_received.pto_valve, control_values_to_send.pto_valve);
  ASSERT_EQ(control_values_to_received.red_qcf_valve, control_values_to_send.red_qcf_valve);
  ASSERT_EQ(control_values_to_received.green_qcf_valve, control_values_to_send.green_qcf_valve);
  ASSERT_EQ(control_values_to_received.blue_qcf_valve, control_values_to_send.blue_qcf_valve);
  ASSERT_EQ(control_values_to_received.angle_actuator, control_values_to_send.angle_actuator);
  ASSERT_EQ(control_values_to_received.height_actuator, control_values_to_send.height_actuator);
  */
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
