/*
 * Copyright (C) 2020-2021 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_common/AgribotComponent.h>
#include <agribot_common/Can.h>
#include <agribot_common/CanBus/ICan.h>
#include <agribot_msgs/AgribotMobileAppFrame.h>
#include <agribot_msgs/ToolManagerScan.h>
#include <hal_mobile_app/TplConfigHandle/TplConfigHandle.h>
#include <hal_mobile_app/MobileAppConstants.h>
#include <stdio.h>

#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <boost/array.hpp>

#define RECEIVED_MANUAL_STEERING (0x0001 << 3)
#define RECEIVED_PROTOCOL_ERROR (0x0001 << 5)
#define RECEIVED_MANUALSTEERING_ERROR (0x0001 << 6)
#define RECEIVED_CHANGE_DELTA_GEOMETRY_FRONT (0x0001 << 10)
#define RECEIVED_CHANGE_DELTA_GEOMETRY_BACK (0x0001 << 11)
#define RECEIVED_CHANGE_STEERING_MODE (0x0001 << 12)

class MobileAppHandler : public AgribotComponent
{
  std::unique_ptr<ICan> can_handler_;
  std::queue<can_frame> FramesQueue;
  bool bSendToController = false;
  bool bArcFreshData = false;
  bool changeDeltaGeometryElevatedFront = false;
  bool changeDeltaGeometryElevatedBack = false;
  MobileAppSteeringMode::SteeringMode steering_mode_;
  double dManualLinearVelo = 0;
  double dRadius = 0;
  std::queue<uint8_t> ProtocolErrorTypeQueue;
  std::queue<uint8_t> ProtocolErrorSpecificQueue;
  std::queue<uint8_t> ManualErrorTypeQueue;
  std::queue<uint8_t> ManualErrorSpecificQueue;
  int16_t i16GetTwoBytes(uint8_t u8StartBit, can_frame CanFrame);

  // engine errors
  uint8_t CodesTotal;
  uint8_t LampState;
  std::queue<uint8_t> CodeNumberQueue;
  std::queue<uint> SPNQueue;
  std::queue<uint8_t> FMIQueue;
  std::queue<uint8_t> OCCQueue;
  std::queue<uint8_t> EngErrStateQueue;

  // configuration data
  uint8_t u8ReceivedConfigParts;
  uint uFrontRFID;
  uint uFrontToolAngle;
  uint uFrontToolLength;
  uint8_t u8FrontToolValveRedFlow;
  uint8_t u8FrontToolValveGreenFlow;
  uint8_t u8FrontToolValveBlueFlow;
  uint8_t u8FrontToolMotorSpeed;
  uint uBackRFID;
  uint uBackToolAngle;
  uint uBackToolLength;
  uint8_t u8BackToolValveRedFlow;
  uint8_t u8BackToolValveGreenFlow;
  uint8_t u8BackToolValveBlueFlow;
  uint8_t u8BackToolMotorSpeed;

  // RFID tags scan
  bool scan_front_ = false;
  bool scan_back_ = false;

  // TPL config
  TplConfig::TplConfigHandle front_tpl_config_;
  TplConfig::TplConfigHandle back_tpl_config_;
  enum TplReturnCode : uint8_t
  {
    success = 0x00,
    failed = 0xFF
  };

  // simulation part
  bool bFreshData;
  int iCommandId;
  int8_t msg_byte_array[8];
  std::string strChosenTask;

  float fFrontLinear;
  float fFrontAngular;
  float fBackLinear;
  float fBackAngular;

public:
  MobileAppHandler();
  ~MobileAppHandler();
  void initializeCanHandler(std::unique_ptr<ICan> can_handler);
  void vReceiveCanFrames();
  template <class T>
  bool bCheckByte(T frame_byte, T expected_data);
  bool bRfidIsAntSelected(const can_frame& msg, uint8_t sel_ant);
  uint16_t u16ParseCanFrames();
  float fGetManualRadius();
  float fGetManualLinearVelo();
  bool bGetChangeDeltaGeometryUpFront();
  bool bGetChangeDeltaGeometryUpBack();
  MobileAppSteeringMode::SteeringMode getSteeringMode();
  void sendDeltasGeometryResponseFrameFront(const uint8_t ret);
  void sendDeltasGeometryResponseFrameBack(const uint8_t ret);
  void sendSteeringModeResponseFrame(const uint8_t return_code);
  void vAssemblyStateMachineFrame(uint* uCanID, uint8_t* u8MsgBytes, uint uFrameType);

  virtual int configure();
  virtual bool startup();
  virtual bool update();
  virtual bool stop();
  virtual bool cleanup();

  // read the chosen task name
  void vSetChosenTask(std::string strTaskName);
  // returns the chosen task name
  std::string strGetTaskName();
  // the timer callback for setting the trigger bSendToController
  // for the proper frequency of commands for the controller
  void vSetTriggerForController();
  // return the state of trigger, reading the trigger true resets it
  bool vReadTriggerForController();
  // return the fresh arc data indicator
  bool bCheckFreshArcData();
  // clears the triggers
  void vResetArcReadTriggers();

  // tags scanning
  bool bScanFrontTriggered();
  bool bScanBackTriggered();

  void setRfidFrameHeader(can_frame& msg,  // NOLINT(runtime/references)
                          uint8_t frame_size, uint8_t com_val, uint8_t selected_antenna, uint8_t epc_words_cnt);

  void vSendRfidTagsIds(const uint8_t sel_ant,
                        const std::vector<std::vector<uint8_t>>& ids);  // NOLINT(runtime/references)

  // tpl control
  bool isTplPositionChangeRequested(const can_frame& message);
  bool setTplConfig(const can_frame& message);
  tool_manager::TplConfig getTplConfig(const TplConfig::SelectedTpl tpl);
  void sendTplResponseFrame(const uint8_t ret_code, const uint8_t id);

  // tpl feedback:
  // TODO(mwiadoms): change the way of assembling and sending can messages
  std::vector<std::uint8_t> makeTwoBytesVector(std::uint16_t value);
  std::vector<std::uint8_t> preparePtoData(const tool_manager::TplConfig& feedback);
  std::vector<std::uint8_t> prepareQcfData(const tool_manager::TplConfig& feedback, TplConfig::ValveColor selection);
  std::vector<std::uint8_t> prepareAngleData(const tool_manager::TplConfig& feedback);
  std::vector<std::uint8_t> prepareHeightData(const tool_manager::TplConfig& feedback);
  std::vector<std::vector<std::uint8_t>> prepareTplFeedbackFrames(const tool_manager::TplConfig& feedback);
  void sendTplFeedbackFrames(const TplConfig::SelectedTpl tpl, const std::vector<std::vector<std::uint8_t>>& frame);
};

template <class T>
bool MobileAppHandler::bCheckByte(T frame_byte, T expected_data)
{
  static_assert(std::is_same<T, std::uint8_t>::value);
  return frame_byte == expected_data;
}
