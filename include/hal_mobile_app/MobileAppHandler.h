/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_msgs/AgribotMobileAppFrame.h>
#include <agribot_msgs/ToolManagerScan.h>
#include <agribot_common/Can.h>
#include <agribot_common/AgribotComponent.h>
#include <vector>
#include <queue>
#include <cmath>
#include <string>
#include <boost/array.hpp>

#define RECEIVED_HANDSHAKE 0x0001
#define RECEIVED_HEARTBEAT 0x0002
#define RECEIVED_MANUAL_MODE_COMF 0x0004
#define RECEIVED_MANUAL_STEERING 0x0008
#define RECEIVED_AUTO_STEERING 0x0010
#define RECEIVED_PROTOCOL_ERROR 0x0020
#define RECEIVED_MANUALSTEERING_ERROR 0x0040
#define RECEIVED_CONFIGURATION 0x0080
#define RECEIVED_ENG_ERROR_START 0x0100
#define RECEIVED_ENG_ERROR 0x0200

class MobileAppHandler : public AgribotComponent
{
  Can CanHandler;
  std::queue<can_frame> FramesQueue;
  bool bSendToController = false;
  bool bArcFreshData = false;
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

  const struct
  {
    uint8_t front_pto = 0x01;
    uint8_t front_green = 0x02;
    uint8_t front_yellow = 0x03;
    uint8_t front_blue = 0x04;
    uint8_t front_tpl_angle = 0x05;
    uint8_t front_tpl_height = 0x06;
    uint8_t back_pto = 0x07;
    uint8_t back_green = 0x08;
    uint8_t back_yellow = 0x09;
    uint8_t back_blue = 0x0A;
    uint8_t back_tpl_angle = 0x0B;
    uint8_t back_tpl_height = 0x0C;
  } config_id_;

  //TPL config
  struct tpl_config
  {
    double pto = 0.0;           // rpm
    double green_valve = 0.0;   // valve open %
    double yellow_valve = 0.0;  // valve open %
    double blue_valve = 0.0;    // valve open %
    double angle = 0.0;         // radians
    double height = 0.0;        // meters
  };
  tpl_config front_tpl_config_;
  tpl_config back_tpl_config_;

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
  explicit MobileAppHandler(const char* pchCanChannel);
  ~MobileAppHandler();
  void vReceiveCanFrames();
  bool bCheckByte(uint8_t frame_byte, uint8_t expected_data);
  bool bRfidIsAntSelected(const can_frame& msg, uint8_t sel_ant);
  uint16_t u16ParseCanFrames();
  float fGetManualRadius();
  float fGetManualLinearVelo();
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
  // simulation part
  void vCreateSimStateMachineAppFrame(uint* uCanID, uint8_t* u8MsgBytes);

  void vSetSimulationAppFrame(const uint32_t uRecFrameID, const boost::array<unsigned char, 8> _chRecBytes);
  void vSetSimulationMoveFrame(float fFrLin, float fFrAng, float fBkLin, float fBkAng);

  // check if new simulation frame has been received
  bool bDataPendingSimulation();

  // tags scanning
  bool bScanFrontTriggered();
  bool bScanBackTriggered();

  void setRfidFrameHeader(can_frame& msg,  // NOLINT(runtime/references)
                          uint8_t frame_size, uint8_t com_val,
                          uint8_t selected_antenna, uint8_t epc_words_cnt);

  void vSendRfidTagsIds(const uint8_t sel_ant,
                        const std::vector<std::vector<uint8_t>>& ids);   // NOLINT(runtime/references)

  //tpl control
  void fillTplConfig(const can_frame& message);
};
