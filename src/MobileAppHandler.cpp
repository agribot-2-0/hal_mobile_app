/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <hal_mobile_app/MobileAppHandler.h>
#include <iostream>

#define ADDR_MASK 0x7FF
#define PROTOCOL_ARC_FRAME 0x035
#define PROTOCOL_HB_FRAME 0x001
#define PROTOCOL_HANDSHAKE_FRAME 0x00A
#define PROTOCOL_MANUALSTEERING 0x015
#define PROTOCOL_AUTOMATIC_STEERING 0x01F
#define PROTOCOL_ERROR_FRAMES 0x0C8
#define PROTOCOL_MAN_ERROR_FRAMES 0x0C9
#define PROTOCOL_CONFIGURATION_FRAME 0x96
constexpr uint8_t rfid_frame = 0x97;
#define PROTOCOL_ENG_ERROR_START_FRAME 0x104
#define PROTOCOL_ENG_ERROR_FRAME 0x105

#define LIN_VELO_SIGN_BYTE 0
#define DIRECTION_SIGN_BYTE 1
#define RADIUS_BYTE 2
#define LIN_VELO_BYTE 4
#define RADIUS_PARSE_CONST 10
#define LIN_VELO_PARSE_CONST 100

// rfid tags scan:
constexpr uint8_t selected_antenna_byte = 0x01;
constexpr uint8_t com_byte = 0;
constexpr uint8_t sel_ant_byte = 1;
constexpr uint8_t epc_len_byte = 2;

constexpr uint8_t scan_trigger_frame = 0x01;
constexpr uint8_t detected_tags_start_frame = 0x02;
constexpr uint8_t front_ant_selected = 0x01;
constexpr uint8_t back_ant_selected = 0x02;
constexpr uint8_t tag_start_frame_size = 3;
constexpr uint8_t tag_data_frame_size = 8;
constexpr uint8_t data_frame_epc_payload = 7;


#define CONFIG_TYPE_BYTE 0
#define CONFIG_FRONT_TOOL_AL 1
#define CONFIG_FRONT_TOOL_AL_MASK_PART 0x01
#define CONFIG_FRONT_TOOL_VM 2
#define CONFIG_FRONT_TOOL_VM_MASK_PART 0x02
#define CONFIG_BACK_TOOL_AL 3
#define CONFIG_BACK_TOOL_AL_MASK_PART 0x04
#define CONFIG_BACK_TOOL_VM 4
#define CONFIG_BACK_TOOL_VM_MASK_PART 0x08
#define CONFIG_TOOLS_RFIDS 5
#define CONFIG_TOOLS_RFIDS_MASK_PART 0x10
#define CONFIG_COMPLETE_MASK                                                                                           \
  (CONFIG_FRONT_TOOL_AL_MASK_PART | CONFIG_FRONT_TOOL_VM_MASK_PART | CONFIG_BACK_TOOL_AL_MASK_PART |                   \
   CONFIG_BACK_TOOL_VM_MASK_PART | CONFIG_TOOLS_RFIDS_MASK_PART)

#define CONFIG_TOOL_RFID_FRONT_START_BYTE 1
#define CONFIG_TOOL_RFID_BACK_START_BYTE 4
#define CONFIG_TOOL_ANGLE_BYTE 1
#define CONFIG_TOOL_LENGTH_BYTE 3

#define CONFIG_TOOL_VALVE_R_BYTE 1
#define CONFIG_TOOL_VALVE_G_BYTE 2
#define CONFIG_TOOL_VALVE_B_BYTE 3
#define CONFIG_TOOL_MOTOR_BYTE 4

#define MAX_RADIUS 10
#define MIN_RADIUS 1.5

#define LEFT_ARCH 1
#define RIGHT_ARCH -1

#define BACKWARD -1
#define FORWARD 1

#define ERROR_TYPE_BYTE 0
#define ERROR_SPECIFIC_BYTE 1

#define ENG_ERR_NUM_TOTAL_BYTE 0
#define LAMP_STATE_BYTE 1
#define ENG_ERROR_NUM_BYTE 0
#define SPN_FIRST_BYTE 1
#define FMI_BYTE 4
#define OCC_BYTE 5
#define ENG_ERR_STATE_BYTE 6

#define JOYSTICK_CONFIRMATION_FRAME_LENGTH 2

MobileAppHandler::MobileAppHandler(const char* pchCanChannel)
{
  char char_conversion[4];
  strncpy(char_conversion, pchCanChannel, sizeof char_conversion);
  CanHandler.iInitialize(char_conversion);
  bSendToController = false;
  bArcFreshData = false;
  dRadius = 0;
  dManualLinearVelo = 0;

  iCommandId = 0;
  bFreshData = false;
  strChosenTask = "";
  int i = 0;
  while (i < 8)
  {
    msg_byte_array[i] = 20;
    i++;
  }
  u8ReceivedConfigParts = 0x00;
  uFrontToolAngle = 0;
  uFrontToolLength = 0;
  u8FrontToolValveRedFlow = 0;
  u8FrontToolValveGreenFlow = 0;
  u8FrontToolValveBlueFlow = 0;
  u8FrontToolMotorSpeed = 0;
  uBackToolAngle = 0;
  uBackToolLength = 0;
  u8BackToolValveRedFlow = 0;
  u8BackToolValveGreenFlow = 0;
  u8BackToolValveBlueFlow = 0;
  u8BackToolMotorSpeed = 0;
  CodesTotal = 0;
  LampState = 0;
}

void MobileAppHandler::vReceiveCanFrames()
{
  can_frame message;
  while (CanHandler.bReceiveFrame(&message))
  {
    FramesQueue.push(message);
  }
}

MobileAppHandler::~MobileAppHandler()
{
  while (!FramesQueue.empty())
  {
    FramesQueue.pop();
  }
}

bool MobileAppHandler::vReadTriggerForController()
{
  return bSendToController;
}

void MobileAppHandler::vSetTriggerForController()
{
  bSendToController = true;
}

bool MobileAppHandler::bCheckFreshArcData()
{
  return bArcFreshData;
}

void MobileAppHandler::vResetArcReadTriggers()
{
  bArcFreshData = false;
  bSendToController = false;
}

int16_t MobileAppHandler::i16GetTwoBytes(uint8_t u8StartBit, can_frame CanFrame)
{
  return ((int16_t)CanFrame.data[u8StartBit]) << 8 | ((int16_t)CanFrame.data[u8StartBit + 1]);
}

bool MobileAppHandler::bRfidIsAntSelected(const can_frame& msg, uint8_t sel_ant)
{
  if (msg.data[CONFIG_TYPE_BYTE] == scan_trigger_frame)
  {
    if ((msg.data[selected_antenna_byte] & sel_ant) == sel_ant)
    {
      return true;
    }
  }
}

uint16_t MobileAppHandler::u16ParseCanFrames()
{
  uint8_t u16Status = 0;
  can_frame message;
  while (!FramesQueue.empty())
  {
    message = FramesQueue.front();
    if ((PROTOCOL_ARC_FRAME & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      if (message.can_dlc > JOYSTICK_CONFIRMATION_FRAME_LENGTH)
      {
        int8_t i8VeloSign = message.data[LIN_VELO_SIGN_BYTE] == 0 ? BACKWARD : FORWARD;
        int8_t i8DirectionSign = message.data[DIRECTION_SIGN_BYTE] == 0 ? RIGHT_ARCH : LEFT_ARCH;
        dManualLinearVelo =
            (static_cast<float>(i16GetTwoBytes(LIN_VELO_BYTE, message)) * i8VeloSign) / LIN_VELO_PARSE_CONST;
        dRadius = (static_cast<float>(i16GetTwoBytes(RADIUS_BYTE, message)) * i8DirectionSign) / RADIUS_PARSE_CONST;
        bArcFreshData = true;
        u16Status |= RECEIVED_MANUAL_STEERING;
      }
    }
    else if ((PROTOCOL_HANDSHAKE_FRAME & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      u16Status |= RECEIVED_HANDSHAKE;
    }
    else if ((PROTOCOL_HB_FRAME & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      u16Status |= RECEIVED_HEARTBEAT;
    }
    else if ((PROTOCOL_MANUALSTEERING & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      u16Status |= RECEIVED_MANUAL_MODE_COMF;
    }
    else if ((PROTOCOL_AUTOMATIC_STEERING & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      u16Status |= RECEIVED_AUTO_STEERING;
    }
    else if ((PROTOCOL_ERROR_FRAMES & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      ProtocolErrorTypeQueue.push(message.data[ERROR_TYPE_BYTE]);
      ProtocolErrorSpecificQueue.push(message.data[ERROR_SPECIFIC_BYTE]);
      u16Status |= RECEIVED_PROTOCOL_ERROR;
    }
    else if ((PROTOCOL_MAN_ERROR_FRAMES & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      ManualErrorTypeQueue.push(message.data[ERROR_TYPE_BYTE]);
      ManualErrorSpecificQueue.push(message.data[ERROR_SPECIFIC_BYTE]);
      u16Status |= RECEIVED_MANUALSTEERING_ERROR;
    }
    else if ((PROTOCOL_ENG_ERROR_START_FRAME & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      CodesTotal = message.data[ENG_ERR_NUM_TOTAL_BYTE];
      LampState = message.data[LAMP_STATE_BYTE];
      u16Status |= RECEIVED_ENG_ERROR_START;
    }
    else if ((PROTOCOL_ENG_ERROR_FRAME & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      CodeNumberQueue.push(message.data[ENG_ERROR_NUM_BYTE]);
      uint SPN = message.data[SPN_FIRST_BYTE] + message.data[SPN_FIRST_BYTE + 1] << 8 + message.data[SPN_FIRST_BYTE + 2]
                                                                                 << 16;
      SPNQueue.push(SPN);
      FMIQueue.push(message.data[FMI_BYTE]);
      OCCQueue.push(message.data[OCC_BYTE]);
      EngErrStateQueue.push(message.data[ENG_ERR_STATE_BYTE]);
      u16Status |= RECEIVED_ENG_ERROR;
    }
    else if ((rfid_frame & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      scan_front_ = bRfidIsAntSelected(message, front_ant_selected);
      scan_back_ = bRfidIsAntSelected(message, back_ant_selected);
    }
    else if ((PROTOCOL_CONFIGURATION_FRAME & ADDR_MASK) == (message.can_id & ADDR_MASK))
    {
      if (message.data[CONFIG_TYPE_BYTE] == CONFIG_FRONT_TOOL_AL)
      {
        uFrontToolAngle = static_cast<uint>(i16GetTwoBytes(CONFIG_TOOL_ANGLE_BYTE, message));
        uFrontToolLength = static_cast<uint>(i16GetTwoBytes(CONFIG_TOOL_LENGTH_BYTE, message));
        u8ReceivedConfigParts |= CONFIG_FRONT_TOOL_AL_MASK_PART;
      }
      else if (message.data[CONFIG_TYPE_BYTE] == CONFIG_FRONT_TOOL_VM)
      {
        u8FrontToolValveRedFlow = message.data[CONFIG_TOOL_VALVE_R_BYTE];
        u8FrontToolValveGreenFlow = message.data[CONFIG_TOOL_VALVE_G_BYTE];
        u8FrontToolValveBlueFlow = message.data[CONFIG_TOOL_VALVE_B_BYTE];
        u8FrontToolMotorSpeed = message.data[CONFIG_TOOL_MOTOR_BYTE];
        u8ReceivedConfigParts |= CONFIG_FRONT_TOOL_VM_MASK_PART;
      }
      else if (message.data[CONFIG_TYPE_BYTE] == CONFIG_BACK_TOOL_AL)
      {
        uBackToolAngle = static_cast<uint>(i16GetTwoBytes(CONFIG_TOOL_ANGLE_BYTE, message));
        uBackToolLength = static_cast<uint>(i16GetTwoBytes(CONFIG_TOOL_LENGTH_BYTE, message));
        u8ReceivedConfigParts |= CONFIG_BACK_TOOL_AL_MASK_PART;
      }
      else if (message.data[CONFIG_TYPE_BYTE] == CONFIG_BACK_TOOL_VM)
      {
        u8BackToolValveRedFlow = message.data[CONFIG_TOOL_VALVE_R_BYTE];
        u8BackToolValveGreenFlow = message.data[CONFIG_TOOL_VALVE_G_BYTE];
        u8BackToolValveBlueFlow = message.data[CONFIG_TOOL_VALVE_B_BYTE];
        u8BackToolMotorSpeed = message.data[CONFIG_TOOL_MOTOR_BYTE];
        u8ReceivedConfigParts |= CONFIG_BACK_TOOL_VM_MASK_PART;
      }
      else if (message.data[CONFIG_TYPE_BYTE] == CONFIG_TOOLS_RFIDS)
      {
        uFrontRFID = static_cast<uint>(message.data[CONFIG_TOOL_RFID_FRONT_START_BYTE] << 16);
        uFrontRFID = uFrontRFID + static_cast<uint>(message.data[CONFIG_TOOL_RFID_FRONT_START_BYTE + 1] << 8);
        uFrontRFID = uFrontRFID + static_cast<uint>(message.data[CONFIG_TOOL_RFID_FRONT_START_BYTE + 2]);
        uBackRFID = static_cast<uint>(message.data[CONFIG_TOOL_RFID_BACK_START_BYTE] << 16);
        uBackRFID = uBackRFID + static_cast<uint>(message.data[CONFIG_TOOL_RFID_BACK_START_BYTE + 1] << 8);
        uBackRFID = uBackRFID + static_cast<uint>(message.data[CONFIG_TOOL_RFID_BACK_START_BYTE + 2]);
        u8ReceivedConfigParts |= CONFIG_TOOLS_RFIDS_MASK_PART;
      }
      u16Status |= RECEIVED_CONFIGURATION;
    }
    FramesQueue.pop();
  }
  return u16Status;
}

void MobileAppHandler::vAssemblyStateMachineFrame(uint* uCanID, uint8_t* u8MsgBytes, uint uFrameType)
{
  switch (uFrameType)
  {
    case RECEIVED_MANUAL_STEERING:
      *uCanID = PROTOCOL_ARC_FRAME;
      if (dManualLinearVelo >= 0)
        u8MsgBytes[LIN_VELO_SIGN_BYTE] = 1;
      else
        u8MsgBytes[LIN_VELO_SIGN_BYTE] = 0;
      u8MsgBytes[LIN_VELO_BYTE] = ((abs(static_cast<int>(dManualLinearVelo * 100))) & 0xFF00) >> 8;
      u8MsgBytes[LIN_VELO_BYTE + 1] = (abs(static_cast<int>(dManualLinearVelo * 100))) & 0x00FF;
      if (dRadius >= 0)
        u8MsgBytes[DIRECTION_SIGN_BYTE] = 1;
      else
        u8MsgBytes[DIRECTION_SIGN_BYTE] = 0;
      u8MsgBytes[RADIUS_BYTE] = ((abs(static_cast<int>(dRadius * 10))) & 0xFF00) >> 8;
      u8MsgBytes[RADIUS_BYTE + 1] = (abs(static_cast<int>(dRadius * 10))) & 0x00FF;
      break;
    case RECEIVED_HANDSHAKE:
      *uCanID = PROTOCOL_HANDSHAKE_FRAME;
      u8MsgBytes[0] = 10;  // dummy load
      break;
    case RECEIVED_MANUAL_MODE_COMF:
      *uCanID = PROTOCOL_MANUALSTEERING;
      u8MsgBytes[0] = 10;
      break;
    case RECEIVED_AUTO_STEERING:
      *uCanID = PROTOCOL_AUTOMATIC_STEERING;
      u8MsgBytes[0] = 10;
      break;
    case RECEIVED_PROTOCOL_ERROR:
      if (!ProtocolErrorTypeQueue.empty())
      {
        *uCanID = PROTOCOL_ERROR_FRAMES;
        u8MsgBytes[ERROR_TYPE_BYTE] = ProtocolErrorTypeQueue.front();
        ProtocolErrorTypeQueue.pop();
        u8MsgBytes[ERROR_SPECIFIC_BYTE] = ProtocolErrorSpecificQueue.front();
        ProtocolErrorSpecificQueue.pop();
      }
      else
      {
        *uCanID = 0;
      }
      break;
    case RECEIVED_MANUALSTEERING_ERROR:
      if (!ManualErrorTypeQueue.empty())
      {
        *uCanID = PROTOCOL_ERROR_FRAMES;
        u8MsgBytes[ERROR_TYPE_BYTE] = ManualErrorTypeQueue.front();
        ManualErrorTypeQueue.pop();
        u8MsgBytes[ERROR_SPECIFIC_BYTE] = ManualErrorSpecificQueue.front();
        ManualErrorSpecificQueue.pop();
      }
      else
      {
        *uCanID = 0;
      }
      break;
    case RECEIVED_CONFIGURATION:
      if (u8ReceivedConfigParts & CONFIG_FRONT_TOOL_AL_MASK_PART == CONFIG_FRONT_TOOL_AL_MASK_PART)
      {
        *uCanID = PROTOCOL_CONFIGURATION_FRAME;
        u8MsgBytes[CONFIG_TYPE_BYTE] = CONFIG_FRONT_TOOL_AL;
        u8MsgBytes[CONFIG_TOOL_ANGLE_BYTE] = (uFrontToolAngle & 0xFF00) >> 8;
        u8MsgBytes[CONFIG_TOOL_ANGLE_BYTE + 1] = uFrontToolAngle & 0x00FF;
        u8MsgBytes[CONFIG_TOOL_LENGTH_BYTE] = (uFrontToolLength & 0xFF00) >> 8;
        u8MsgBytes[CONFIG_TOOL_LENGTH_BYTE + 1] = uFrontToolLength & 0x00FF;

        u8ReceivedConfigParts &= ~CONFIG_FRONT_TOOL_AL_MASK_PART;
      }
      else if (u8ReceivedConfigParts & CONFIG_FRONT_TOOL_VM_MASK_PART == CONFIG_FRONT_TOOL_VM_MASK_PART)
      {
        *uCanID = PROTOCOL_CONFIGURATION_FRAME;
        u8MsgBytes[CONFIG_TYPE_BYTE] = CONFIG_FRONT_TOOL_VM;
        u8MsgBytes[CONFIG_TOOL_VALVE_R_BYTE] = u8FrontToolValveRedFlow;
        u8MsgBytes[CONFIG_TOOL_VALVE_G_BYTE] = u8FrontToolValveGreenFlow;
        u8MsgBytes[CONFIG_TOOL_VALVE_B_BYTE] = u8FrontToolValveBlueFlow;
        u8MsgBytes[CONFIG_TOOL_MOTOR_BYTE] = u8FrontToolMotorSpeed;

        u8ReceivedConfigParts &= ~CONFIG_FRONT_TOOL_VM_MASK_PART;
      }
      else if (u8ReceivedConfigParts & CONFIG_BACK_TOOL_AL_MASK_PART == CONFIG_BACK_TOOL_AL_MASK_PART)
      {
        *uCanID = PROTOCOL_CONFIGURATION_FRAME;
        u8MsgBytes[CONFIG_TYPE_BYTE] = CONFIG_BACK_TOOL_AL;
        u8MsgBytes[CONFIG_TOOL_ANGLE_BYTE] = (uBackToolAngle & 0xFF00) >> 8;
        u8MsgBytes[CONFIG_TOOL_ANGLE_BYTE + 1] = uBackToolAngle & 0x00FF;
        u8MsgBytes[CONFIG_TOOL_LENGTH_BYTE] = (uBackToolLength & 0xFF00) >> 8;
        u8MsgBytes[CONFIG_TOOL_LENGTH_BYTE + 1] = uBackToolLength & 0x00FF;

        u8ReceivedConfigParts &= ~CONFIG_BACK_TOOL_AL_MASK_PART;
      }
      else if (u8ReceivedConfigParts & CONFIG_BACK_TOOL_VM_MASK_PART == CONFIG_BACK_TOOL_VM_MASK_PART)
      {
        *uCanID = PROTOCOL_CONFIGURATION_FRAME;
        u8MsgBytes[CONFIG_TYPE_BYTE] = CONFIG_BACK_TOOL_VM;
        u8MsgBytes[CONFIG_TOOL_VALVE_R_BYTE] = u8BackToolValveRedFlow;
        u8MsgBytes[CONFIG_TOOL_VALVE_G_BYTE] = u8BackToolValveGreenFlow;
        u8MsgBytes[CONFIG_TOOL_VALVE_B_BYTE] = u8BackToolValveBlueFlow;
        u8MsgBytes[CONFIG_TOOL_MOTOR_BYTE] = u8BackToolMotorSpeed;

        u8ReceivedConfigParts &= ~CONFIG_BACK_TOOL_VM_MASK_PART;
      }
      else if (u8ReceivedConfigParts & CONFIG_TOOLS_RFIDS_MASK_PART == CONFIG_TOOLS_RFIDS_MASK_PART)
      {
        *uCanID = PROTOCOL_CONFIGURATION_FRAME;
        u8MsgBytes[CONFIG_TYPE_BYTE] = CONFIG_TOOLS_RFIDS;
        u8MsgBytes[CONFIG_TOOL_RFID_FRONT_START_BYTE] = static_cast<uint8_t>((uFrontRFID & 0xFF0000) >> 16);
        u8MsgBytes[CONFIG_TOOL_RFID_FRONT_START_BYTE + 1] = static_cast<uint8_t>((uFrontRFID & 0x00FF00) >> 8);
        u8MsgBytes[CONFIG_TOOL_RFID_FRONT_START_BYTE + 2] = static_cast<uint8_t>((uFrontRFID & 0x0000FF));
        u8MsgBytes[CONFIG_TOOL_RFID_BACK_START_BYTE] = static_cast<uint8_t>((uBackRFID & 0xFF0000) >> 16);
        u8MsgBytes[CONFIG_TOOL_RFID_BACK_START_BYTE + 1] = static_cast<uint8_t>((uBackRFID & 0x00FF00) >> 8);
        u8MsgBytes[CONFIG_TOOL_RFID_BACK_START_BYTE + 2] = static_cast<uint8_t>((uBackRFID & 0x0000FF));
        u8ReceivedConfigParts &= ~CONFIG_TOOLS_RFIDS_MASK_PART;
      }
      else
      {
        *uCanID = 0;
      }
      break;
    case RECEIVED_ENG_ERROR_START:
      *uCanID = PROTOCOL_ENG_ERROR_START_FRAME;
      u8MsgBytes[ENG_ERR_NUM_TOTAL_BYTE] = CodesTotal;
      u8MsgBytes[LAMP_STATE_BYTE] = LampState;
      break;
    case RECEIVED_ENG_ERROR:
      if (!CodeNumberQueue.empty())
      {
        *uCanID = PROTOCOL_ENG_ERROR_FRAME;
        u8MsgBytes[ENG_ERROR_NUM_BYTE] = CodeNumberQueue.front();
        CodeNumberQueue.pop();
        uint SPN = SPNQueue.front();
        SPNQueue.pop();
        u8MsgBytes[SPN_FIRST_BYTE] = static_cast<uint8_t>(SPN & 0x0000FF);
        u8MsgBytes[SPN_FIRST_BYTE + 1] = static_cast<uint8_t>((SPN & 0x00FF00) >> 8);
        u8MsgBytes[SPN_FIRST_BYTE + 2] = static_cast<uint8_t>((SPN & 0xFF0000) >> 16);
        u8MsgBytes[FMI_BYTE] = FMIQueue.front();
        FMIQueue.pop();
        u8MsgBytes[OCC_BYTE] = OCCQueue.front();
        OCCQueue.pop();
        u8MsgBytes[ENG_ERR_STATE_BYTE] = EngErrStateQueue.front();
        EngErrStateQueue.pop();
      }
      else
      {
        *uCanID = 0;
      }
      break;
    default:
      *uCanID = 0;
      break;
  }
}

float MobileAppHandler::fGetManualRadius()
{
  return dRadius;
}

float MobileAppHandler::fGetManualLinearVelo()
{
  return dManualLinearVelo;
}

void MobileAppHandler::vSetSimulationAppFrame(const uint32_t uRecFrameID,
                                              const boost::array<unsigned char, 8> _chRecBytes)
{
  iCommandId = uRecFrameID;
  msg_byte_array[0] = _chRecBytes[0];
  msg_byte_array[1] = _chRecBytes[1];
  msg_byte_array[2] = _chRecBytes[2];
  msg_byte_array[3] = _chRecBytes[3];
  msg_byte_array[4] = _chRecBytes[4];
  msg_byte_array[5] = _chRecBytes[5];
  msg_byte_array[6] = _chRecBytes[6];
  msg_byte_array[7] = _chRecBytes[7];
  bFreshData = true;
}

// Method used to register and inform state machine about incoming msg
void MobileAppHandler::vSetSimulationMoveFrame(float fFrLin, float fFrAng, float fBkLin, float fBkAng)
{
  if (!bFreshData)
  {
    if (fFrLin != 0 && fFrAng != 0 && fBkLin != 0 && fBkAng != 0)
    {
      iCommandId = 53;
      fFrontLinear = fFrLin;
      fFrontAngular = fFrAng;
      fBackLinear = fBkLin;
      fBackAngular = fBkAng;
      bFreshData = true;
    }
  }
}

void MobileAppHandler::vSetChosenTask(std::string strTaskName)
{
  strChosenTask = strTaskName;
}

void MobileAppHandler::vCreateSimStateMachineAppFrame(uint* uCanID, uint8_t* u8MsgBytes)
{
  if (bFreshData)
  {
    *uCanID = iCommandId;
    for (int i = 0; i < 8; i++)
      u8MsgBytes[i] = msg_byte_array[i];
    bFreshData = false;
  }
}

std::string MobileAppHandler::strGetTaskName()
{
  std::string strTemp = strChosenTask;
  strChosenTask = "";
  return strTemp;
}

int MobileAppHandler::configure()
{
  return 0;
}

bool MobileAppHandler::startup()
{
  return true;
}

bool MobileAppHandler::update()
{
  return true;
}

bool MobileAppHandler::stop()
{
  return true;
}

bool MobileAppHandler::cleanup()
{
  return true;
}

bool MobileAppHandler::bDataPendingSimulation()
{
  return bFreshData;
}

bool MobileAppHandler::bScanFrontTriggered()
{
  if (scan_front_)
  {
    scan_front_ = false;
    return true;
  }
  return false;
}

bool MobileAppHandler::bScanBackTriggered()
{
  if (scan_back_)
  {
    scan_back_ = false;
    return true;
  }
  return false;
}

void MobileAppHandler::vSendRfidTagsIds(const uint8_t sel_ant,
                                        const std::vector<std::vector<uint8_t>>& ids)  // NOLINT(runtime/references)
{
  std::size_t cnt = 0;
  uint8_t com = detected_tags_start_frame;
  can_frame message;
  for (const auto& tag_id : ids)
  {
    cnt = 0;
    // Start frame with com = 2. It contains information about EPC length and selected antenna
    message.can_id = rfid_frame;
    message.can_dlc = tag_start_frame_size;
    message.data[com_byte] = com;
    message.data[sel_ant_byte] = sel_ant;
    message.data[epc_len_byte] = tag_id.size();  // number of EPC bytes
    usleep(10000);
    CanHandler.iSendFrame(&message, false);
    bzero(message.data, sizeof(uint8_t)*tag_data_frame_size);
    message.can_dlc = tag_data_frame_size;
    // EPC bytes transsmision: (com 3 and higher)
    for (const auto& epc_byte : tag_id)
    {
      ++cnt;
      message.data[cnt % tag_data_frame_size] = epc_byte;
      if (cnt % data_frame_epc_payload == 0)  // data frame payload exceeded or every byte in vector verified
      {
        ++com;  //  increment com byte
        message.data[com_byte] = com;
        usleep(10000);
        CanHandler.iSendFrame(&message, false);
        bzero(message.data, sizeof(uint8_t)*tag_data_frame_size);
        ++cnt;
      }
    }
    ++com;  //  increment com byte
    message.data[com_byte] = com;
    usleep(10000);
    CanHandler.iSendFrame(&message, false);
  }
}
