/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <hal_mobile_app/MobileAppHandler.h>
#include <hal_mobile_app/MobileAppConstants.h>
#include <iostream>

MobileAppHandler::MobileAppHandler()
{
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

void MobileAppHandler::initializeCanHandler(std::unique_ptr<ICan> can_handler)
{
  can_handler_ = std::move(can_handler);
}

void MobileAppHandler::vReceiveCanFrames()
{
  can_frame message;
  while (can_handler_->bReceiveFrame(&message))
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

bool MobileAppHandler::bCheckByte(uint8_t frame_byte, uint8_t expected_data)
{
  return frame_byte == expected_data;
}

bool MobileAppHandler::bRfidIsAntSelected(const can_frame& msg, uint8_t sel_ant)
{
  return bCheckByte(msg.data[MobileAppRfid::com_byte], MobileAppRfid::scan_trigger_frame) &&
         bCheckByte((msg.data[MobileAppRfid::selected_antenna_byte] & sel_ant), sel_ant);
}

uint16_t MobileAppHandler::u16ParseCanFrames()
{
  uint8_t u16Status = 0;
  can_frame message;
  while (!FramesQueue.empty())
  {
    message = FramesQueue.front();
    if ((MobileAppCanFrameId::protocol_ark & addr_mask) == (message.can_id & addr_mask))
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
    else if ((MobileAppCanFrameId::protocol_handshake & addr_mask) == (message.can_id & addr_mask))
    {
      u16Status |= RECEIVED_HANDSHAKE;
    }
    else if ((MobileAppCanFrameId::protocol_hb & addr_mask) == (message.can_id & addr_mask))
    {
      u16Status |= RECEIVED_HEARTBEAT;
    }
    else if ((MobileAppCanFrameId::protocol_manual_steering & addr_mask) == (message.can_id & addr_mask))
    {
      u16Status |= RECEIVED_MANUAL_MODE_COMF;
    }
    else if ((MobileAppCanFrameId::protocol_automatic_steering & addr_mask) == (message.can_id & addr_mask))
    {
      u16Status |= RECEIVED_AUTO_STEERING;
    }
    else if ((MobileAppCanFrameId::protocol_error & addr_mask) == (message.can_id & addr_mask))
    {
      ProtocolErrorTypeQueue.push(message.data[ERROR_TYPE_BYTE]);
      ProtocolErrorSpecificQueue.push(message.data[ERROR_SPECIFIC_BYTE]);
      u16Status |= RECEIVED_PROTOCOL_ERROR;
    }
    else if ((MobileAppCanFrameId::protocol_man_error & addr_mask) == (message.can_id & addr_mask))
    {
      ManualErrorTypeQueue.push(message.data[ERROR_TYPE_BYTE]);
      ManualErrorSpecificQueue.push(message.data[ERROR_SPECIFIC_BYTE]);
      u16Status |= RECEIVED_MANUALSTEERING_ERROR;
    }
    else if ((MobileAppCanFrameId::protocol_eng_error_start & addr_mask) == (message.can_id & addr_mask))
    {
      CodesTotal = message.data[ENG_ERR_NUM_TOTAL_BYTE];
      LampState = message.data[LAMP_STATE_BYTE];
      u16Status |= RECEIVED_ENG_ERROR_START;
    }
    else if ((MobileAppCanFrameId::protocol_eng_error & addr_mask) == (message.can_id & addr_mask))
    {
      CodeNumberQueue.push(message.data[ENG_ERROR_NUM_BYTE]);
      uint SPN = (message.data[SPN_FIRST_BYTE] + message.data[SPN_FIRST_BYTE + 1])
                 << (8 + message.data[SPN_FIRST_BYTE + 2]) << 16;
      SPNQueue.push(SPN);
      FMIQueue.push(message.data[FMI_BYTE]);
      OCCQueue.push(message.data[OCC_BYTE]);
      EngErrStateQueue.push(message.data[ENG_ERR_STATE_BYTE]);
      u16Status |= RECEIVED_ENG_ERROR;
    }
    else if ((MobileAppCanFrameId::rfid & addr_mask) == (message.can_id & addr_mask))
    {
      scan_front_ = bRfidIsAntSelected(message, MobileAppRfid::front_ant_selected);
      scan_back_ = bRfidIsAntSelected(message, MobileAppRfid::back_ant_selected);
    }
    else if (isTplPositionChangeRequested(message))
    {
      if (setTplConfig(can_handler_->getMostRecentCanFrame()))
      {
        sendTplResponseFrame(TplReturnCode::success, message.data[MobileAppTpl::config_id_byte]);
      }
      else
      {
        sendTplResponseFrame(TplReturnCode::failed, message.data[MobileAppTpl::config_id_byte]);
      }
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
      *uCanID = MobileAppCanFrameId::protocol_ark;
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
      *uCanID = MobileAppCanFrameId::protocol_handshake;
      u8MsgBytes[0] = 10;  // dummy load
      break;
    case RECEIVED_MANUAL_MODE_COMF:
      *uCanID = MobileAppCanFrameId::protocol_manual_steering;
      u8MsgBytes[0] = 10;
      break;
    case RECEIVED_AUTO_STEERING:
      *uCanID = MobileAppCanFrameId::protocol_automatic_steering;
      u8MsgBytes[0] = 10;
      break;
    case RECEIVED_PROTOCOL_ERROR:
      if (!ProtocolErrorTypeQueue.empty())
      {
        *uCanID = MobileAppCanFrameId::protocol_error;
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
        *uCanID = MobileAppCanFrameId::protocol_error;
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
    case RECEIVED_ENG_ERROR_START:
      *uCanID = MobileAppCanFrameId::protocol_eng_error_start;
      u8MsgBytes[ENG_ERR_NUM_TOTAL_BYTE] = CodesTotal;
      u8MsgBytes[LAMP_STATE_BYTE] = LampState;
      break;
    case RECEIVED_ENG_ERROR:
      if (!CodeNumberQueue.empty())
      {
        *uCanID = MobileAppCanFrameId::protocol_eng_error;
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

void MobileAppHandler::setRfidFrameHeader(can_frame& msg,  // NOLINT(runtime/references)
                                          uint8_t frame_size, uint8_t com_val, uint8_t selected_antenna,
                                          uint8_t epc_words_cnt)
{
  msg.can_id = MobileAppCanFrameId::rfid;
  msg.can_dlc = frame_size;
  msg.data[MobileAppRfid::com_byte] = com_val;
  msg.data[MobileAppRfid::sel_ant_byte] = selected_antenna;
  msg.data[MobileAppRfid::epc_len_byte] = epc_words_cnt;  // number of EPC bytes
}

void MobileAppHandler::vSendRfidTagsIds(const uint8_t sel_ant,
                                        const std::vector<std::vector<uint8_t>>& ids)  // NOLINT(runtime/references)
{
  std::size_t cnt = 0;
  uint8_t com = MobileAppRfid::detected_tags_start_frame;
  can_frame message;
  if (ids.size() == 0)
  {
    setRfidFrameHeader(message, MobileAppRfid::tag_start_frame_size, com, sel_ant, ids.size());
    usleep(MobileAppRfid::frame_send_delay);
    can_handler_->iSendFrame(&message, false);
  }
  for (const auto& tag_id : ids)
  {
    com = MobileAppRfid::detected_tags_start_frame;
    cnt = 0;
    // Start frame with com = 2. It contains information about EPC length and selected antenna
    message.can_id = MobileAppCanFrameId::rfid;
    setRfidFrameHeader(message, MobileAppRfid::tag_start_frame_size, com, sel_ant, tag_id.size());
    usleep(MobileAppRfid::frame_send_delay);
    can_handler_->iSendFrame(&message, false);
    bzero(message.data, sizeof(uint8_t) * (MobileAppRfid::tag_data_frame_size));
    message.can_dlc = MobileAppRfid::tag_data_frame_size;
    // EPC bytes transsmision: (com 3 and higher)
    for (const auto& epc_byte : tag_id)
    {
      ++cnt;
      message.data[cnt % MobileAppRfid::tag_data_frame_size] = epc_byte;
      // if data frame payload exceeded
      if (cnt % MobileAppRfid::data_frame_epc_payload == 0)
      {
        ++com;  //  increment com byte
        message.data[MobileAppRfid::com_byte] = com;
        usleep(MobileAppRfid::frame_send_delay);
        can_handler_->iSendFrame(&message, false);
        bzero(message.data, sizeof(uint8_t) * MobileAppRfid::tag_data_frame_size);
        ++cnt;
      }
    }
    if (--cnt % MobileAppRfid::data_frame_epc_payload != 0)
    {
      ++com;  //  increment com byte
      message.data[MobileAppRfid::com_byte] = com;
      usleep(MobileAppRfid::frame_send_delay);
      can_handler_->iSendFrame(&message, false);
    }
  }
}

// tpl control
bool MobileAppHandler::isTplPositionChangeRequested(const can_frame& message)
{
  return (MobileAppCanFrameId::tpl_config & addr_mask) == (message.can_id & addr_mask);
}

bool MobileAppHandler::setTplConfig(const CanFrame& frame)
{
  using TplConfig::TplConfigId;
  using TplConfig::ValveColor;
  TplConfigId id = static_cast<TplConfigId>(frame.getData()[MobileAppTpl::config_id_byte]);
  switch (id)
  {
    case TplConfigId::front_pto:
      return front_tpl_config_.setPto(frame.getData());
    case TplConfigId::front_green:
      return front_tpl_config_.setValveFlow(frame.getData(), ValveColor::green);
    case TplConfigId::front_blue:
      return front_tpl_config_.setValveFlow(frame.getData(), ValveColor::blue);
    case TplConfigId::front_red:
      return front_tpl_config_.setValveFlow(frame.getData(), ValveColor::red);
    case TplConfigId::front_tpl_angle:
      return front_tpl_config_.setAngle(frame.getData());
    case TplConfigId::front_tpl_height:
      return front_tpl_config_.setHeight(frame.getData());
    case TplConfigId::back_pto:
      return back_tpl_config_.setPto(frame.getData());
    case TplConfigId::back_green:
      return back_tpl_config_.setValveFlow(frame.getData(), ValveColor::green);
    case TplConfigId::back_blue:
      return back_tpl_config_.setValveFlow(frame.getData(), ValveColor::blue);
    case TplConfigId::back_red:
      return back_tpl_config_.setValveFlow(frame.getData(), ValveColor::red);
    case TplConfigId::back_tpl_angle:
      return back_tpl_config_.setAngle(frame.getData());
    case TplConfigId::back_tpl_height:
      return back_tpl_config_.setHeight(frame.getData());
  }
  return false;
}

tool_manager::TplConfig MobileAppHandler::getTplConfig(const TplConfig::SelectedTpl tpl)
{
  using TplConfig::SelectedTpl;
  switch (tpl)
  {
    case SelectedTpl::front:
      return front_tpl_config_.getConfig();
    case SelectedTpl::back:
      return back_tpl_config_.getConfig();
  }
}

void MobileAppHandler::sendTplResponseFrame(const uint8_t ret, const uint8_t id)
{
  can_frame message;
  message.can_id = MobileAppCanFrameId::tpl_response;
  message.can_dlc = MobileAppTpl::tpl_resp_frame_len;
  message.data[MobileAppTpl::config_id_byte] = id;
  message.data[MobileAppTpl::ret_code_byte] = static_cast<uint8_t>(ret);
  can_handler_->iSendFrame(&message, false);
}

// tpl feedback:
std::vector<std::uint8_t> MobileAppHandler::makeTwoBytesVector(std::uint16_t value)
{
  return { static_cast<std::uint8_t>(value >> 8), static_cast<std::uint8_t>(value) };
}

std::vector<std::uint8_t> MobileAppHandler::preparePtoData(const tool_manager::TplConfig& feedback)
{
  return makeTwoBytesVector(static_cast<uint16_t>(feedback.pto_rpm));
}

std::vector<std::uint8_t> MobileAppHandler::prepareQcfData(const tool_manager::TplConfig& feedback,
                                                           TplConfig::ValveColor selection)
{
  switch (selection)
  {
    case TplConfig::ValveColor::red:
      return makeTwoBytesVector(static_cast<uint16_t>(feedback.red_valve_flow));
    case TplConfig::ValveColor::green:
      return makeTwoBytesVector(static_cast<uint16_t>(feedback.green_valve_flow));
    case TplConfig::ValveColor::blue:
      return makeTwoBytesVector(static_cast<uint16_t>(feedback.blue_valve_flow));
    default:
      return {};
  }
}

std::vector<std::uint8_t> MobileAppHandler::prepareAngleData(const tool_manager::TplConfig& feedback)
{
  bool negative = (feedback.tpl_angle < 0) ? true : false;
  uint16_t processed_angle = abs(feedback.tpl_angle) * TplConfig::encode_constant;
  if (negative)
  {
    processed_angle |= TplConfig::negative_sign_mask;
  }
  return makeTwoBytesVector(static_cast<uint16_t>(processed_angle));
}

std::vector<std::uint8_t> MobileAppHandler::prepareHeightData(const tool_manager::TplConfig& feedback)
{
  return makeTwoBytesVector(static_cast<uint16_t>(feedback.tpl_height * TplConfig::encode_constant));
}

std::vector<std::vector<std::uint8_t>>
MobileAppHandler::prepareTplFeedbackFrames(const tool_manager::TplConfig& feedback)
{
  std::vector<std::vector<std::uint8_t>> frames_data;
  frames_data.push_back(preparePtoData(feedback));
  frames_data.push_back(prepareQcfData(feedback, TplConfig::ValveColor::red));
  frames_data.push_back(prepareQcfData(feedback, TplConfig::ValveColor::green));
  frames_data.push_back(prepareQcfData(feedback, TplConfig::ValveColor::blue));
  frames_data.push_back(prepareAngleData(feedback));
  frames_data.push_back(prepareHeightData(feedback));
  return frames_data;
}

void MobileAppHandler::sendTplFeedbackFrames(const TplConfig::SelectedTpl tpl,
                                             const std::vector<std::vector<std::uint8_t>>& frames_data)
{
  std::uint8_t config_id =
      (tpl == TplConfig::SelectedTpl::front) ? TplConfig::front_config_start_id : TplConfig::back_config_start_id;
  for (auto data : frames_data)
  {
    can_frame message;
    message.can_id = MobileAppCanFrameId::tpl_feedback;
    message.can_dlc = TplConfig::feedback_frame_size;
    message.data[MobileAppTpl::config_id_byte] = config_id;
    message.data[MobileAppTpl::feedback_byte::first_frame] = data[MobileAppTpl::feedback_byte::first_vector];
    message.data[MobileAppTpl::feedback_byte::zero_frame] = data[MobileAppTpl::feedback_byte::zero_vector];
    ++config_id;
    can_handler_->iSendFrame(&message, false);
    usleep(MobileAppRfid::frame_send_delay);
  }
}
