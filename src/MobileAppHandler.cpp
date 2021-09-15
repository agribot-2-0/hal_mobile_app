/*
 * Copyright (C) 2020-2021 Agribot. All rights reserved.
 */

#include <hal_mobile_app/MobileAppConstants.h>
#include <hal_mobile_app/MobileAppHandler.h>

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

bool MobileAppHandler::bRfidIsAntSelected(const can_frame& msg, uint8_t sel_ant)
{
  return bCheckByte(msg.data[MobileAppRfid::com_byte], MobileAppRfid::scan_trigger_frame) &&
         bCheckByte(static_cast<uint8_t>(msg.data[MobileAppRfid::selected_antenna_byte]), sel_ant);
}

uint16_t MobileAppHandler::u16ParseCanFrames()
{
  uint16_t u16Status = 0x00;
  can_frame message;
  while (!FramesQueue.empty())
  {
    message = FramesQueue.front();
    if ((MobileAppCanFrameId::protocol_ark & addr_mask) == (message.can_id & addr_mask))
    {
      if (message.can_dlc > JOYSTICK_CONFIRMATION_FRAME_LENGTH)
      {
        int8_t i8VeloSign;
        int8_t i8DirectionSign;
        u16Status |= RECEIVED_MANUAL_STEERING;
        if (message.data[LIN_VELO_SIGN_BYTE] == 0)
        {
          i8VeloSign = BACKWARD;
        }
        else if (message.data[LIN_VELO_SIGN_BYTE] == 1)
        {
          i8VeloSign = FORWARD;
        }
        else
        {
          u16Status = 0x00;
        }
        if (message.data[DIRECTION_SIGN_BYTE] == 0)
        {
          i8DirectionSign = RIGHT_ARCH;
        }
        else if (message.data[DIRECTION_SIGN_BYTE] == 1)
        {
          i8DirectionSign = LEFT_ARCH;
        }
        else
        {
          u16Status = 0x00;
        }
        if (u16Status != 0x00)
        {
          dManualLinearVelo =
              (static_cast<float>(i16GetTwoBytes(LIN_VELO_BYTE, message)) * i8VeloSign) / LIN_VELO_PARSE_CONST;
          dRadius = (static_cast<float>(i16GetTwoBytes(RADIUS_BYTE, message)) * i8DirectionSign) / RADIUS_PARSE_CONST;
          bArcFreshData = true;
        }
      }
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
    else if ((MobileAppCanFrameId::rfid & addr_mask) == (message.can_id & addr_mask))
    {
      scan_front_ = bRfidIsAntSelected(message, MobileAppRfid::front_ant_selected);
      scan_back_ = bRfidIsAntSelected(message, MobileAppRfid::back_ant_selected);
    }
    else if ((MobileAppCanFrameId::change_delta_geometry_front & addr_mask) == (message.can_id & addr_mask) &&
             message.data[MobileAppDeltaGeometry::request_response_byte] == MobileAppDeltaGeometry::request_byte_value)
    {
      u16Status |= RECEIVED_CHANGE_DELTA_GEOMETRY_FRONT;
      changeDeltaGeometryElevatedFront = message.data[MobileAppDeltaGeometry::caterpillar_state_byte] ==
                                                 MobileAppDeltaGeometry::normal_caterpillar_state ?
                                             MobileAppDeltaGeometry::normal_caterpillar_state :
                                             MobileAppDeltaGeometry::elevated_caterpillar_state;
    }
    else if ((MobileAppCanFrameId::change_delta_geometry_back & addr_mask) == (message.can_id & addr_mask) &&
             message.data[MobileAppDeltaGeometry::request_response_byte] == MobileAppDeltaGeometry::request_byte_value)
    {
      u16Status |= RECEIVED_CHANGE_DELTA_GEOMETRY_BACK;
      changeDeltaGeometryElevatedBack = message.data[MobileAppDeltaGeometry::caterpillar_state_byte] ==
                                                MobileAppDeltaGeometry::normal_caterpillar_state ?
                                            MobileAppDeltaGeometry::normal_caterpillar_state :
                                            MobileAppDeltaGeometry::elevated_caterpillar_state;
    }
    else if ((MobileAppCanFrameId::change_steering_mode & addr_mask) == (message.can_id & addr_mask) &&
             message.data[MobileAppSteeringMode::request_response_byte] == MobileAppSteeringMode::request_byte_value)
    {
      u16Status |= RECEIVED_CHANGE_STEERING_MODE;
      switch (message.data[MobileAppSteeringMode::steering_mode_byte])
      {
        case static_cast<uint8_t>(MobileAppSteeringMode::SteeringMode::four_wheel_mode):
          steering_mode_ = MobileAppSteeringMode::SteeringMode::four_wheel_mode;
          break;
        case static_cast<uint8_t>(MobileAppSteeringMode::SteeringMode::front_wheel_mode):
          steering_mode_ = MobileAppSteeringMode::SteeringMode::front_wheel_mode;
          break;
        case static_cast<uint8_t>(MobileAppSteeringMode::SteeringMode::crab_mode):
          steering_mode_ = MobileAppSteeringMode::SteeringMode::crab_mode;
          break;
        default:
          steering_mode_ = MobileAppSteeringMode::SteeringMode::wrong_value;
          break;
      }
    }

    else if ((MobileAppCanFrameId::tpl_config & addr_mask) == (message.can_id & addr_mask))
    {
      if (setTplConfig(message))
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

float MobileAppHandler::fGetManualRadius()
{
  return dRadius;
}

float MobileAppHandler::fGetManualLinearVelo()
{
  return dManualLinearVelo;
}

bool MobileAppHandler::bGetChangeDeltaGeometryUpFront()
{
  return changeDeltaGeometryElevatedFront;
}

bool MobileAppHandler::bGetChangeDeltaGeometryUpBack()
{
  return changeDeltaGeometryElevatedBack;
}

MobileAppSteeringMode::SteeringMode MobileAppHandler::getSteeringMode()
{
  return steering_mode_;
}

void MobileAppHandler::vSetChosenTask(std::string strTaskName)
{
  strChosenTask = strTaskName;
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
  msg.data[MobileAppRfid::selected_antenna_byte] = selected_antenna;
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
      // if data frame payload exceeded - every 7 bytes send frame
      if (cnt % MobileAppRfid::data_frame_epc_payload == 0)
      {
        ++com;  //  increment com byte
        message.data[MobileAppRfid::com_byte] = com;
        usleep(MobileAppRfid::frame_send_delay);
        can_handler_->iSendFrame(&message, false);
        bzero(message.data, sizeof(uint8_t) * MobileAppRfid::tag_data_frame_size);
        ++cnt;
      }
    }  // send remaining frame - less than 7 bytes
    if ((--cnt % MobileAppRfid::data_frame_epc_payload != 0) || (cnt == 0))
    {
      ++com;  //  increment com byte
      message.data[MobileAppRfid::com_byte] = com;
      usleep(MobileAppRfid::frame_send_delay);
      can_handler_->iSendFrame(&message, false);
    }
  }
}

bool MobileAppHandler::setTplConfig(const can_frame& message)
{
  CanFrame frame;
  frame.setupFrame(message);
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

void MobileAppHandler::sendDeltasGeometryResponseFrameFront(const uint8_t ret)
{
  can_frame message;
  message.can_id = MobileAppCanFrameId::change_delta_geometry_front;
  message.can_dlc = MobileAppDeltaGeometry::delta_geometry_response_frame_length;
  message.data[MobileAppDeltaGeometry::request_response_byte] = MobileAppDeltaGeometry::response_byte_value;
  message.data[MobileAppDeltaGeometry::caterpillar_state_byte] = static_cast<uint8_t>(ret);
  can_handler_->iSendFrame(&message, false);
}

void MobileAppHandler::sendDeltasGeometryResponseFrameBack(const uint8_t ret)
{
  can_frame message;
  message.can_id = MobileAppCanFrameId::change_delta_geometry_back;
  message.can_dlc = MobileAppDeltaGeometry::delta_geometry_response_frame_length;
  message.data[MobileAppDeltaGeometry::request_response_byte] = MobileAppDeltaGeometry::response_byte_value;
  message.data[MobileAppDeltaGeometry::caterpillar_state_byte] = static_cast<uint8_t>(ret);
  can_handler_->iSendFrame(&message, false);
}

void MobileAppHandler::sendSteeringModeResponseFrame(const uint8_t return_code)
{
  can_frame message;
  message.can_id = MobileAppCanFrameId::change_steering_mode;
  message.can_dlc = MobileAppSteeringMode::response_frame_length;
  message.data[MobileAppSteeringMode::request_response_byte] = MobileAppSteeringMode::response_byte_value;
  message.data[MobileAppSteeringMode::return_code_byte] = return_code;
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
  uint16_t processed_angle = abs(feedback.tpl_angle) * TplConfig::encode_constant;
  if (feedback.tpl_angle < 0)
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
  for (const auto& data : frames_data)
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
