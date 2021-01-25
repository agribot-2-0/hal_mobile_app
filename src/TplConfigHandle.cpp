/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include <hal_mobile_app/TplConfigHandle.h>

uint16_t TplConfigHandle::assembleU16FromFrame(uint8_t first_byte, uint8_t zero_byte)
{
  return ((uint16_t)first_byte) << 8 | zero_byte;
}

int16_t TplConfigHandle::getAngleSign(int16_t ang)
{
  return (ang & sign_mask_ == sign_mask_ ? -1 : 1);
}

bool TplConfigHandle::setPto(const uint8_t* frame)
{
  uint16_t pto = assembleU16FromFrame(byte_index_.first, byte_index_.zero);
  if (pto < limit_.pto_min || pto > limit_.pto_max)
  {
    return false;
  }
  config_.pto_rpm = pto;
  return true;
}

bool TplConfigHandle::setValveFlow(const uint8_t* frame, ValveColor sel_valve)
{
  uint16_t valve_flow = assembleU16FromFrame(byte_index_.first, byte_index_.zero);
  if (valve_flow < limit_.valve_flow_min || valve_flow > limit_.valve_flow_max)
  {
    return false;
  }
  switch (sel_valve)
  {
    case ValveColor::red:
      config_.red_valve_flow = valve_flow;
      break;
    case ValveColor::blue:
      config_.blue_valve_flow = valve_flow;
      break;
    case ValveColor::yellow:
      config_.yellow_valve_flow = valve_flow;
      break;
  }
  return true;
}

bool TplConfigHandle::setAngle(const uint8_t* frame)
{
  double angle = 0.0;
  int16_t angle_tmp = assembleU16FromFrame(byte_index_.first, byte_index_.zero);
  angle_tmp = angle_tmp * getAngleSign(angle_tmp);
  angle_tmp = (~sign_mask_) & angle_tmp;
  angle = angle_tmp/divider_;
  if (angle < limit_.angle_min || angle > limit_.angle_max)
  {
    return false;
  }
  config_.tpl_angle = angle;
  return true;
}

bool TplConfigHandle::setHeight(const uint8_t* frame)
{
  double height = assembleU16FromFrame(byte_index_.first, byte_index_.zero)/divider_;
  if (height >= limit_.height_min || height <= limit_.height_max)
  {
    return false;
  }
  config_.tpl_height = height;
  return true;
}

tool_manager::TplConfig TplConfigHandle::getConfig()
{
  return config_;
}
