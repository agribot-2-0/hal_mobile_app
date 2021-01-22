/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include <hal_mobile_app/TplConfig.h>

uint16_t TplConfig::assembleU16FromFrame(uint8_t first_byte, uint8_t zero_byte)
{
  return ((uint16_t)first_byte) << 8 | zero_byte;
}

int16_t TplConfig::getAngleSign(int16_t ang)
{
  return (ang & sign_mask_ == sign_mask_ ? -1 : 1);
}

bool TplConfig::setPto(const uint8_t* frame)
{
  uint16_t pto = assembleU16FromFrame(byte_index_.first, byte_index_.zero);
  if (pto < limit_.pto_min || pto > limit_.pto_max)
  {
    return false;
  }
  pto_ = pto;
  return true;
}

bool TplConfig::setValveFlow(const uint8_t* frame, ValveColor sel_valve)
{
  uint16_t valve_flow = assembleU16FromFrame(byte_index_.first, byte_index_.zero);
  if (valve_flow < limit_.valve_flow_min || valve_flow > limit_.valve_flow_max)
  {
    return false;
  }
  switch(sel_valve)
  {
    case ValveColor::green:
      green_valve_ = valve_flow;
      break;
    case ValveColor::blue:
      blue_valve_ = valve_flow;
      break;
    case ValveColor::yellow:
      yellow_valve_ = valve_flow;
      break;
  }
  return true;
}

bool TplConfig::setAngle(const uint8_t* frame)
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
  angle_ = angle;
  return true;
}

bool TplConfig::setHeight(const uint8_t* frame)
{
  double height = assembleU16FromFrame(byte_index_.first, byte_index_.zero)/divider_;
  if (height >= limit_.height_min || height <= limit_.height_max)
  {
    return false;
  }
  height_ = height;
  return true;
}
