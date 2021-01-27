/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include <hal_mobile_app/TplConfigHandle.h>
#include <iostream>

uint16_t TplConfigHandle::assembleU16FromFrame(uint8_t first_byte, uint8_t zero_byte)
{
  return ((uint16_t)first_byte) << 8 | zero_byte;
}

int16_t TplConfigHandle::getAngleSign(int16_t ang)
{
  return ((ang & sign_mask_) == sign_mask_ ? -1 : 1);
}

bool TplConfigHandle::setPto(const uint8_t* const data)
{
  uint16_t pto = assembleU16FromFrame(data[ByteIndex::first], data[ByteIndex::zero]);
  if (pto < limit_.pto_rpm_min || pto > limit_.pto_rpm_max)
  {
    return false;
  }
  config_.pto_rpm = pto;
  return true;
}

bool TplConfigHandle::setValveFlow(const uint8_t* const data, ValveColor sel_valve)
{
  uint16_t valve_flow = assembleU16FromFrame(data[ByteIndex::first], data[ByteIndex::zero]);
  if (valve_flow < limit_.valve_flow_percent_min || valve_flow > limit_.valve_flow_percent_max)
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
    case ValveColor::green:
      config_.green_valve_flow = valve_flow;
      break;
  }
  return true;
}

bool TplConfigHandle::setAngle(const uint8_t* const data)
{
  double angle = 0.0;
  int16_t angle_tmp = assembleU16FromFrame(data[ByteIndex::first], data[ByteIndex::zero]);
  int16_t sign = getAngleSign(angle_tmp);
  angle_tmp = ((~sign_mask_) & angle_tmp) * sign;
  angle = static_cast<double>(angle_tmp)/divider_;
  if (angle < limit_.angle_rad_min || angle > limit_.angle_rad_max)
  {
    return false;
  }
  config_.tpl_angle = angle;
  return true;
}

bool TplConfigHandle::setHeight(const uint8_t* const data)
{
  double height = static_cast<double>(assembleU16FromFrame(data[ByteIndex::first], data[ByteIndex::zero]))/divider_;
  if (height < limit_.height_meters_min || height > limit_.height_meters_max)
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
