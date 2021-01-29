/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include <hal_mobile_app/TplConfigHandle/TplConfigHandle.h>
#include <iostream>

namespace TplConfig
{
uint16_t TplConfigHandle::assembleU16FromFrame(const uint8_t first_byte, const uint8_t zero_byte) const
{
  return static_cast<uint16_t>(first_byte) << 8 | zero_byte;
}

int16_t TplConfigHandle::getAngleSign(const int16_t preprocessed_angle) const
{
  return ((preprocessed_angle & negative_sign_mask_) == negative_sign_mask_ ? -1 : 1);
}

bool TplConfigHandle::isValueWithinRange(const double val, const double limit_min, const double limit_max) const
{
  return (val >= limit_min && val <= limit_max);
}

bool TplConfigHandle::setPto(const std::vector<uint8_t>& data)
{
  uint16_t pto = assembleU16FromFrame(data[ByteIndex::first], data[ByteIndex::zero]);
  if (isValueWithinRange(pto, limit_.pto_rpm_min, limit_.pto_rpm_max))
  {
    config_.pto_rpm = pto;
    return true;
  }
  return false;
}

bool TplConfigHandle::setValveFlow(const std::vector<uint8_t>& data, const ValveColor sel_valve)
{
  uint16_t valve_flow = assembleU16FromFrame(data[ByteIndex::first], data[ByteIndex::zero]);
  if (isValueWithinRange(valve_flow, limit_.valve_flow_percent_min, limit_.valve_flow_percent_max))
  {
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
  return false;
}

double TplConfigHandle::assembleAngle(int16_t preprocessed_angle) const
{
  int16_t sign = getAngleSign(preprocessed_angle);
  preprocessed_angle = ((~negative_sign_mask_) & preprocessed_angle) * sign;
  return static_cast<double>(preprocessed_angle) / tpl_height_angle_divider_;
}

bool TplConfigHandle::setAngle(const std::vector<uint8_t>& data)
{
  double angle = assembleAngle(assembleU16FromFrame(data[ByteIndex::first], data[ByteIndex::zero]));
  if (isValueWithinRange(angle, limit_.angle_rad_min, limit_.angle_rad_max))
  {
    config_.tpl_angle = angle;
    return true;
  }
  return false;
}

bool TplConfigHandle::setHeight(const std::vector<uint8_t>& data)
{
  double height = static_cast<double>(assembleU16FromFrame(data[ByteIndex::first], data[ByteIndex::zero])) /
                  tpl_height_angle_divider_;
  if (isValueWithinRange(height, limit_.height_meters_min, limit_.height_meters_max))
  {
    config_.tpl_height = height;
    return true;
  }
  return false;
}

tool_manager::TplConfig TplConfigHandle::getConfig() const
{
  return config_;
}
}  // namespace TplConfig
