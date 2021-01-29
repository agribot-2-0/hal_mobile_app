/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_common/TplConfig.h>
#include <hal_mobile_app/TplConfigHandle/TplConfigHandleConstants.h>

#include <vector>

namespace TplConfig
{
// TPL config
class TplConfigHandle
{
 private:
  tool_manager::TplConfig config_;

  enum ByteIndex
  {
    first = 1,
    zero = 2
  };

  const struct
  {
    double pto_rpm_min = 0;
    double pto_rpm_max = 1300;
    double valve_flow_percent_min = 0;
    double valve_flow_percent_max = 100;
    double angle_rad_min = -0.26;  // -15 deg
    double angle_rad_max = 0.52;   // 30 deg
    double height_meters_min = 0.0;
    double height_meters_max = 0.45;
  } limit_;

  const uint16_t negative_sign_mask_ = 0x8000;
  const int16_t tpl_height_angle_divider_ = 1000;

  uint16_t assembleU16FromFrame(const uint8_t zero_byte, const uint8_t first_byte) const;
  int16_t getAngleSign(const int16_t preprocessed_angle) const;
  bool isValueWithinRange(const double val, const double limit_min, const double limit_max) const;
  double assembleAngle(int16_t angle_tmp) const;

 public:
  TplConfigHandle(){}
  bool setPto(const std::vector<uint8_t>& data);
  bool setValveFlow(const std::vector<uint8_t>& data, const ValveColor sel_valve);
  bool setAngle(const std::vector<uint8_t>& data);
  bool setHeight(const std::vector<uint8_t>& data);
  tool_manager::TplConfig getConfig() const;
};
}  // namespace TplConfig
