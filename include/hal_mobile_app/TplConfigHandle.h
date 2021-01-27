/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_common/TplConfig.h>
#include <cstdint>

enum class TplConfigId
{
  front_pto = 0x01,
  front_red = 0x02,
  front_green = 0x03,
  front_blue = 0x04,
  front_tpl_angle = 0x05,
  front_tpl_height = 0x06,
  back_pto = 0x07,
  back_red = 0x08,
  back_green = 0x09,
  back_blue = 0x0A,
  back_tpl_angle = 0x0B,
  back_tpl_height = 0x0C
};

enum class ValveColor
{
  red,
  blue,
  green
};

enum class SelectedTpl
{
  front,
  back
};

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

  const uint16_t sign_mask_ = 0x8000;
  const int16_t divider_ = 1000;

  uint16_t assembleU16FromFrame(uint8_t zero_byte, uint8_t first_byte);
  int16_t getAngleSign(int16_t ang);
 public:
  TplConfigHandle() {}
  bool setPto(const uint8_t* const data);
  bool setValveFlow(const uint8_t* const data, ValveColor sel_valve);
  bool setAngle(const uint8_t* const data);
  bool setHeight(const uint8_t* const data);
  tool_manager::TplConfig getConfig();
};
