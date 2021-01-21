/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <cstdint>

enum class ConfigId
{
  front_pto = 0x01,
  front_green = 0x02,
  front_yellow = 0x03,
  front_blue = 0x04,
  front_tpl_angle = 0x05,
  front_tpl_height = 0x06,
  back_pto = 0x07,
  back_green = 0x08,
  back_yellow = 0x09,
  back_blue = 0x0A,
  back_tpl_angle = 0x0B,
  back_tpl_height = 0x0C
};

enum class ValveColor
{
  green,
  blue,
  yellow
};

//TPL config
class TplConfig
{
 private:
  double pto = 0.0;           // rpm
  double green_valve = 0.0;   // valve open %
  double yellow_valve = 0.0;  // valve open %
  double blue_valve = 0.0;    // valve open %
  double angle = 0.0;         // radians
  double height = 0.0;        // meters
 public:
  TplConfig() {};
  void setPtoFromFrame(const uint8_t* frame);
  void setValveFromFrame(const uint8_t* frame, ValveColor sel_valve);
  void setAngleFromFrame(const uint8_t* frame);
  void setHeightFromFrame(const uint8_t* frame);
};
