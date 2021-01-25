/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <cstdint>
#include <agribot_common/TplConfig.h>

enum class ConfigId
{
  front_pto = 0x01,
  front_red = 0x02,
  front_yellow = 0x03,
  front_blue = 0x04,
  front_tpl_angle = 0x05,
  front_tpl_height = 0x06,
  back_pto = 0x07,
  back_red = 0x08,
  back_yellow = 0x09,
  back_blue = 0x0A,
  back_tpl_angle = 0x0B,
  back_tpl_height = 0x0C
};

enum class ValveColor
{
  red,
  blue,
  yellow
};

enum class SelectedTpl
{
  front = 0,
  back = 1
};

//TPL config
class TplConfigHandle
{
 private:
  tool_manager::TplConfig config_;

  const struct
  {
    uint8_t first = 1;
    uint8_t zero = 2;
  } byte_index_;

  const struct
  {
    double pto_min = 0;
    double pto_max = 1300;
    double valve_flow_min = 0;
    double valve_flow_max = 100;
    double angle_min = -0.26;
    double angle_max = 0.52;
    double height_min = 0.0;
    double height_max = 0.45;
  } limit_;

  const uint16_t sign_mask_ = 0x8000;
  const int16_t divider_ = 1000;

  uint16_t assembleU16FromFrame(uint8_t zero_byte, uint8_t first_byte);
  int16_t getAngleSign(int16_t ang);
 public:
  TplConfigHandle() {};
  bool setPto(const uint8_t* frame);
  bool setValveFlow(const uint8_t* frame, ValveColor sel_valve);
  bool setAngle(const uint8_t* frame);
  bool setHeight(const uint8_t* frame);
  tool_manager::TplConfig getConfig();
};
