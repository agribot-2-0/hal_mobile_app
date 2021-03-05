/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <cstdint>

namespace TplConfig
{
enum class TplConfigId : uint8_t
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

constexpr std::uint8_t feedback_frame_size = 3;

constexpr std::uint8_t front_config_start_id = 1;
constexpr std::uint8_t back_config_start_id = 7;

inline constexpr uint16_t negative_sign_mask{ 0x8000 };
inline constexpr uint16_t encode_constant { 1000 };

}  // namespace TplConfig
