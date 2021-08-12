/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <cstdint>

namespace MobileAppCanFrameId
{
inline constexpr uint16_t protocol_ark{ 0x035 };
inline constexpr uint16_t protocol_hb{ 0x001 };
inline constexpr uint16_t protocol_handshake{ 0x00A };
inline constexpr uint16_t protocol_manual_steering{ 0x015 };
inline constexpr uint16_t protocol_automatic_steering{ 0x01F };
inline constexpr uint16_t protocol_error{ 0x0C8 };
inline constexpr uint16_t protocol_man_error{ 0x0C9 };  // man - manual
inline constexpr uint16_t protocol_configuration{ 0x96 };
inline constexpr uint16_t protocol_eng_error_start{ 0x104 };
inline constexpr uint16_t protocol_eng_error{ 0x105 };
inline constexpr uint16_t rfid{ 0x1f4 };
inline constexpr uint16_t tpl_config{ 0x13D };
inline constexpr uint16_t tpl_response{ 0x13E };
inline constexpr uint16_t tpl_feedback{ 0x13F };
inline constexpr uint16_t change_delta_geometry_front{ 0x1f5 };
inline constexpr uint16_t change_delta_geometry_back{ 0x1f6 };
inline constexpr uint16_t change_steering_mode{ 0x16C };
inline constexpr uint16_t logging_start{ 0x12E };
inline constexpr uint16_t logging_stop{ 0x12F };
}  // namespace MobileAppCanFrameId

namespace MobileAppRfid
{
inline constexpr uint32_t frame_send_delay{ 50000 };

inline constexpr uint8_t selected_antenna_byte{ 0x01 };
inline constexpr uint8_t com_byte{ 0 };
inline constexpr uint8_t sel_ant_byte{ 1 };
inline constexpr uint8_t epc_len_byte{ 2 };
inline constexpr uint8_t scan_trigger_frame{ 0x01 };
inline constexpr uint8_t detected_tags_start_frame{ 0x02 };
inline constexpr uint8_t front_ant_selected{ 0x01 };
inline constexpr uint8_t back_ant_selected{ 0x02 };
inline constexpr uint8_t tag_start_frame_size{ 3 };
inline constexpr uint8_t tag_data_frame_size{ 8 };
inline constexpr uint8_t data_frame_epc_payload{ 7 };
}  // namespace MobileAppRfid

namespace MobileAppDeltaGeometry
{
inline constexpr uint8_t delta_geometry_response_frame_length{ 2 };
inline constexpr uint8_t request_response_byte{ 0 };
inline constexpr uint8_t response_byte_value{ 1 };
inline constexpr uint8_t request_byte_value{ 0 };
inline constexpr uint8_t caterpillar_state_byte{ 1 };
inline constexpr uint8_t elevated_caterpillar_state{ 1 };
inline constexpr uint8_t normal_caterpillar_state{ 0 };
}  // namespace MobileAppDeltaGeometry

namespace MobileAppTpl
{
inline constexpr uint8_t config_id_byte{ 0 };
inline constexpr uint8_t ret_code_byte{ 1 };
inline constexpr uint8_t tpl_resp_frame_len{ 2 };

enum feedback_byte
{
  first_vector = 0,
  zero_vector = 1,
  first_frame = 1,
  zero_frame = 2
};

}  // namespace MobileAppTpl

// TODO(all): encapsulate in namespaces/enums
#define LIN_VELO_SIGN_BYTE 0
#define DIRECTION_SIGN_BYTE 1
#define RADIUS_BYTE 2
#define LIN_VELO_BYTE 4
#define RADIUS_PARSE_CONST 10
#define LIN_VELO_PARSE_CONST 100

constexpr uint16_t addr_mask = 0x7FF;

#define MAX_RADIUS 10
#define MIN_RADIUS 1.5

#define LEFT_ARCH 1
#define RIGHT_ARCH -1

#define BACKWARD -1
#define FORWARD 1

#define ERROR_TYPE_BYTE 0
#define ERROR_SPECIFIC_BYTE 1

#define ENG_ERR_NUM_TOTAL_BYTE 0
#define LAMP_STATE_BYTE 1
#define ENG_ERROR_NUM_BYTE 0
#define SPN_FIRST_BYTE 1
#define FMI_BYTE 4
#define OCC_BYTE 5
#define ENG_ERR_STATE_BYTE 6

#define JOYSTICK_CONFIRMATION_FRAME_LENGTH 2
