/*
    This file is a modified version of the originial by Michael Foley foley586@umn.edu
    The original can be found at
    https://github.com/vedderb/bldc/blob/c0094f58db86efab1fa0f496ebb59c387a573f6b/comm/comm_can.c
    The original notice has been preserved below.

    ============================================================================

	Copyright 2016 - 2020 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "comm_can.h"
#pragma GCC optimize ("Os")

#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <linux/can.h>
#include "datatypes.h"
#include "buffer.h"

#define RX_FRAMES_SIZE	50
#define RX_BUFFER_NUM	3
#define RX_BUFFER_SIZE	PACKET_MAX_PL_LEN

// extern struct can_frame created_frame;
struct can_frame created_frame;

// Variables
static unsigned int detect_all_foc_res_index = 0;
static int8_t detect_all_foc_res[50];

// Function pointers
static bool(*sid_callback)(uint32_t id, uint8_t *data, uint8_t len) = 0;
static bool(*eid_callback)(uint32_t id, uint8_t *data, uint8_t len) = 0;

/**
 * Transmit CAN packet with extended ID.
 *
 * @param id
 * EID
 *
 * @param data
 * Data
 *
 * @param len
 * Length of data, max 8 bytes.
 *
 * @param replace
 * Process packets for motor2 directly instead of sending them. Unused
 * on single motor hardware.
 *
 * @param interface
 * CAN-interface
 * 0: Both
 * 1: CAN1
 * 2: CAN2
 *
 * @return
 * MSG_OK for success, anything else otherwise.
 */
void comm_can_transmit_eid_replace(uint32_t id, const uint8_t *data, uint8_t len, bool replace, int interface) {
    created_frame.can_id = id | (1u << 31);
    created_frame.len = len;
    for(int i = 0; i < len; i++) {
        created_frame.data[i] = data[i];
    }
    created_frame.len8_dlc = 0;
}


void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
	return comm_can_transmit_eid_replace(id, data, len, false, 0);
}

void comm_can_transmit_eid_if(uint32_t id, const uint8_t *data, uint8_t len, int interface) {
	return comm_can_transmit_eid_replace(id, data, len, false, interface);
}

void comm_can_set_duty(uint8_t controller_id, float duty) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
	comm_can_transmit_eid_replace(controller_id |
			((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index, true, 0);
}

void comm_can_set_current(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
	comm_can_transmit_eid_replace(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index, true, 0);
}

void comm_can_set_current_off_delay(uint8_t controller_id, float current, float off_delay) {
	int32_t send_index = 0;
	uint8_t buffer[6];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
	buffer_append_float16(buffer, off_delay, 1e3, &send_index);
	comm_can_transmit_eid_replace(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index, true, 0);
}

void comm_can_set_current_brake(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
	comm_can_transmit_eid_replace(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index, true, 0);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	comm_can_transmit_eid_replace(controller_id |
			((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index, true, 0);
}

void comm_can_set_pos(uint8_t controller_id, float pos) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), &send_index);
	comm_can_transmit_eid_replace(controller_id |
			((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index, true, 0);
}

/**
 * Set current relative to the minimum and maximum current limits.
 *
 * @param controller_id
 * The ID of the VESC to set the current on.
 *
 * @param current_rel
 * The relative current value, range [-1.0 1.0]
 */
void comm_can_set_current_rel(uint8_t controller_id, float current_rel) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
	comm_can_transmit_eid_replace(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index, true, 0);
}

/**
 * Same as above, but also sets the off delay
 */
void comm_can_set_current_rel_off_delay(uint8_t controller_id, float current_rel, float off_delay) {
	int32_t send_index = 0;
	uint8_t buffer[6];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
	buffer_append_float16(buffer, off_delay, 1e3, &send_index);
	comm_can_transmit_eid_replace(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index, true, 0);
}

/**
 * Set brake current relative to the minimum current limit.
 *
 * @param controller_id
 * The ID of the VESC to set the current on.
 *
 * @param current_rel
 * The relative current value, range [0.0 1.0]
 */
void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
	comm_can_transmit_eid_replace(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), buffer, send_index, true, 0);
}

/**
 * Set handbrake current.
 *
 * @param controller_id
 * The ID of the VESC to set the handbrake current on.
 *
 * @param current_rel
 * The handbrake current value
 */
void comm_can_set_handbrake(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current, 1e3, &send_index);
	comm_can_transmit_eid_replace(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8), buffer, send_index, true, 0);
}

/**
 * Set handbrake current relative to the minimum current limit.
 *
 * @param controller_id
 * The ID of the VESC to set the handbrake current on.
 *
 * @param current_rel
 * The relative handbrake current value, range [0.0 1.0]
 */
void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
	comm_can_transmit_eid_replace(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8), buffer, send_index, true, 0);
}


/**
 * Update current limits on VESC on CAN-bus.
 *
 * @param controller_id
 * ID of the VESC.
 *
 * @param store
 * Store parameters in emulated EEPROM (FLASH).
 *
 * @param min
 * Minimum current (negative value).
 *
 * @param max
 * Maximum current.
 */
void comm_can_conf_current_limits(uint8_t controller_id,
		bool store, float min, float max) {
	int32_t send_index = 0;
	uint8_t buffer[8];
	buffer_append_float32(buffer, min, 1e3, &send_index);
	buffer_append_float32(buffer, max, 1e3, &send_index);
	comm_can_transmit_eid_replace(controller_id |
			((uint32_t)(store ? CAN_PACKET_CONF_STORE_CURRENT_LIMITS :
					CAN_PACKET_CONF_CURRENT_LIMITS) << 8), buffer, send_index, true, 0);
}

/**
 * Update input current limits on VESC on CAN-bus.
 *
 * @param controller_id
 * ID of the VESC.
 *
 * @param store
 * Store parameters in emulated EEPROM (FLASH).
 *
 * @param min
 * Minimum current (negative value).
 *
 * @param max
 * Maximum current.
 */
void comm_can_conf_current_limits_in(uint8_t controller_id,
		bool store, float min, float max) {
	int32_t send_index = 0;
	uint8_t buffer[8];
	buffer_append_float32(buffer, min, 1e3, &send_index);
	buffer_append_float32(buffer, max, 1e3, &send_index);
	comm_can_transmit_eid_replace(controller_id |
			((uint32_t)(store ? CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN :
					CAN_PACKET_CONF_CURRENT_LIMITS_IN) << 8), buffer, send_index, true, 0);
}

/**
 * Update FOC ERPM settings on VESC on CAN-bus.
 *
 * @param controller_id
 * ID of the VESC.
 *
 * @param store
 * Store parameters in emulated EEPROM (FLASH).
 *
 * @param foc_openloop_rpm
 * Run in openloop below this ERPM in sensorless mode.
 *
 * @param foc_sl_erpm
 * Use sensors below this ERPM in sensored mode.
 */
void comm_can_conf_foc_erpms(uint8_t controller_id,
		bool store, float foc_openloop_rpm, float foc_sl_erpm) {
	int32_t send_index = 0;
	uint8_t buffer[8];
	buffer_append_float32(buffer, foc_openloop_rpm, 1e3, &send_index);
	buffer_append_float32(buffer, foc_sl_erpm, 1e3, &send_index);
	comm_can_transmit_eid_replace(controller_id |
			((uint32_t)(store ? CAN_PACKET_CONF_STORE_FOC_ERPMS :
					CAN_PACKET_CONF_FOC_ERPMS) << 8), buffer, send_index, true, 0);
}

int comm_can_detect_all_foc_res(unsigned int index) {
	if (index < detect_all_foc_res_index) {
		return detect_all_foc_res[detect_all_foc_res_index];
	} else {
		return -999;
	}
}

int comm_can_detect_all_foc_res_size(void) {
	return detect_all_foc_res_index;
}

void comm_can_detect_all_foc_res_clear(void) {
	detect_all_foc_res_index = 0;
}

void comm_can_conf_battery_cut(uint8_t controller_id,
		bool store, float start, float end) {
	int32_t send_index = 0;
	uint8_t buffer[8];
	buffer_append_float32(buffer, start, 1e3, &send_index);
	buffer_append_float32(buffer, end, 1e3, &send_index);
	comm_can_transmit_eid_replace(controller_id |
			((uint32_t)(store ? CAN_PACKET_CONF_STORE_BATTERY_CUT :
					CAN_PACKET_CONF_BATTERY_CUT) << 8), buffer, send_index, true, 0);
}

void comm_can_shutdown(uint8_t controller_id) {
	int32_t send_index = 0;
	uint8_t buffer[8];
	comm_can_transmit_eid_replace(controller_id |
			((uint32_t)(CAN_PACKET_SHUTDOWN) << 8), buffer, send_index, true, 0);
}


