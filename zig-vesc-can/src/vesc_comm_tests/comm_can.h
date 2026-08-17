/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

#ifndef COMM_CAN_H_
#define COMM_CAN_H_

#include <stdint.h>
#include <stdbool.h>
#include <linux/can.h>

// Settings
#define CAN_STATUS_MSGS_TO_STORE	10


extern struct can_frame created_frame;

// Functions
void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len);
void comm_can_transmit_eid_if(uint32_t id, const uint8_t *data, uint8_t len, int interface);
void comm_can_transmit_eid_replace(uint32_t id, const uint8_t *data, uint8_t len, bool replace, int interface);
void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len);
void comm_can_set_duty(uint8_t controller_id, float duty);
void comm_can_set_current(uint8_t controller_id, float current);
void comm_can_set_current_off_delay(uint8_t controller_id, float current, float off_delay);
void comm_can_set_current_brake(uint8_t controller_id, float current);
void comm_can_set_rpm(uint8_t controller_id, float rpm);
void comm_can_set_pos(uint8_t controller_id, float pos);
void comm_can_set_current_rel(uint8_t controller_id, float current_rel);
void comm_can_set_current_rel_off_delay(uint8_t controller_id, float current_rel, float off_delay);
void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel);
void comm_can_conf_current_limits(uint8_t controller_id,
		bool store, float min, float max);
void comm_can_conf_current_limits_in(uint8_t controller_id,
		bool store, float min, float max);
void comm_can_conf_battery_cut(uint8_t controller_id,
		bool store, float start, float end);
void comm_can_shutdown(uint8_t controller_id);

#endif /* COMM_CAN_H_ */
