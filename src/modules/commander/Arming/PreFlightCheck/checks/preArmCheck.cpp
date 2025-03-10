/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "../PreFlightCheck.hpp"

#include <ArmAuthorization.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <HealthFlags.h>

bool PreFlightCheck::preArmCheck(orb_advert_t *mavlink_log_pub, const vehicle_status_flags_s &status_flags,
				 const vehicle_control_mode_s &control_mode,
				 const safety_s &safety, const arm_requirements_t &arm_requirements, vehicle_status_s &status, bool report_fail)
{
	bool prearm_ok = true;

	// rate control mode require valid angular velocity
	if (control_mode.flag_control_rates_enabled && !status_flags.angular_velocity_valid) {
		if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! angular velocity invalid"); }

		prearm_ok = false;
	}

	// attitude control mode require valid attitude
	if (control_mode.flag_control_attitude_enabled && !status_flags.attitude_valid) {
		if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! attitude invalid"); }

		prearm_ok = false;
	}

	// velocity control mode require valid velocity
	if (control_mode.flag_control_velocity_enabled && !status_flags.local_velocity_valid) {
		if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! velocity invalid"); }

		prearm_ok = false;
	}

	// position control mode require valid position
	if (control_mode.flag_control_position_enabled && !status_flags.local_position_valid) {
		if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! position invalid"); }

		prearm_ok = false;
	}

    bool companion_connected = true;
	// manual control mode require valid manual control (rc)
	if (control_mode.flag_control_manual_enabled && status.rc_signal_lost && !companion_connected) {
		if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! manual control lost"); }

		prearm_ok = false;
	}

	if (status_flags.flight_terminated) {
		if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Flight termination active"); }

		prearm_ok = false;
	}

	// USB not connected
	if (!status_flags.circuit_breaker_engaged_usb_check && status_flags.usb_connected) {
		if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Flying with USB is not safe"); }

		prearm_ok = false;
	}

	// battery and system power status
	if (!status_flags.circuit_breaker_engaged_power_check) {

		// Fail transition if power is not good
		if (!status_flags.power_input_valid) {
			if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Connect power module"); }

			prearm_ok = false;
		}

		// main battery level
		set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_SENSORBATTERY, true, true,
				 status_flags.battery_healthy, status);

		// Only arm if healthy
		if (!status_flags.battery_healthy) {
			if (prearm_ok) {
				if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Check battery"); }
			}

			prearm_ok = false;
		}
	}

	// Arm Requirements: mission
	if (arm_requirements.mission) {

		if (!status_flags.auto_mission_available) {
			if (prearm_ok) {
				if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! No valid mission"); }
			}

			prearm_ok = false;
		}

		if (!status_flags.global_position_valid) {
			if (prearm_ok) {
				if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Missions require a global position"); }
			}

			prearm_ok = false;
		}
	}

	if (arm_requirements.global_position && !status_flags.circuit_breaker_engaged_posfailure_check) {

		if (!status_flags.global_position_valid) {
			if (prearm_ok) {
				if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Global position required"); }
			}

			prearm_ok = false;
		}

		if (!status_flags.home_position_valid) {
			if (prearm_ok) {
				if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Home position invalid"); }
			}

			prearm_ok = false;
		}
	}

	// safety button
	if (safety.safety_switch_available && !safety.safety_off) {
		// Fail transition if we need safety switch press
		if (prearm_ok) {
			if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Press safety switch first"); }
		}

		prearm_ok = false;
	}

	if (status_flags.avoidance_system_required && !status_flags.avoidance_system_valid) {
		if (prearm_ok) {
			if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Avoidance system not ready"); }
		}

		prearm_ok = false;

	}

	if (arm_requirements.esc_check && status_flags.escs_error) {
		if (prearm_ok) {
			if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! One or more ESCs are offline"); }

			prearm_ok = false;
		}
	}

	if (arm_requirements.esc_check && status_flags.escs_failure) {
		if (prearm_ok) {
			if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! One or more ESCs have a failure"); }

			prearm_ok = false;
		}
	}

	if (status.is_vtol) {

		if (status.in_transition_mode) {
			if (prearm_ok) {
				if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Vehicle is in transition state"); }

				prearm_ok = false;
			}
		}

		if (!status_flags.circuit_breaker_vtol_fw_arming_check
		    && status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			if (prearm_ok) {
				if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Vehicle is not in multicopter mode"); }

				prearm_ok = false;
			}
		}
	}

	if (arm_requirements.geofence && status.geofence_violated) {
		if (report_fail) {
			mavlink_log_critical(mavlink_log_pub, "Arming denied, vehicle outside geofence");
		}

		prearm_ok = false;
	}

	// Arm Requirements: authorization
	// check last, and only if everything else has passed
	if (arm_requirements.arm_authorization && prearm_ok) {
		if (arm_auth_check() != vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED) {
			// feedback provided in arm_auth_check
			prearm_ok = false;
		}
	}


	return prearm_ok;
}
