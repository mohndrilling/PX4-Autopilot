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

/**
 * @file PreFlightCheck.cpp
 */

#include "PreFlightCheck.hpp"

#include <drivers/drv_hrt.h>
#include <HealthFlags.h>
#include <lib/parameters/param.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>

using namespace time_literals;

static constexpr unsigned max_mandatory_mag_count = 1;
static constexpr unsigned max_mandatory_gyro_count = 1;
static constexpr unsigned max_mandatory_accel_count = 1;
static constexpr unsigned max_mandatory_baro_count = 1;

bool PreFlightCheck::preflightCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status,
				    vehicle_status_flags_s &status_flags, const vehicle_control_mode_s &control_mode,
				    bool report_failures, const bool prearm, const hrt_abstime &time_since_boot)
{
	report_failures = (report_failures && status_flags.system_hotplug_timeout
			   && !status_flags.calibration_enabled);

	bool failed = false;
	bool debug_mode = false;

    /* ---- AIRFRAME ---- */
    {
        bool airframeCheck_failed = !airframeCheck(mavlink_log_pub, status);
        if (airframeCheck_failed && debug_mode) {
            PX4_INFO("airframe check failed");
        } else {
            // PX4_INFO("airframe check succeeded");
        }
        failed |= airframeCheck_failed;
    }

    /* ---- SD CARD ---- */
    {
        bool sdcardCheck_failed = !airframeCheck(mavlink_log_pub, status);
        if (sdcardCheck_failed && debug_mode) {
            PX4_INFO("sdcard check failed");
        } else {
            // PX4_INFO("sdcard check succeeded");
        }
        failed |= sdcardCheck_failed;
    }

	/* ---- MAG ---- */
	{
		int32_t sys_has_mag = 1;
		param_get(param_find("SYS_HAS_MAG"), &sys_has_mag);

		if (sys_has_mag == 1) {
		    bool mag_available = sensorAvailabilityCheck(report_failures, max_mandatory_mag_count,
							   mavlink_log_pub, status, magnetometerCheck);
	        if (!mag_available && debug_mode) {
	            PX4_INFO("magnetometer isn't available");
	        } else {
	            // PX4_INFO("magnetometer is available");
	        }
			failed |= !mag_available;

			/* mag consistency checks (need to be performed after the individual checks) */
			bool mag_consistent = magConsistencyCheck(mavlink_log_pub, status, report_failures);
			if (!mag_consistent && debug_mode) {
	            PX4_INFO("magnetometer isn't consistent");
	        } else {
	            // PX4_INFO("magnetometer is consistent");
	        }
			failed |= !mag_consistent;
		}
	}

	/* ---- ACCEL ---- */
	{
	    bool accel_available = sensorAvailabilityCheck(report_failures, max_mandatory_accel_count,
						   mavlink_log_pub, status, accelerometerCheck);
	    if (!accel_available && debug_mode) {
	        PX4_INFO("accelerometer isn't available");
	    } else {
	        // PX4_INFO("accelerometer is available");
	    }
		failed |= !accel_available;
	}

	/* ---- GYRO ---- */
	{
	    bool gyro_available = sensorAvailabilityCheck(report_failures, max_mandatory_gyro_count,
						   mavlink_log_pub, status, gyroCheck);
		if (!gyro_available && debug_mode) {
	        PX4_INFO("gyroscope isn't available");
	    } else {
	        // PX4_INFO("gyroscope is available");
	    }
		failed |= !gyro_available;
	}

	/* ---- BARO ---- */
	{
		int32_t sys_has_baro = 1;
		param_get(param_find("SYS_HAS_BARO"), &sys_has_baro);

		if (sys_has_baro == 1) {
			bool baro_available = sensorAvailabilityCheck(report_failures, max_mandatory_baro_count,
					  mavlink_log_pub, status, baroCheck);
		    if (!baro_available && debug_mode) {
	            PX4_INFO("barometer isn't available");
            } else {
                // PX4_INFO("barometer is available");
            }
            failed |= !baro_available;
		}
	}

	/* ---- IMU CONSISTENCY ---- */
	// To be performed after the individual sensor checks have completed
	{
	    bool imu_consistent = imuConsistencyCheck(mavlink_log_pub, status, report_failures);
        if (!imu_consistent && debug_mode) {
            PX4_INFO("imu isn't consistent");
        } else {
            // PX4_INFO("imu is consistent");
        }
        failed |= !imu_consistent;
	}

	/* ---- Distance Sensor ---- */
	{
		int32_t sys_has_num_dist_sens = 0;
		param_get(param_find("SYS_HAS_NUM_DIST"), &sys_has_num_dist_sens);

		if (sys_has_num_dist_sens > 0) {
			bool dist_available = sensorAvailabilityCheck(report_failures, sys_has_num_dist_sens,
					  mavlink_log_pub, status, distSensCheck);
		    if (!dist_available && debug_mode) {
                PX4_INFO("distance sensor isn't available");
            } else {
                // PX4_INFO("distance sensor is available");
            }
            failed |= !dist_available;
		}

	}

	/* ---- AIRSPEED ---- */
	/* Perform airspeed check only if circuit breaker is not engaged and it's not a rotary wing */
	if (!status_flags.circuit_breaker_engaged_airspd_check &&
	    (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING || status.is_vtol)) {

		int32_t airspeed_mode = 0;
		param_get(param_find("FW_ARSP_MODE"), &airspeed_mode);
		const bool optional = (airspeed_mode == 1);

		int32_t max_airspeed_check_en = 0;
		param_get(param_find("COM_ARM_ARSP_EN"), &max_airspeed_check_en);

		float airspeed_trim = 10.0f;
		param_get(param_find("FW_AIRSPD_TRIM"), &airspeed_trim);

		const float arming_max_airspeed_allowed = airspeed_trim / 2.0f; // set to half of trim airspeed

		if (!airspeedCheck(mavlink_log_pub, status, optional, report_failures, prearm, (bool)max_airspeed_check_en,
				   arming_max_airspeed_allowed)
		    && !(bool)optional) {
			failed = true;
		}
	}

	/* ---- RC CALIBRATION ---- */
	int32_t com_rc_in_mode{0};
	param_get(param_find("COM_RC_IN_MODE"), &com_rc_in_mode);

	if (com_rc_in_mode == 0) {
		if (rcCalibrationCheck(mavlink_log_pub, report_failures) != OK) {
			if (report_failures) {
				mavlink_log_critical(mavlink_log_pub, "RC calibration check failed");
			}

            PX4_INFO("RC calibration check failed");
			failed = true;

			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, status_flags.rc_signal_found_once, true, false, status);
			status_flags.rc_calibration_valid = false;

		} else {
			// The calibration is fine, but only set the overall health state to true if the signal is not currently lost
			status_flags.rc_calibration_valid = true;
			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_RCRECEIVER, status_flags.rc_signal_found_once, true,
					 !status.rc_signal_lost, status);
		}
	}

	/* ---- SYSTEM POWER ---- */
	if (status_flags.power_input_valid && !status_flags.circuit_breaker_engaged_power_check) {
		if (!powerCheck(mavlink_log_pub, status, report_failures, prearm)) {
			PX4_INFO("system power check failed");
			failed = true;
		}
	}

	/* ---- Navigation EKF ---- */
	// only check EKF2 data if EKF2 is selected as the estimator and GNSS checking is enabled
	int32_t estimator_type = -1;

	if (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !status.is_vtol) {
		param_get(param_find("SYS_MC_EST_GROUP"), &estimator_type);

	} else {
		// EKF2 is currently the only supported option for FW & VTOL
		estimator_type = 2;
	}

	if (estimator_type == 2) {

		const bool in_grace_period = time_since_boot < 10_s;
		const bool do_report_ekf2_failures = report_failures && (!in_grace_period);
		const bool ekf_healthy = ekf2Check(mavlink_log_pub, status, false, do_report_ekf2_failures) &&
					 ekf2CheckSensorBias(mavlink_log_pub, do_report_ekf2_failures);

		// For the first 10 seconds the ekf2 can be unhealthy, and we just mark it
		// as not present.
		// After that or if we're forced to report, we'll set the flags as is.

		if (!ekf_healthy && !do_report_ekf2_failures) {
			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_AHRS, true, false, false, status);

		} else {
			set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_AHRS, true, true, ekf_healthy, status);
		}


		if (control_mode.flag_control_attitude_enabled
		    || control_mode.flag_control_velocity_enabled
		    || control_mode.flag_control_position_enabled) {
			// healthy estimator only required for dependent control modes
			if (!ekf_healthy && debug_mode) {
			    PX4_INFO("ekf health check failed");
			}
			failed |= !ekf_healthy;
		}
	}

	/* ---- Failure Detector ---- */
	if (!failureDetectorCheck(mavlink_log_pub, status, report_failures, prearm)) {
	    PX4_INFO("failure detected");
		failed = true;
	}

    /* ---- Manual Control ---- */
	{
	    bool manual_control_check_failed = !manualControlCheck(mavlink_log_pub, report_failures);
        if (manual_control_check_failed) {
            PX4_INFO("manual control check failed");
        } else {
            // PX4_INFO("manual control check succeeded");
        }
        failed |= manual_control_check_failed;
	}

	/* ---- Mode ---- */
	{
	    bool mode_check_failed = !modeCheck(mavlink_log_pub, report_failures, status);
        if (mode_check_failed) {
            PX4_INFO("mode check failed");
        } else {
            // PX4_INFO("mode check succeeded");
        }
        failed |= mode_check_failed;
	}

	/* ---- CPU resource ---- */
	{
	    bool cpu_resource_check_failed = !cpuResourceCheck(mavlink_log_pub, report_failures);
        if (cpu_resource_check_failed) {
            PX4_INFO("cpu resource check failed");
        } else {
            // PX4_INFO("cpu resource check succeeded");
        }
        failed |= cpu_resource_check_failed;
	}

	/* ---- Parachute ---- */
	{
	    bool parachute_check_failed = !parachuteCheck(mavlink_log_pub, report_failures, status_flags);
        if (parachute_check_failed) {
            PX4_INFO("parachute check failed");
        } else {
            // PX4_INFO("parachute check succeeded");
        }
        failed |= parachute_check_failed;
	}

	/* Report status */
	return !failed;
}

bool PreFlightCheck::sensorAvailabilityCheck(const bool report_failure,
		const uint8_t nb_mandatory_instances, orb_advert_t *mavlink_log_pub,
		vehicle_status_s &status, sens_check_func_t sens_check)
{
	bool pass_check = true;
	bool report_fail = report_failure;

	/* check all sensors, but fail only for mandatory ones */
	for (uint8_t i = 0u; i < ORB_MULTI_MAX_INSTANCES; i++) {
		const bool is_mandatory = i < nb_mandatory_instances;

		if (!sens_check(mavlink_log_pub, status, i, is_mandatory, report_fail)) {
			pass_check = false;
		}
	}

	return pass_check;
}
