/**
 * @file mohntech_example_app.cpp
 *
 * This file let the hippocampus drive in a circle and prints the orientation as well as the acceleration data.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

// system libraries
#include <parameters/param.h>
#include <systemlib/err.h>
#include <perf/perf_counter.h>

// internal libraries
#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <lib/geo/geo.h>

// Include uORB and the required topics for this app
#include <uORB/uORB.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>

extern "C" __EXPORT int mohntech_example_app_main(int argc, char *argv[]);

int mohntech_example_app_main(int argc, char *argv[])
{
	PX4_INFO("mohntech_example_app has been started!");

    float pwm = atof(argv[1]);

	/* advertise to actuator_outputs topic */
	struct actuator_motors_s act_mot;
	struct actuator_servos_s act_ser;
	memset(&act_mot, 0, sizeof(act_mot));
	memset(&act_ser, 0, sizeof(act_ser));
	orb_advert_t mot_pub = orb_advertise(ORB_ID(actuator_motors), &act_mot);
	orb_advert_t ser_pub = orb_advertise(ORB_ID(actuator_servos), &act_ser);

    PX4_INFO("Testing all motors and servos");
    for (int j = 0; j < 8; j++){
        act_mot.control[j] = pwm;
        act_ser.control[j] = pwm;
    }
    orb_publish(ORB_ID(actuator_motors), mot_pub, &act_mot);
    orb_publish(ORB_ID(actuator_servos), ser_pub, &act_ser);

	PX4_INFO("Exiting mohntech_example_app!");
	return 0;
}


