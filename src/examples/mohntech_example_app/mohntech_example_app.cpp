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
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/actuator_controls.h>

extern "C" __EXPORT int mohntech_example_app_main(int argc, char *argv[]);

int mohntech_example_app_main(int argc, char *argv[])
{
	PX4_INFO("mohntech_example_app has been started!");

	/* subscribe to vehicle_acceleration topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_acceleration));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);

	/* advertise to actuator_outputs topic */
	struct actuator_controls_s act;
	memset(&act, 0, sizeof(act));
	orb_advert_t act_pub = orb_advertise(ORB_ID(actuator_controls_0), &act);

    for (int i = 0; i < 8; i++) {
        PX4_INFO("Testing motor %i", i);
        act.control[0] = 0.0f;
        act.control[1] = 0.0f;
        act.control[2] = 0.0f;
        act.control[3] = 0.0f;
        act.control[4] = 0.0f;
        act.control[5] = 0.0f;
        act.control[6] = 0.0f;
        act.control[7] = 0.0f;
        for (float j = -1.0f; j < 1.0f; j+=0.1f) {
            act.control[i] = j;
            orb_publish(ORB_ID(actuator_controls_0), act_pub, &act);
            px4_usleep(150000);
        }
        act.control[i] = 0.0f;
    }
	PX4_INFO("Exiting mohntech_example_app!");
	return 0;
}


