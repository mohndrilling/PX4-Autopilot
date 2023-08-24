
/**
 * @file esc_rpm.cpp
 *
 * @author Kevin Strandenes 
 */

#include "esc_rpm.hpp"
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#define MOTOR_BIT(x) (1<<(x))

using namespace time_literals;

UavcanEscRPM::UavcanEscRPM(uavcan::INode &node) :
	_node(node),
	_uavcan_sub_status(node)
{

}

int
UavcanEscRPM::init()
{
	// ESC status subscription
	int res = _uavcan_sub_status.start(StatusCbBinder(this, &UavcanEscRPM::esc_rpm_sub_cb));

	if (res < 0) {
		PX4_ERR("ESC status sub failed %i", res);
		return res;
	}

	return res;
}



void
UavcanEscRPM::esc_rpm_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status> &msg)
{
	if (msg.esc_index < MAX_ACTUATORS) {
		
        _esc_rpm.timestamp = hrt_absolute_time();
        _esc_rpm.esc_address = msg.getSrcNodeID().get();
        _esc_rpm.esc_rpm         = msg.rpm;


        _esc_rpm_pub.publish(_esc_rpm);
        
        // printf(
		//     "timestamp %f, esc_address  %i, esc_rpm %i\n",
        //     (double) _esc_rpm.timestamp,
		//     (int) _esc_rpm.esc_address,
		//     (int) _esc_rpm.esc_rpm
		// );
        
        
	}
}


