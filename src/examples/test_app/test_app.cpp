

#include "test_app.hpp"
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#define MOTOR_BIT(x) (1<<(x))

using namespace time_literals;

UavcanEscController::UavcanEscController(uavcan::INode &node) :
	_node(node),
	_uavcan_pub_raw_cmd(node),
	_uavcan_sub_status(node)
{
	_uavcan_pub_raw_cmd.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);
}

int
UavcanEscController::init()
{
	// ESC status subscription
	int res = _uavcan_sub_status.start(StatusCbBinder(this, &UavcanEscController::esc_status_sub_cb));

	if (res < 0) {
		PX4_ERR("ESC status sub failed %i", res);
		return res;
	}

	return res;
}

void
UavcanEscController::update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
{
	/*
	 * Rate limiting - we don't want to congest the bus
	 */
	const auto timestamp = _node.getMonotonicTime();

	if ((timestamp - _prev_cmd_pub).toUSec() < (1000000 / MAX_RATE_HZ)) {
		return;
	}

	_prev_cmd_pub = timestamp;

	/*
	 * Fill the command message
	 * If unarmed, we publish an empty message anyway
	 */
	uavcan::equipment::esc::RawCommand msg;
	for (unsigned i = 0; i < num_outputs; i++) {
		if (stop_motors || outputs[i] == DISARMED_OUTPUT_VALUE) {
		    // TODO mohntech: msg.cmd.push_back(static_cast<unsigned>(0));
			msg.cmd.push_back(static_cast<unsigned>(4096));

		} else {
			msg.cmd.push_back(static_cast<int>(outputs[i]));
		}
	}

	/*
	 * Remove channels that are always zero.
	 * The objective of this optimization is to avoid broadcasting multi-frame transfers when a single frame
	 * transfer would be enough. This is a valid optimization as the UAVCAN specification implies that all
	 * non-specified ESC setpoints should be considered zero.
	 * The positive outcome is a (marginally) lower bus traffic and lower CPU load.
	 *
	 * From the standpoint of the PX4 architecture, however, this is a hack. It should be investigated why
	 * the mixer returns more outputs than are actually used.
	 */
	for (int index = int(msg.cmd.size()) - 1; index >= _max_number_of_nonzero_outputs; index--) {
		if (msg.cmd[index] != 0) {
			_max_number_of_nonzero_outputs = index + 1;
			break;
		}
	}

	msg.cmd.resize(_max_number_of_nonzero_outputs);

	/*
	 * Publish the command message to the bus
	 * Note that for a quadrotor it takes one CAN frame
	 */
	 /*
    PX4_INFO(
        "output: %i, %i, %i, %i, %i, %i, %i, %i",
        msg.cmd[0],
        msg.cmd[1],
        msg.cmd[2],
        msg.cmd[3],
        msg.cmd[4],
        msg.cmd[5],
        msg.cmd[6],
        msg.cmd[7]
    );
    */
	_uavcan_pub_raw_cmd.broadcast(msg);
}

void
UavcanEscController::set_rotor_count(uint8_t count)
{
	_rotor_count = count;
}

void
UavcanEscController::esc_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status> &msg)
{
	if (msg.esc_index < esc_status_s::CONNECTED_ESC_MAX) {
		auto &ref = _esc_status.esc[msg.esc_index];

		// ref.timestamp       = hrt_absolute_time();
		// ref.esc_address = msg.getSrcNodeID().get();
		// ref.esc_voltage     = msg.voltage;
		// ref.esc_current     = msg.current;
		// ref.esc_temperature = msg.temperature;
		// ref.esc_rpm         = msg.rpm;
		// ref.esc_errorcount  = msg.error_count;

		// _esc_status.esc_count = _rotor_count;
		// _esc_status.counter += 1;
		// _esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
		// _esc_status.esc_online_flags = check_escs_status();
		// _esc_status.esc_armed_flags = (1 << _rotor_count) - 1;
		// _esc_status.timestamp = hrt_absolute_time();
		// _esc_status_pub.publish(_esc_status);
		
		printf(
		    "timestamp %f, esc_address %i, esc_index %i, esc_voltage %f, esc_current %f, esc_temperature %f, esc_rpm %i, power_rating_pct %i, esc_errorcount %i\n",
		    (double) ref.timestamp,
		    (int) ref.esc_address,
		    (int) msg.esc_index,
		    (double) ref.esc_voltage,
		    (double) ref.esc_current,
		    (double) ref.esc_temperature,
		    (int) ref.esc_rpm,
		    (int) msg.power_rating_pct,
		    (int) ref.esc_errorcount
		);
		
		/*
		printf(
		    "timestamp %f, esc_address %i, esc_voltage %f, esc_current %f, esc_temperature %f, esc_rpm %i, power_rating_pct %i\n",
		    (double) ref.timestamp,
		    (int) ref.esc_address,
		    (double) ref.esc_voltage,
		    (double) ref.esc_current,
		    (double) ref.esc_temperature,
		    (int) ref.esc_rpm,
		    (int) msg.power_rating_pct
		);
		*/
	}
}

uint8_t
UavcanEscController::check_escs_status()
{
	int esc_status_flags = 0;
	const hrt_abstime now = hrt_absolute_time();

	for (int index = 0; index < esc_status_s::CONNECTED_ESC_MAX; index++) {

		if (_esc_status.esc[index].timestamp > 0 && now - _esc_status.esc[index].timestamp < 1200_ms) {
			esc_status_flags |= (1 << index);
		}

	}

	return esc_status_flags;
}
