
/**
 * @file esc.hpp
 *
 * UAVCAN <--> ORB bridge for ESC messages:
 *     uavcan.equipment.esc.RPMCommand
 *     uavcan.equipment.esc.Status
 *

 */

#pragma once

#include <uavcan/uavcan.hpp>

#include <uavcan/equipment/esc/Status.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/esc_rpm.h>
#include <drivers/drv_hrt.h>
#include <lib/mixer_module/mixer_module.hpp>

class UavcanEscRPM
{
public:
	static constexpr int MAX_ACTUATORS = 8;


	UavcanEscRPM(uavcan::INode &node);
	~UavcanEscRPM() = default;

	int init();

	void update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs);



private:

	void esc_rpm_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status> &msg);



	typedef uavcan::MethodBinder<UavcanEscRPM *,
		void (UavcanEscRPM::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status>&)> StatusCbBinder;

	typedef uavcan::MethodBinder<UavcanEscRPM *,
		void (UavcanEscRPM::*)(const uavcan::TimerEvent &)> TimerCbBinder;

	esc_rpm_s	_esc_rpm{};

	uORB::PublicationMulti<esc_rpm_s> _esc_rpm_pub{ORB_ID(esc_rpm)};


	uavcan::INode								&_node;

	uavcan::Subscriber<uavcan::equipment::esc::Status, StatusCbBinder>	_uavcan_sub_status;

};
