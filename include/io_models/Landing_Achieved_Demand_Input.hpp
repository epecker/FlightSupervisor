/**
 *	\brief		An atomic model for receiving input on demand into the simulation from shared memory.
 *	\details	This header file defines a polling shared memory input atomic model that is activated upon
				a signal receipt for use in the RT-Cadmium DEVS simulation software.
 *	\author		James Horner
 */

#ifndef LANDING_ACHIEVED_DEMAND_INPUT_HPP
#define LANDING_ACHIEVED_DEMAND_INPUT_HPP

 // System libraries
#include <iostream>
#include <assert.h>
#include <string>

// RT-Cadmium
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <cadmium/modeling/dynamic_model.hpp>

// Shared Memory Model
#include "sharedmemorymodel/SharedMemoryModel.h"

// Message Structures

// Includes
#include "enum_string_conversion.hpp"
#include "Constants.hpp"

#ifdef RT_LINUX

using namespace cadmium;
using namespace std;

// Input and output port definitions
struct Landing_Achieved_Demand_Input_defs {
	struct o_message : public out_port<bool> { };
	struct i_start : public in_port<bool> { };
};

// Atomic model
template<typename TIME>
class Landing_Achieved_Demand_Input {

	// Private members.
private:
	TIME polling_rate;
	SharedMemoryModel model;
	float landing_height_ft;

public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(POLL)
	);

	// Default constructor
	Landing_Achieved_Demand_Input() {
		//Initialise the current state
		state.current_state = States::IDLE;

		//Set the rate at which the shared memory segement will be polled.
		polling_rate = TIME("00:00:00:100");
		landing_height_ft = DEFAULT_LAND_CRITERIA_VERT_DIST;

		model = SharedMemoryModel();
		model.connectSharedMem();
		if (!model.isConnected()) {
			throw(std::runtime_error("Could not connect to shared memory."));
		}
	}

	// Constructor with polling rate and name parameters
	Landing_Achieved_Demand_Input(TIME rate, float landing_height_ft) : polling_rate(rate), landing_height_ft(landing_height_ft) {
		//Initialise the current state
		state.current_state = States::IDLE;

		model = SharedMemoryModel();
		model.connectSharedMem();
		if (!model.isConnected()) {
			throw(std::runtime_error("Could not connect to shared memory."));
		}
	}

	// Destructor for disconnecting from shared memory.
	~Landing_Achieved_Demand_Input() {
		model.disconnectSharedMem();
	}

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct state_type {
		States current_state;
	};
	state_type state;

	// Create a tuple of input ports (required for the simulator)
	using input_ports = std::tuple<typename Landing_Achieved_Demand_Input_defs::i_start>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = std::tuple<typename Landing_Achieved_Demand_Input_defs::o_message>;

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
	void internal_transition() {
		if (state.current_state == States::POLL && model.sharedMemoryStruct->hg1700.mixedhgt < landing_height_ft) {
			state.current_state = States::IDLE;
		}
	}

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		if (get_messages<typename Landing_Achieved_Demand_Input_defs::i_start>(mbs).size() >= 1) {
			state.current_state = States::POLL;
		}
	}

	// Confluence transition
	// Used to call set call precedence
	void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), std::move(mbs));
	}

	// Output function
	typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		vector<bool> bag_port_message;
		if (state.current_state == States::POLL && model.sharedMemoryStruct->hg1700.mixedhgt < landing_height_ft) {
			bag_port_message.push_back(true);
			get_messages<typename Landing_Achieved_Demand_Input_defs::o_message>(bags) = bag_port_message;
		}
		return bags;
	}

	// Time advance
	// Used to set the internal time of the current state
	TIME time_advance() const {
		switch (state.current_state) {
			case States::IDLE:
				return std::numeric_limits<TIME>::infinity();
			case States::POLL:
				return polling_rate;
			default:
				return TIME(TA_ZERO);
		}
	}

	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Landing_Achieved_Demand_Input<TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state);
		return os;
	}
};

#endif /* RT_LINUX */
#endif /* LANDING_ACHIEVED_DEMAND_INPUT_HPP */
