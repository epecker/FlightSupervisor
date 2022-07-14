/**
 *	\brief		An atomic model for receiving input into the simulation from shared memory.
 *	\details	This header file defines a polling shared memory input atomic model for use
 				in the RT-Cadmium DEVS simulation software.
 *	\author		James Horner
 */

#ifndef AIRCRAFT_STATE_INPUT_HPP
#define AIRCRAFT_STATE_INPUT_HPP

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
#include "message_structures/message_aircraft_state_t.hpp"

// Includes
#include "enum_string_conversion.hpp"
#include "Constants.hpp"

#ifdef RT_LINUX

using namespace cadmium;
using namespace std;

// Input and output port definitions
struct Aircraft_State_Input_defs {
	struct o_message : public out_port<message_aircraft_state_t> { };
	struct i_request : public in_port<bool> { };
};

// Atomic model
template<typename TIME>
class Aircraft_State_Input {

	// Private members.
private:
	SharedMemoryModel model;
	
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(SEND)
	);

	// Default constructor
	Aircraft_State_Input() {
		//Initialise the current state
		state.current_state = States::IDLE;
		
		model = SharedMemoryModel();
		model.connectSharedMem();
		if (!model.isConnected()) {
			throw(std::runtime_error("Could not connect to shared memory."));
		}
	}

	// Destructor for disconnecting from shared memory.
	~Aircraft_State_Input() {
		model.disconnectSharedMem();
	}

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct state_type {
		States current_state;
	};
	state_type state;

	// Create a tuple of input ports (required for the simulator)
	using input_ports = std::tuple<typename Aircraft_State_Input_defs::i_request>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = std::tuple<typename Aircraft_State_Input_defs::o_message>;

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
	void internal_transition() {
		switch (state.current_state)
		{
			case States::SEND:
				state.current_state = States::IDLE;
				break;

			default:
				break;
		}
	}

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		if (get_messages<typename Aircraft_State_Input_defs::i_request>(mbs).size() >= 1) {
			state.current_state = States::SEND;
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
		vector<message_aircraft_state_t> bag_port_message;

		if (state.current_state == States::SEND) {
			message_aircraft_state_t message = message_aircraft_state_t(
				model.sharedMemoryStruct->hg1700.lat,
				model.sharedMemoryStruct->hg1700.lng,
				model.sharedMemoryStruct->hg1700.mixedhgt,
				model.sharedMemoryStruct->hg1700.alt,
				model.sharedMemoryStruct->hg1700.hdg,
				sqrt(pow(model.sharedMemoryStruct->hg1700.ve, 2) + pow(model.sharedMemoryStruct->hg1700.vn, 2))
			);
			bag_port_message.push_back(message);
			get_messages<typename Aircraft_State_Input_defs::o_message>(bags) = bag_port_message;
		}
		return bags;
	}

	// Time advance
	// Used to set the internal time of the current state
	TIME time_advance() const {
		switch (state.current_state) {
			case States::IDLE:
				return std::numeric_limits<TIME>::infinity();
			case States::SEND:
				return TIME(TA_ZERO);
			default:
				return TIME(TA_ZERO);
		}
	}

	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Aircraft_State_Input<TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state);
		return os;
	}
};

#endif /* RT_LINUX */
#endif /* AIRCRAFT_STATE_INPUT_HPP */
