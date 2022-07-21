/**
 *	\brief		An atomic model for retrieving the aircraft's gps time from shared memory.
 *	\details	This header file defines an atomic model for retrieving gps times from shared
 				memory for use in the RT-Cadmium DEVS simulation software.
 *	\author		Tanner Trautrim
 */

#ifdef RT_LINUX
#ifndef AIRCRAFT_STATE_INPUT_HPP
#define AIRCRAFT_STATE_INPUT_HPP

// System libraries
#include <iostream>
#include <cassert>
#include <string>

// RT-Cadmium
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

#include "sharedmemorymodel/SharedMemoryModel.h"
#include "enum_string_conversion.hpp"
#include "Constants.hpp"

using namespace cadmium;

// Input and output port definitions
struct Aircraft_State_Input_defs {
	struct o_message : public out_port<double> { };
	struct i_request : public in_port<bool> { };
};

// Atomic model
template<typename TIME>
class Aircraft_State_Input {
public:
	// Creates an enum of states and a function to convert from enum to a string
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(SEND)
	);

	struct state_type {
		States current_state;
	} state;

	Aircraft_State_Input() {
		state.current_state = States::IDLE;
		
		model = SharedMemoryModel();
		model.connectSharedMem();
		if (!model.isConnected()) {
			throw(std::runtime_error("Could not connect to shared memory."));
		}
	}

	~Aircraft_State_Input() {
		model.disconnectSharedMem();
	}

	// Create a tuple of input ports (required for the simulator)
	using input_ports = std::tuple<typename Aircraft_State_Input_defs::i_request>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = std::tuple<typename Aircraft_State_Input_defs::o_message>;

	// Internal transitions (required for the simulator)
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

	// External transitions (required for the simulator)
	void external_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {
		bool received_request;

		if (state.current_state == States::IDLE) {
			received_request = !get_messages<typename Aircraft_State_Input_defs::i_request>(mbs).empty();
			if (received_request) {
				state.current_state = States::SEND;
			}
		}
	}

	// Confluence transition sets the internal/external precedence
	// Triggered when a message is received at the same time as an internal transition.
	// (required for the simulator)
	void confluence_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), std::move(mbs));
	}

	// Creates output messages (required for the simulator)
	[[nodiscard]] typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		vector<double> bag_port_message;

		if (state.current_state == States::SEND) {
			bag_port_message.push_back(model.sharedMemoryStruct->hg1700.time);
			get_messages<typename Aircraft_State_Input_defs::o_message>(bags) = bag_port_message;
		}
		return bags;
	}

	// Time advance sets the wait time of the current state (required for the simulator)
	TIME time_advance() const {
		switch (state.current_state) {
			case States::IDLE:
				return std::numeric_limits<TIME>::infinity();
			default:
				return TIME(TA_ZERO);
		}
	}

	// Used for logging outputs the state's name. (required for the simulator)
	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Aircraft_State_Input<TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state);
		return os;
	}

private:
	SharedMemoryModel model;
};

#endif // AIRCRAFT_STATE_INPUT_HPP
#endif // RT_LINUX
