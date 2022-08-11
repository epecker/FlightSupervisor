/**
 *	\brief		An atomic model for receiving input on demand into the simulation based on a condition.
 *	\details	This header file defines a polling conditional input atomic model that is activated upon
				a condition becoming true for use in the RT-Cadmium DEVS simulation software.
 *	\author		James Horner
 */

#ifndef POLLING_CONDITION_INPUT_HPP
#define POLLING_CONDITION_INPUT_HPP

 // System libraries
#include <iostream>
#include <assert.h>
#include <string>
#include <vector>

// RT-Cadmium
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <cadmium/modeling/dynamic_model.hpp>

// Shared Memory Model
#include "sharedmemorymodel/SharedMemoryModel.h"

// Includes
#include "enum_string_conversion.hpp"
#include "Constants.hpp"

using namespace cadmium;

// Atomic model
template<typename START_TYPE, typename QUIT_TYPE, typename TIME>
class Polling_Condition_Input {

	// Private members.
private:
	TIME polling_rate;

protected:
	virtual bool setup() {return false;};
	virtual bool check_condition() {return false;};

public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(POLL)
	);

	// Input and output port definitions
	struct defs {
		struct o_message : public out_port<bool> { };
		struct i_start : public in_port<START_TYPE> { };
		struct i_quit : public in_port<QUIT_TYPE> { };
	};

	// Default constructor
	Polling_Condition_Input() {
		//Initialise the current state
		state.current_state = States::IDLE;

		//Set the rate at which the shared memory segement will be polled.
		polling_rate = TIME("00:00:00:100");
	}

	// Constructor with polling rate and name parameters
	Polling_Condition_Input(TIME rate) : polling_rate(rate) {
		//Initialise the current state
		state.current_state = States::IDLE;
	}

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct state_type {
		States current_state;
		bool condition_met;
	};
	state_type state;

	// Create a tuple of input ports (required for the simulator)
	using input_ports = std::tuple<
		typename Polling_Condition_Input::defs::i_start,
		typename Polling_Condition_Input::defs::i_quit
	>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = std::tuple<typename Polling_Condition_Input::defs::o_message>;

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
	void internal_transition() {
		if (state.condition_met) {
			state.current_state = States::IDLE;
			state.condition_met = false;
		}
		else if (state.current_state == States::POLL && check_condition()) {
			state.condition_met = true;
		}
	}

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		if (get_messages<typename Polling_Condition_Input::defs::i_quit>(mbs).size() >= 1) {
			state.current_state = States::IDLE;
		}
		else if (get_messages<typename Polling_Condition_Input::defs::i_start>(mbs).size() >= 1) {
			state.current_state = States::POLL;
		}
	}

	// Confluence transition
	// Used to call set call precedence
	void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		external_transition(TIME(), std::move(mbs));
		internal_transition();
	}

	// Output function
	typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		std::vector<bool> bag_port_message;
		if (state.current_state == States::POLL && state.condition_met) {
			bag_port_message.push_back(true);
			get_messages<typename Polling_Condition_Input::defs::o_message>(bags) = bag_port_message;
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
				if (state.condition_met) {
					return TIME(TA_ZERO);				
				}
				else {
					return polling_rate;
				}
			default:
				return TIME(TA_ZERO);
		}
	}

	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Polling_Condition_Input<START_TYPE, QUIT_TYPE, TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state) << "-" << (i.condition_met ? "MET" : "NOT_MET");
		return os;
	}
};

/**
 * \brief   Polling Condition Input for testing. 
 * \author	James Horner
 */
template<typename TIME>
class Polling_Condition_Input_Test : public Polling_Condition_Input<bool, bool, TIME> {
public:
    Polling_Condition_Input_Test() = default;
    Polling_Condition_Input_Test(TIME rate) : Polling_Condition_Input<bool, bool, TIME>(rate) {
		if (!setup()) {
			throw(std::runtime_error("Could not set up polling condition input test."));
		}
	}

	bool setup() {
		number_polls = 0;
		return true;
	}

	bool check_condition() {
		number_polls++;
		return (number_polls == 10);
	}

private:
	int number_polls;
};

/**
 * \brief   Polling Condition Input for Landing Achieved. 
 * \author	James Horner
 */
template<typename TIME>
class Polling_Condition_Input_Landing_Achieved : public Polling_Condition_Input<message_fcc_command_t, bool, TIME> {
public:
    Polling_Condition_Input_Landing_Achieved() = default;
    Polling_Condition_Input_Landing_Achieved(TIME rate, float landing_height_ft) : 
		Polling_Condition_Input<message_fcc_command_t, bool, TIME>(rate),
		landing_height_ft(landing_height_ft) {
		if (!setup()) {
			throw(std::runtime_error("Could not set up polling condition input for landing achieved."));
		}
	}

	~Polling_Condition_Input_Landing_Achieved() {
		model.disconnectSharedMem();
	}

	bool setup() {
		model = SharedMemoryModel();
		model.connectSharedMem();
		return model.isConnected();
	}

	bool check_condition() {
		return (model.sharedMemoryStruct->hg1700.mixedhgt < landing_height_ft);
	}

private:
	SharedMemoryModel model;
	float landing_height_ft;
};

/**
 * \brief   Polling Condition Input for Pilot Takeover. 
 * \author	James Horner
 */
template<typename TIME>
class Polling_Condition_Input_Pilot_Takeover : public Polling_Condition_Input<message_start_supervisor_t, bool, TIME> {
public:
    Polling_Condition_Input_Pilot_Takeover() = default;
    Polling_Condition_Input_Pilot_Takeover(TIME rate) : 
		Polling_Condition_Input<message_start_supervisor_t, bool, TIME>(rate) {
		if (!setup()) {
			throw(std::runtime_error("Could not set up polling condition input for pilot takeover."));
		}
	}

	~Polling_Condition_Input_Pilot_Takeover() {
		model.disconnectSharedMem();
	}

	bool setup() {
		engaged = ((1 << 0) | (1 << 1));
		model = SharedMemoryModel();
		model.connectSharedMem();
		return model.isConnected();
	}

	bool check_condition() {
		// Store the status bits
		uint32_t status = model.sharedMemoryStruct->hmu_safety.safety_status;
		// Clear all but the FCC engaged bits
		status &= engaged;
		// If both FCC engaged bits are set, return true 
		return (status != engaged);
	}

private:
	SharedMemoryModel model;
	uint32_t engaged;
};

#endif /* POLLING_CONDITION_INPUT_HPP */
