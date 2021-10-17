/**
 *	\brief		An atomic model representing the handover control model.
 *	\details	This header file define the handover control model as
				an atomic model for use in the Cadmium DEVS
				simulation software.
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef HANDOVER_CTRL_HPP
#define HANDOVER_CTRL_HPP

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

#include <limits> // Used to set the time advance to infinity
#include <assert.h> // Used to check values and stop the simulation
#include <string>

 // Includes the macro DEFINE_ENUM_WITH_STRING_CONVERSIONS
#include "../include/enum_string_conversion.hpp"

// Data structures that are used in message transport
#include "../data_structures/hover_criteria_message.hpp"

using namespace cadmium;
using namespace std;

// Input and output port definition
struct Handover_Ctrl_defs {
	struct i_lp_crit_met : public in_port<bool> {};
	struct i_pilot_handover : public in_port<bool> {};
	struct i_pilot_takeover : public in_port<bool> {};

	struct o_notify_pilot : public out_port<bool> {};
	struct o_control_yielded : public out_port<bool> {};
};

// Atomic Model
template<typename TIME> class Handover_Ctrl {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(STABILIZING)
		(NOTIFY_PILOT)
		(WAIT_FOR_PILOT)
		(YIELD_CONTROL)
		(PILOT_CONTROL)
	);

	// Create a tuple of input ports (required for the simulator)
	using input_ports = tuple<
		typename Handover_Ctrl_defs::i_lp_crit_met,
		typename Handover_Ctrl_defs::i_pilot_handover,
		typename Handover_Ctrl_defs::i_pilot_takeover
	>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
		typename Handover_Ctrl_defs::o_notify_pilot,
		typename Handover_Ctrl_defs::o_control_yielded
	>;

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct state_type {
		States current_state;
		TIME next_internal;

		Message_t landing_point;
	};

	state_type state;

	// Default constructor
	Handover_Ctrl() {
		state.current_state = IDLE;
		state.next_internal = numeric_limits<TIME>::infinity();
	}

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
	void internal_transition() {
		switch (state.current_state) {
			case NOTIFY_PILOT:
				state.current_state = WAIT_FOR_PILOT;
				state.next_internal = numeric_limits<TIME>::infinity();
				break;
			case YIELD_CONTROL:
				state.current_state = PILOT_CONTROL;
				state.next_internal = numeric_limits<TIME>::infinity();
				break;
			default:
				break;
		}
	}

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		bool received_pilot_handover;
		bool received_pilot_takeover;
		bool received_hover_crit_met;

		switch (state.current_state) {
			case IDLE:
				received_pilot_takeover = get_messages<typename Handover_Ctrl_defs::i_pilot_takeover>(mbs).size() >= 1;
				received_pilot_handover = get_messages<typename Handover_Ctrl_defs::i_pilot_handover>(mbs).size() >= 1;

				if (received_pilot_takeover) {
					state.current_state = PILOT_CONTROL;
					state.next_internal = numeric_limits<TIME>::infinity();
				} else if (received_pilot_handover) {
					state.current_state = STABILIZING;
					state.next_internal = numeric_limits<TIME>::infinity();
				}
				break;
			case STABILIZING:
				received_pilot_takeover = get_messages<typename Handover_Ctrl_defs::i_pilot_takeover>(mbs).size() >= 1;
				received_hover_crit_met = get_messages<typename Handover_Ctrl_defs::i_pilot_handover>(mbs).size() >= 1;

				if (received_pilot_takeover) {
					state.current_state = PILOT_CONTROL;
					state.next_internal = numeric_limits<TIME>::infinity();
				} else if (received_hover_crit_met) {
					state.current_state = NOTIFY_PILOT;
					state.next_internal = TIME("00:00:00:000");
				}
				break;
			case WAIT_FOR_PILOT:
				received_pilot_takeover = get_messages<typename Handover_Ctrl_defs::i_pilot_takeover>(mbs).size() >= 1;

				if (received_pilot_takeover) {
					state.current_state = YIELD_CONTROL;
					state.next_internal = TIME("00:00:00:000");
				}
				break;
			default:
				break;
		}
	}

	// confluence transition
	// Used to call set call precedence
	void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), move(mbs));
	}

	// output function
	typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		vector<bool> bag_port_out;

		switch (state.current_state) {
			case WAIT_FOR_PILOT:
				bag_port_out.push_back(true);
				get_messages<typename Handover_Ctrl_defs::o_notify_pilot>(bags) = bag_port_out;
				break;
			case PILOT_CONTROL:
				bag_port_out.push_back(true);
				get_messages<typename Handover_Ctrl_defs::o_control_yielded>(bags) = bag_port_out;
				break;
			default:
				break;
		}

		return bags;
	}

	// Time advance
	// Used to set the internal time of the current state
	TIME time_advance() const {
		return state.next_internal;
	}

	friend ostringstream& operator<<(ostringstream& os, const typename Handover_Ctrl<TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state);
		return os;
	}
};

#endif // REPO_HPP
