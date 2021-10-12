/**
 *	\brief		An atomic model representing the reposition model.
 *	\details	This header file define the reposition model as
				an atomic model for use in the Cadmium DEVS
				simulation software.
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef HAND_HPP
#define HAND_HPP

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

#include <limits> // Used to set the time advance to infinity
#include <assert.h> // Used to check values and stop the simulation
#include <string>

#include "../data_structures/message.hpp"
#include "../data_structures/enum_string_conversion.hpp"

 // Macros for the time advance functions
#define LP_REPOSITION_TIME "00:02:00:000"

using namespace cadmium;
using namespace std;

// Input and output port definition
struct Hand_defs {
	struct hover_criteria_met_in : public in_port<Message_t> {};
	struct pilot_handover_in : public in_port<Message_t> {};
	struct pilot_takeover_in : public out_port<Message_t> {};
};

// Atomic Model
template<typename TIME> class Hand {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(HOVER)
		(STABILISING)
	)

		// Create a tuple of input ports (required for the simulator)
	using input_ports = tuple<
		typename Hand_defs::hover_criteria_met_in,
		typename Hand_defs::pilot_handover_in,
		typename Hand_defs::pilot_takeover_in
	>;

	using output_ports = tuple<>;

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct state_type {
		States current_state;
		TIME next_internal;
	};
	state_type state;

	// Default constructor
	Hand() {
		state.current_state = IDLE;
		state.next_internal = numeric_limits<TIME>::infinity();
	}

	// Internal transitions
	// These are transitions occuring from internal inputs
	// There are no internal transitions
	// (required for the simulator)
	void internal_transition() {

	}

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		bool received_hover_criteria_met;
		bool received_pilot_handover;
		bool received_pilot_takeover;

		switch (state.current_state) {
			case IDLE:
				received_pilot_handover = get_messages<typename Hand_defs::pilot_handover_in>(mbs).size() >= 1;
				received_pilot_takeover = get_messages<typename Hand_defs::pilot_takeover_in>(mbs).size() >= 1;

				if (received_pilot_handover || received_pilot_takeover) {
					state.current_state = STABILISING;
					state.next_internal = numeric_limits<TIME>::infinity();
				}
				break;
			case STABILISING:
				received_hover_criteria_met = get_messages<typename Hand_defs::hover_criteria_met_in>(mbs).size() >= 1;

				if (received_hover_criteria_met) {
					state.current_state = HOVER;
					state.next_internal = numeric_limits<TIME>::infinity();
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
	// Nothing to output
	typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		return bags;
	}

	// Time advance
	// Used to set the internal time of the current state
	TIME time_advance() const {
		return state.next_internal;
	}

	friend ostringstream& operator<<(ostringstream& os, const typename Hand<TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state);
		return os;
	}
};

#endif // HAND_HPP
