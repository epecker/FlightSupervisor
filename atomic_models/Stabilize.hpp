/**
 *	\brief		An atomic model representing the Stabilize model.
 *	\details	This header file define the Stabilize model as
				an atomic model for use in the Cadmium DEVS
				simulation software.
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef STABILIZE_HPP
#define STABILIZE_HPP

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

#include <limits> // Used to set the time advance to infinity
#include <assert.h> // Used to check values and stop the simulation
#include <string>

#include "../data_structures/message.hpp"
#include "../data_structures/hover_criteria_message.hpp"
#include "../data_structures/enum_string_conversion.hpp"
#include "../include/Constants.hpp"

using namespace cadmium;
using namespace std;

// Input and output port definition
struct Stabilize_defs {
	struct stabilize_in : public in_port<HoverCriteriaMessage_t> {};
	struct hover_criteria_in : public in_port<HoverCriteriaMessage_t> {};
	struct hover_criteria_met_out : public out_port<Message_t> {};
};

// Atomic Model
template<typename TIME> class Stabilize {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(STABILIZING)
		(CRIT_CHECK_FAILED)
		(HOVER)
	)

	// Create a tuple of input ports (required for the simulator)
	using input_ports = tuple<
		typename Stabilize_defs::stabilize_in,
		typename Stabilize_defs::hover_criteria_in
	>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
		typename Stabilize_defs::hover_criteria_met_out
	>;

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct state_type {
		States current_state;
		HoverCriteriaMessage_t hover_criteria; 
	};
	state_type state;

	// Default constructor
	Stabilize() {
		state.current_state = IDLE;
		state.next_internal = numeric_limits<TIME>::infinity();
	}

	// Internal transitions
	// These are transitions occuring from internal inputs
	// There are no internal transitions
	// (required for the simulator)
	void internal_transition() {
		switch (state.current_state) {
			case CRIT_CHECK_FAILED:
				state.current_state = STABILIZING;
				break;
			case STABILIZING:
				state.current_state = HOVER;
				break;
			default:
				break;
		}

	}

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {

		switch (state.current_state) {
			case IDLE:
				if (get_messages<typename Stabilize_defs::stabilize_in>(mbs).size() >= 1) {
					state.hover_criteria = get_messages<typename Stabilize_defs::stabilize_in>(mbs)[0];
					state.current_state = STABILIZING;
				}
				break;
			case STABILIZING:
				if (get_messages<typename Stabilize_defs::hover_criteria_in>(mbs).size() >= 1) {
					state.hover_criteria = get_messages<typename Stabilize_defs::hover_criteria_in>(mbs)[0];
					bool hover_criteria_met = calculate_hover_criteria_met(state.hover_criteria);
					if (!hover_criteria_met) {
						state.current_state = CRIT_CHECK_FAILED;
					}
				}
				break;
			default:
				break;
		}
	}

	// confluence transition
	// Used to call set call precedence
	void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		external_transition(TIME(), move(mbs));
		internal_transition();
	}

	// output function
	// Nothing to output
	typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		vector<Message_t> message_out;
		Message_t success;

		switch (state.current_state) {
			case HOVER:
				message_out.push_back(success);
				get_messages<typename Stabilize_defs::hover_criteria_met_out>(bags) = message_out;
				break;
			default:
				break;
		}

		return bags;
	}

	// Time advance
	// Used to set the internal time of the current state
	TIME time_advance() const {
		TIME next_internal;
		switch (state.cur_state) {
			case IDLE: case HOVER:
				next_internal = numeric_limits<TIME>::infinity();
				break;
			case CRIT_CHECK_FAILED:
				next_internal = TIME("00:00:00:000");
				break;
			case STABILIZING:
				next_internal = calculate_time_from_double_seconds(state.hover_criteria.timeTol);
				break;
			default:
				next_internal = numeric_limits<TIME>::infinity();
		}
		return next_internal;
	}

	friend ostringstream& operator<<(ostringstream& os, const typename Stabilize<TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state);
		return os;
	}

	// Stub implementation for now so we can always hover.
	bool calculate_hover_criteria_met(HoverCriteriaMessage_t hover_criteria_message) {
		return true;
	}

	TIME calculate_time_from_double_seconds(double time) {
		int hours = time / 3600;
		int mins = (time - hours) / 60;
		int secs = (time - hours - mins);
		int millis = (time - hours - mins - secs) * 100;
		return TIME(hours + ":" + mins + ":" + secs + ":" + millis);
	}
};

#endif // STABILIZE_HPP
