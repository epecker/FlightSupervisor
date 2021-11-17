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

#include "../data_structures/hover_criteria_message.hpp"
#include "../data_structures/aircraft_state_message.hpp"
#include "../data_structures/lp_message.hpp"
#include "../data_structures/fcc_command.hpp"
#include "../include/enum_string_conversion.hpp"
#include "../include/Constants.hpp"

using namespace cadmium;
using namespace std;

// Input and output port definition
struct Stabilize_defs {
	struct i_aircraft_state : public in_port<AircraftStateMessage_t> {};
	struct i_stabilize : public in_port<HoverCriteriaMessage_t> {};
	struct i_cancel_hover : public in_port<bool> {};

	struct o_fcc_command_hover : public out_port<FccCommandMessage_t> {};
	struct o_hover_criteria_met : public out_port<bool> {};
};

// Atomic Model
template<typename TIME> class Stabilize {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(INIT_HOVER)
		(STABILIZING)
		(CRIT_CHECK_FAILED)
		(HOVER)
	)

	// Create a tuple of input ports (required for the simulator)
	using input_ports = tuple<
		typename Stabilize_defs::i_aircraft_state,
		typename Stabilize_defs::i_stabilize,
		typename Stabilize_defs::i_cancel_hover
	>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
		typename Stabilize_defs::o_fcc_command_hover,
		typename Stabilize_defs::o_hover_criteria_met
	>;

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct state_type {
		States current_state;
		HoverCriteriaMessage_t hover_criteria;
		AircraftStateMessage_t aircraft_state;
	};
	state_type state;

	// Default constructor
	Stabilize() {
		state.current_state = States::IDLE;
		state.hover_criteria = HoverCriteriaMessage_t();
		state.aircraft_state = AircraftStateMessage_t();
	}

	// Internal transitions
	// These are transitions occuring from internal inputs
	// There are no internal transitions
	// (required for the simulator)
	void internal_transition() {
		switch (state.current_state) {
			case States::INIT_HOVER:
				state.current_state = States::STABILIZING;
				break;
			case States::CRIT_CHECK_FAILED:
				state.current_state = States::STABILIZING;
				break;
			case States::STABILIZING:
				state.current_state = States::HOVER;
				break;
			default:
				break;
		}

	}

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		bool received_cancel_hover = get_messages<typename Stabilize_defs::i_cancel_hover>(mbs).size() >= 1;

		if (received_cancel_hover) {
			state.current_state = States::IDLE;
			state.aircraft_state = AircraftStateMessage_t();
			state.hover_criteria = HoverCriteriaMessage_t();
		} else {
			switch (state.current_state) {
				case States::IDLE:
					if (get_messages<typename Stabilize_defs::i_stabilize>(mbs).size() >= 1) {
						state.current_state = States::INIT_HOVER;
						state.hover_criteria = get_messages<typename Stabilize_defs::i_stabilize>(mbs)[0];
					}
					break;
				case States::STABILIZING:
					if (get_messages<typename Stabilize_defs::i_aircraft_state>(mbs).size() >= 1) {
						state.aircraft_state = get_messages<typename Stabilize_defs::i_aircraft_state>(mbs)[0];
						if (!calculate_hover_criteria_met(state.aircraft_state)) {
							state.current_state = States::CRIT_CHECK_FAILED;
						}
					}
					break;
				default:
					break;
			}
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
		vector<bool> message_out;
		vector<FccCommandMessage_t> message_fcc_out;

		switch (state.current_state) {
			case States::INIT_HOVER:
				message_fcc_out.push_back(FccCommandMessage_t());
				get_messages<typename Stabilize_defs::o_fcc_command_hover>(bags) = message_fcc_out;
				break;
			case States::STABILIZING:
				message_out.push_back(true);
				get_messages<typename Stabilize_defs::o_hover_criteria_met>(bags) = message_out;
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
		switch (state.current_state) {
			case States::IDLE: case States::HOVER:
				next_internal = numeric_limits<TIME>::infinity();
				break;
			case States::CRIT_CHECK_FAILED:
				next_internal = TIME("00:00:00:000");
				break;
			case States::STABILIZING:
				next_internal = calculate_time_from_double_seconds(state.hover_criteria.timeTol);
				break;
			case States::INIT_HOVER:
				next_internal = TIME("00:00:00:000");
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
	bool calculate_hover_criteria_met(AircraftStateMessage_t state) {
		return true;
	}

	static TIME calculate_time_from_double_seconds(double time) {
		int hours = time / 3600;
		int mins = (time - hours) / 60;
		int secs = (time - hours - mins);
		int millis = (time - hours - mins - secs) * 100;
		return TIME(to_string(hours) + ":" + to_string(mins) + ":" + to_string(secs) + ":" + to_string(millis));
	}
};

#endif // STABILIZE_HPP
