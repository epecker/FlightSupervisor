/**
 *	\brief		An atomic model representing the reposition model.
 *	\details	This header file define the reposition model as
				an atomic model for use in the Cadmium DEVS
				simulation software.
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef LP_REPOSITION_HPP
#define LP_REPOSITION_HPP

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

#include <limits> // Used to set the time advance to infinity
#include <assert.h> // Used to check values and stop the simulation
#include <string>

 // Includes the macro DEFINE_ENUM_WITH_STRING_CONVERSIONS
#include "../include/enum_string_conversion.hpp"

// Data structures that are used in message transport
#include "../data_structures/lp_message.hpp"

// Macros
#define LP_REPOSITION_TIME "00:00:60:000"
#define LAND_OUTPUT true
#define PILOT_HANDOVER true

using namespace cadmium;
using namespace std;

// Input and output port definition
struct LP_Reposition_defs {
	struct i_control_yielded : public in_port<bool> {};
	struct i_lp_crit_met : public in_port<bool> {};
	struct i_lp_new : public in_port<LPMessage_t> {};
	struct i_pilot_takeover : public in_port<bool> {};

	struct o_land : public out_port<bool> {};
	struct o_pilot_handover : public out_port<bool> {};
};

// Atomic Model
template<typename TIME> class LP_Reposition {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(LP_REPO)
		(NEW_LP_REPO)
		(TRIGGER_LAND)
		(HANDOVER_CTRL)
		(LANDING_ROUTINE)
		(PILOT_CONTROL)
	);

	// Create a tuple of input ports (required for the simulator)
	using input_ports = tuple<
		typename LP_Reposition_defs::i_control_yielded,
		typename LP_Reposition_defs::i_lp_crit_met,
		typename LP_Reposition_defs::i_lp_new,
		typename LP_Reposition_defs::i_pilot_takeover
	>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
		typename LP_Reposition_defs::o_land,
		typename LP_Reposition_defs::o_pilot_handover
	>;

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct state_type {
		States current_state;
		TIME next_internal;

		LPMessage_t landing_point;
	};

	state_type state;

	// Default constructor
	LP_Reposition() {
		state.current_state = States::IDLE;
		state.next_internal = numeric_limits<TIME>::infinity();
	}

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
	void internal_transition() {
		switch (state.current_state) {
			case States::NEW_LP_REPO:
				state.current_state = States::LP_REPO;
				state.next_internal = TIME(LP_REPOSITION_TIME);
				break;
			case States::LP_REPO:
				state.current_state = States::PILOT_CONTROL;
				state.next_internal = numeric_limits<TIME>::infinity();
				break;
			case States::TRIGGER_LAND:
				state.current_state = States::LANDING_ROUTINE;
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
		bool received_control_yielded;
		bool received_lp_new;
		bool received_lp_crit_met;
		bool received_pilot_takeover;

		received_pilot_takeover = get_messages<typename LP_Reposition_defs::i_pilot_takeover>(mbs).size() >= 1;

		if (received_pilot_takeover) {
			state.current_state = States::PILOT_CONTROL;
			state.next_internal = numeric_limits<TIME>::infinity();
		} else {
			switch (state.current_state) {
				case States::IDLE:
					received_lp_new = get_messages<typename LP_Reposition_defs::i_lp_new>(mbs).size() >= 1;
					if (received_lp_new) {
						state.current_state = States::LP_REPO;
						state.next_internal = TIME(LP_REPOSITION_TIME);
					}
					break;
				case States::LP_REPO:
					received_lp_new = get_messages<typename LP_Reposition_defs::i_lp_new>(mbs).size() >= 1;
					received_lp_crit_met = get_messages<typename LP_Reposition_defs::i_lp_crit_met>(mbs).size() >= 1;

					if (received_lp_new) {
						vector<LPMessage_t> new_landing_points = get_messages<typename LP_Reposition_defs::i_lp_new>(mbs);
						state.landing_point = new_landing_points[0]; // set the new Landing 
						state.current_state = States::NEW_LP_REPO;
						state.next_internal = TIME("00:00:00:000");
					} else if (received_lp_crit_met) {
						state.current_state = States::TRIGGER_LAND;
						state.next_internal = TIME("00:00:00:000");
					}
					break;
				case States::HANDOVER_CTRL:
					received_control_yielded = get_messages<typename LP_Reposition_defs::i_control_yielded>(mbs).size() >= 1;

					if (received_control_yielded) {
						state.current_state = States::PILOT_CONTROL;
						state.next_internal = numeric_limits<TIME>::infinity();
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
		internal_transition();
		external_transition(TIME(), move(mbs));
	}

	// output function
	typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		vector<bool> bag_port_out;

		switch (state.current_state) {
			case States::LANDING_ROUTINE:
				bag_port_out.push_back(LAND_OUTPUT);
				get_messages<typename LP_Reposition_defs::o_land>(bags) = bag_port_out;
				break;
			case States::HANDOVER_CTRL:
				bag_port_out.push_back(PILOT_HANDOVER);
				get_messages<typename LP_Reposition_defs::o_pilot_handover>(bags) = bag_port_out;
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

	friend ostringstream& operator<<(ostringstream& os, const typename LP_Reposition<TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state) << "\tLP: " << i.landing_point;
		return os;
	}
};

#endif // LP_REPOSITION_HPP
