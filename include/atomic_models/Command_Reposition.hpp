/**
 *	\brief		An atomic model representing the Command Reposition model.
 *	\details	This header file define the Command Reposition model as
				an atomic model for use in the Cadmium DEVS
				simulation software.
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef COMMAND_REPOSITION_HPP
#define COMMAND_REPOSITION_HPP

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

#include <limits> // Used to set the time advance to infinity
#include <assert.h> // Used to check values and stop the simulation
#include <string>

 // Includes the macro DEFINE_ENUM_WITH_STRING_CONVERSIONS
#include "../../include/enum_string_conversion.hpp"

// Data structures that are used in message transport
#include "../../include/message_structures/message_aircraft_state_t.hpp"
#include "../../include/message_structures/message_hover_criteria_t.hpp"
#include "../../include/message_structures/message_mavlink_mission_item_t.hpp"
#include "../../include/message_structures/message_fcc_command_t.hpp"

// Macros
#include "../../include/Constants.hpp"

using namespace cadmium;
using namespace std;

// Input and output port definitions
struct Command_Reposition_defs {
	struct i_aircraft_state : public in_port<message_aircraft_state_t> {};
	struct i_hover_criteria_met : public in_port<bool> {};
	struct i_pilot_handover : public in_port<bool> {};
	struct i_pilot_takeover : public in_port<bool> {};
	struct i_request_reposition : public in_port<message_mavlink_mission_item_t> {};

	struct o_cancel_hover : public out_port<bool> {};
	struct o_fcc_command_velocity : public out_port<message_fcc_command_t> {};
	struct o_stabilize : public out_port<message_hover_criteria_t> {};
	struct o_lp_criteria_met : public out_port<message_mavlink_mission_item_t> {};
};

// Atomic Model
template<typename TIME> class Command_Reposition {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(GET_STATE)
		(COMMAND_VEL)
		(COMMAND_HOVER)
		(STABILIZING)
		(LP_CRITERIA_MET)
		(LANDING)
		(CANCEL_HOVER)
		(TIMER_EXPIRED)
		(PILOT_CONTROL)
	);

	// Create a tuple of input ports (required for the simulator)
	using input_ports = tuple<
		typename Command_Reposition_defs::i_aircraft_state,
		typename Command_Reposition_defs::i_hover_criteria_met,
		typename Command_Reposition_defs::i_pilot_handover,
		typename Command_Reposition_defs::i_pilot_takeover,
		typename Command_Reposition_defs::i_request_reposition
	>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
		typename Command_Reposition_defs::o_cancel_hover,
		typename Command_Reposition_defs::o_fcc_command_velocity,
		typename Command_Reposition_defs::o_stabilize,
		typename Command_Reposition_defs::o_lp_criteria_met
	>;

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct state_type {
		States current_state;
		TIME next_internal;

		message_mavlink_mission_item_t landing_point;
	};

	state_type state;

	// Default constructor
	Command_Reposition() {
		state.current_state = States::IDLE;
		state.next_internal = numeric_limits<TIME>::infinity();
	}

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
	void internal_transition() {
		switch (state.current_state) {
			case States::COMMAND_VEL:
				state.current_state = States::COMMAND_HOVER;
				state.next_internal = TIME("00:00:00:000");
				break;
			case States::COMMAND_HOVER:
				state.current_state = States::STABILIZING;
				state.next_internal = numeric_limits<TIME>::infinity();
				break;
			case States::LP_CRITERIA_MET:
				state.current_state = States::LANDING;
				state.next_internal = numeric_limits<TIME>::infinity();
				break;
			case States::CANCEL_HOVER:
				state.current_state = States::GET_STATE;
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
		bool received_aircraft_state;
		bool received_hover_criteria_met;
		bool received_pilot_handover;
		bool received_pilot_takeover;
		bool received_request_reposition;

		received_pilot_handover = get_messages<typename Command_Reposition_defs::i_pilot_handover>(mbs).size() >= 1;
		received_pilot_takeover = get_messages<typename Command_Reposition_defs::i_pilot_takeover>(mbs).size() >= 1;

		if (received_pilot_takeover) {
			state.current_state = States::PILOT_CONTROL;
			state.next_internal = numeric_limits<TIME>::infinity();
		} else if (received_pilot_handover) {
			state.current_state = States::TIMER_EXPIRED;
			state.next_internal = numeric_limits<TIME>::infinity();
		} else {
			switch (state.current_state) {
				case States::IDLE:
					received_request_reposition = get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs).size() >= 1;

					if (received_request_reposition) {
						vector<message_mavlink_mission_item_t> new_landing_points = get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs);
						state.landing_point = new_landing_points[0]; // set the new Landing 
						state.current_state = States::GET_STATE;
						state.next_internal = numeric_limits<TIME>::infinity();
					}
					break;
				case States::GET_STATE:
					received_aircraft_state = get_messages<typename Command_Reposition_defs::i_aircraft_state>(mbs).size() >= 1;

					if (received_aircraft_state) {
						state.current_state = States::COMMAND_VEL;
						state.next_internal = TIME("00:00:00:000");
					}
					break;
				case States::COMMAND_VEL:
					received_request_reposition = get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs).size() >= 1;

					if (received_request_reposition) {
						vector<message_mavlink_mission_item_t> new_landing_points = get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs);
						state.landing_point = new_landing_points[0]; // set the new Landing 
						state.current_state = States::GET_STATE;
						state.next_internal = numeric_limits<TIME>::infinity();
					}
					break;
				case States::COMMAND_HOVER:
					received_request_reposition = get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs).size() >= 1;

					if (received_request_reposition) {
						vector<message_mavlink_mission_item_t> new_landing_points = get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs);
						state.landing_point = new_landing_points[0]; // set the new Landing 
						state.current_state = States::GET_STATE;
						state.next_internal = numeric_limits<TIME>::infinity();
					}
					break;
				case States::STABILIZING:
					received_hover_criteria_met = get_messages<typename Command_Reposition_defs::i_hover_criteria_met>(mbs).size() >= 1;
					received_request_reposition = get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs).size() >= 1;

					if (received_request_reposition) {
						vector<message_mavlink_mission_item_t> new_landing_points = get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs);
						state.landing_point = new_landing_points[0]; // set the new Landing 
						state.current_state = States::CANCEL_HOVER;
						state.next_internal = TIME("00:00:00:000");
					} else if (received_hover_criteria_met) {
						state.current_state = States::LP_CRITERIA_MET;
						state.next_internal = TIME("00:00:00:000");
					}
					break;
				case States::LP_CRITERIA_MET:
					received_request_reposition = get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs).size() >= 1;

					if (received_request_reposition) {
						vector<message_mavlink_mission_item_t> new_landing_points = get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs);
						state.landing_point = new_landing_points[0]; // set the new Landing 
						state.current_state = States::CANCEL_HOVER;
						state.next_internal = TIME("00:00:00:000");
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
		bool received_request_reposition = get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs).size() >= 1;

		if (state.current_state == States::LP_CRITERIA_MET && received_request_reposition) {
			external_transition(TIME(), move(mbs));
			internal_transition();
		} else {
			internal_transition();
			external_transition(TIME(), move(mbs));
		}
	}

	// output function
	typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		vector<bool> bag_port_out;
		vector<message_mavlink_mission_item_t> bag_port_LP_out;
		vector<message_fcc_command_t> bag_port_fcc_out;
		vector<message_hover_criteria_t> bag_port_hover_out;

		switch (state.current_state) {
			case States::COMMAND_HOVER:
				bag_port_fcc_out.push_back(message_fcc_command_t());
				get_messages<typename Command_Reposition_defs::o_fcc_command_velocity>(bags) = bag_port_fcc_out;
				break;
			case States::STABILIZING:
				bag_port_hover_out.push_back(message_hover_criteria_t());
				get_messages<typename Command_Reposition_defs::o_stabilize>(bags) = bag_port_hover_out;
				break;
			case States::GET_STATE:
				bag_port_out.push_back(true);
				get_messages<typename Command_Reposition_defs::o_cancel_hover>(bags) = bag_port_out;
				break;
			case States::LANDING:
				bag_port_LP_out.push_back(message_mavlink_mission_item_t());
				get_messages<typename Command_Reposition_defs::o_lp_criteria_met>(bags) = bag_port_LP_out;
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

	friend ostringstream& operator<<(ostringstream& os, const typename Command_Reposition<TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state) << "\tLP: " << i.landing_point;
		return os;
	}
};

#endif // LP_REPOSITION_HPP
