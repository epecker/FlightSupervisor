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
#include "../../include/enum_string_conversion.hpp"
#include "../../include/Constants.hpp"

using namespace cadmium;
using namespace std;

// Input and output port definition
struct Handover_Control_defs {
	struct i_hover_criteria_met : public in_port<bool> {};
	struct i_pilot_handover : public in_port<message_mavlink_mission_item_t> {};
	struct i_pilot_takeover : public in_port<bool> {};

	struct o_notify_pilot : public out_port<bool> {};
	struct o_control_yielded : public out_port<bool> {};
	struct o_stabilize : public out_port<message_hover_criteria_t> {};
};

// Atomic Model
template<typename TIME> class Handover_Control {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(HOVER)
		(STABILIZING)
		(NOTIFY_PILOT)
		(WAIT_FOR_PILOT)
		(YIELD_CONTROL)
		(PILOT_CONTROL)
	);

	// Create a tuple of input ports (required for the simulator)
	using input_ports = tuple<
		typename Handover_Control_defs::i_hover_criteria_met,
		typename Handover_Control_defs::i_pilot_handover,
		typename Handover_Control_defs::i_pilot_takeover
	>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
		typename Handover_Control_defs::o_notify_pilot,
		typename Handover_Control_defs::o_control_yielded,
		typename Handover_Control_defs::o_stabilize
	>;

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct state_type {
		States current_state;
	};
	state_type state;

	// Public members of the class
	message_mavlink_mission_item_t hover_location;

	// Default constructor
	Handover_Control() {
		state.current_state = States::IDLE;
		hover_location = message_mavlink_mission_item_t();
	}

	// Constructor with initial state parameter for debugging or partial execution startup.
	Handover_Control(States initial_state) {
		state.current_state = initial_state;
		hover_location = message_mavlink_mission_item_t();
	}

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
	void internal_transition() {
		switch (state.current_state) {
			case States::HOVER:
				state.current_state = States::STABILIZING;
				break;
			case States::NOTIFY_PILOT:
				state.current_state = States::WAIT_FOR_PILOT;
				break;
			case States::YIELD_CONTROL:
				state.current_state = States::PILOT_CONTROL;
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
			case States::IDLE:
				received_pilot_takeover = get_messages<typename Handover_Control_defs::i_pilot_takeover>(mbs).size() >= 1;
				received_pilot_handover = get_messages<typename Handover_Control_defs::i_pilot_handover>(mbs).size() >= 1;

				if (received_pilot_takeover) {
					state.current_state = States::PILOT_CONTROL;
				} else if (received_pilot_handover) {
					state.current_state = States::HOVER;
					hover_location = get_messages<typename Handover_Control_defs::i_pilot_handover>(mbs)[0];
				}
				break;
			case States::HOVER:
				received_pilot_takeover = get_messages<typename Handover_Control_defs::i_pilot_takeover>(mbs).size() >= 1;

				if (received_pilot_takeover) {
					state.current_state = States::PILOT_CONTROL;
				}
			case States::STABILIZING:
				received_pilot_takeover = get_messages<typename Handover_Control_defs::i_pilot_takeover>(mbs).size() >= 1;
				received_hover_crit_met = get_messages<typename Handover_Control_defs::i_hover_criteria_met>(mbs).size() >= 1;

				if (received_pilot_takeover) {
					state.current_state = States::PILOT_CONTROL;
				} else if (received_hover_crit_met) {
					state.current_state = States::NOTIFY_PILOT;
				}
				break;
			case States::NOTIFY_PILOT:
				received_pilot_takeover = get_messages<typename Handover_Control_defs::i_pilot_takeover>(mbs).size() >= 1;

				if (received_pilot_takeover) {
					state.current_state = States::PILOT_CONTROL;
				}
				break;
			case States::WAIT_FOR_PILOT:
				received_pilot_takeover = get_messages<typename Handover_Control_defs::i_pilot_takeover>(mbs).size() >= 1;

				if (received_pilot_takeover) {
					state.current_state = States::YIELD_CONTROL;
				}
				break;
			default:
				break;
		}
	}

	// confluence transition
	// Used to call set call precedence
	void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		bool received_pilot_takeover = get_messages<typename Handover_Control_defs::i_pilot_takeover>(mbs).size() >= 1;

		if (received_pilot_takeover) {
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
		vector<message_hover_criteria_t> bag_port_hover_out;
		message_hover_criteria_t hover_criteria;

		switch (state.current_state) {
			case States::HOVER:
				// Need to add a heading member to the mavlink mission item struct.
				hover_criteria = message_hover_criteria_t(
					hover_location.lat,
					hover_location.lon,
					hover_location.alt,
					DEFAULT_LAND_CRITERIA_HDG,
					DEFAULT_LAND_CRITERIA_HOR_DIST,
					DEFAULT_LAND_CRITERIA_VERT_DIST,
					DEFAULT_LAND_CRITERIA_VEL,
					DEFAULT_LAND_CRITERIA_HDG,
					DEFAULT_LAND_CRITERIA_TIME,
					0,
					0,
					0
				);
				bag_port_hover_out.push_back(hover_criteria);
				get_messages<typename Handover_Control_defs::o_stabilize>(bags) = bag_port_hover_out;
				break;
			case States::NOTIFY_PILOT:
				bag_port_out.push_back(true);
				get_messages<typename Handover_Control_defs::o_notify_pilot>(bags) = bag_port_out;
				break;
			case States::YIELD_CONTROL:
				bag_port_out.push_back(true);
				get_messages<typename Handover_Control_defs::o_control_yielded>(bags) = bag_port_out;
				break;
			default:
				break;
		}

		return bags;
	}

	// Time advance
	// Used to set the internal time of the current state
	TIME time_advance() const {
		switch (state.current_state) {
			case States::IDLE:
				return numeric_limits<TIME>::infinity();
				break;
			case States::HOVER:
				return TIME(TA_ZERO);
				break;
			case States::STABILIZING:
				return numeric_limits<TIME>::infinity();
				break;
			case States::NOTIFY_PILOT:
				return TIME(TA_ZERO);
				break;
			case States::WAIT_FOR_PILOT:
				return numeric_limits<TIME>::infinity();
				break;
			case States::YIELD_CONTROL:
				return TIME(TA_ZERO);
				break;
			case States::PILOT_CONTROL:
				return numeric_limits<TIME>::infinity();
				break;
			default:
				assert(false && "Unhandled state time advance.");
				return numeric_limits<TIME>::infinity(); // Used to stop unhandled path warning will not be called because of assert
				break;
		}
	}

	friend ostringstream& operator<<(ostringstream& os, const typename Handover_Control<TIME>::state_type& i) {
		os << (string("State: ") + enumToString(i.current_state) + string("\n"));
		return os;
	}
};

#endif // HANDOVER_CTRL_HPP
