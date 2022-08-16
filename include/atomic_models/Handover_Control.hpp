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
#include <cassert> // Used to check values and stop the simulation
#include <string>

 // Includes the macro DEFINE_ENUM_WITH_STRING_CONVERSIONS
#include "enum_string_conversion.hpp"
#include "Constants.hpp"

#include "message_structures/message_landing_point_t.hpp"
#include "message_structures/message_hover_criteria_t.hpp"

using namespace cadmium;

// Atomic Model
template<typename TIME> class Handover_Control {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(MISSION_STARTED)
		(HOVER)
		(STABILIZING)
		(NOTIFY_PILOT)
		(WAIT_FOR_PILOT)
		(YIELD_CONTROL)
		(PILOT_CONTROL)
	);

    // Input and output port definition
    struct defs {
        struct i_hover_criteria_met : public in_port<bool> {};
        struct i_pilot_handover : public in_port<message_landing_point_t> {};
        struct i_pilot_takeover : public in_port<bool> {};
        struct i_start_mission : public in_port<bool> {};

        struct o_notify_pilot : public out_port<bool> {};
        struct o_control_yielded : public out_port<bool> {};
        struct o_stabilize : public out_port<message_hover_criteria_t> {};
    };

	// Create a tuple of input ports (required for the simulator)
	using input_ports = tuple<
		typename defs::i_hover_criteria_met,
		typename defs::i_pilot_handover,
		typename defs::i_pilot_takeover,
		typename defs::i_start_mission
	>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
		typename defs::o_notify_pilot,
		typename defs::o_control_yielded,
		typename defs::o_stabilize
	>;

	// This is used to track the state of the atomic model.
	// (required for the simulator)
	struct state_type {
		States current_state;
	} state;

	// Public members of the class
	message_landing_point_t hover_location;

	// Default constructor
	Handover_Control() {
		state.current_state = States::IDLE;
		hover_location = message_landing_point_t();
	}

	// Constructor with initial state parameter for debugging or partial execution startup.
	explicit Handover_Control(States initial_state) {
		state.current_state = initial_state;
		hover_location = message_landing_point_t();
	}

	// Internal transitions
	// These are transitions occurring from internal inputs
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
	// These are transitions occurring from external inputs
	// (required for the simulator)
	void external_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {

        bool received_pilot_takeover = !get_messages<typename defs::i_pilot_takeover>(mbs).empty();
        if (received_pilot_takeover && state.current_state != States::WAIT_FOR_PILOT) {
            state.current_state = States::PILOT_CONTROL;
            return;
        }

        bool received_start_mission = !get_messages<typename defs::i_start_mission>(mbs).empty();
        if (received_start_mission) {
            state.current_state = States::MISSION_STARTED;
            return;
        }

        bool received_pilot_handover;
		bool received_hover_crit_met;
		switch (state.current_state) {
			case States::MISSION_STARTED:
				received_pilot_handover = !get_messages<typename defs::i_pilot_handover>(mbs).empty();
				if (received_pilot_handover) {
					// Set the hover location to the newest input (found at the back of the vector of inputs)
					hover_location = get_messages<typename defs::i_pilot_handover>(mbs).back();
					state.current_state = States::HOVER;
				}
				break;
			case States::STABILIZING:
				received_hover_crit_met = !get_messages<typename defs::i_hover_criteria_met>(mbs).empty();
                if (received_hover_crit_met) {
					state.current_state = States::NOTIFY_PILOT;
				}
				break;
			case States::WAIT_FOR_PILOT:
				received_pilot_takeover = !get_messages<typename defs::i_pilot_takeover>(mbs).empty();
				if (received_pilot_takeover) {
					state.current_state = States::YIELD_CONTROL;
				}
				break;
			default:
				break;
		}
	}

	// confluence transition
	// Used to call set call precedent
	void confluence_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {
		bool received_pilot_takeover = !get_messages<typename defs::i_pilot_takeover>(mbs).empty();

		if (received_pilot_takeover) {
			external_transition(TIME(), std::move(mbs));
			internal_transition();
		} else {
			internal_transition();
			external_transition(TIME(), std::move(mbs));
		}
	}

	// output function
	[[nodiscard]] typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		vector<bool> bag_port_out;
		vector<message_hover_criteria_t> bag_port_hover_out;
		message_hover_criteria_t hover_criteria;

		switch (state.current_state) {
			case States::HOVER:
				hover_criteria = message_hover_criteria_t(
					hover_location.lat,
					hover_location.lon,
					hover_location.alt,
					nanf(""),
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
				get_messages<typename defs::o_stabilize>(bags) = bag_port_hover_out;
				break;
			case States::NOTIFY_PILOT:
				bag_port_out.push_back(true);
				get_messages<typename defs::o_notify_pilot>(bags) = bag_port_out;
				break;
			case States::YIELD_CONTROL:
				bag_port_out.push_back(true);
				get_messages<typename defs::o_control_yielded>(bags) = bag_port_out;
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
			case States::MISSION_STARTED:
            case States::STABILIZING:
            case States::WAIT_FOR_PILOT:
            case States::PILOT_CONTROL:
				return numeric_limits<TIME>::infinity();
			case States::HOVER:
			case States::NOTIFY_PILOT:
			case States::YIELD_CONTROL:
				return TIME(TA_ZERO);
			default:
				assert(false && "Unhandled state time advance.");
		}
	}

	friend ostringstream& operator<<(ostringstream& os, const typename Handover_Control<TIME>::state_type& i) {
		os << (string("State: ") + enumToString(i.current_state) + string("\n"));
		return os;
	}
};

#endif // HANDOVER_CTRL_HPP
