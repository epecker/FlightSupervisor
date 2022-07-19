/**
 *	\brief		An atomic model representing the Handle_Waypoint model.
 *	\details	This header file define the Handle_Waypoint model as
				an atomic model for use in the Cadmium DEVS. It handles
				the on-route behaviour of the aircraft.
				simulation software.
 *	\author		Tanner Trautrim
 */

#ifndef HANDLE_WAYPOINT_HPP
#define HANDLE_WAYPOINT_HPP

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

#include <limits>
#include <string>

#include "message_structures/message_fcc_command_waypoint_t.hpp"
#include "message_structures/message_fcc_command_waypoint_t.hpp"

#include "enum_string_conversion.hpp"
#include "Constants.hpp"

using namespace cadmium;

// Input and output port definitions
struct Handle_Waypoint_defs {
	struct i_start_mission : public out_port<bool> {};
	struct i_waypoint : public out_port<message_fcc_command_waypoint_t> {};

	struct o_fcc_waypoint_update : public out_port<message_fcc_command_waypoint_t> {};
};

template<typename TIME>
class Handle_Waypoint {
public:
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(WAIT_FOR_WAYPOINT)
		(UPDATE_FCC)
	);

	// Create a tuple of input ports (required for the simulator)
	using input_ports = tuple<
			typename Handle_Waypoint_defs::i_start_mission,
			typename Handle_Waypoint_defs::i_waypoint
	>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
			typename Handle_Waypoint_defs::o_fcc_waypoint_update
	>;

	// Tracks the state of the model
	struct state_type {
		States current_state;
	};
	state_type state;

	message_fcc_command_waypoint_t next_waypoint;

	Handle_Waypoint() {
		state.current_state = States::IDLE;
		next_waypoint = message_fcc_command_waypoint_t();
	}

	explicit Handle_Waypoint(States initial_state) {
		state.current_state = initial_state;
		next_waypoint = message_fcc_command_waypoint_t();
	}

	// Internal transitions (required for the simulator)
	void internal_transition() {
		switch (state.current_state) {
			case States::UPDATE_FCC:
				state.current_state = States::WAIT_FOR_WAYPOINT;
				break;
			default:
				return;
		}
	}

	// External transitions (required for the simulator)
	void external_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {
		bool received_start_mission;
		bool received_waypoint;

		switch (state.current_state) {
			case States::IDLE:
				received_start_mission = !get_messages<typename Handle_Waypoint_defs::i_start_mission>(mbs).empty();
				if (received_start_mission) {
					state.current_state = States::WAIT_FOR_WAYPOINT;
				}
				break;
			case States::WAIT_FOR_WAYPOINT:
				received_waypoint = !get_messages<typename Handle_Waypoint_defs::i_waypoint>(mbs).empty();
				if (received_waypoint) {
					vector<message_fcc_command_waypoint_t> new_waypoint = get_messages<typename Handle_Waypoint_defs::i_waypoint>(mbs);
					next_waypoint = new_waypoint[0];
					state.current_state = States::UPDATE_FCC;
				}
				break;
			default:
				break;
		}
	}

	// Confluence transition sets the internal/external precedence
	// Triggered when a message is received at the same time as an internal transition.
	// (required for the simulator)
	void confluence_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), move(mbs));
	}

	// Creates output messages (required for the simulator)
	[[nodiscard]]
	typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		vector<message_fcc_command_waypoint_t> waypoint_out_port;

		if (state.current_state == States::UPDATE_FCC) {
			message_fcc_command_waypoint_t waypoint = next_waypoint;
			waypoint.set_supervisor_status(MAV_COMMAND);

			waypoint_out_port.push_back(next_waypoint);
			get_messages<typename Handle_Waypoint_defs::o_fcc_waypoint_update>(bags) = waypoint_out_port;
		}

		return bags;
	}

	// Time advance sets the wait time of the current state (required for the simulator)
	TIME time_advance() const {
		TIME next_internal;
		switch (state.current_state) {
			case States::IDLE:
			case States::WAIT_FOR_WAYPOINT:
				next_internal = numeric_limits<TIME>::infinity();
				break;
			case States::UPDATE_FCC:
				next_internal = TIME(TA_ZERO);
				break;
			default:
				assert(false && "Unhandled time advance");
				break;
		}
		return next_internal;
	}

	// Used for logging outputs the state's name. (required for the simulator)
	friend ostringstream& operator<<(ostringstream& os, const typename Handle_Waypoint<TIME>::state_type& i) {
		os << (string("State: ") + enumToString(i.current_state) + string("\n"));
		return os;
	}
};

#endif // HANDLE_WAYPOINT_HPP
