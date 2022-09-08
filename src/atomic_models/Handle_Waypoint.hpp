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

#include "cadmium/modeling/ports.hpp"
#include "cadmium/modeling/message_bag.hpp"

#include <limits>
#include <string>

#include "../message_structures/message_fcc_command_t.hpp"

#include "../enum_string_conversion.hpp"
#include "../Constants.hpp"

// Input and output port definitions
struct Handle_Waypoint_defs {
	struct i_pilot_takeover : public cadmium::out_port<bool> {};
	struct i_start_mission : public cadmium::out_port<int> {};
	struct i_waypoint : public cadmium::out_port<message_fcc_command_t> {};

	struct o_fcc_waypoint_update : public cadmium::out_port<message_fcc_command_t> {};
};

template<typename TIME>
class Handle_Waypoint {
public:
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(WAIT_FOR_WAYPOINT)
		(PILOT_TAKEOVER)
		(UPDATE_FCC)
	);

	// Create a tuple of input ports (required for the simulator)
	using input_ports = std::tuple<
			typename Handle_Waypoint_defs::i_pilot_takeover,
			typename Handle_Waypoint_defs::i_start_mission,
			typename Handle_Waypoint_defs::i_waypoint
	>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = std::tuple<
			typename Handle_Waypoint_defs::o_fcc_waypoint_update
	>;

	// Tracks the state of the model
	struct state_type {
		States current_state;
	} state;



	Handle_Waypoint() {
		state.current_state = States::IDLE;
	}

	explicit Handle_Waypoint(States initial_state) {
		state.current_state = initial_state;
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
	void external_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		bool received_pilot_takeover;
		bool received_start_mission;
		bool received_waypoint;

		received_pilot_takeover = !cadmium::get_messages<typename Handle_Waypoint_defs::i_pilot_takeover>(mbs).empty();
		if (received_pilot_takeover) {
			state.current_state = States::PILOT_TAKEOVER;
			return;
		}

		switch (state.current_state) {
			case States::IDLE:
				received_start_mission = !cadmium::get_messages<typename Handle_Waypoint_defs::i_start_mission>(mbs).empty();
				if (received_start_mission) {
					state.current_state = States::WAIT_FOR_WAYPOINT;
				}
				break;
			case States::WAIT_FOR_WAYPOINT:
				received_waypoint = !cadmium::get_messages<typename Handle_Waypoint_defs::i_waypoint>(mbs).empty();
				if (received_waypoint) {
					next_waypoint = cadmium::get_messages<typename Handle_Waypoint_defs::i_waypoint>(mbs);
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
	void confluence_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), std::move(mbs));
	}

	// Creates output messages (required for the simulator)
	[[nodiscard]]
	typename cadmium::make_message_bags<output_ports>::type output() const {
		typename cadmium::make_message_bags<output_ports>::type bags;

		if (state.current_state == States::UPDATE_FCC) {
            for (message_fcc_command_t & waypoint : next_waypoint) {
                waypoint.set_supervisor_status(Control_Mode_E::MAV_COMMAND);
            }
			cadmium::get_messages<typename Handle_Waypoint_defs::o_fcc_waypoint_update>(bags) = next_waypoint;
		}

		return bags;
	}

	// Time advance sets the wait time of the current state (required for the simulator)
	TIME time_advance() const {
		TIME next_internal;
		switch (state.current_state) {
			case States::IDLE:
			case States::WAIT_FOR_WAYPOINT:
			case States::PILOT_TAKEOVER:
				next_internal = std::numeric_limits<TIME>::infinity();
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
	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Handle_Waypoint<TIME>::state_type& i) {
		os << (std::string("State: ") + enumToString(i.current_state) + std::string("\n"));
		return os;
	}

private:
    mutable std::vector<message_fcc_command_t> next_waypoint;
};

#endif // HANDLE_WAYPOINT_HPP
