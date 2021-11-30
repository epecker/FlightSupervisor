/**
 *	\brief		An atomic model representing the landing routine model.
 *	\details	This header file define the handover control model as
				an atomic model for use in the Cadmium DEVS
				simulation software.
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef LANDING_ROUTING_HPP
#define LANDING_ROUTING_HPP

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
struct Landing_Routine_defs {
	struct i_landing_achieved : public in_port<bool> {};
	struct i_pilot_takeover : public in_port<bool> {};
	struct i_land : public in_port<bool> {};

	struct o_land_requested : public out_port<bool> {};
	struct o_mission_complete : public out_port<bool> {};
};

// Atomic Model
template<typename TIME> class Landing_Routine {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(REQUEST_LAND)
		(LANDING)
		(NOTIFY_LANDED)
		(LANDED)
		(PILOT_CONTROL)
	);

	// Create a tuple of input ports (required for the simulator)
	using input_ports = tuple<
		typename Landing_Routine_defs::i_landing_achieved,
		typename Landing_Routine_defs::i_pilot_takeover,
		typename Landing_Routine_defs::i_land
	>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
		typename Landing_Routine_defs::o_land_requested,
		typename Landing_Routine_defs::o_mission_complete
	>;

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct state_type {
		States current_state;
	};
	state_type state;

	// Default constructor
	Landing_Routine() {
		state.current_state = States::IDLE;
	}

	// Constructor with initial state parameter for debugging or partial execution startup.
	Landing_Routine(States initial_state) {
		state.current_state = initial_state;
	}

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
	void internal_transition() {
		switch (state.current_state) {
			case States::REQUEST_LAND:
				state.current_state = States::LANDING;
				break;
			case States::NOTIFY_LANDED:
				state.current_state = States::LANDED;
				break;
			default:
				break;
		}
	}

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		bool received_landing_achieved;
		bool received_pilot_takeover;
		bool received_hover_crit_met;
		bool received_land;

		received_pilot_takeover = get_messages<typename Landing_Routine_defs::i_pilot_takeover>(mbs).size() >= 1;

		if (received_pilot_takeover) {
			state.current_state = States::PILOT_CONTROL;
		} else {
			switch (state.current_state) {
				case States::IDLE:
					received_land = get_messages<typename Landing_Routine_defs::i_land>(mbs).size() >= 1;

					if (received_land) {
						state.current_state = States::REQUEST_LAND;
					}
					break;
				case States::LANDING:
					received_landing_achieved = get_messages<typename Landing_Routine_defs::i_landing_achieved>(mbs).size() >= 1;

					if (received_landing_achieved) {
						state.current_state = States::NOTIFY_LANDED;
					}
					break;
				case States::PILOT_CONTROL:
					received_landing_achieved = get_messages<typename Landing_Routine_defs::i_landing_achieved>(mbs).size() >= 1;

					if (received_landing_achieved) {
						state.current_state = States::NOTIFY_LANDED;
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
			case States::REQUEST_LAND:
				bag_port_out.push_back(true);
				get_messages<typename Landing_Routine_defs::o_land_requested>(bags) = bag_port_out;
				break;
			case States::NOTIFY_LANDED:
				bag_port_out.push_back(true);
				get_messages<typename Landing_Routine_defs::o_mission_complete>(bags) = bag_port_out;
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
			case States::REQUEST_LAND:
				return TIME(TA_ZERO);
				break;
			case States::LANDING:
				return numeric_limits<TIME>::infinity();
				break;
			case States::NOTIFY_LANDED:
				return TIME(TA_ZERO);
				break;
			case States::LANDED:
				return numeric_limits<TIME>::infinity();
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

	friend ostringstream& operator<<(ostringstream& os, const typename Landing_Routine<TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state);
		return os;
	}
};

#endif // LANDING_ROUTING_HPP
