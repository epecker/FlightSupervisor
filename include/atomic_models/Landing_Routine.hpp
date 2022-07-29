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

// Message structures
#include "message_structures/message_fcc_command_t.hpp"
#include "message_structures/message_boss_mission_update_t.hpp"

// Includes the macro DEFINE_ENUM_WITH_STRING_CONVERSIONS
#include "enum_string_conversion.hpp"
#include "Constants.hpp"

using namespace cadmium;
using namespace std;

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

	// Input and output port definition
	struct defs {
		struct i_landing_achieved : public in_port<bool> {};
		struct i_pilot_takeover : public in_port<bool> {};
		struct i_land : public in_port<bool> {};

		struct o_fcc_command_land : public out_port<message_fcc_command_t> {};
		struct o_mission_complete : public out_port<bool> {};
		struct o_update_boss : public out_port<message_boss_mission_update_t> {};
		struct o_update_gcs : public out_port<string> {};
	};

	// Create a tuple of input ports (required for the simulator)
	using input_ports = tuple<
		typename Landing_Routine<TIME>::defs::i_landing_achieved,
		typename Landing_Routine<TIME>::defs::i_pilot_takeover,
		typename Landing_Routine<TIME>::defs::i_land
	>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
		typename Landing_Routine<TIME>::defs::o_fcc_command_land,
		typename Landing_Routine<TIME>::defs::o_mission_complete,
		typename Landing_Routine<TIME>::defs::o_update_boss,
		typename Landing_Routine<TIME>::defs::o_update_gcs
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

		received_pilot_takeover = get_messages<typename Landing_Routine<TIME>::defs::i_pilot_takeover>(mbs).size() >= 1;

		if (received_pilot_takeover) {
			state.current_state = States::PILOT_CONTROL;
		} else {
			switch (state.current_state) {
				case States::IDLE:
					received_land = get_messages<typename Landing_Routine<TIME>::defs::i_land>(mbs).size() >= 1;

					if (received_land) {
						state.current_state = States::REQUEST_LAND;
					}
					break;
				case States::LANDING:
					received_landing_achieved = get_messages<typename Landing_Routine<TIME>::defs::i_landing_achieved>(mbs).size() >= 1;

					if (received_landing_achieved) {
						state.current_state = States::NOTIFY_LANDED;
					}
					break;
				case States::PILOT_CONTROL:
					received_landing_achieved = get_messages<typename Landing_Routine<TIME>::defs::i_landing_achieved>(mbs).size() >= 1;

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
		vector<message_fcc_command_t> fcc_messages;
		vector<message_boss_mission_update_t> boss_messages;
		vector<string> gcs_messages;

		switch (state.current_state) {
			case States::REQUEST_LAND: 
				{
					message_fcc_command_t temp_fcc_command = message_fcc_command_t();
					temp_fcc_command.set_supervisor_status(Control_Mode_E::LANDING_REQUESTED);
					message_boss_mission_update_t temp_boss_update = message_boss_mission_update_t();
					strcpy(temp_boss_update.description, "LAND");
					string temp_gcs_update = "Landing";

					fcc_messages.push_back(temp_fcc_command);
					boss_messages.push_back(temp_boss_update);
					gcs_messages.push_back(temp_gcs_update);

					get_messages<typename Landing_Routine<TIME>::defs::o_fcc_command_land>(bags) = fcc_messages;
					get_messages<typename Landing_Routine<TIME>::defs::o_update_boss>(bags) = boss_messages;
					get_messages<typename Landing_Routine<TIME>::defs::o_update_gcs>(bags) = gcs_messages;

				}
				break;
			case States::NOTIFY_LANDED:
				bag_port_out.push_back(true);
				get_messages<typename Landing_Routine<TIME>::defs::o_mission_complete>(bags) = bag_port_out;
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
		os << (string("State: ") + enumToString(i.current_state) + string("\n"));
		return os;
	}
};

#endif // LANDING_ROUTING_HPP
