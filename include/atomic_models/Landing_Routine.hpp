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

#include <limits>
#include <cassert>
#include <string>

#include "message_structures/message_fcc_command_t.hpp"
#include "message_structures/message_boss_mission_update_t.hpp"
#include "message_structures/message_update_gcs_t.hpp"

#include "enum_string_conversion.hpp"
#include "Constants.hpp"

using namespace cadmium;

// Atomic Model
template<typename TIME> class Landing_Routine {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(MISSION_STARTED)
		(REQUEST_LAND)
		(LANDING)
		(NOTIFY_LANDED)
		(LANDED)
		(PILOT_CONTROL)
	);

	// Input and output port definition
	struct defs {
		struct i_land : public in_port<bool> {};
		struct i_landing_achieved : public in_port<bool> {};
		struct i_pilot_takeover : public in_port<bool> {};
		struct i_start_mission : public in_port<bool> {};

		struct o_fcc_command_land : public out_port<message_fcc_command_t> {};
		struct o_mission_complete : public out_port<bool> {};
		struct o_update_boss : public out_port<message_boss_mission_update_t> {};
		struct o_update_gcs : public out_port<message_update_gcs_t> {};
		struct o_update_mission_item : public out_port<bool> {};
	};

	// Create a tuple of input ports (required for the simulator)
	using input_ports = tuple<
		typename Landing_Routine<TIME>::defs::i_land,
		typename Landing_Routine<TIME>::defs::i_landing_achieved,
		typename Landing_Routine<TIME>::defs::i_pilot_takeover,
		typename Landing_Routine<TIME>::defs::i_start_mission
	>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
		typename Landing_Routine<TIME>::defs::o_fcc_command_land,
		typename Landing_Routine<TIME>::defs::o_mission_complete,
		typename Landing_Routine<TIME>::defs::o_update_boss,
		typename Landing_Routine<TIME>::defs::o_update_gcs,
		typename Landing_Routine<TIME>::defs::o_update_mission_item
	>;

	// This is used to track the state of the atomic model.
	// (required for the simulator)
	struct state_type {
		States current_state;
	} state;

	// Default constructor
	Landing_Routine() {
		state.current_state = States::IDLE;
	}

	// Constructor with initial state parameter for debugging or partial execution startup.
	explicit Landing_Routine(States initial_state) {
		state.current_state = initial_state;
	}

	// Internal transitions
	// These are transitions occurring from internal inputs
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
	// These are transitions occurring from external inputs
	// (required for the simulator)
	void external_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {
        bool received_pilot_takeover = !get_messages<typename Landing_Routine<TIME>::defs::i_pilot_takeover>(mbs).empty();
		if (received_pilot_takeover) {
			state.current_state = States::PILOT_CONTROL;
            return;
		}

        bool received_start_mission = !get_messages<typename Landing_Routine<TIME>::defs::i_start_mission>(mbs).empty();
        if (received_start_mission) {
            state.current_state = States::MISSION_STARTED;
            return;
        }

        bool received_land;
        bool received_landing_achieved;
        switch (state.current_state) {
            case States::MISSION_STARTED:
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

	// confluence transition
	// Used to call set call precedent
	void confluence_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), std::move(mbs));
	}

	// output function
	typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		vector<bool> mission_complete_messages;
		vector<bool> mission_item_messages;
		vector<message_fcc_command_t> fcc_messages;
		vector<message_boss_mission_update_t> boss_messages;
		vector<message_update_gcs_t> gcs_messages;

		switch (state.current_state) {
			case States::REQUEST_LAND:
				{
					message_fcc_command_t temp_fcc_command = message_fcc_command_t();
					temp_fcc_command.set_supervisor_status(Control_Mode_E::LANDING_REQUESTED);
					message_boss_mission_update_t temp_boss_update = message_boss_mission_update_t();
					strcpy(temp_boss_update.description, "LAND");
					message_update_gcs_t temp_gcs_update;
					temp_gcs_update.text = "Landing";
					temp_gcs_update.severity = Mav_Severities_E::MAV_SEVERITY_ALERT;

					fcc_messages.push_back(temp_fcc_command);
					boss_messages.push_back(temp_boss_update);
					gcs_messages.push_back(temp_gcs_update);

					get_messages<typename Landing_Routine<TIME>::defs::o_fcc_command_land>(bags) = fcc_messages;
					get_messages<typename Landing_Routine<TIME>::defs::o_update_boss>(bags) = boss_messages;
					get_messages<typename Landing_Routine<TIME>::defs::o_update_gcs>(bags) = gcs_messages;

				}
				break;
			case States::NOTIFY_LANDED:
				{
					message_update_gcs_t temp_gcs_update;
					temp_gcs_update.text = "Just landed!";
					temp_gcs_update.severity = Mav_Severities_E::MAV_SEVERITY_INFO;
					gcs_messages.push_back(temp_gcs_update);
					mission_complete_messages.push_back(true);
					mission_item_messages.push_back(true);
					get_messages<typename Landing_Routine<TIME>::defs::o_mission_complete>(bags) = mission_complete_messages;
					get_messages<typename Landing_Routine<TIME>::defs::o_update_mission_item>(bags) = mission_item_messages;
					get_messages<typename Landing_Routine<TIME>::defs::o_update_gcs>(bags) = gcs_messages;
				}
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
            case States::LANDING:
            case States::LANDED:
            case States::PILOT_CONTROL:
				return numeric_limits<TIME>::infinity();
			case States::REQUEST_LAND:
			case States::NOTIFY_LANDED:
				return TIME(TA_ZERO);
			default:
				assert(false && "Unhandled state time advance.");
		}
	}

	friend ostringstream& operator<<(ostringstream& os, const typename Landing_Routine<TIME>::state_type& i) {
		os << (string("State: ") + enumToString(i.current_state) + string("\n"));
		return os;
	}
};

#endif // LANDING_ROUTING_HPP
