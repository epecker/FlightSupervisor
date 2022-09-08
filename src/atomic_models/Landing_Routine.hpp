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

#include "cadmium/modeling/ports.hpp"
#include "cadmium/modeling/message_bag.hpp"

#include <limits>
#include <cassert>
#include <string>

#include "../message_structures/message_boss_mission_update_t.hpp"
#include "../message_structures/message_fcc_command_t.hpp"
#include "../message_structures/message_landing_point_t.hpp"
#include "../message_structures/message_update_gcs_t.hpp"

#include "../enum_string_conversion.hpp"
#include "../Constants.hpp"

// Atomic Model
template<typename TIME> class Landing_Routine {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(WAIT_LAND_REQUEST)
		(REQUEST_LAND)
		(LANDING)
		(NOTIFY_LANDED)
		(LANDED)
		(PILOT_CONTROL)
	);

	// Input and output port definition
	struct defs {
		struct i_land : public cadmium::in_port<message_landing_point_t> {};
		struct i_landing_achieved : public cadmium::in_port<bool> {};
		struct i_pilot_takeover : public cadmium::in_port<bool> {};
		struct i_start_mission : public cadmium::in_port<int> {};

		struct o_fcc_command_land : public cadmium::out_port<message_fcc_command_t> {};
		struct o_mission_complete : public cadmium::out_port<bool> {};
		struct o_update_boss : public cadmium::out_port<message_boss_mission_update_t> {};
		struct o_update_gcs : public cadmium::out_port<message_update_gcs_t> {};
		struct o_update_mission_item : public cadmium::out_port<bool> {};
	};

	// Create a tuple of input ports (required for the simulator)
	using input_ports = std::tuple<
		typename Landing_Routine<TIME>::defs::i_land,
		typename Landing_Routine<TIME>::defs::i_landing_achieved,
		typename Landing_Routine<TIME>::defs::i_pilot_takeover,
		typename Landing_Routine<TIME>::defs::i_start_mission
	>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = std::tuple<
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
        landing_point = message_landing_point_t();
        mission_number = 0;
        state.current_state = States::IDLE;
	}

	// Constructor with initial state parameter for debugging or partial execution startup.
	explicit Landing_Routine(States initial_state) {
        landing_point = message_landing_point_t();
        mission_number = 0;
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
	void external_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
        bool received_pilot_takeover = !cadmium::get_messages<typename Landing_Routine<TIME>::defs::i_pilot_takeover>(mbs).empty();
		if (received_pilot_takeover) {
			state.current_state = States::PILOT_CONTROL;
            return;
		}

        bool received_start_mission = !cadmium::get_messages<typename Landing_Routine<TIME>::defs::i_start_mission>(mbs).empty();
        if (received_start_mission) {
            mission_number = cadmium::get_messages<typename Landing_Routine<TIME>::defs::i_start_mission>(mbs).back();
            state.current_state = States::WAIT_LAND_REQUEST;
            return;
        }

        bool received_land;
        bool received_landing_achieved;
        switch (state.current_state) {
            case States::WAIT_LAND_REQUEST:
                received_land = !cadmium::get_messages<typename Landing_Routine<TIME>::defs::i_land>(mbs).empty();
                if (received_land) {
                    landing_point = cadmium::get_messages<typename Landing_Routine<TIME>::defs::i_land>(mbs).back();
                    state.current_state = States::REQUEST_LAND;
                }
                break;
            case States::LANDING:
                received_landing_achieved = !cadmium::get_messages<typename Landing_Routine<TIME>::defs::i_landing_achieved>(mbs).empty();
                if (received_landing_achieved) {
                    state.current_state = States::NOTIFY_LANDED;
                }
                break;
            case States::PILOT_CONTROL:
                received_landing_achieved = !cadmium::get_messages<typename Landing_Routine<TIME>::defs::i_landing_achieved>(mbs).empty();
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
	void confluence_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), std::move(mbs));
	}

	// output function
	typename cadmium::make_message_bags<output_ports>::type output() const {
		typename cadmium::make_message_bags<output_ports>::type bags;
		std::vector<bool> mission_complete_messages;
		std::vector<bool> mission_item_messages;
		std::vector<message_fcc_command_t> fcc_messages;
		std::vector<message_boss_mission_update_t> boss_messages;
		std::vector<message_update_gcs_t> gcs_messages;

		switch (state.current_state) {
			case States::REQUEST_LAND:
				{
					message_fcc_command_t temp_fcc_command = message_fcc_command_t();
					temp_fcc_command.set_supervisor_status(Control_Mode_E::LANDING_REQUESTED);

					message_boss_mission_update_t temp_boss{};
					temp_boss.update_landing_point(
							landing_point.id,
							landing_point.lat,
							landing_point.lon,
							landing_point.alt * FT_TO_METERS,
							landing_point.hdg,
							"LAND");
					temp_boss.missionNo = mission_number;
					temp_boss.missionItemNo = landing_point.missionItemNo;
					message_update_gcs_t temp_gcs_update{"Landing", Mav_Severities_E::MAV_SEVERITY_ALERT};

					fcc_messages.push_back(temp_fcc_command);
					boss_messages.push_back(temp_boss);
					gcs_messages.push_back(temp_gcs_update);

					cadmium::get_messages<typename Landing_Routine<TIME>::defs::o_fcc_command_land>(bags) = fcc_messages;
					cadmium::get_messages<typename Landing_Routine<TIME>::defs::o_update_boss>(bags) = boss_messages;
					cadmium::get_messages<typename Landing_Routine<TIME>::defs::o_update_gcs>(bags) = gcs_messages;
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
					cadmium::get_messages<typename Landing_Routine<TIME>::defs::o_mission_complete>(bags) = mission_complete_messages;
					cadmium::get_messages<typename Landing_Routine<TIME>::defs::o_update_mission_item>(bags) = mission_item_messages;
					cadmium::get_messages<typename Landing_Routine<TIME>::defs::o_update_gcs>(bags) = gcs_messages;
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
			case States::WAIT_LAND_REQUEST:
            case States::LANDING:
            case States::LANDED:
            case States::PILOT_CONTROL:
				return std::numeric_limits<TIME>::infinity();
			case States::REQUEST_LAND:
			case States::NOTIFY_LANDED:
				return TIME(TA_ZERO);
			default:
				assert(false && "Unhandled state time advance.");
		}
	}

	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Landing_Routine<TIME>::state_type& i) {
		os << (std::string("State: ") + enumToString(i.current_state) + std::string("\n"));
		return os;
	}

private:
    message_landing_point_t landing_point;
    int mission_number;
};

#endif // LANDING_ROUTING_HPP
