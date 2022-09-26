/**
 * 	\file		Landing_Routine.hpp
 *	\brief		Definition of the Landing Routine atomic model.
 *	\details	This header file defines the Landing Routine atomic model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor when
				performing a landing after coming to a hover over a landing point.
 *	\image		html atomic_models/landing_routine.png
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef LANDING_ROUTING_HPP
#define LANDING_ROUTING_HPP

// Messages structures
#include "../message_structures/message_boss_mission_update_t.hpp"
#include "../message_structures/message_fcc_command_t.hpp"
#include "../message_structures/message_landing_point_t.hpp"
#include "../message_structures/message_update_gcs_t.hpp"

// Utility functions
#include "../enum_string_conversion.hpp"
#include "../Constants.hpp"

// Cadmium Simulator Headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

// System Libraries
#include <limits>
#include <cassert>
#include <string>

/**
 * 	\class		Landing_Routine
 *	\brief		Definition of the Landing Routine atomic model.
*	\details	This class defines the Landing Routine atomic model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor when
				performing a landing after coming to a hover over a landing point.
 *	\image		html atomic_models/landing_routine.png
*/
template<typename TIME>
class Landing_Routine {
public:
	/**
	 *	\par	States
	 * 	Declaration of the states of the atomic model.
	 */
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(WAIT_LAND_REQUEST)
		(REQUEST_LAND)
		(LANDING)
		(NOTIFY_LANDED)
		(LANDED)
		(PILOT_CONTROL)
	);

	/**
	 *	\brief	For definition of the input and output ports see:
	 *	\ref 	Landing_Routine_input_ports "Input Ports" and
	 *	\ref 	Landing_Routine_output_ports "Output Ports"
	 * 	\note 	All input and output ports must be listed in this struct.
	 */
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

	/**
	 * 	\anchor	Landing_Routine_input_ports
	 *	\par	Input Ports
	 * 	Definition of the input ports for the model.
	 * 	\param	i_land				Port for receiving a request to land at a landing point.
	 * 	\param	i_landing_achieved	Port for receiving signal indicating that the aircraft has successfully landed.
	 * 	\param 	i_pilot_takeover 	Port for receiving signal indicating that the pilot has taken control from the supervisor.
	 * 	\param 	i_start_mission 	Port for receiving signal indicating the mission has started.
	 */
	using input_ports = std::tuple<
		typename Landing_Routine<TIME>::defs::i_land,
		typename Landing_Routine<TIME>::defs::i_landing_achieved,
		typename Landing_Routine<TIME>::defs::i_pilot_takeover,
		typename Landing_Routine<TIME>::defs::i_start_mission
	>;

	/**
	 *	\anchor	Landing_Routine_output_ports
	 * 	\par 	Output Ports
	 * 	Definition of the output ports for the model.
	 * 	\param	o_fcc_command_land		Port for sending land commands to the FCC.
	 * 	\param	o_mission_complete		Port for declaring the mission as being complete after landing.
	 * 	\param	o_update_boss 			Port for sending updates to BOSS.
	 * 	\param	o_update_gcs 			Port for sending updates to the GCS.
	 * 	\param	o_update_mission_item	Port for updating the mission manager that the last mission item has been reached.
	 */
	using output_ports = std::tuple<
		typename Landing_Routine<TIME>::defs::o_fcc_command_land,
		typename Landing_Routine<TIME>::defs::o_mission_complete,
		typename Landing_Routine<TIME>::defs::o_update_boss,
		typename Landing_Routine<TIME>::defs::o_update_gcs,
		typename Landing_Routine<TIME>::defs::o_update_mission_item
	>;

	/**
	 *	\anchor	Landing_Routine_state_type
	 *	\par	State
	 * 	Definition of the states of the atomic model.
	 * 	\param 	current_state 	Current state of atomic model.
	 */
	struct state_type {
		States current_state;
	} state;

	/**
	 * \brief 	Default constructor for the model.
	 */
	Landing_Routine() {
        landing_point = message_landing_point_t();
        mission_number = 0;
        state.current_state = States::IDLE;
	}

	/**
	 * \brief 	Constructor for the model with initial state parameter
	 * 			for debugging or partial execution startup.
	 * \param	initial_state	States initial state of the model.
	 */
	explicit Landing_Routine(States initial_state) {
        landing_point = message_landing_point_t();
        mission_number = 0;
        state.current_state = initial_state;
	}

	/// Internal transitions of the model
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

	/// External transitions of the model
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

	/// Function used to decide precedence between internal and external transitions when both are scheduled simultaneously.
	void confluence_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), std::move(mbs));
	}

	/// Function for generating output from the model before internal transitions.
	typename cadmium::make_message_bags<output_ports>::type output() const {
		typename cadmium::make_message_bags<output_ports>::type bags;

		switch (state.current_state) {
			case States::REQUEST_LAND: {
				message_fcc_command_t fcc_command = message_fcc_command_t();
				fcc_command.set_supervisor_status(Control_Mode_E::LANDING_REQUESTED);
				cadmium::get_messages<typename Landing_Routine<TIME>::defs::o_fcc_command_land>(bags).push_back(fcc_command);

				// Update the boss displays landing point location
				cadmium::get_messages<typename Landing_Routine::defs::o_update_boss>(bags).emplace_back(
						landing_point.id,
						landing_point.lat,
						landing_point.lon,
						mission_number,
						landing_point.missionItemNo,
						landing_point.alt * FT_TO_METERS,
						landing_point.hdg,
						0,
						"LAND"
				);

				// Update the ground control computer
				cadmium::get_messages<typename Landing_Routine::defs::o_update_gcs>(bags).emplace_back(
						"Landing",
						Mav_Severities_E::MAV_SEVERITY_ALERT
				);
				break;
			}
			case States::NOTIFY_LANDED: {
				cadmium::get_messages<typename Landing_Routine<TIME>::defs::o_mission_complete>(bags).emplace_back(true);
				cadmium::get_messages<typename Landing_Routine<TIME>::defs::o_update_mission_item>(bags).emplace_back(true);

				// Update the ground control computer
				cadmium::get_messages<typename Landing_Routine::defs::o_update_gcs>(bags).emplace_back(
						"Just landed!",
						Mav_Severities_E::MAV_SEVERITY_INFO
				);
				break;
			}
			default:
				break;
		}

		return bags;
	}

	/// Function to declare the time advance value for each state of the model.
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

	/**
	 *  \brief 		Operator for defining how the model state will be represented as a string.
	 * 	\warning 	Prepended "State: " is required for log parsing, do not remove.
	 */
	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Landing_Routine<TIME>::state_type& i) {
		os << (std::string("State: ") + enumToString(i.current_state) + std::string("\n"));
		return os;
	}

private:
    /// Variable for storing the landing point that the helicopter will land at.
    message_landing_point_t landing_point;
    /// Variable for storing the number of the mission for updating BOSS.
    int mission_number;
};

#endif // LANDING_ROUTING_HPP
