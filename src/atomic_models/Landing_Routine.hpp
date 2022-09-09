/**
 * 	\file		Landing_Routine.hpp
 *	\brief		Definition of the Landing Routine atomic model.
 *	\details	This header file defines the Landing Routine atomic model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor when 
				performing a landing after coming to a hover over a landing point.
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
*	\author		Tanner Trautrim
*	\author		James Horner
*/
template<typename TIME> 
class Landing_Routine {
public:
	/**
	 *	\enum	States
	 * 	\brief	Declaration of the states of the atomic model.
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
	 * \struct	defs
	 * \brief 	Declaration of the ports for the model.
	 * \see		input_ports
	 * \see 	output_ports
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
	 *	\struct	input_ports
	 * 	\brief 	Defintion of the input ports for the model.
	 * 	\var	i_land				[input] Port for receiving a request to land at a landing point.
	 * 	\var	i_landing_achieved	[input] Port for receiving signal indicating that the aircraft has successfully landed.
	 * 	\var 	i_pilot_takeover 	[input] Port for receiving signal indicating that the pilot has taken control from the supervisor.
	 * 	\var 	i_start_mission 	[input] Port for receiving signal indicating the mission has started.
	 */
	using input_ports = std::tuple<
		typename Landing_Routine<TIME>::defs::i_land,
		typename Landing_Routine<TIME>::defs::i_landing_achieved,
		typename Landing_Routine<TIME>::defs::i_pilot_takeover,
		typename Landing_Routine<TIME>::defs::i_start_mission
	>;

	/**
	 *	\struct	output_ports
	 * 	\brief 	Defintion of the output ports for the model.
	 * 	\var	o_fcc_command_land		[output] Port for sending land commands to the FCC.
	 * 	\var	o_mission_complete		[output] Port for declaring the mission as being complete after landing.
	 * 	\var	o_update_boss 			[output] Port for sending updates to BOSS.
	 * 	\var	o_update_gcs 			[output] Port for sending updates to the GCS.
	 * 	\var	o_update_mission_item	[output] Port for updating the mission manager that the last mission item has been reached.
	 */
	using output_ports = std::tuple<
		typename Landing_Routine<TIME>::defs::o_fcc_command_land,
		typename Landing_Routine<TIME>::defs::o_mission_complete,
		typename Landing_Routine<TIME>::defs::o_update_boss,
		typename Landing_Routine<TIME>::defs::o_update_gcs,
		typename Landing_Routine<TIME>::defs::o_update_mission_item
	>;

	/**
	 *	\struct	state_type
	 * 	\brief 	Defintion of the states of the atomic model.
	 * 	\var 	current_state 	Current state of atomic model.
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
