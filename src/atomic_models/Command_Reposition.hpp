/**
 * 	\file		Command_Reposition.hpp
 *	\brief		Definition of the Command Reposition atomic model.
 *	\details	This header file defines the Command Reposition atomic model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor when
				repositioning to a landing point.
 *	\image		html atomic_models/command_reposition.png
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef COMMAND_REPOSITION_HPP
#define COMMAND_REPOSITION_HPP

// Messages structures
#include "../message_structures/message_aircraft_state_t.hpp"
#include "../message_structures/message_hover_criteria_t.hpp"
#include "../message_structures/message_landing_point_t.hpp"
#include "../message_structures/message_fcc_command_t.hpp"
#include "../message_structures/message_boss_mission_update_t.hpp"
#include "../message_structures/message_update_gcs_t.hpp"

// Utility functions
#include "../enum_string_conversion.hpp"
#include <mavNRC/geo.h>

// Cadmium Simulator Headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

// System Libraries
#include <limits> // Used to set the time advance to infinity
#include <cassert>
#include <string>

/**
 *	\class		Command_Reposition
 *	\brief		Definition of the Command Reposition atomic model.
 *	\details	This class defines the Command Reposition coupled model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor when
				repositioning to a landing point.
 *	\image		html atomic_models/command_reposition.png
 */
template<typename TIME>
class Command_Reposition {
public:
	/**
	 *	\par	States
	 * 	Declaration of the states of the atomic model.
	 */
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(WAIT_REQUEST_REPOSITION)
		(REQUEST_STATE)
		(GET_STATE)
		(COMMAND_VEL)
		(COMMAND_HOVER)
		(STABILIZING)
		(LP_CRITERIA_MET)
		(LANDING)
		(CANCEL_HOVER)
		(TIMER_EXPIRED)
		(PILOT_CONTROL)
	);

	/**
	 *	\brief	For definition of the input and output ports see:
	 *	\ref 	Command_Reposition_input_ports "Input Ports" and
	 *	\ref 	Command_Reposition_output_ports "Output Ports"
	 * 	\note 	All input and output ports must be listed in this struct.
	 */
	struct defs {
		struct i_aircraft_state : public cadmium::in_port<message_aircraft_state_t> {};
		struct i_hover_criteria_met : public cadmium::in_port<bool> {};
		struct i_pilot_handover : public cadmium::in_port<message_landing_point_t> {};
		struct i_pilot_takeover : public cadmium::in_port<bool> {};
		struct i_request_reposition : public cadmium::in_port<message_landing_point_t> {};
		struct i_start_mission : public cadmium::in_port<int> {};

		struct o_cancel_hover : public cadmium::out_port<bool> {};
		struct o_fcc_command_velocity : public cadmium::out_port<message_fcc_command_t> {};
		struct o_lp_criteria_met : public cadmium::out_port<message_landing_point_t> {};
		struct o_request_aircraft_state : public cadmium::out_port<bool> {};
		struct o_set_mission_monitor_status : public cadmium::out_port<uint8_t> {};
		struct o_stabilize : public cadmium::out_port<message_hover_criteria_t> {};
		struct o_update_boss : public cadmium::out_port<message_boss_mission_update_t> {};
		struct o_update_gcs : public cadmium::out_port<message_update_gcs_t> {};
	};

	/**
	 * 	\anchor	Command_Reposition_input_ports
	 *	\par	Input Ports
	 * 	Definition of the input ports for the model.
	 * 	\param 	i_aircraft_state 		Port for receiving the current state of the aircraft.
	 * 	\param 	i_hover_criteria_met 	Port for receiving updates on whether the previously commanded hover was achieved.
	 * 	\param 	i_pilot_handover 		Port for receiving signal indicating control should be handed over to the pilot.
	 * 	\param 	i_pilot_takeover 		Port for receiving signal indicating that the pilot has taken control from the supervisor.
	 * 	\param 	i_request_reposition 	Port for receiving requests to reposition to landing points.
	 * 	\param 	i_start_mission 		Port for receiving signal indicating the mission has started.
	 */
	using input_ports = std::tuple<
		typename defs::i_aircraft_state,
		typename defs::i_hover_criteria_met,
		typename defs::i_pilot_handover,
		typename defs::i_pilot_takeover,
		typename defs::i_request_reposition,
		typename defs::i_start_mission
	>;

	/**
	 *	\anchor	Command_Reposition_output_ports
	 * 	\par 	Output Ports
	 * 	Definition of the output ports for the model.
	 * 	\param	o_cancel_hover 					Port for cancelling a previously requested stabilization.
	 * 	\param	o_fcc_command_velocity 			Port for sending velocity commands to the FCC.
	 * 	\param	o_lp_criteria_met 				Port for notifying that the helicopter is now hovering over an LP.
	 * 	\param	o_request_aircraft_state 		Port for requesting the current aircraft state.
	 * 	\param	o_set_mission_monitor_status 	Port for telling the mission monitor to stop monitoring mission progress.
	 * 	\param	o_stabilize 					Port for requesting the helicopter hover at a specific location.
	 * 	\param	o_update_boss 					Port for sending updates to BOSS.
	 * 	\param	o_update_gcs 					Port for sending updates to the GCS.
	 */
	using output_ports = std::tuple<
		typename defs::o_cancel_hover,
		typename defs::o_fcc_command_velocity,
		typename defs::o_lp_criteria_met,
		typename defs::o_request_aircraft_state,
		typename defs::o_set_mission_monitor_status,
		typename defs::o_stabilize,
		typename defs::o_update_boss,
		typename defs::o_update_gcs
	>;

	/**
	 *	\anchor	Command_Reposition_state_type
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
	Command_Reposition() {
		state.current_state = States::IDLE;
		aircraft_state = message_aircraft_state_t();
		landing_point = message_landing_point_t();
        velocity = 0;
        mission_number = 0;
	}

	/**
	 * \brief 	Constructor for the model with initial state parameter
	 * 			for debugging or partial execution startup.
	 * \param	initial_state	States initial state of the model.
	 */
	explicit Command_Reposition(States initial_state) {
		state.current_state = initial_state;
		aircraft_state = message_aircraft_state_t();
		landing_point = message_landing_point_t();
        velocity = 0;
        mission_number = 0;
	}

	/// Internal transitions of the model
	void internal_transition() {
		switch (state.current_state) {
			case States::REQUEST_STATE:
				state.current_state = States::GET_STATE;
				break;
			case States::COMMAND_VEL:
				state.current_state = States::COMMAND_HOVER;
				break;
			case States::COMMAND_HOVER:
				state.current_state = States::STABILIZING;
				break;
			case States::LP_CRITERIA_MET:
				state.current_state = States::LANDING;
				break;
			case States::CANCEL_HOVER:
				state.current_state = States::REQUEST_STATE;
				break;
			default:
				break;
		}
	}

	/// External transitions of the model
	void external_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
        bool received_pilot_takeover =  !cadmium::get_messages<typename defs::i_pilot_takeover>(mbs).empty();
        if (received_pilot_takeover) {
            state.current_state = States::PILOT_CONTROL;
            return;
        }

        bool received_start_mission = !cadmium::get_messages<typename defs::i_start_mission>(mbs).empty();
        if (received_start_mission) {
            reset_state();
            mission_number = cadmium::get_messages<typename defs::i_start_mission>(mbs).back();
            state.current_state = States::WAIT_REQUEST_REPOSITION;
            return;
        }

        bool received_pilot_handover = !cadmium::get_messages<typename defs::i_pilot_handover>(mbs).empty();
        if (received_pilot_handover && state.current_state != States::IDLE) {
			state.current_state = States::TIMER_EXPIRED;
            return;
		}

        switch (state.current_state) {
            case States::WAIT_REQUEST_REPOSITION: {
				bool received_request_reposition = !cadmium::get_messages<typename defs::i_request_reposition>(mbs).empty();

				if (received_request_reposition) {
					std::vector<message_landing_point_t> new_landing_points = cadmium::get_messages<typename defs::i_request_reposition>(mbs);
					// Set the landing point to reposition over to the newest input (found at the back of the vector of input LPs)
					landing_point = new_landing_points.back();
					state.current_state = States::REQUEST_STATE;
				}
				break;
			}
            case States::GET_STATE: {
				bool received_aircraft_state = !cadmium::get_messages<typename defs::i_aircraft_state>(mbs).empty();

				if (received_aircraft_state) {
					std::vector<message_aircraft_state_t> new_aircraft_state = cadmium::get_messages<typename defs::i_aircraft_state>(mbs);
					aircraft_state = new_aircraft_state[0];
					state.current_state = States::COMMAND_VEL;
				}
				break;
			}
            case States::COMMAND_VEL: {
				bool received_request_reposition = !cadmium::get_messages<typename defs::i_request_reposition>(mbs).empty();

				if (received_request_reposition) {
					std::vector<message_landing_point_t> new_landing_points = cadmium::get_messages<typename defs::i_request_reposition>(mbs);
					// Set the landing point to reposition over to the newest input (found at the back of the vector of input LPs)
					landing_point = new_landing_points.back();
					state.current_state = States::REQUEST_STATE;
				}
				break;
			}
            case States::COMMAND_HOVER: {
				bool received_request_reposition = !cadmium::get_messages<typename defs::i_request_reposition>(mbs).empty();

				if (received_request_reposition) {
					std::vector<message_landing_point_t> new_landing_points = cadmium::get_messages<typename defs::i_request_reposition>(mbs);
					// Set the landing point to reposition over to the newest input (found at the back of the vector of input LPs)
					landing_point = new_landing_points.back();
					state.current_state = States::REQUEST_STATE;
				}
				break;
			}
            case States::STABILIZING: {
				bool received_hover_criteria_met = !cadmium::get_messages<typename defs::i_hover_criteria_met>(mbs).empty();
				bool received_request_reposition = !cadmium::get_messages<typename defs::i_request_reposition>(mbs).empty();

				if (received_request_reposition) {
					std::vector<message_landing_point_t> new_landing_points = cadmium::get_messages<typename defs::i_request_reposition>(mbs);
					// Set the landing point to reposition over to the newest input (found at the back of the vector of input LPs)
					landing_point = new_landing_points.back();
					state.current_state = States::CANCEL_HOVER;
				} else if (received_hover_criteria_met) {
					state.current_state = States::LP_CRITERIA_MET;
				}
				break;
			}
            case States::LP_CRITERIA_MET: {
				bool received_request_reposition = !cadmium::get_messages<typename defs::i_request_reposition>(mbs).empty();

				if (received_request_reposition) {
					std::vector<message_landing_point_t> new_landing_points = cadmium::get_messages<typename defs::i_request_reposition>(mbs);
					// Set the landing point to reposition over to the newest input (found at the back of the vector of input LPs)
					landing_point = new_landing_points.back();
					state.current_state = States::CANCEL_HOVER;
				}
				break;
			}
            default:
                break;
        }
	}

	/// Function used to decide precedence between internal and external transitions when both are scheduled simultaneously.
	void confluence_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		external_transition(TIME(), std::move(mbs));
	}

	/// Function for generating output from the model before internal transitions.
	[[nodiscard]] typename cadmium::make_message_bags<output_ports>::type output() const {
		typename cadmium::make_message_bags<output_ports>::type bags;

		switch (state.current_state) {
			case States::REQUEST_STATE:
				cadmium::get_messages<typename defs::o_request_aircraft_state>(bags).emplace_back(true);
				break;
			case States::COMMAND_VEL: {
				float distance, altitude;
				get_distance_to_point_global_wgs84(aircraft_state.lat, aircraft_state.lon,
												   aircraft_state.alt_MSL * FT_TO_METERS, landing_point.lat,
												   landing_point.lon, landing_point.alt * METERS_TO_FT, &distance,
												   &altitude);
				velocity = distance / REPO_TRANSIT_TIME;

				if (velocity > MAX_REPO_VEL * KTS_TO_MPS) {
					velocity = MAX_REPO_VEL * KTS_TO_MPS;
				} else if (velocity < MIN_REPO_VEL * KTS_TO_MPS) {
					velocity = MIN_REPO_VEL * KTS_TO_MPS;
				}

				message_fcc_command_t mfc = message_fcc_command_t();
				mfc.change_velocity(velocity, aircraft_state.gps_time);
				cadmium::get_messages<typename defs::o_fcc_command_velocity>(bags).push_back(mfc);
				break;
			}
			case States::COMMAND_HOVER: {
				cadmium::get_messages<typename defs::o_set_mission_monitor_status>(bags)
						.emplace_back(0);

				// Send a hover criteria message
				cadmium::get_messages<typename defs::o_stabilize>(bags).emplace_back(
								landing_point.lat,
								landing_point.lon,
								landing_point.alt,
								landing_point.hdg,
								DEFAULT_LAND_CRITERIA_HOR_DIST,
								DEFAULT_LAND_CRITERIA_VERT_DIST,
								DEFAULT_LAND_CRITERIA_VEL,
								DEFAULT_LAND_CRITERIA_HDG,
								DEFAULT_LAND_CRITERIA_TIME,
								-1,
								0,
								0
						);

				// Update the boss displays landing point location
				cadmium::get_messages<typename defs::o_update_boss>(bags).emplace_back(
								landing_point.id,
								landing_point.lat,
								landing_point.lon,
								mission_number,
								landing_point.missionItemNo,
								landing_point.alt * FT_TO_METERS,
								landing_point.hdg,
								velocity * MPS_TO_KTS,
								"LP REP"
						);

				// Update the ground control computer
				cadmium::get_messages<typename defs::o_update_gcs>(bags).emplace_back(
								"Repositioning to LP!",
								Mav_Severities_E::MAV_SEVERITY_ALERT
						);
				break;
			}
			case States::CANCEL_HOVER:
				cadmium::get_messages<typename defs::o_cancel_hover>(bags).emplace_back(true);
				break;
			case States::LP_CRITERIA_MET:
				cadmium::get_messages<typename defs::o_lp_criteria_met>(bags).push_back(landing_point);
				break;
			default:
				break;
		}

		return bags;
	}

	/// Function to declare the time advance value for each state of the model.
	TIME time_advance() const {
		TIME next_internal;
		switch (state.current_state) {
			case States::IDLE:
			case States::WAIT_REQUEST_REPOSITION:
			case States::GET_STATE:
			case States::STABILIZING:
			case States::LANDING:
			case States::TIMER_EXPIRED:
			case States::PILOT_CONTROL:
				next_internal = std::numeric_limits<TIME>::infinity();
				break;
			case States::REQUEST_STATE:
			case States::COMMAND_VEL:
			case States::COMMAND_HOVER:
			case States::LP_CRITERIA_MET:
			case States::CANCEL_HOVER:
				next_internal = TIME(TA_ZERO);
				break;
			default:
				assert(false && "Unhandled state time advance.");
		}
		return next_internal;
	}

	/**
	 *  \brief 		Operator for defining how the model state will be represented as a string.
	 * 	\warning 	Prepended "State: " is required for log parsing, do not remove.
	 */
	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Command_Reposition<TIME>::state_type& i) {
		os << (std::string("State: ") + enumToString(i.current_state) + std::string("\n"));
		return os;
	}

private:
	/// Variable for storing the current landing points being repositioned to.
	message_landing_point_t landing_point;
	/// Variable for storing aircraft state when scheduling Reposition velocities.
	message_aircraft_state_t aircraft_state;
	/// Variable for storing the reposition velocity.
    mutable float velocity;
    /// Variable for storing the number of the mission for updating BOSS.
    int mission_number;

    /// Function for resetting private variables.
    void reset_state() {
        aircraft_state = message_aircraft_state_t();
        landing_point = message_landing_point_t();
        velocity = 0;
        mission_number = 0;
    }
};

#endif // COMMAND_REPOSITION_HPP
