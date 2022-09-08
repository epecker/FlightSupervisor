/**
 *	\brief		Definition of the Command Repostions atomic model.
 *	\details	This header file defines the Landing coupled model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor when
				repositioning to a landing point.
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
 *	\brief		Definition of the Command Repostions atomic model.
 *	\details	This class defines the Landing coupled model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor when
				repositioning to a landing point.
 */
template<typename TIME> class Command_Reposition {
public:
	/**
	 *	\enum	States
	 * 	\brief	Declaration of the states of the atomic model.
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
	 * \struct	defs
	 * \brief 	Declaration of the ports for the model.
	 * \see		input_ports
	 * \see 	output_ports
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
	 *	\struct	input_ports
	 * 	\brief 	Defintion of the input ports for the model.
	 * 	\var 	i_aircraft_state [input]
	 * 	Port for
	 * 	\var 	i_hover_criteria_met [input]
	 * 	Port for
	 * 	\var 	i_pilot_handover [input]
	 * 	Port for
	 * 	\var 	i_pilot_takeover [input]
	 * 	Port for
	 * 	\var 	i_request_reposition [input]
	 * 	Port for
	 * 	\var 	i_start_mission [input]
	 * 	Port for
	 */
	using input_ports = std::tuple<
		typename Command_Reposition::defs::i_aircraft_state,
		typename Command_Reposition::defs::i_hover_criteria_met,
		typename Command_Reposition::defs::i_pilot_handover,
		typename Command_Reposition::defs::i_pilot_takeover,
		typename Command_Reposition::defs::i_request_reposition,
		typename Command_Reposition::defs::i_start_mission
	>;

	/**
	 *	\struct	output_ports
	 * 	\brief 	Defintion of the output ports for the model.
	 * 	\var	o_cancel_hover
	 * 	[output] Port for
	 * 	\var	o_fcc_command_velocity
	 * 	[output] Port for
	 * 	\var	o_lp_criteria_met
	 * 	[output] Port for
	 * 	\var	o_request_aircraft_state
	 * 	[output] Port for
	 * 	\var	o_set_mission_monitor_status
	 * 	[output] Port for
	 * 	\var	o_stabilize
	 * 	[output] Port for
	 * 	\var	o_update_boss
	 * 	[output] Port for
	 * 	\var	o_update_gcs
	 * 	[output] Port for
	 */
	using output_ports = std::tuple<
		typename Command_Reposition::defs::o_cancel_hover,
		typename Command_Reposition::defs::o_fcc_command_velocity,
		typename Command_Reposition::defs::o_lp_criteria_met,
		typename Command_Reposition::defs::o_request_aircraft_state,
		typename Command_Reposition::defs::o_set_mission_monitor_status,
		typename Command_Reposition::defs::o_stabilize,
		typename Command_Reposition::defs::o_update_boss,
		typename Command_Reposition::defs::o_update_gcs
	>;

	/**
	 *	\struct	state_type
	 * 	\brief 	Defintion of the states of the atomic model.
	 * 	\var 	current_state
	 * 	Current state of atomic model.
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
        bool received_pilot_takeover =  !cadmium::get_messages<typename Command_Reposition::defs::i_pilot_takeover>(mbs).empty();
        if (received_pilot_takeover) {
            state.current_state = States::PILOT_CONTROL;
            return;
        }

        bool received_start_mission = !cadmium::get_messages<typename Command_Reposition::defs::i_start_mission>(mbs).empty();
        if (received_start_mission) {
            reset_state();
            mission_number = cadmium::get_messages<typename Command_Reposition::defs::i_start_mission>(mbs).back();
            state.current_state = States::WAIT_REQUEST_REPOSITION;
            return;
        }

        bool received_pilot_handover = !cadmium::get_messages<typename Command_Reposition::defs::i_pilot_handover>(mbs).empty();
        if (received_pilot_handover && state.current_state != States::IDLE) {
			state.current_state = States::TIMER_EXPIRED;
            return;
		}

        bool received_aircraft_state;
        bool received_hover_criteria_met;
        bool received_request_reposition;
        switch (state.current_state) {
            case States::WAIT_REQUEST_REPOSITION:
                received_request_reposition = !cadmium::get_messages<typename Command_Reposition::defs::i_request_reposition>(mbs).empty();

                if (received_request_reposition) {
                    std::vector<message_landing_point_t> new_landing_points = cadmium::get_messages<typename Command_Reposition::defs::i_request_reposition>(mbs);
                    // Set the landing point to reposition over to the newest input (found at the back of the vector of input LPs)
                    landing_point = new_landing_points.back();
                    state.current_state = States::REQUEST_STATE;
                }
                break;
            case States::GET_STATE:
                received_aircraft_state = !cadmium::get_messages<typename Command_Reposition::defs::i_aircraft_state>(mbs).empty();

                if (received_aircraft_state) {
                    std::vector<message_aircraft_state_t> new_aircraft_state = cadmium::get_messages<typename Command_Reposition::defs::i_aircraft_state>(mbs);
                    aircraft_state = new_aircraft_state[0];
                    state.current_state = States::COMMAND_VEL;
                }
                break;
            case States::COMMAND_VEL:
                received_request_reposition = !cadmium::get_messages<typename Command_Reposition::defs::i_request_reposition>(mbs).empty();

                if (received_request_reposition) {
                    std::vector<message_landing_point_t> new_landing_points = cadmium::get_messages<typename Command_Reposition::defs::i_request_reposition>(mbs);
                    // Set the landing point to reposition over to the newest input (found at the back of the vector of input LPs)
                    landing_point = new_landing_points.back();
                    state.current_state = States::REQUEST_STATE;
                }
                break;
            case States::COMMAND_HOVER:
                received_request_reposition = !cadmium::get_messages<typename Command_Reposition::defs::i_request_reposition>(mbs).empty();

                if (received_request_reposition) {
                    std::vector<message_landing_point_t> new_landing_points = cadmium::get_messages<typename Command_Reposition::defs::i_request_reposition>(mbs);
                    // Set the landing point to reposition over to the newest input (found at the back of the vector of input LPs)
                    landing_point = new_landing_points.back();
                    state.current_state = States::REQUEST_STATE;
                }
                break;
            case States::STABILIZING:
                received_hover_criteria_met = !cadmium::get_messages<typename Command_Reposition::defs::i_hover_criteria_met>(mbs).empty();
                received_request_reposition = !cadmium::get_messages<typename Command_Reposition::defs::i_request_reposition>(mbs).empty();

                if (received_request_reposition) {
                    std::vector<message_landing_point_t> new_landing_points = cadmium::get_messages<typename Command_Reposition::defs::i_request_reposition>(mbs);
                    // Set the landing point to reposition over to the newest input (found at the back of the vector of input LPs)
                    landing_point = new_landing_points.back();
                    state.current_state = States::CANCEL_HOVER;
                } else if (received_hover_criteria_met) {
                    state.current_state = States::LP_CRITERIA_MET;
                }
                break;
            case States::LP_CRITERIA_MET:
                received_request_reposition = !cadmium::get_messages<typename Command_Reposition::defs::i_request_reposition>(mbs).empty();

                if (received_request_reposition) {
                    std::vector<message_landing_point_t> new_landing_points = cadmium::get_messages<typename Command_Reposition::defs::i_request_reposition>(mbs);
                    // Set the landing point to reposition over to the newest input (found at the back of the vector of input LPs)
                    landing_point = new_landing_points.back();
                    state.current_state = States::CANCEL_HOVER;
                }
                break;
            default:
                break;
        }
	}

	/// Function used to decide precedence between internal and external transitions when both are scheduled simultaneously.
	void confluence_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		external_transition(TIME(), std::move(mbs));
	}

	/// Function for generating output from the model after internal transitions.
	[[nodiscard]] typename cadmium::make_message_bags<output_ports>::type output() const {
		typename cadmium::make_message_bags<output_ports>::type bags;
		std::vector<bool> bag_port_out;
		std::vector<message_landing_point_t> bag_port_LP_out;
		std::vector<message_fcc_command_t> bag_port_fcc_out;
		std::vector<message_hover_criteria_t> bag_port_hover_out;
		std::vector<uint8_t> mission_monitor_messages;
		std::vector<message_boss_mission_update_t> boss_messages;
		std::vector<message_update_gcs_t> gcs_messages;

		switch (state.current_state) {
			case States::REQUEST_STATE:
				{
					bag_port_out.push_back(true);
					cadmium::get_messages<typename Command_Reposition::defs::o_request_aircraft_state>(bags) = bag_port_out;
				}
				break;
			case States::COMMAND_VEL:
			{
				message_fcc_command_t mfc = message_fcc_command_t();
				float distance, altitude;
				get_distance_to_point_global_wgs84(aircraft_state.lat, aircraft_state.lon, aircraft_state.alt_MSL * FT_TO_METERS, landing_point.lat, landing_point.lon, landing_point.alt * METERS_TO_FT, &distance, &altitude);
				velocity = distance / REPO_TRANSIT_TIME;

				if (velocity > MAX_REPO_VEL * KTS_TO_MPS) {
					velocity = MAX_REPO_VEL * KTS_TO_MPS;
				} else if (velocity < MIN_REPO_VEL * KTS_TO_MPS) {
                    velocity = MIN_REPO_VEL * KTS_TO_MPS;
                }

				mfc.change_velocity(velocity, aircraft_state.gps_time);
				bag_port_fcc_out.push_back(mfc);
				cadmium::get_messages<typename Command_Reposition::defs::o_fcc_command_velocity>(bags) = bag_port_fcc_out;
			}
				break;
			case States::COMMAND_HOVER:
			{
				message_hover_criteria_t mhc;
				mhc.desiredLat = landing_point.lat;
				mhc.desiredLon = landing_point.lon;
				mhc.desiredAltMSL = landing_point.alt;
				mhc.desiredHdgDeg = landing_point.hdg;
				mhc.horDistTolFt = DEFAULT_LAND_CRITERIA_HOR_DIST;
				mhc.vertDistTolFt = DEFAULT_LAND_CRITERIA_VERT_DIST;
				mhc.velTolKts = DEFAULT_LAND_CRITERIA_VEL;
				mhc.hdgToleranceDeg = DEFAULT_LAND_CRITERIA_HDG;
				mhc.timeTol = DEFAULT_LAND_CRITERIA_TIME;
				mhc.timeCritFirstMet = -1;
				mhc.hoverCompleted = 0;
				mhc.manCtrlRequiredAfterCritMet = 0;

                message_update_gcs_t temp_gcs_update{"Repositioning to LP!", Mav_Severities_E::MAV_SEVERITY_ALERT};

                message_boss_mission_update_t temp_boss_update{};
                temp_boss_update.update_landing_point(
                        landing_point.id,
                        landing_point.lat,
                        landing_point.lon,
                        landing_point.alt * FT_TO_METERS,
                        landing_point.hdg,
                        "LP REP"
                        );
                temp_boss_update.missionNo = mission_number;
                temp_boss_update.missionItemNo = landing_point.missionItemNo;
                temp_boss_update.speed = velocity * MPS_TO_KTS;
				bag_port_hover_out.push_back(mhc);

				mission_monitor_messages.emplace_back(0);
				boss_messages.push_back(temp_boss_update);
				gcs_messages.push_back(temp_gcs_update);

				cadmium::get_messages<typename Command_Reposition::defs::o_stabilize>(bags) = bag_port_hover_out;
				cadmium::get_messages<typename Command_Reposition::defs::o_set_mission_monitor_status>(bags) = mission_monitor_messages;
				cadmium::get_messages<typename Command_Reposition::defs::o_update_boss>(bags) = boss_messages;
				cadmium::get_messages<typename Command_Reposition::defs::o_update_gcs>(bags) = gcs_messages;
			}
				break;
			case States::CANCEL_HOVER:
				bag_port_out.push_back(true);
				cadmium::get_messages<typename Command_Reposition::defs::o_cancel_hover>(bags) = bag_port_out;
				break;
			case States::LP_CRITERIA_MET:
				bag_port_LP_out.emplace_back(landing_point);
				cadmium::get_messages<typename Command_Reposition::defs::o_lp_criteria_met>(bags) = bag_port_LP_out;
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
	/// Varible for storing the current landing points being repositioned to.
	message_landing_point_t landing_point;
	/// Varible for storing aircraft state when scheduling repostion velocities.
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
