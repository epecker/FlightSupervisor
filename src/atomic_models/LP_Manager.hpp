/**
 * 	\file		LP_Manager.hpp
 *	\brief		Definition of the LP Manager atomic model.
 *	\details	This header file defines the Landing Point Manager atomic model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor at the start
				of the landing phase, when either the planned landing point is achieved or a landing point
				is received.
 *	\image		html atomic_models/lp_manager.png
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef LP_MANAGER_HPP
#define LP_MANAGER_HPP

// Messages structures
#include "../message_structures/message_landing_point_t.hpp"
#include "../message_structures/message_aircraft_state_t.hpp"
#include "../message_structures/message_boss_mission_update_t.hpp"
#include "../message_structures/message_update_gcs_t.hpp"
#include "../message_structures/message_fcc_command_t.hpp"

// Utility functions
#include "../enum_string_conversion.hpp"
#include "../time_conversion.hpp"
#include <mavNRC/geo.h>
#include "../Constants.hpp"

// Cadmium Simulator Headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

// System Libraries
#include <limits> // Used to set the time advance to infinity
#include <cassert>
#include <string>

/**
 * 	\class		LP_Manager
 *	\brief		Definition of the LP Manager atomic model.
 *	\details	This header file defines the Landing Point Manager atomic model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor at the start
				of the landing phase, when either the planned landing point is achieved or a landing point
				is received.
 *	\image		html atomic_models/lp_manager.png
 */
template<typename TIME>
class LP_Manager {
public:
	/**
	 *	\par	States
	 * 	Declaration of the states of the atomic model.
	 */
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(WAIT_LP_PLP)
		(REQUEST_STATE_PLP)
		(GET_STATE_PLP)
		(REQUEST_STATE_LP)
		(GET_STATE_LP)
		(START_LZE_SCAN)
		(LZE_SCAN)
		(HANDOVER_CONTROL)
		(PILOT_CONTROL)
		(NOTIFY_LP)
		(LP_APPROACH)
		(LP_ACCEPT_EXP)
	);

	/**
	 *	\brief	For definition of the input and output ports see:
	 *	\ref 	LP_Manager_input_ports "Input Ports" and
	 *	\ref 	LP_Manager_output_ports "Output Ports"
	 * 	\note 	All input and output ports must be listed in this struct.
	 */
	struct defs {
		struct i_aircraft_state : public cadmium::in_port<message_aircraft_state_t> {};
		struct i_control_yielded : public cadmium::in_port<bool> {};
		struct i_fcc_command_land : public cadmium::in_port<message_fcc_command_t> {};
		struct i_lp_recv : public cadmium::in_port<message_landing_point_t> {};
		struct i_pilot_takeover : public cadmium::in_port<bool> {};
		struct i_plp_ach : public cadmium::in_port<message_landing_point_t> {};
		struct i_start_mission : public cadmium::in_port<int> {};

		struct o_fcc_command_orbit : public cadmium::out_port<message_fcc_command_t> {};
		struct o_lp_expired : public cadmium::out_port<message_landing_point_t> {};
		struct o_lp_new : public cadmium::out_port<message_landing_point_t> {};
		struct o_pilot_handover : public cadmium::out_port<message_landing_point_t> {};
		struct o_request_aircraft_state : public cadmium::out_port<bool> {};
		struct o_set_mission_monitor_status : public cadmium::out_port<uint8_t> {};
		struct o_update_boss : public cadmium::out_port<message_boss_mission_update_t> {};
		struct o_update_gcs : public cadmium::out_port<message_update_gcs_t> {};
	};

	/**
	 * 	\anchor	LP_Manager_input_ports
	 *	\par	Input Ports
	 * 	Defintion of the input ports for the model.
	 * 	\param 	i_aircraft_state 	Port for receiving the current state of the aircraft.
	 * 	\param	i_control_yielded	Port for receiving signal indicating control has been handed over to the pilot
	 * 	\param	i_fcc_command_land	Port for receiving notification that a landing will be attempted.
	 * 	\param	i_lp_recv			Port for receiving landing points from the perception system.
	 * 	\param 	i_pilot_takeover 	Port for receiving signal indicating that the pilot has taken control from the supervisor.
	 * 	\param	i_plp_ach			Port for receiving signal indicating that the planned landing point has been achieved.
	 * 	\param 	i_start_mission 	Port for receiving signal indicating the mission has started.
	 */
	using input_ports = std::tuple<
		typename defs::i_aircraft_state,
		typename defs::i_control_yielded,
		typename defs::i_fcc_command_land,
		typename defs::i_lp_recv,
		typename defs::i_pilot_takeover,
		typename defs::i_plp_ach,
		typename defs::i_start_mission
	>;

	/**
	 *	\anchor	LP_Manager_output_ports
	 * 	\par 	Output Ports
	 * 	Defintion of the output ports for the model.
	 * 	\param	o_fcc_command_orbit				Port for sending orbit commands to the FCC.
	 * 	\param	o_lp_expired					Port for sending notifcation that the LP accept timer has expired.
	 * 	\param	o_lp_new						Port for sending new valid landing points.
	 * 	\param	o_pilot_handover				Port for requesting that control be handed over to the pilot.
	 * 	\param	o_request_aircraft_state 		Port for requesting the current aircraft state.
	 * 	\param	o_set_mission_monitor_status 	Port for telling the mission monitor to stop monitoring mission progress.
	 * 	\param	o_update_boss 					Port for sending updates to BOSS.
	 * 	\param	o_update_gcs 					Port for sending updates to the GCS.
	 */
	using output_ports = std::tuple<
		typename defs::o_fcc_command_orbit,
		typename defs::o_lp_expired,
		typename defs::o_lp_new,
		typename defs::o_pilot_handover,
		typename defs::o_request_aircraft_state,
		typename defs::o_set_mission_monitor_status,
		typename defs::o_update_boss,
		typename defs::o_update_gcs
	>;

	/**
	 *	\anchor	LP_Manager_state_type
	 *	\par	State
	 * 	Defintion of the states of the atomic model.
	 * 	\param 	current_state 	Current state of atomic model.
	 */
	struct state_type {
		States current_state;
	} state;

	/**
	 * \brief 	Default constructor for the model.
	 */
	LP_Manager() {
        first_waypoint_number = -1;
		state.current_state = States::IDLE;
		lp_accept_time_prev = seconds_to_time<TIME>(LP_ACCEPT_TIMER);
		orbit_time = seconds_to_time<TIME>(ORBIT_TIMER);
		lp_count = 0;
        mission_number = 0;
		lp = message_landing_point_t();
		plp = message_landing_point_t();
		aircraft_state = message_aircraft_state_t();
	}

	/**
	 * \brief 	Constructor for the model with parameters for the length of the accept and orbit timers.
	 * \param	i_lp_accept_time	TIME length of the LP accept timer.
	 * \param	i_orbit_time		TIME length of the orbit timer.
	 */
	LP_Manager(TIME i_lp_accept_time, TIME i_orbit_time) {
        first_waypoint_number = -1;
		state.current_state = States::IDLE;
		lp_accept_time_prev = i_lp_accept_time;
		orbit_time = i_orbit_time;
		lp_count = 0;
        mission_number = 0;
		lp = message_landing_point_t();
		plp = message_landing_point_t();
	}

	/**
	 * \brief 	Constructor for the model with initial state parameter
	 * 			for debugging or partial execution startup.
	 * \param	i_lp_accept_time	TIME length of the LP accept timer.
	 * \param	i_orbit_time		TIME length of the orbit timer.
	 * \param	initial_state	States initial state of the model.
	 */
	LP_Manager(TIME i_lp_accept_time, TIME i_orbit_time, States initial_state) {
        first_waypoint_number = -1;
		state.current_state = initial_state;
		lp_accept_time_prev = i_lp_accept_time;
		orbit_time = i_orbit_time;
		lp_count = 0;
        mission_number = 0;
		lp = message_landing_point_t();
		plp = message_landing_point_t();
	}

	/// Internal transitions of the model
	void internal_transition() {
		switch (state.current_state) {
			case States::START_LZE_SCAN:
				state.current_state = States::LZE_SCAN;
				break;
			case States::REQUEST_STATE_LP:
				state.current_state = States::GET_STATE_LP;
				break;
			case States::REQUEST_STATE_PLP:
				state.current_state = States::GET_STATE_PLP;
				break;
			case States::LZE_SCAN:
				state.current_state = States::HANDOVER_CONTROL;
				break;
			case States::NOTIFY_LP:
				state.current_state = States::LP_APPROACH;
				break;
			case States::LP_APPROACH:
				state.current_state = States::LP_ACCEPT_EXP;
				break;
			default:
				break;
		}
	}

	/// External transitions of the model
	void external_transition(TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		//If we get any messages on the pilot takeover port in any state (apart from HANDOVER_CONTROL), immediately transition into the pilot in control state.
        bool received_pilot_takeover = !cadmium::get_messages<typename defs::i_pilot_takeover>(mbs).empty();
		if (received_pilot_takeover && state.current_state != States::HANDOVER_CONTROL) {
			state.current_state = States::PILOT_CONTROL;
            return;
        }

        bool received_start_mission = !cadmium::get_messages<typename defs::i_start_mission>(mbs).empty();
        if (received_start_mission) {
            reset_state();
            mission_number = cadmium::get_messages<typename defs::i_start_mission>(mbs).back();
            state.current_state = States::WAIT_LP_PLP;
            return;
        }

        update_lp_accept_time(e);

        switch (state.current_state) {
            case States::WAIT_LP_PLP: {
                bool received_lp = !cadmium::get_messages<typename defs::i_lp_recv>(mbs).empty();
                bool received_plp_ach = !cadmium::get_messages<typename defs::i_plp_ach>(mbs).empty();
                if (received_lp) {
                    set_lp_if_valid(&mbs);
                    if (received_plp_ach) {
                        first_waypoint_number = lp.missionItemNo;
                    } else {
                        first_waypoint_number = lp.missionItemNo + 1;
                    }
                    lp.missionItemNo = first_waypoint_number;
                    state.current_state = States::REQUEST_STATE_LP;
                } else if (received_plp_ach) {
                    plp = cadmium::get_messages<typename defs::i_plp_ach>(mbs)[0];
                    first_waypoint_number = plp.missionItemNo;
                    state.current_state = States::REQUEST_STATE_PLP;
                }
                break;
            }
            case States::LZE_SCAN: {
                bool received_lp = !cadmium::get_messages<typename defs::i_lp_recv>(mbs).empty();
                if (received_lp) {
                    set_lp_if_valid(&mbs);
                    state.current_state = States::REQUEST_STATE_LP;
                }
                break;
            }
            case States::GET_STATE_PLP: {
                bool received_aircraft_state = !cadmium::get_messages<typename defs::i_aircraft_state>(mbs).empty();

                if (received_aircraft_state) {
                    std::vector<message_aircraft_state_t> new_aircraft_state = cadmium::get_messages<typename defs::i_aircraft_state>(
                            mbs);
                    aircraft_state = new_aircraft_state[0];
                    if (aircraft_state.alt_AGL < DEFAULT_HOVER_ALTITUDE_AGL) {
                        plp.alt = (aircraft_state.alt_MSL - aircraft_state.alt_AGL + DEFAULT_HOVER_ALTITUDE_AGL);
                    } else {
                        plp.alt = aircraft_state.alt_MSL;
                    }
                    state.current_state = States::START_LZE_SCAN;
                }
                break;
            }
            case States::GET_STATE_LP: {
                bool received_aircraft_state = !cadmium::get_messages<typename defs::i_aircraft_state>(mbs).empty();

                if (received_aircraft_state) {
                    std::vector<message_aircraft_state_t> new_aircraft_state = cadmium::get_messages<typename defs::i_aircraft_state>(
                            mbs);
                    aircraft_state = new_aircraft_state[0];
                    if (aircraft_state.alt_AGL < DEFAULT_HOVER_ALTITUDE_AGL) {
                        lp.alt = (aircraft_state.alt_MSL - aircraft_state.alt_AGL + DEFAULT_HOVER_ALTITUDE_AGL);
                    } else {
                        lp.alt = aircraft_state.alt_MSL;
                    }
                    state.current_state = States::NOTIFY_LP;
                }
                break;
            }
            case States::HANDOVER_CONTROL: {
                bool received_control_yielded = !cadmium::get_messages<typename defs::i_control_yielded>(mbs).empty();
                if (received_control_yielded) {
                    state.current_state = States::PILOT_CONTROL;
                }
                break;
            }
            case States::LP_APPROACH: {
                bool received_command_land = !cadmium::get_messages<typename defs::i_fcc_command_land>(mbs).empty();
                bool received_lp = !cadmium::get_messages<typename defs::i_lp_recv>(mbs).empty();
                if (received_command_land) {
                    state.current_state = States::LP_ACCEPT_EXP;
                } else if (received_lp) {
                    set_lp_if_valid(&mbs);
                    lp.missionItemNo = first_waypoint_number;
                    state.current_state = States::REQUEST_STATE_LP;
                }
                break;
            }
            default:
                break;
        }
	}

	/// Function used to decide precedence between internal and external transitions when both are scheduled simultaneously.
	void confluence_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		//If the external input is a pilot takeover message,
		if (cadmium::get_messages<typename defs::i_pilot_takeover>(mbs).size() >= 1) {
			//Execute the external transition first, then the internal.
			external_transition(TIME(), std::move(mbs));
			internal_transition();
		} else {
			internal_transition();
			external_transition(TIME(), std::move(mbs));
		}
	}

	/// Function for generating output from the model before internal transitions.
	typename cadmium::make_message_bags<output_ports>::type output() const {
		typename cadmium::make_message_bags<output_ports>::type bags;

		switch (state.current_state) {
			case States::START_LZE_SCAN: {
				message_fcc_command_t fcc_command = message_fcc_command_t();
				fcc_command.orbit(
						aircraft_state.gps_time,
						plp.lat * (1E7),
						plp.lon * (1E7),
						plp.alt * FT_TO_METERS,
						DEFAULT_ORBIT_RADIUS,
						DEFAULT_ORBIT_VELOCITY,
						DEFAULT_ORBIT_YAW_BEHAVIOUR
				);
				cadmium::get_messages<typename defs::o_fcc_command_orbit>(bags).push_back(fcc_command);

				// Update the ground control computer
				cadmium::get_messages<typename defs::o_update_gcs>(bags).emplace_back(
						"Starting an orbit to scan LZ",
						Mav_Severities_E::MAV_SEVERITY_INFO
				);

				// Update BOSS display message
				cadmium::get_messages<typename defs::o_update_boss>(bags).emplace_back(
                        mission_number,
                        plp.missionItemNo,
                        plp.lat,
                        plp.lon,
                        plp.alt * FT_TO_METERS,
                        plp.hdg,
                        0.1, // If set to 0 displays a doghouse, else displays a circle
                        DEFAULT_ACCEPTANCE_RADIUS_HORZ,
                        0,
                        "LZ SCAN"
				);

				cadmium::get_messages<typename defs::o_set_mission_monitor_status>(bags).emplace_back(0);
				break;
			}
			case States::LZE_SCAN:
				{
					// Update the ground control computer
					cadmium::get_messages<typename defs::o_update_gcs>(bags).emplace_back(
							"Landing point not found. Hovering over PLP",
							Mav_Severities_E::MAV_SEVERITY_ALERT
					);

					// Update BOSS display message
					cadmium::get_messages<typename defs::o_update_boss>(bags).emplace_back(
                            mission_number,
                            plp.missionItemNo,
                            plp.lat,
                            plp.lon,
                            plp.alt * FT_TO_METERS,
                            plp.hdg,
                            0.1,
                            DEFAULT_ACCEPTANCE_RADIUS_HORZ,
                            0,
                            "MAN CTRL"
					);

					cadmium::get_messages<typename defs::o_pilot_handover>(bags).push_back(plp);

				}
				break;
			case States::NOTIFY_LP:
				{
					if (lp_count == 0) {
						cadmium::get_messages<typename defs::o_update_gcs>(bags).emplace_back(
								"LP timer started",
								Mav_Severities_E::MAV_SEVERITY_INFO
						);
					}
					cadmium::get_messages<typename defs::o_lp_new>(bags).push_back(lp);
				}
				break;
			case States::LP_APPROACH:
				{
					cadmium::get_messages<typename defs::o_lp_expired>(bags).push_back(lp);

					// Update the ground control computer
					cadmium::get_messages<typename defs::o_update_gcs>(bags).emplace_back(
							"LP accept timer expired",
							Mav_Severities_E::MAV_SEVERITY_INFO
					);
				}
				break;
			case States::REQUEST_STATE_LP: case States::REQUEST_STATE_PLP:
				{
					cadmium::get_messages<typename defs::o_request_aircraft_state>(bags).emplace_back(true);
				}
				break;
			default:
				assert(false && "Unhandled output after internal transition.");
				break;
		}

		return bags;
	}

	/// Function to declare the time advance value for each state of the model.
	TIME time_advance() const {
		TIME next_internal;

		switch (state.current_state) {
            case States::IDLE:
            case States::WAIT_LP_PLP:
            case States::GET_STATE_PLP:
            case States::GET_STATE_LP:
            case States::HANDOVER_CONTROL:
            case States::PILOT_CONTROL:
            case States::LP_ACCEPT_EXP:
				next_internal = std::numeric_limits<TIME>::infinity();
				break;
            case States::START_LZE_SCAN:
            case States::NOTIFY_LP:
            case States::REQUEST_STATE_LP:
            case States::REQUEST_STATE_PLP:
				next_internal = TIME(TA_ZERO);
				break;
			case States::LZE_SCAN:
				next_internal = orbit_time;
				break;
			case States::LP_APPROACH:
				//Schedule the amount of time that was left on the LP accept timer.
				next_internal = lp_accept_time_prev;
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
	friend std::ostringstream& operator<<(std::ostringstream& os, const typename LP_Manager<TIME>::state_type& i) {
		os << (std::string("State: ") + enumToString(i.current_state) + std::string("\n"));
		return os;
	}

private:
    /// Required to display the landing point doghouses.
    int first_waypoint_number;
    /// Variable to count the number of valid LPs that have been sent.
    int lp_count;
    /// Variable for storing the number of the mission for updating BOSS.
    int mission_number;
    /// Variable for storing the location of the current valid landing point.
    message_landing_point_t lp;
    /// Variable for storing the location of the planned landing point.
    message_landing_point_t plp;
    /// Variable for storing the current aircraft state.
    message_aircraft_state_t aircraft_state;
    /// Variable to store the remaining amount of time on the LP accept timer for decrementing upon new LP received.
    TIME lp_accept_time_prev;
	/// Variable to store the length of the orbit timer.
    TIME orbit_time;

	/**
	 *	\brief 		Function set_lp_if_valid is used to set the current valid landing point.
	 *	\details	The function checks if any landing points have been received, if none have the most
	 * 				recent is selected. If there have been multiple valid LPs, the most recent valid LP is
	 * 				chosen. An LP is considered valid if it has a large enough separation from the previous LP.
	 * 	\param		mbs	Bag of messages received on the input ports received from Cadmium simulation engine.
	 */
    void set_lp_if_valid(typename cadmium::make_message_bags<input_ports>::type * mbs) {
        std::vector<message_landing_point_t> landing_points = cadmium::get_messages<typename defs::i_lp_recv>(*mbs);

        if (lp_count == 0) {
            lp = landing_points.back(); // Pick the newest landing point for the first new LP (found at the back of the vector of inputs)
            lp_count++;
            lp.id = lp_count;
        } else {
            for (message_landing_point_t new_lp : landing_points) {
                float distance_xy;
                float distance_z;
                get_distance_to_point_global_wgs84(
                        lp.lat, lp.lon, lp.alt,
                        new_lp.lat, new_lp.lon, new_lp.alt,
                        &distance_xy, &distance_z);
                bool new_lp_valid = (distance_xy > LP_SEPARATION);
                //If the landing point is far enough away from the previous landing point,
                if (new_lp_valid) {
                    //Set the current landing point to be the new landing point.
                    lp = new_lp;
                    lp_count++;
                    lp.id = lp_count;
                    break;
                }
            }
        }
    }

	/**
	 *	\brief 	Function update_lp_accept_time is used to update the LP accept timer based on the current state and an elapsed time.
	 */
    void update_lp_accept_time(TIME e) {
        if (state.current_state == States::REQUEST_STATE_LP ||
            state.current_state == States::GET_STATE_LP ||
            state.current_state == States::NOTIFY_LP ||
            state.current_state == States::LP_APPROACH) {
            lp_accept_time_prev = lp_accept_time_prev - e;
            if (lp_accept_time_prev <= TIME(TA_ZERO)) {
                lp_accept_time_prev = TIME(TA_ZERO);
            }
        }
    }

    /// Function reset_state resets the private variables to a default value.
    void reset_state() {
        lp_accept_time_prev = seconds_to_time<TIME>(LP_ACCEPT_TIMER);
        orbit_time = seconds_to_time<TIME>(ORBIT_TIMER);
        mission_number = 0;
        lp_count = 0;
    }
};

#endif // LP_MANAGER_HPP
