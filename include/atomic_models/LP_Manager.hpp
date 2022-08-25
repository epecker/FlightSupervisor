/**
 *	\brief		An atomic model representing the landing point manager model.
 *	\details	This header file defines the landing point manager model as
				an atomic model for use in the Cadmium DEVS
				simulation software.
 *	\author		James Horner
 *	\author		Tanner Trautrim
 */

#ifndef LP_MANAGER_HPP
#define LP_MANAGER_HPP

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

#include <limits>
#include <cassert>
#include <string>

#include "message_structures/message_landing_point_t.hpp"
#include "message_structures/message_aircraft_state_t.hpp"
#include "message_structures/message_boss_mission_update_t.hpp"
#include "message_structures/message_update_gcs_t.hpp"
#include "message_structures/message_fcc_command_t.hpp"

#include "enum_string_conversion.hpp"
#include "Constants.hpp"
#include "time_conversion.hpp"
#include "mavNRC/geo.h"

using namespace cadmium;

template<typename TIME>
class LP_Manager {
public:
	// Enum of the automata-like states of the atomic model.
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

	//Port definition
	struct defs {
		struct i_aircraft_state : public in_port<message_aircraft_state_t> {};
		struct i_control_yielded : public in_port<bool> {};
		struct i_fcc_command_land : public in_port<message_fcc_command_t> {};
		struct i_lp_recv : public in_port<message_landing_point_t> {};
		struct i_pilot_takeover : public in_port<bool> {};
		struct i_plp_ach : public in_port<message_landing_point_t> {};
		struct i_start_mission : public in_port<int> {};

		struct o_fcc_command_orbit : public out_port<message_fcc_command_t> {};
		struct o_lp_expired : public out_port<message_landing_point_t> {};
		struct o_lp_new : public out_port<message_landing_point_t> {};
		struct o_pilot_handover : public out_port<message_landing_point_t> {};
		struct o_request_aircraft_state : public out_port<bool> {};
		struct o_set_mission_monitor_status : public out_port<uint8_t> {};
		struct o_update_boss : public out_port<message_boss_mission_update_t> {};
		struct o_update_gcs : public out_port<message_update_gcs_t> {};
	};

	// ports definition
	using input_ports = tuple<
		typename defs::i_aircraft_state,
		typename defs::i_control_yielded,
		typename defs::i_fcc_command_land,
		typename defs::i_lp_recv,
		typename defs::i_pilot_takeover,
		typename defs::i_plp_ach,
		typename defs::i_start_mission
	>;

	using output_ports = tuple<
		typename defs::o_fcc_command_orbit,
		typename defs::o_lp_expired,
		typename defs::o_lp_new,
		typename defs::o_pilot_handover,
		typename defs::o_request_aircraft_state,
		typename defs::o_set_mission_monitor_status,
		typename defs::o_update_boss,
		typename defs::o_update_gcs
	>;

	// state definition
	struct state_type {
		States current_state;
	} state;

	// Default constructor
	LP_Manager() {
		state.current_state = States::IDLE;
		lp_accept_time_prev = seconds_to_time<TIME>(LP_ACCEPT_TIMER);
		orbit_time = seconds_to_time<TIME>(ORBIT_TIMER);
		lp_count = 0;
        mission_number = 0;
		lp = message_landing_point_t();
		plp = message_landing_point_t();
		aircraft_state = message_aircraft_state_t();
	}

	// Constructor with timer parameter
	LP_Manager(TIME i_lp_accept_time, TIME i_orbit_time) {
		state.current_state = States::IDLE;
		lp_accept_time_prev = i_lp_accept_time;
		orbit_time = i_orbit_time;
		lp_count = 0;
        mission_number = 0;
		lp = message_landing_point_t();
		plp = message_landing_point_t();
	}

	// Constructor with timer parameter and initial state parameter for debugging or partial execution startup.
	LP_Manager(TIME i_lp_accept_time, TIME i_orbit_time, States initial_state) {
		state.current_state = initial_state;
		lp_accept_time_prev = i_lp_accept_time;
		orbit_time = i_orbit_time;
		lp_count = 0;
        mission_number = 0;
		lp = message_landing_point_t();
		plp = message_landing_point_t();
	}

	// internal transition
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

	// external transition
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		//If we get any messages on the pilot takeover port in any state (apart from HANDOVER_CONTROL), immediately transition into the pilot in control state.
        bool received_pilot_takeover = !get_messages<typename defs::i_pilot_takeover>(mbs).empty();
		if (received_pilot_takeover && state.current_state != States::HANDOVER_CONTROL) {
			state.current_state = States::PILOT_CONTROL;
            return;
        }

        bool received_start_mission = !get_messages<typename defs::i_start_mission>(mbs).empty();
        if (received_start_mission) {
            reset_state();
            mission_number = get_messages<typename defs::i_start_mission>(mbs).back();
            state.current_state = States::WAIT_LP_PLP;
            return;
        }

        update_lp_accept_time(e);

        switch (state.current_state) {
            case States::WAIT_LP_PLP: {
                bool received_lp = !get_messages<typename defs::i_lp_recv>(mbs).empty();
                bool received_plp_ach = !get_messages<typename defs::i_plp_ach>(mbs).empty();
                if (received_lp) {
                    set_lp_if_valid(&mbs);
                    state.current_state = States::REQUEST_STATE_LP;
                } else if (received_plp_ach) {
                    state.current_state = States::REQUEST_STATE_PLP;
                    plp = get_messages<typename defs::i_plp_ach>(mbs)[0];
                }
                break;
            }
            case States::LZE_SCAN: {
                bool received_lp = !get_messages<typename defs::i_lp_recv>(mbs).empty();
                if (received_lp) {
                    set_lp_if_valid(&mbs);
                    state.current_state = States::REQUEST_STATE_LP;
                }
                break;
            }
            case States::GET_STATE_PLP: {
                bool received_aircraft_state = !get_messages<typename defs::i_aircraft_state>(mbs).empty();

                if (received_aircraft_state) {
                    vector<message_aircraft_state_t> new_aircraft_state = get_messages<typename defs::i_aircraft_state>(
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
                bool received_aircraft_state = !get_messages<typename defs::i_aircraft_state>(mbs).empty();

                if (received_aircraft_state) {
                    vector<message_aircraft_state_t> new_aircraft_state = get_messages<typename defs::i_aircraft_state>(
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
                bool received_control_yielded = !get_messages<typename defs::i_control_yielded>(mbs).empty();
                if (received_control_yielded) {
                    state.current_state = States::PILOT_CONTROL;
                }
                break;
            }
            case States::LP_APPROACH: {
                bool received_command_land = !get_messages<typename defs::i_fcc_command_land>(mbs).empty();
                bool received_lp = !get_messages<typename defs::i_lp_recv>(mbs).empty();
                if (received_command_land) {
                    state.current_state = States::LP_ACCEPT_EXP;
                } else if (received_lp) {
                    set_lp_if_valid(&mbs);
                    state.current_state = States::REQUEST_STATE_LP;
                }
                break;
            }
            default:
                break;
        }
	}

	// confluence transition
	void confluence_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {
		//If the external input is a pilot takeover message,
		if (get_messages<typename defs::i_pilot_takeover>(mbs).size() >= 1) {
			//Execute the external transition first, then the internal.
			external_transition(TIME(), std::move(mbs));
			internal_transition();
		} else {
			internal_transition();
			external_transition(TIME(), std::move(mbs));
		}
	}

	// output function
	typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		vector<message_landing_point_t> lp_messages;
		vector<message_landing_point_t> plp_messages;
		vector<bool> bool_messages;
		vector<message_boss_mission_update_t> boss_messages;
		vector<message_update_gcs_t> gcs_messages;
		vector<uint8_t> mission_monitor_messages;
		vector<message_fcc_command_t> fcc_messages;

		switch (state.current_state) {
			case States::START_LZE_SCAN:
				{
					message_fcc_command_t temp_fcc_command = message_fcc_command_t();
					temp_fcc_command.orbit(
						aircraft_state.gps_time,
						plp.lat * (1E7),
						plp.lon * (1E7),
						plp.alt * FT_TO_METERS,
						DEFAULT_ORBIT_RADIUS,
						DEFAULT_ORBIT_VELOCITY,
						DEFAULT_ORBIT_YAW_BEHAVIOUR
					);

					message_update_gcs_t temp_gcs_update;
					temp_gcs_update.text = "Starting an orbit to scan LZ";
					temp_gcs_update.severity = Mav_Severities_E::MAV_SEVERITY_INFO;

					message_boss_mission_update_t temp_boss{};
                    temp_boss.update_message("LZ SCAN", false, mission_number);
					temp_boss.alt = plp.alt * FT_TO_METERS;

					fcc_messages.push_back(temp_fcc_command);
					boss_messages.push_back(temp_boss);
					gcs_messages.push_back(temp_gcs_update);
					mission_monitor_messages.emplace_back(0);

					get_messages<typename defs::o_fcc_command_orbit>(bags) = fcc_messages;
					get_messages<typename defs::o_update_boss>(bags) = boss_messages;
					get_messages<typename defs::o_update_gcs>(bags) = gcs_messages;
					get_messages<typename defs::o_set_mission_monitor_status>(bags) = mission_monitor_messages;
				}
				break;
			case States::LZE_SCAN:
				{
					message_update_gcs_t temp_gcs_update{"Landing point not found. Hovering over PLP",
                                                         Mav_Severities_E::MAV_SEVERITY_ALERT};

					message_boss_mission_update_t temp_boss{};
                    temp_boss.update_message("PLP REP", false, mission_number);
					temp_boss.alt = plp.alt * FT_TO_METERS;

                    boss_messages.push_back(temp_boss);
					gcs_messages.push_back(temp_gcs_update);
					plp_messages.push_back(plp);
					get_messages<typename defs::o_update_boss>(bags) = boss_messages;
					get_messages<typename defs::o_update_gcs>(bags) = gcs_messages;
					get_messages<typename defs::o_pilot_handover>(bags) = plp_messages;

				}
				break;
			case States::NOTIFY_LP:
				{
					if (lp_count == 0) {
						message_update_gcs_t temp_gcs_update{"LP timer started", Mav_Severities_E::MAV_SEVERITY_INFO};
						get_messages<typename defs::o_update_gcs>(bags).push_back(temp_gcs_update);
					}
					get_messages<typename defs::o_lp_new>(bags).push_back(lp);
				}
				break;
			case States::LP_APPROACH:
				{
					message_update_gcs_t temp_gcs_update;
					temp_gcs_update.text = "LP accept timer expired";
					temp_gcs_update.severity = Mav_Severities_E::MAV_SEVERITY_INFO;
					gcs_messages.push_back(temp_gcs_update);
					lp_messages.push_back(lp);
					get_messages<typename defs::o_lp_expired>(bags) = lp_messages;
					get_messages<typename defs::o_update_gcs>(bags) = gcs_messages;
				}
				break;
			case States::REQUEST_STATE_LP: case States::REQUEST_STATE_PLP:
				{
					bool_messages.push_back(true);
					get_messages<typename defs::o_request_aircraft_state>(bags) = bool_messages;
				}
				break;
			default:
				assert(false && "Unhandled output after internal transition.");
				break;
		}

		return bags;
	}

	// time_advance function
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
				next_internal = numeric_limits<TIME>::infinity();
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

	friend ostringstream& operator<<(ostringstream& os, const typename LP_Manager<TIME>::state_type& i) {
		os << (string("State: ") + enumToString(i.current_state) + string("\n"));
		return os;
	}

private:
    int lp_count;
    int mission_number;
    message_landing_point_t lp;
    message_landing_point_t plp;
    message_aircraft_state_t aircraft_state;
    TIME lp_accept_time_prev;
    TIME orbit_time;

    void set_lp_if_valid(typename make_message_bags<input_ports>::type * mbs) {
        vector<message_landing_point_t> landing_points = get_messages<typename defs::i_lp_recv>(*mbs);

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

    void reset_state() {
        lp_accept_time_prev = seconds_to_time<TIME>(LP_ACCEPT_TIMER);
        orbit_time = seconds_to_time<TIME>(ORBIT_TIMER);
        mission_number = 0;
        lp_count = 0;
    }
};

#endif // LP_MANAGER_HPP
