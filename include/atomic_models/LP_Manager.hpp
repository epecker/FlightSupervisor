/**
* James Horner and Tanner Trautrim
*
*/

#ifndef LP_MANAGER_HPP
#define LP_MANAGER_HPP

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

#include <limits>
#include <assert.h>
#include <string>

#include "message_structures/message_hover_criteria_t.hpp"
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
using namespace std;

template<typename TIME>
class LP_Manager {
public:
	// Enum of the automata-like states of the atomic model.
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(WAYPOINT_MET)
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
		typename LP_Manager<TIME>::defs::i_aircraft_state,
		typename LP_Manager<TIME>::defs::i_control_yielded,
		typename LP_Manager<TIME>::defs::i_fcc_command_land,
		typename LP_Manager<TIME>::defs::i_lp_recv,
		typename LP_Manager<TIME>::defs::i_pilot_takeover,
		typename LP_Manager<TIME>::defs::i_plp_ach
	>;

	using output_ports = tuple<
		typename LP_Manager<TIME>::defs::o_fcc_command_orbit,
		typename LP_Manager<TIME>::defs::o_lp_expired,
		typename LP_Manager<TIME>::defs::o_lp_new,
		typename LP_Manager<TIME>::defs::o_pilot_handover,
		typename LP_Manager<TIME>::defs::o_request_aircraft_state,
		typename LP_Manager<TIME>::defs::o_set_mission_monitor_status,
		typename LP_Manager<TIME>::defs::o_update_boss,
		typename LP_Manager<TIME>::defs::o_update_gcs
	>;

	// state definition
	struct state_type {
		States current_state;
	};
	state_type state;

	// Public members of the class
	bool lp_recvd;
	bool first_lp;
	int lp_count;
	message_landing_point_t lp;
	message_landing_point_t plp;
	message_aircraft_state_t aircraft_state;
	TIME lp_accept_time_prev;
	TIME orbit_time;

	// Default constructor
	LP_Manager() {
		state.current_state = States::WAYPOINT_MET;
		lp_accept_time_prev = seconds_to_time<TIME>(LP_ACCEPT_TIMER);
		orbit_time = seconds_to_time<TIME>(ORBIT_TIMER);
		lp_recvd = false;
		first_lp = false;
		lp_count = 0;
		lp = message_landing_point_t();
		plp = message_landing_point_t();
		aircraft_state = message_aircraft_state_t();
	}

	// Constructor with timer parameter
	LP_Manager(TIME i_lp_accept_time, TIME i_orbit_time) {
		state.current_state = States::WAYPOINT_MET;
		lp_accept_time_prev = i_lp_accept_time;
		orbit_time = i_orbit_time;
		lp_recvd = false;
		first_lp = false;
		lp_count = 0;
		lp = message_landing_point_t();
		plp = message_landing_point_t();
	}

	// Constructor with timer parameter and initial state parameter for debugging or partial execution startup.
	LP_Manager(TIME i_lp_accept_time, TIME i_orbit_time, States initial_state) {
		state.current_state = initial_state;
		lp_accept_time_prev = i_lp_accept_time;
		orbit_time = i_orbit_time;
		lp_recvd = false;
		first_lp = false;
		lp_count = 0;
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
		if (get_messages<typename LP_Manager<TIME>::defs::i_pilot_takeover>(mbs).size() >= 1 && state.current_state != States::HANDOVER_CONTROL)
			state.current_state = States::PILOT_CONTROL;

		if (state.current_state == States::LP_APPROACH && !get_messages<typename LP_Manager<TIME>::defs::i_fcc_command_land>(mbs).empty()) {
			state.current_state = States::LP_ACCEPT_EXP;
		}

		//If we are in a state that can receive a landing point input,
		if (state.current_state == States::WAYPOINT_MET || state.current_state == States::LZE_SCAN || state.current_state == States::LP_APPROACH) {
			//If there are landing points that have been received,
			if (get_messages<typename LP_Manager<TIME>::defs::i_lp_recv>(mbs).size() >= 1) {
				//Store the landing points in a vector.
				vector<message_landing_point_t> landing_points = get_messages<typename LP_Manager<TIME>::defs::i_lp_recv>(mbs);

				//Create a flag for if one of them is a valid landing point to be transitioned to.
				bool valid_lp_recv = false;

				//If there was a previous landing point,
				if (lp_recvd) {
					//For each of the landing points received,
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
							valid_lp_recv = true;
							first_lp = false;
							lp_count++;
							lp.id = lp_count;
							break;
						}
					}
				}
				//If this is the first bag of landing points that have been received,
				else {
					//Pick the first landing point in the list.
					lp = landing_points[0];
					valid_lp_recv = true;
					lp_recvd = true;
					first_lp = true;
					lp_count++;
					lp.id = lp_count;
				}

				//If a valid landing point was identified out of the list of landing points,
				//Transition into the notify reposition loop state.
				state.current_state = States::REQUEST_STATE_LP;
			}
		}

		bool received_aircraft_state;
		switch (state.current_state) {
			//If we are in a state that can receive a planned landing point acheived input,
			case States::WAYPOINT_MET:
				if (get_messages<typename LP_Manager<TIME>::defs::i_plp_ach>(mbs).size() >= 1) {
					state.current_state = States::REQUEST_STATE_PLP;
					plp = get_messages<typename LP_Manager<TIME>::defs::i_plp_ach>(mbs)[0];
				}
				break;
			case States::GET_STATE_PLP:
				received_aircraft_state = get_messages<typename LP_Manager<TIME>::defs::i_aircraft_state>(mbs).size() >= 1;

				if (received_aircraft_state) {
					vector<message_aircraft_state_t> new_aircraft_state = get_messages<typename LP_Manager<TIME>::defs::i_aircraft_state>(mbs);
					aircraft_state = new_aircraft_state[0];
					if (aircraft_state.alt_AGL < DEFAULT_HOVER_ALTITUDE_AGL) {
						plp.alt = (aircraft_state.alt_MSL - aircraft_state.alt_AGL + DEFAULT_HOVER_ALTITUDE_AGL);
					} else {
						plp.alt = aircraft_state.alt_MSL;
					}
					state.current_state = States::START_LZE_SCAN;
				}
				break;
			case States::GET_STATE_LP:
				received_aircraft_state = get_messages<typename LP_Manager<TIME>::defs::i_aircraft_state>(mbs).size() >= 1;

				if (received_aircraft_state) {
					vector<message_aircraft_state_t> new_aircraft_state = get_messages<typename LP_Manager<TIME>::defs::i_aircraft_state>(mbs);
					message_aircraft_state_t aircraft_state = new_aircraft_state[0];
					if (aircraft_state.alt_AGL < DEFAULT_HOVER_ALTITUDE_AGL) {
						lp.alt = (aircraft_state.alt_MSL - aircraft_state.alt_AGL + DEFAULT_HOVER_ALTITUDE_AGL);
					} else {
						lp.alt = aircraft_state.alt_MSL;
					}
					state.current_state = States::NOTIFY_LP;
				}
				break;

			//If we are in a state that can receive a control yielded input,
			case States::HANDOVER_CONTROL:
				if (get_messages<typename LP_Manager<TIME>::defs::i_control_yielded>(mbs).size() >= 1) {
					state.current_state = States::PILOT_CONTROL;
				}
				break;

			default:
				break;
		}
		if (
			state.current_state == States::REQUEST_STATE_LP ||
			state.current_state == States::GET_STATE_LP ||
			state.current_state == States::NOTIFY_LP ||
			state.current_state == States::LP_APPROACH
		) {
			lp_accept_time_prev = lp_accept_time_prev - e;
			if (lp_accept_time_prev <= TIME(TA_ZERO)) {
				lp_accept_time_prev = TIME(TA_ZERO);
			}
		}
	}

	// confluence transition
	void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		//If the external input is a pilot takeover messasge,
		if (get_messages<typename LP_Manager<TIME>::defs::i_pilot_takeover>(mbs).size() >= 1) {
			//Execute the external transition first, then the internal.
			external_transition(TIME(), move(mbs));
			internal_transition();
		} else {
			internal_transition();
			external_transition(TIME(), move(mbs));
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
						plp.alt,
						DEFAULT_ORBIT_RADIUS,
						DEFAULT_ORBIT_VELOCITY,
						DEFAULT_ORBIT_YAW_BEHAVIOUR
					);

					message_update_gcs_t temp_gcs_update;
					temp_gcs_update.text = "Starting an orbit to scan LZ";
					temp_gcs_update.severity = Mav_Severities_E::MAV_SEVERITY_INFO;

					message_boss_mission_update_t temp_boss = message_boss_mission_update_t();
					strcpy(temp_boss.description, "LZ scan");

					fcc_messages.push_back(temp_fcc_command);
					boss_messages.push_back(temp_boss);
					gcs_messages.push_back(temp_gcs_update);
					mission_monitor_messages.emplace_back(0);

					get_messages<typename LP_Manager<TIME>::defs::o_fcc_command_orbit>(bags) = fcc_messages;
					get_messages<typename LP_Manager<TIME>::defs::o_update_boss>(bags) = boss_messages;
					get_messages<typename LP_Manager<TIME>::defs::o_update_gcs>(bags) = gcs_messages;
					get_messages<typename LP_Manager<TIME>::defs::o_set_mission_monitor_status>(bags) = mission_monitor_messages;
				}
				break;

			case States::LZE_SCAN:
				{
					message_update_gcs_t temp_gcs_update;
					temp_gcs_update.text = "Landing point not found. Hovering over PLP";
					temp_gcs_update.severity = Mav_Severities_E::MAV_SEVERITY_ALERT;
					message_boss_mission_update_t temp_boss = message_boss_mission_update_t();
					strcpy(temp_boss.description, "PLP rep");
					boss_messages.push_back(temp_boss);
					gcs_messages.push_back(temp_gcs_update);
					plp_messages.push_back(plp);
					get_messages<typename LP_Manager<TIME>::defs::o_update_boss>(bags) = boss_messages;
					get_messages<typename LP_Manager<TIME>::defs::o_update_gcs>(bags) = gcs_messages;
					get_messages<typename LP_Manager<TIME>::defs::o_pilot_handover>(bags) = plp_messages;

				}
				break;

			case States::NOTIFY_LP:
				{
					if (first_lp) {
						message_update_gcs_t temp_gcs_update_1;
						message_update_gcs_t temp_gcs_update_2;

						temp_gcs_update_1.text = "LP timer started";
						temp_gcs_update_1.severity = Mav_Severities_E::MAV_SEVERITY_INFO;

						temp_gcs_update_2.text = "LP found. Holding";
						temp_gcs_update_2.severity = Mav_Severities_E::MAV_SEVERITY_ALERT;

						gcs_messages.push_back(temp_gcs_update_1);
						gcs_messages.push_back(temp_gcs_update_2);

						get_messages<typename LP_Manager<TIME>::defs::o_update_gcs>(bags) = gcs_messages;
					}
					message_boss_mission_update_t temp_boss;

					temp_boss.lpNo = lp_count;
					temp_boss.lpLat = lp.lat;
					temp_boss.lpLon = lp.lon;
					temp_boss.alt = lp.alt;
					temp_boss.yaw = lp.hdg;
					strcpy(temp_boss.description, "LP UPD");

					boss_messages.push_back(temp_boss);
					lp_messages.push_back(lp);
					
					get_messages<typename LP_Manager<TIME>::defs::o_update_boss>(bags) = boss_messages;
					get_messages<typename LP_Manager<TIME>::defs::o_lp_new>(bags) = lp_messages;
				}
				break;

			case States::LP_APPROACH:
				{
					message_update_gcs_t temp_gcs_update;
					temp_gcs_update.text = "LP accept timer expired";
					temp_gcs_update.severity = Mav_Severities_E::MAV_SEVERITY_INFO;
					gcs_messages.push_back(temp_gcs_update);
					lp_messages.push_back(lp);
					get_messages<typename LP_Manager<TIME>::defs::o_lp_expired>(bags) = lp_messages;
					get_messages<typename LP_Manager<TIME>::defs::o_update_gcs>(bags) = gcs_messages;
				}
				break;

			case States::REQUEST_STATE_LP: case States::REQUEST_STATE_PLP:
				{
					bool_messages.push_back(true);
					get_messages<typename LP_Manager<TIME>::defs::o_request_aircraft_state>(bags) = bool_messages;
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
			case States::WAYPOINT_MET: case States::HANDOVER_CONTROL: case States::PILOT_CONTROL: case States::LP_ACCEPT_EXP:
				next_internal = numeric_limits<TIME>::infinity();
				break;

			case States::START_LZE_SCAN: case States::NOTIFY_LP: case States::REQUEST_STATE_LP: case States::REQUEST_STATE_PLP:
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
				next_internal = numeric_limits<TIME>::infinity();
		}
		return next_internal;
	}

	friend ostringstream& operator<<(ostringstream& os, const typename LP_Manager<TIME>::state_type& i) {
		os << (string("State: ") + enumToString(i.current_state) + string("\n"));
		return os;
	}
};

#endif // LP_MANAGER_HPP