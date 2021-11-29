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

#include "../../include/message_structures/message_hover_criteria_t.hpp"
#include "../../include/message_structures/message_mavlink_mission_item_t.hpp"
#include "../../include/message_structures/message_mavlink_mission_item_t.hpp"

#include "../../include/enum_string_conversion.hpp"
#include "../../include/Constants.hpp"
#include "../../include/time_conversion.hpp"

using namespace cadmium;
using namespace std;

//Port definition
struct LP_Manager_defs {
	struct i_lp_recv : public in_port<message_mavlink_mission_item_t> {};
	struct i_plp_ach : public in_port<message_mavlink_mission_item_t> {};
	struct i_pilot_takeover : public in_port<bool> {};
	struct i_hover_criteria_met : public in_port<bool> {};
	struct i_control_yielded : public in_port<bool> {};

	struct o_lp_new : public out_port<message_mavlink_mission_item_t> {};
	struct o_lp_expired : public out_port<message_mavlink_mission_item_t> {};
	struct o_pilot_handover : public out_port<message_mavlink_mission_item_t> {};
	struct o_stabilize : public out_port<message_hover_criteria_t> {};
	struct o_start_lze_scan : public out_port<bool> {};
};

template<typename TIME>
class LP_Manager {
public:
	// Enum of the automata-like states of the atomic model.
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(WAYPOINT_MET)
		(HOVER_PLP)
		(STABILIZING)
		(START_LZE_SCAN)
		(LZE_SCAN)
		(HANDOVER_CONTROL)
		(PILOT_CONTROL)
		(NOTIFY_LP)
		(LP_APPROACH)
		(LP_ACCEPT_EXP)
	);

	// ports definition
	using input_ports = tuple<
		typename LP_Manager_defs::i_lp_recv,
		typename LP_Manager_defs::i_plp_ach,
		typename LP_Manager_defs::i_pilot_takeover,
		typename LP_Manager_defs::i_hover_criteria_met,
		typename LP_Manager_defs::i_control_yielded
	>;

	using output_ports = tuple<
		typename LP_Manager_defs::o_lp_new,
		typename LP_Manager_defs::o_lp_expired,
		typename LP_Manager_defs::o_pilot_handover,
		typename LP_Manager_defs::o_stabilize,
		typename LP_Manager_defs::o_start_lze_scan
	>;

	// state definition
	struct state_type {
		States current_state;
	};
	state_type state;

	// Public members of the class
	bool lp_recvd;
	message_mavlink_mission_item_t lp;
	message_mavlink_mission_item_t plp;
	TIME lp_accept_time_prev;
	TIME orbit_time;

	// Default constructor
	LP_Manager() {
		state.current_state = States::WAYPOINT_MET;
		lp_accept_time_prev = seconds_to_time<TIME>(LP_ACCEPT_TIMER);
		orbit_time = seconds_to_time<TIME>(ORBIT_TIMER);
		lp_recvd = false;
		lp = message_mavlink_mission_item_t();
		plp = message_mavlink_mission_item_t();
	}

	// Constructor with timer parameter
	LP_Manager(TIME i_lp_accept_time, TIME i_orbit_time) {
		state.current_state = States::WAYPOINT_MET;
		lp_accept_time_prev = i_lp_accept_time;
		orbit_time = i_orbit_time;
		lp_recvd = false;
		lp = message_mavlink_mission_item_t();
		plp = message_mavlink_mission_item_t();
	}

	// Constructor with timer parameter and initial state parameter for debugging or partial execution startup.
	LP_Manager(TIME i_lp_accept_time, TIME i_orbit_time, States initial_state) {
		state.current_state = initial_state;
		lp_accept_time_prev = i_lp_accept_time;
		orbit_time = i_orbit_time;
		lp_recvd = false;
		lp = message_mavlink_mission_item_t();
		plp = message_mavlink_mission_item_t();
	}

	// internal transition
	void internal_transition() {
		switch (state.current_state) {
			case States::HOVER_PLP:
				state.current_state = States::STABILIZING;
				break;
			case States::START_LZE_SCAN:
				state.current_state = States::LZE_SCAN;
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
				assert(false && "Unhandled internal transition.");
				break;
		}
	}

	// external transition
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		//If we get any messages on the pilot takeover port in any state (apart from HANDOVER_CONTROL), immediately transition into the pilot in control state.
		if (get_messages<typename LP_Manager_defs::i_pilot_takeover>(mbs).size() >= 1 && state.current_state != States::HANDOVER_CONTROL)
			state.current_state = States::PILOT_CONTROL;

		//If we are in a state that can receive a landing point input,
		if (state.current_state == States::WAYPOINT_MET || state.current_state == States::LZE_SCAN || state.current_state == States::LP_APPROACH) {
			//If there are landing points that have been received,
			if (get_messages<typename LP_Manager_defs::i_lp_recv>(mbs).size() >= 1) {
				//Store the landing points in a vector.
				vector<message_mavlink_mission_item_t> landing_points = get_messages<typename LP_Manager_defs::i_lp_recv>(mbs);

				//Create a flag for if one of them is a valid landing point to be transitioned to.
				bool valid_lp_recv = false;

				//If there was a previous landing point,
				if (!lp_recvd) {
					//For each of the landing points received,
					for (message_mavlink_mission_item_t new_lp : landing_points) {
						//If the landing point is far enough away from the previous landing point,
						if (calculate_new_lp_valid(new_lp)) {
							//Set the current landing point to be the new landing point.
							lp = new_lp;
							valid_lp_recv = true;
							break;
						}
					}
				}
				//If this is the first bag of landing points that have been received,
				else {
					//Pick the first landing point in the list.
					lp = landing_points[0];
					valid_lp_recv = true;
				}

				//If a valid landing point was identified out of the list of landing points,
				switch (state.current_state) {
					case States::WAYPOINT_MET: case States::LZE_SCAN:
						if (valid_lp_recv) {
							//Transition into the notify reposition loop state.
							state.current_state = States::NOTIFY_LP;
						}
						break;
					case States::LP_APPROACH:
						if (valid_lp_recv) {
							//Transition into the notify reposition loop state and store the current value of the LP accept timer.
							state.current_state = States::NOTIFY_LP;
						}
						lp_accept_time_prev = lp_accept_time_prev - e;
						break;
					default:
						assert(false && "Unhandled external transition on receipt of landing point.");
						break;
				}
			}
		}

		switch (state.current_state) {
			//If we are in a state that can receive a planned landing point acheived input,
			case States::WAYPOINT_MET:
				if (get_messages<typename LP_Manager_defs::i_plp_ach>(mbs).size() >= 1) {
					state.current_state = States::HOVER_PLP;
					plp = get_messages<typename LP_Manager_defs::i_plp_ach>(mbs)[0];
				}
				break;

				//If we are in a state that can receive a hover criteria met input,
			case States::STABILIZING:
				if (get_messages<typename LP_Manager_defs::i_hover_criteria_met>(mbs).size() >= 1) {
					state.current_state = States::START_LZE_SCAN;
				}
				break;

				//If we are in a state that can receive a control yielded input,
			case States::HANDOVER_CONTROL:
				if (get_messages<typename LP_Manager_defs::i_control_yielded>(mbs).size() >= 1) {
					state.current_state = States::PILOT_CONTROL;
				}
				break;

			default:
				break;
		}
	}

	// confluence transition
	void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		//If the external input is a pilot takeover messasge,
		if (get_messages<typename LP_Manager_defs::i_pilot_takeover>(mbs).size() >= 1) {
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
		vector<message_mavlink_mission_item_t> message_out;
		message_mavlink_mission_item_t temp_lp;
		vector<bool> bool_out;
		vector<message_hover_criteria_t> stabilize_messages;

		switch (state.current_state) {
			case States::HOVER_PLP:
				stabilize_messages.push_back(message_hover_criteria_t());
				get_messages<typename LP_Manager_defs::o_stabilize>(bags) = stabilize_messages;
				break;

			case States::START_LZE_SCAN:
				bool_out.push_back(true);
				get_messages<typename LP_Manager_defs::o_start_lze_scan>(bags) = bool_out;
				break;

			case States::LZE_SCAN:
				message_out.push_back(plp);
				get_messages<typename LP_Manager_defs::o_pilot_handover>(bags) = message_out;
				break;

			case States::NOTIFY_LP:
				message_out.push_back(lp);
				get_messages<typename LP_Manager_defs::o_lp_new>(bags) = message_out;
				break;

			case States::LP_APPROACH:
				message_out.push_back(lp);
				get_messages<typename LP_Manager_defs::o_lp_expired>(bags) = message_out;
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
			case States::WAYPOINT_MET: case States::STABILIZING: case States::HANDOVER_CONTROL: case States::PILOT_CONTROL: case States::LP_ACCEPT_EXP:
				next_internal = numeric_limits<TIME>::infinity();
				break;

			case States::HOVER_PLP: case States::START_LZE_SCAN: case States::NOTIFY_LP:
				next_internal = TIME("00:00:00:000");
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
		os << "State: " << enumToString(i.current_state);
		return os;
	}

	bool calculate_new_lp_valid(message_mavlink_mission_item_t i_lp) {
		//Radius of the earth in meters.
		const float R = 6371000;

		double my_x = R * cos(lp.lat) * cos(lp.lon);
		double my_y = R * cos(lp.lat) * sin(lp.lon);
		double my_z = R * sin(lp.lat);

		double i_x = R * cos(i_lp.lat) * cos(i_lp.lon);
		double i_y = R * cos(i_lp.lat) * sin(i_lp.lon);
		double i_z = R * sin(i_lp.lat);

		return (sqrt(pow((i_x - my_x), 2) + pow((i_y - my_y), 2) + pow((i_z - my_z), 2)) >= LP_HOR_ACCEPT_TOLERANCE_DISTANCE);
	}
};

#endif // LP_MANAGER_HPP