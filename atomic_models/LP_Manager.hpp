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

#include "../data_structures/message.hpp"
#include "../include/enum_string_conversion.hpp"
#include "../include/Constants.hpp"

using namespace cadmium;
using namespace std;

//Port definition
struct LP_Manager_defs {
	struct i_lp_recv : public in_port<Message_t> {};
	struct i_plp_ach : public in_port<Message_t> {};
	struct i_pilot_takeover : public in_port<Message_t> {};

	struct o_lp_new : public out_port<Message_t> {};
	struct o_pilot_handover : public out_port<Message_t> {};
	struct o_lp_expired : public out_port<Message_t> {};
};

template<typename TIME>
class LP_Manager {
public:
	// Enum of the automata-like states of the atomic model.
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(WAYPOINT_MET)
		(LZE_SCAN)
		(PILOT_CONTROL)
		(NOTIFY_LP)
		(LP_APPROACH)
		(LP_ACCEPT_EXP)
	)

	// ports definition
	using input_ports = tuple<
		typename LP_Manager_defs::i_lp_recv,
		typename LP_Manager_defs::i_plp_ach,
		typename LP_Manager_defs::i_pilot_takeover
	>;

	using output_ports = tuple<
		typename LP_Manager_defs::o_lp_new,
		typename LP_Manager_defs::o_pilot_handover,
		typename LP_Manager_defs::o_lp_expired
	>;

	// state definition
	struct state_type {
		States cur_state;
		bool lp_recvd;
		Message_t lp;
		TIME lp_accept_time_prev;
	};
	state_type state;

	// default constructor
	LP_Manager() {
		state.cur_state = States::WAYPOINT_MET;
		state.lp_accept_time_prev = TIME(LP_APPROACH_TIME);
		state.lp_recvd = false;
	}

	// internal transition
	void internal_transition() {
		switch (state.cur_state) {
			case States::LZE_SCAN:
				state.cur_state = States::PILOT_CONTROL;
				break;
			case States::NOTIFY_LP:
				state.cur_state = States::LP_APPROACH;
				break;
			case States::LP_APPROACH:
				state.cur_state = States::LP_ACCEPT_EXP;
				break;
			default:
				assert(false && "Unhandled internal transition.");
				break;
		}
	}

	// external transition
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		//If we get any messages on the pilot takeover port in any state, immediately transition into the pilot in control state.
		if (get_messages<typename LP_Manager_defs::i_pilot_takeover>(mbs).size() >= 1)
			state.cur_state = States::PILOT_CONTROL;

		//If we are in a state that can receive a landing point input,
		if (state.cur_state == States::WAYPOINT_MET || state.cur_state == States::LZE_SCAN || state.cur_state == States::LP_APPROACH) {
			//If there are landing points that have been received,
			if (get_messages<typename LP_Manager_defs::i_lp_recv>(mbs).size() >= 1) {
				//Store the landing points in a vector.
				vector<Message_t> landing_points;
				landing_points = get_messages<typename LP_Manager_defs::i_lp_recv>(mbs);

				//Create a flag for if one of them is a valid landing point to be transitioned to.
				bool valid_lp_recv = false;

				//If there was a previous landing point,
				if (!state.lp_recvd) {
					//For each of the landing points received,
					for (Message_t new_lp : landing_points) {
						//If the landing point is far enough away from the previous landing point,
						if (state.lp.separation(new_lp) >= LP_SEPARATION) {
							//Set the current landing point to be the new landing point.
							state.lp = new_lp;
							valid_lp_recv = true;
							break;
						}
					}
				}
				//If this is the first bag of landing points that have been received,
				else {
					//Pick the first landing point in the list.
					state.lp = landing_points[0];
					valid_lp_recv = true;
				}

				//If a valid landing point was identified out of the list of landing points,
				if (valid_lp_recv) {
					//Based on the current state,
					switch (state.cur_state) {
						case States::WAYPOINT_MET: case States::LZE_SCAN:
							//Transition into the notify reposition loop state.
							state.cur_state = States::NOTIFY_LP;
							break;
						case States::LP_APPROACH:
							//Transition into the notify reposition loop state and store the current value of the LP accept timer.
							state.cur_state = States::NOTIFY_LP;
							state.lp_accept_time_prev = e;
							break;
						default:
							assert(false && "Unhandled external transition on receipt of landing point.");
							break;
					}
				}
			}
		}

		//If we are in a state that can receive a planned landing point acheived input,
		if (state.cur_state == States::WAYPOINT_MET) {
			//If there is at least one PLP acheived message,
			if (get_messages<typename LP_Manager_defs::i_plp_ach>(mbs).size() >= 1) {
				state.cur_state = States::LZE_SCAN;
			}
		}
	}

	// confluence transition
	void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		//If the external input is a pilot takeover messasge,
		if (get_messages<typename LP_Manager_defs::i_pilot_takeover>(mbs).size() >= 1) {
			//Execute the external transition first, then the internal.
			external_transition(TIME(), move(mbs));
			internal_transition();
		}
		else {
			internal_transition();
			external_transition(TIME(), move(mbs));
		}
	}

	// output function
	typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		vector<Message_t> message_out;
		Message_t temp_lp;
		vector<int> integer_out;

		switch (state.cur_state) {
			case States::NOTIFY_LP:
				message_out.push_back(state.lp);
				get_messages<typename LP_Manager_defs::o_lp_new>(bags) = message_out;
				break;
			case States::LZE_SCAN:
				temp_lp = { PLP_HANDOVER_CODE, 0, 0, 0 };
				message_out.push_back(temp_lp);
				get_messages<typename LP_Manager_defs::o_pilot_handover>(bags) = message_out;
				break;
			case States::LP_APPROACH:
				temp_lp = { LP_TIME_EXPIRED_CODE, 0, 0, 0 };
				message_out.push_back(temp_lp);
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

		switch (state.cur_state) {
			case States::WAYPOINT_MET: case States::PILOT_CONTROL: case States::LP_ACCEPT_EXP:
				next_internal = numeric_limits<TIME>::infinity();
				break;
			case States::NOTIFY_LP:
				next_internal = TIME("00:00:00:000");
				break;
			case States::LZE_SCAN:
				next_internal = TIME(LZE_SCAN_TIME);
				break;
			case States::LP_APPROACH:
				//Schedule the amount of time that was left on the LP accept timer.
				next_internal = state.lp_accept_time_prev;
				break;
			default:
				next_internal = numeric_limits<TIME>::infinity();
		}
		return next_internal;
	}

	friend ostringstream& operator<<(ostringstream& os, const typename LP_Manager<TIME>::state_type& i) {
		os << "State: " << enumToString(i.cur_state) << "\tLP: " << i.lp;
		return os;
	}
};

#endif // LP_MANAGER_HPP