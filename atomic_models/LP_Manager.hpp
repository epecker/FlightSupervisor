/**
* James Horner and Tanner Trautrim
*
*/

#ifndef Man_HPP
#define Man_HPP

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

#include <limits>
#include <assert.h>
#include <string>

#include "../data_structures/message.hpp"
#include "../data_structures/enum_string_conversion.hpp"
#include "../include/Constants.hpp"

using namespace cadmium;
using namespace std;

//Port definition
struct Man_defs {
	struct lp_recv_in : public in_port<Message_t> {};
	struct plp_ach_in : public in_port<Message_t> {};
	struct pilot_takeover_in : public in_port<Message_t> {};

	struct lp_new_out : public out_port<Message_t> {};
	struct pilot_handover_out : public out_port<Message_t> {};
	struct lp_expired_out : public out_port<Message_t> {};
};

template<typename TIME>
class Man {
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
		typename Man_defs::lp_recv_in,
		typename Man_defs::plp_ach_in,
		typename Man_defs::pilot_takeover_in
	>;

	using output_ports = tuple<
		typename Man_defs::lp_new_out,
		typename Man_defs::pilot_handover_out,
		typename Man_defs::lp_expired_out
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
	Man() {
		state.cur_state = WAYPOINT_MET;
		state.lp_accept_time_prev = TIME(LP_APPROACH_TIME);
		state.lp_recvd = false;
	}

	// internal transition
	void internal_transition() {
		switch (state.cur_state) {
			case LZE_SCAN:
				state.cur_state = PILOT_CONTROL;
				break;
			case NOTIFY_LP:
				state.cur_state = LP_APPROACH;
				break;
			case LP_APPROACH:
				state.cur_state = LP_ACCEPT_EXP;
				break;
			default:
				assert(false && "Unhandled internal transition.");
				break;
		}
	}

	// external transition
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		//If we get any messages on the pilot takeover port in any state, immediately transition into the pilot in control state.
		if (get_messages<typename Man_defs::pilot_takeover_in>(mbs).size() >= 1)
			state.cur_state = PILOT_CONTROL;

		//If we are in a state that can receive a landing point input,
		if (state.cur_state == WAYPOINT_MET || state.cur_state == LZE_SCAN || state.cur_state == LP_APPROACH) {
			//If there are landing points that have been received,
			if (get_messages<typename Man_defs::lp_recv_in>(mbs).size() >= 1) {
				//Store the landing points in a vector.
				vector<Message_t> landing_points;
				landing_points = get_messages<typename Man_defs::lp_recv_in>(mbs);

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
						case WAYPOINT_MET: case LZE_SCAN:
							//Transition into the notify reposition loop state.
							state.cur_state = NOTIFY_LP;
							break;
						case LP_APPROACH:
							//Transition into the notify reposition loop state and store the current value of the LP accept timer.
							state.cur_state = NOTIFY_LP;
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
		if (state.cur_state == WAYPOINT_MET) {
			//If there is at least one PLP acheived message,
			if (get_messages<typename Man_defs::plp_ach_in>(mbs).size() >= 1) {
				state.cur_state = LZE_SCAN;
			}
		}
	}

	// confluence transition
	void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		//If the external input is a pilot takeover messasge,
		if (get_messages<typename Man_defs::pilot_takeover_in>(mbs).size() >= 1) {
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
			case NOTIFY_LP:
				message_out.push_back(state.lp);
				get_messages<typename Man_defs::lp_new_out>(bags) = message_out;
				break;
			case LZE_SCAN:
				temp_lp = { PLP_HANDOVER_CODE, 0, 0, 0 };
				message_out.push_back(temp_lp);
				get_messages<typename Man_defs::pilot_handover_out>(bags) = message_out;
				break;
			case LP_APPROACH:
				temp_lp = { LP_TIME_EXPIRED_CODE, 0, 0, 0 };
				message_out.push_back(temp_lp);
				get_messages<typename Man_defs::lp_expired_out>(bags) = message_out;
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
			case WAYPOINT_MET: case PILOT_CONTROL: case LP_ACCEPT_EXP:
				next_internal = numeric_limits<TIME>::infinity();
				break;
			case NOTIFY_LP:
				next_internal = TIME("00:00:00:000");
				break;
			case LZE_SCAN:
				next_internal = TIME(LZE_SCAN_TIME);
				break;
			case LP_APPROACH:
				//Schedule the amount of time that was left on the LP accept timer.
				next_internal = state.lp_accept_time_prev;
				break;
			default:
				next_internal = numeric_limits<TIME>::infinity();
		}
		return next_internal;
	}

	friend ostringstream& operator<<(ostringstream& os, const typename Man<TIME>::state_type& i) {
		os << "State: " << enumToString(i.cur_state) << "\tLP: " << i.lp;
		return os;
	}
};

#endif // Man_HPP