/**
* James Horner and Tanner Trautrim
*
*/

#ifndef __LANDINGPOINTMANAGER_HPP__
#define __LANDINGPOINTMANAGER_HPP__

//Macro for the distance that subsequent landing points should be separated by in meters.
#define LP_SEPARATION (10.0)

//Macros for the time in the states.
#define LZE_SCAN_TIME "00:01:00:000"
#define LP_APPROACH_TIME "00:01:00:000"

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

#include <limits>
#include <assert.h>
#include <string>

#include "../data_structures/message.hpp"

using namespace cadmium;
using namespace std;

//Port definition
struct LandingPointManager_defs {
	struct LP_recv : public in_port<Message_t> { };
	struct PLP_ach : public in_port<int> { };
	struct LP_new : public out_port<Message_t> { };
	struct PLP_pilot_handover : public out_port<int> { };
	struct LP_expired : public out_port<int> { };
};

template<typename TIME>
class LandingPointManager {
public:
	// Enum of the automata-like states of the atomic model.
	typedef enum {
		WAYPOINT_MET,
		LZE_SCAN,
		PLP_HOVER,
		NOTIFY_LP,
		LP_APPROACH,
		LP_ACCEPT_EXP
	} States;

	// Array for the string names of the states.
	string state_names[] = {
		"WAYPOINT_MET",
		"LZE_SCAN",
		"PLP_HOVER",
		"NOTIFY_LP",
		"LP_APPROACH",
		"LP_ACCEPT_EXP"
	};

	// state definition
	struct state_type {
		States cur_state;
		Message_t lp_prev;
		TIME lp_accept_time_prev;
	};
	state_type state;

	// default constructor
	LandingPointManager() noexcept {
		state.cur_state = States.WAYPOINT_MET;
		state.lp = nullptr;
		state.lp_accept_time_prev = TIME(LP_APPROACH_TIME);
	}

	// ports definition
	using input_ports = std::tuple<typename LandingPointManager_defs::LP_recv, typename LandingPointManager_defs::PLP_ach>;
	using output_ports = std::tuple<typename LandingPointManager_defs::LP_new, typename LandingPointManager_defs::PLP_pilot_handover, typename LandingPointManager_defs::LP_expired>;

	// internal transition
	void internal_transition() {
		switch (state.cur_state) {
			case LZE_SCAN:
				state.cur_state = PLP_HOVER;
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
		//If we are in a state that can receive a landing point input,
		if (state.cur_state == WAYPOINT_MET || state.cur_state == LZE_SCAN || state.cur_state == LP_APPROACH) {
			//If there are landing points that have been received,
			if (get_messages<typename LandingPointManager_defs::LP_recv>(mbs).size() >= 1) {
				//Store the landing points in a vector.
				vector<Message_t> landing_points;
				landing_points = get_messages<typename LandingPointManager_defs::LP_recv>(mbs);

				//Create a flag for if one of them is a valid landing point to be transitioned to.
				bool valid_lp_recv = false;

				//If there was a previous landing point,
				if (lp_prev != nullptr) {
					//For each of the landing points received,
					for (Message_t new_lp : landing_points) {
						//If the landing point is far enough away from the previous landing point,
						if (state.lp.separation(new_lp) >= LP_SEPARATION) {
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
			if (get_messages<typename LandingPointManager_defs::PLP_ach>(mbs).size() >= 1) {
				//Transition into the landing zone evaluation scan state from the waypoint met state.
				state.cur_state = LZE_SCAN;
			}
		}

	// confluence transition
	void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), std::move(mbs));
	}

	// output function
	typename make_message_bags<output_ports>::type output() const {
		//Based on the current state,
		switch (state.cur_state) {
			//If we just transitioned into the notify reposition loop state,
			case NOTIFY_LP:
				//Construct an output message to send to the LP_new port with the valid landing point that was just received.
				typename make_message_bags<output_ports>::type bags;
				Message_t lp_to_repo_to = state.lp;
				get_messages<typename LandingPointManager_defs::LP_new>(bags).push_back(lp_to_repo_to);
				return bags;

			//If we just transitioned into the hover at planned landing point state,
			case PLP_HOVER:
				//Construct a message to send on the PLP_pilot_handover port saying that we need to hover.
				typename make_message_bags<output_ports>::type bags;
				int plp_handover_code = 0;
				get_messages<typename LandingPointManager_defs::PLP_pilot_handover>(bags).push_back(plp_handover_code);
				return bags;

			//If we just transitioned into the landing point accept timer expired state,
			case LP_ACCEPT_EXP:
				//Construct a message to send on the LP_expired port saying that no more LPs will be repositioned to.
				typename make_message_bags<output_ports>::type bags;
				int lp_timer_expired_code = 0;
				get_messages<typename LandingPointManager_defs::LP_expired>(bags).push_back(lp_timer_expired_code);
				return bags;

			default:
				assert(false && "Unhandled output after internal transition.");
				break;
		}
		return nullptr;
	}

	// time_advance function
	TIME time_advance() const {
		TIME next_internal;
		//Based on the state that was just transitioned into,
		switch (state.cur_state) {
			//If the state should be passive,
			case WAYPOINT_MET: PLP_HOVER: LP_ACCEPT_EXP: 
				//return an infinite duration.
				next_internal = std::numeric_limits<TIME>::infinity();
				break;
			//If the state should be immediately transitioned out of,
			case NOTIFY_LP:
				//return a duration of 0.
				next_internal = TIME("00:00:00:000");
				break;
			//If the landing zone evaluation scan is taking place,
			case LZE_SCAN:
				//Schedule a predefined duration.
				next_internal = TIME(LZE_SCAN_TIME);
				break;
			//If the LP approach state is being enetered,
			case LP_APPROACH:
				//Schedule the amount of time that was left on the LP accept timer.
				next_internal = state.lp_accept_time_prev;
				break;
			default:
				next_internal = std::numeric_limits<TIME>::infinity();
		}
		return next_internal;
	}

	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Receiver<TIME>::state_type& i) {
		os << "State: " << state_names[i.cur_state] << "\tLP: " << i.lp;
		return os;
	}
};

#endif // __LANDINGPOINTMANAGER_HPP__