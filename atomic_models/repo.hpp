#ifndef REPO_HPP
#define REPO_HPP

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

#include <limits> // Used to set the time advance to infinity
#include <assert> // Used to check values and stop the simulation
#include <string>

#include "../data_structures/message.hpp"

// Macros for the time advance functions
#define LP_REPOSITION_TIME "00:02:00:000"
#define CODE_LANDING 9
#define CODE_PILOT_HANDOVER 8

using namespace std;
using namespace cadmium;

// Input and output port definition
struct Repo_defs {
	struct lp_new_in : public in_port<Message_t> {};
	struct lp_crit_met_in : public in_port<Message_t> {};

	struct lp_repo_new_out : public out_port<Message_t> {};
	struct pilot_handover_out : public out_port<Message_t> {};
	struct land_out : public out_port<Message_t> {};
};

// Atomic Model
template<typename TIME> class Repo {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	enum Repo_states {
		IDLE,
		LP_REPO,
		NEW_LP_REPO,
		NOTIFY_LAND,
		LAND,
		LP_HOVER
	};

	// Create a tuple of input ports (required for the simulator)
	using input_ports = tuple<
		typename Repo_defs::lp_crit_met_in,
		typename Repo_defs::lp_new_in
	>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
		typename Repo_defs::lp_repo_new_out,
		typename Repo_defs::pilot_handover_out,
		typename Repo_defs::land_out
	>;

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct State_type {
		Repo_states current_state;
		Message_t landing_point;
		TIME next_internal;
	};

	State_type state;

	// Default constructor
	Repo() {
		state.current_state = IDLE;
		state.time_advance = TIME(LP_REPOSITION_TIME);
	}

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
	void internal_transition() {
		switch (state.current_state) {
			case NEW_LP_REPO:
				state.current_state = LP_REPO;
				state.next_internal = TIME(LP_REPOSITION_TIME);
				break;
			case LP_REPO:
				state.current_state = LP_HOVER;
				state.next_internal = std::numeric_limits<TIME>::infinity();
				break;
			case NOTIFY_LAND:
				state.current_state = LAND;
				state.next_internal = std::numeric_limits<TIME>::infinity();
				break;
			default:
				break;
		}
	}

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		switch (state.current_state) {
			case IDLE:
				bool received_lp_new = get_messages<typename Repo_defs::lp_new_in>(mbs).size() >= 1;

				if (received_lp_new) {
					state.current_state = LP_REPO;
					state.next_internal = TIME(LP_REPOSITION_TIME);
				}
				break;

			case LP_REPO:
				bool received_lp_new = get_messages<typename Repo_defs::lp_new_in>(mbs).size() >= 1;
				bool received_lp_crit_met = get_messages<typename Repo_defs::lp_crit_met_in>(mbs).size() >= 1;

				// Receiving a new landing point takes precedence over meeting the landing criteria 
				if (received_lp_new) {
					vector<Message_t> new_landing_points = get_messages<typename Repo_defs::lp_new_in>(mbs)
					state.landing_point = new_landing_points[0]; // set the new Landing 
					state.current_state = NEW_LP_REPO;
					state.next_internal = TIME("00:00:00:000");
				} else if (received_lp_crit_met) {
					state.current_state = NOTIFY_LAND;
					state.next_internal = TIME("00:00:00:000");
				}
				break;

			default:
				break;
		}
	}

	// confluence transition
	// Used to call set call precedence
	void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), std::move(mbs));
	}

	typename make_message_bags<output_ports>::type output() const {
		switch (state.current_state) {
			typename make_message_bags<output_ports>::type bags;

			case NOTIFY_LAND:
				get_messages<typename Repo::land_out>bags.push_back(CODE_LANDING);
				break;
			case LP_HOVER:
				get_messages<typename Repo::pilot_handover_out>bags.push_back(CODE_PILOT_HANDOVER);
				break;
			case NEW_LP_REPO:
				Message_t landing_point_message;
				landing_point_message = state.landing_point;
				get_messages<typename Repo::lp_repo_new_out>bags.push_back(landing_point_message);
				break;
			default:
				break;
		}

		return bags;
	}

	// Time advance
	// Used to set the internal time of the current state
	TIME time_advance() const {
		return state.next_internal;
	}

	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Receiver<TIME>::state_type& i) {
		os << "State: " << state_names[i.cur_state] << "\tLP: " << i.lp;
		return os;
	}
};

#endif // REPO_HPP