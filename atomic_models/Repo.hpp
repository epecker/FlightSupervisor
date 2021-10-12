/**
 *	\brief		An atomic model representing the reposition model.
 *	\details	This header file define the reposition model as
				an atomic model for use in the Cadmium DEVS
				simulation software.
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef REPO_HPP
#define REPO_HPP

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

#include <limits> // Used to set the time advance to infinity
#include <assert.h> // Used to check values and stop the simulation
#include <string>

#include "../data_structures/message.hpp"
#include "../data_structures/enum_string_conversion.hpp"

 // Macros for the time advance functions
#define LP_REPOSITION_TIME "00:02:00:000"

using namespace cadmium;
using namespace std;

// Input and output port definition
struct Repo_defs {
	struct lp_new_in : public in_port<Message_t> {};
	struct lp_crit_met_in : public in_port<Message_t> {};
	struct pilot_takeover_in : public in_port<Message_t> {};

	struct lp_repo_new_out : public out_port<Message_t> {};
	struct pilot_handover_out : public out_port<Message_t> {};
	struct land_out : public out_port<Message_t> {};
};

// Atomic Model
template<typename TIME> class Repo {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(LAND)
		(LP_REPO)
		(NEW_LP_REPO)
		(NOTIFY_LAND)
		(PILOT_CONTROL)
	)

		// Create a tuple of input ports (required for the simulator)
		using input_ports = tuple<
		typename Repo_defs::lp_crit_met_in,
		typename Repo_defs::lp_new_in,
		typename Repo_defs::pilot_takeover_in
		>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
		typename Repo_defs::lp_repo_new_out,
		typename Repo_defs::pilot_handover_out,
		typename Repo_defs::land_out
	>;

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct state_type {
		States current_state;
		Message_t landing_point;
		TIME next_internal;
	};
	state_type state;

	// Default constructor
	Repo() {
		state.current_state = IDLE;
		state.next_internal = numeric_limits<TIME>::infinity();
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
				state.current_state = PILOT_CONTROL;
				state.next_internal = numeric_limits<TIME>::infinity();
				break;
			case NOTIFY_LAND:
				state.current_state = LAND;
				state.next_internal = numeric_limits<TIME>::infinity();
				break;
			default:
				break;
		}
	}

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		bool received_lp_new;
		bool received_lp_crit_met;
		bool received_pilot_takeover;

		switch (state.current_state) {
			case IDLE:
				received_lp_new = get_messages<typename Repo_defs::lp_new_in>(mbs).size() >= 1;
				received_pilot_takeover = get_messages<typename Repo_defs::lp_new_in>(mbs).size() >= 1;

				if (received_pilot_takeover) {
					state.current_state = PILOT_CONTROL;
					state.next_internal = numeric_limits<TIME>::infinity();
				} else if (received_lp_new) {
					state.current_state = LP_REPO;
					state.next_internal = TIME(LP_REPOSITION_TIME);
				}
				break;
			case LAND:
				received_pilot_takeover = get_messages<typename Repo_defs::lp_new_in>(mbs).size() >= 1;

				if (received_pilot_takeover) {
					state.current_state = PILOT_CONTROL;
					state.next_internal = numeric_limits<TIME>::infinity();
				}
				break;
			case LP_REPO:
				received_lp_new = get_messages<typename Repo_defs::lp_new_in>(mbs).size() >= 1;
				received_lp_crit_met = get_messages<typename Repo_defs::lp_crit_met_in>(mbs).size() >= 1;
				received_pilot_takeover = get_messages<typename Repo_defs::lp_new_in>(mbs).size() >= 1;

				if (received_pilot_takeover) {
					state.current_state = PILOT_CONTROL;
					state.next_internal = numeric_limits<TIME>::infinity();
				} else if (received_lp_new) {
					vector<Message_t> new_landing_points = get_messages<typename Repo_defs::lp_new_in>(mbs);
					state.landing_point = new_landing_points[0]; // set the new Landing 
					state.current_state = NEW_LP_REPO;
					state.next_internal = TIME("00:00:00:000");
				} else if (received_lp_crit_met) {
					state.current_state = NOTIFY_LAND;
					state.next_internal = TIME("00:00:00:000");
				}
				break;
			case NEW_LP_REPO:
				received_pilot_takeover = get_messages<typename Repo_defs::lp_new_in>(mbs).size() >= 1;

				if (received_pilot_takeover) {
					state.current_state = PILOT_CONTROL;
					state.next_internal = numeric_limits<TIME>::infinity();
				}
				break;
			case NOTIFY_LAND:
				received_pilot_takeover = get_messages<typename Repo_defs::lp_new_in>(mbs).size() >= 1;

				if (received_pilot_takeover) {
					state.current_state = PILOT_CONTROL;
					state.next_internal = numeric_limits<TIME>::infinity();
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
		external_transition(TIME(), move(mbs));
	}

	// output function
	typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		vector<Message_t> bag_port_out;

		switch (state.current_state) {
			case NOTIFY_LAND:
				bag_port_out.push_back(state.landing_point);
				get_messages<typename Repo_defs::land_out>(bags) = bag_port_out;
				break;
			case PILOT_CONTROL:
				bag_port_out.push_back(state.landing_point);
				get_messages<typename Repo_defs::pilot_handover_out>(bags) = bag_port_out;
				break;
			case NEW_LP_REPO:
				bag_port_out.push_back(state.landing_point);
				get_messages<typename Repo_defs::lp_repo_new_out>(bags) = bag_port_out;
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

	friend ostringstream& operator<<(ostringstream& os, const typename Repo<TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state) << "\tLP: " << i.landing_point;
		return os;
	}
};

#endif // REPO_HPP
