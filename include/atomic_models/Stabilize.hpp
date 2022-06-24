/**
 *	\brief		An atomic model representing the Stabilize model.
 *	\details	This header file define the Stabilize model as
				an atomic model for use in the Cadmium DEVS
				simulation software.
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef STABILIZE_HPP
#define STABILIZE_HPP

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

#include <limits> // Used to set the time advance to infinity
#include <assert.h> // Used to check values and stop the simulation
#include <string>

#include "message_structures/message_hover_criteria_t.hpp"
#include "message_structures/message_aircraft_state_t.hpp"
#include "message_structures/message_fcc_command_t.hpp"

#include "enum_string_conversion.hpp"
#include "time_conversion.hpp"
#include "Constants.hpp"

using namespace cadmium;
using namespace std;

// Input and output port definition
struct Stabilize_defs {
	struct i_aircraft_state : public in_port<message_aircraft_state_t> {};
	struct i_stabilize : public in_port<message_hover_criteria_t> {};
	struct i_cancel_hover : public in_port<bool> {};

	struct o_fcc_command_hover : public out_port<message_fcc_command_t> {};
	struct o_hover_criteria_met : public out_port<bool> {};
};

// Atomic Model
template<typename TIME> class Stabilize {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(INIT_HOVER)
		(STABILIZING)
		(CRIT_CHECK_FAILED)
		(HOVER)
	)

	// Create a tuple of input ports (required for the simulator)
	using input_ports = tuple<
		typename Stabilize_defs::i_aircraft_state,
		typename Stabilize_defs::i_stabilize,
		typename Stabilize_defs::i_cancel_hover
	>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
		typename Stabilize_defs::o_fcc_command_hover,
		typename Stabilize_defs::o_hover_criteria_met
	>;

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct state_type {
		States current_state;
	};
	state_type state;

	// Public members of the class
	message_hover_criteria_t hover_criteria;
	message_aircraft_state_t aircraft_state;
	TIME stabilization_time_prev;

	// Default constructor
	Stabilize() {
		state.current_state = States::IDLE;
		hover_criteria = message_hover_criteria_t();
		aircraft_state = message_aircraft_state_t();
	}

	// Constructor with initial state parameter for debugging or partial execution startup.
	Stabilize(States initial_state) {
		state.current_state = initial_state;
		hover_criteria = message_hover_criteria_t();
		aircraft_state = message_aircraft_state_t();
	}

	// Internal transitions
	// These are transitions occuring from internal inputs
	// There are no internal transitions
	// (required for the simulator)
	void internal_transition() {
		switch (state.current_state) {
			case States::INIT_HOVER:
				state.current_state = States::STABILIZING;
				break;
			case States::CRIT_CHECK_FAILED:
				state.current_state = States::STABILIZING;
				break;
			case States::STABILIZING:
				state.current_state = States::HOVER;
				break;
			case States::HOVER:
				state.current_state = States::IDLE;
				break;
			default:
				break;
		}
	}

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		bool received_cancel_hover = get_messages<typename Stabilize_defs::i_cancel_hover>(mbs).size() >= 1;

		if (received_cancel_hover) {
			state.current_state = States::IDLE;
			aircraft_state = message_aircraft_state_t();
			hover_criteria = message_hover_criteria_t();
		} else {
			switch (state.current_state) {
				case States::IDLE:
					if (get_messages<typename Stabilize_defs::i_stabilize>(mbs).size() >= 1) {
						state.current_state = States::INIT_HOVER;
						hover_criteria = get_messages<typename Stabilize_defs::i_stabilize>(mbs)[0];
						stabilization_time_prev = seconds_to_time<TIME>(hover_criteria.timeTol);
					}
					break;
				case States::STABILIZING:
					if (get_messages<typename Stabilize_defs::i_aircraft_state>(mbs).size() >= 1) {
						aircraft_state = get_messages<typename Stabilize_defs::i_aircraft_state>(mbs)[0];
						if (!calculate_hover_criteria_met(aircraft_state)) {
							state.current_state = States::CRIT_CHECK_FAILED;
							stabilization_time_prev = seconds_to_time<TIME>(hover_criteria.timeTol);
						}
						else {
							stabilization_time_prev = stabilization_time_prev - e;
						}
					}
					break;
				default:
					break;
			}
		}
	}

	// confluence transition
	// Used to call set call precedence
	void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		bool received_cancel_hover = get_messages<typename Stabilize_defs::i_cancel_hover>(mbs).size() >= 1;

		if (received_cancel_hover) {
			external_transition(TIME(), move(mbs));
		} else {
			internal_transition();
		}
	}

	// output function
	// Nothing to output
	typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		vector<bool> message_out;
		vector<message_fcc_command_t> message_fcc_out;

		switch (state.current_state) {
			case States::INIT_HOVER:
			{
				message_fcc_command_t mfc = message_fcc_command_t();
				mfc.x = hover_criteria.desiredLat * (1E7);
				mfc.y = hover_criteria.desiredLon * (1E7);
				mfc.z = hover_criteria.desiredAltMSL;
				mfc.param4 = -NAN;
				mfc.param1 = 0.0;
				mfc.command = MAV_CMD_DO_REPOSITION;
				message_fcc_out.push_back(mfc);
				get_messages<typename Stabilize_defs::o_fcc_command_hover>(bags) = message_fcc_out;
			}
				break;
			case States::STABILIZING:
				message_out.push_back(true);
				get_messages<typename Stabilize_defs::o_hover_criteria_met>(bags) = message_out;
				break;
			case States::CRIT_CHECK_FAILED:
				break;
			default:
				break;
		}

		return bags;
	}

	// Time advance
	// Used to set the internal time of the current state
	TIME time_advance() const {
		TIME next_internal;
		switch (state.current_state) {
			case States::IDLE: 
				next_internal = numeric_limits<TIME>::infinity();
				break;
			case States::INIT_HOVER:
				next_internal = TIME(TA_ZERO);
				break;
			case States::STABILIZING:
				next_internal = stabilization_time_prev;
				break;
			case States::CRIT_CHECK_FAILED:
				next_internal = TIME(TA_ZERO);
				break;
			case States::HOVER:
				next_internal = TIME(TA_ZERO);
				break;
			default:
				next_internal = numeric_limits<TIME>::infinity();
		}
		return next_internal;
	}

	friend ostringstream& operator<<(ostringstream& os, const typename Stabilize<TIME>::state_type& i) {
		os << (string("State: ") + enumToString(i.current_state) + string("\n"));
		return os;
	}

	// Stub implementation for now so we can always hover.
	bool calculate_hover_criteria_met(message_aircraft_state_t i_state) {

		if (abs(i_state.alt_MSL - hover_criteria.desiredAltMSL) >= hover_criteria.vertDistTolFt) {
			return false;
		}
		if (abs(i_state.hdg_Deg - hover_criteria.desiredHdgDeg) >= hover_criteria.hdgToleranceDeg) {
			return false;
		}
		if (abs(i_state.vel_Kts) >= hover_criteria.velTolKts) {
			return false;
		}

		//Radius of the earth in meters.
		const float R = 6371000;

		double i_x = R * cos(i_state.lat) * cos(i_state.lon);
		double i_y = R * cos(i_state.lat) * sin(i_state.lon);
		double i_z = R * sin(i_state.lat);

		double goal_x = R * cos(hover_criteria.desiredLat) * cos(hover_criteria.desiredLon);
		double goal_y = R * cos(hover_criteria.desiredLat) * sin(hover_criteria.desiredLon);
		double goal_z = R * sin(hover_criteria.desiredLat);

		double distance_m = sqrt(pow((i_x - goal_x), 2) + pow((i_y - goal_y), 2) + pow((i_z - goal_z), 2));
		if ((distance_m / 0.3048) >= hover_criteria.horDistTolFt) {
			return false;
		}

		return true;
	}
};

#endif // STABILIZE_HPP
