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
#include <cassert> // Used to check values and stop the simulation
#include <string>

#include "message_structures/message_hover_criteria_t.hpp"
#include "message_structures/message_aircraft_state_t.hpp"
#include "message_structures/message_fcc_command_t.hpp"
#include "message_structures/message_update_gcs_t.hpp"

#include "mavNRC/geo.h"
#include "enum_string_conversion.hpp"
#include "time_conversion.hpp"
#include "Constants.hpp"

using namespace cadmium;

// Atomic Model
template<typename TIME> class Stabilize {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(WAIT_STABILIZE)
		(REQUEST_AIRCRAFT_STATE)
		(GET_AIRCRAFT_STATE)
		(INIT_HOVER)
		(STABILIZING)
		(CHECK_STATE)
		(HOVER)
	)

	// Input and output port definition
	struct defs {
		struct i_aircraft_state : public in_port<message_aircraft_state_t> {};
		struct i_cancel_hover : public in_port<bool> {};
		struct i_stabilize : public in_port<message_hover_criteria_t> {};
		struct i_start_mission : public in_port<int> {};

		struct o_fcc_command_hover : public out_port<message_fcc_command_t> {};
		struct o_hover_criteria_met : public out_port<bool> {};
		struct o_request_aircraft_state : public out_port<bool> {};
		struct o_update_gcs : public out_port<message_update_gcs_t> {};
	};

	// Create a tuple of input ports (required for the simulator)
	using input_ports = tuple<
		typename Stabilize::defs::i_aircraft_state,
		typename Stabilize::defs::i_cancel_hover,
		typename Stabilize::defs::i_stabilize,
		typename Stabilize::defs::i_start_mission
    >;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
		typename Stabilize::defs::o_fcc_command_hover,
		typename Stabilize::defs::o_hover_criteria_met,
		typename Stabilize::defs::o_request_aircraft_state,
		typename Stabilize::defs::o_update_gcs
	>;

	// This is used to track the state of the atomic model.
	// (required for the simulator)
	struct state_type {
		States current_state;
		bool in_tolerance;
		bool time_tolerance_met;
		TIME stabilization_time_prev;
        #ifdef DEBUG_MODELS
        string failures;
        #endif
	} state;

	// Default constructor
	Stabilize() {
		state.current_state = States::IDLE;
		state.in_tolerance = false;
		state.time_tolerance_met = false;
		polling_rate = TIME("00:00:00:100");
		state.stabilization_time_prev = TIME("00:00:00:000");
		hover_criteria = message_hover_criteria_t();
		aircraft_state = message_aircraft_state_t();
	}

	// Default constructor
	explicit Stabilize(TIME polling_rate) {
		state.current_state = States::IDLE;
		state.in_tolerance = false;
		state.time_tolerance_met = false;
		this->polling_rate = polling_rate;
		state.stabilization_time_prev = TIME("00:00:00:000");
		hover_criteria = message_hover_criteria_t();
		aircraft_state = message_aircraft_state_t();
	}

	// Constructor with initial state parameter for debugging or partial execution startup.
	explicit Stabilize(States initial_state) {
		state.current_state = initial_state;
		state.in_tolerance = false;
		state.time_tolerance_met = false;
		polling_rate = TIME("00:00:00:100");
		state.stabilization_time_prev = TIME("00:00:00:000");
		hover_criteria = message_hover_criteria_t();
		aircraft_state = message_aircraft_state_t();
	}

	// Constructor with initial state parameter for debugging or partial execution startup.
	Stabilize(TIME polling_rate, States initial_state) {
		state.current_state = initial_state;
		state.in_tolerance = false;
		state.time_tolerance_met = false;
		this->polling_rate = polling_rate;
		state.stabilization_time_prev = TIME("00:00:00:000");
		hover_criteria = message_hover_criteria_t();
		aircraft_state = message_aircraft_state_t();
	}

	// Internal transitions
	// These are transitions occurring from internal inputs
	// There are no internal transitions
	// (required for the simulator)
	void internal_transition() {
		switch (state.current_state) {
			case States::REQUEST_AIRCRAFT_STATE:
				state.current_state = States::GET_AIRCRAFT_STATE;
				break;
			case States::INIT_HOVER:
				state.current_state = States::STABILIZING;
				break;
			case States::STABILIZING:
				if (state.time_tolerance_met && state.in_tolerance) {
					state.current_state = States::HOVER;
				} else {
					state.current_state = States::CHECK_STATE;
				}
				break;
			case States::HOVER:
                reset_state();
				state.current_state = States::WAIT_STABILIZE;
				break;
			default:
				break;
		}
	}

	// External transitions
	// These are transitions occurring from external inputs
	// (required for the simulator)
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		bool received_cancel_hover = !get_messages<typename Stabilize::defs::i_cancel_hover>(mbs).empty();
		bool received_start_mission = !get_messages<typename Stabilize::defs::i_start_mission>(mbs).empty();
		if (received_cancel_hover || received_start_mission) {
			reset_state();
            state.current_state = States::WAIT_STABILIZE;
			return;
		}

        bool received_aircraft_state;
        bool received_stabilize;
		switch (state.current_state) {
			case States::WAIT_STABILIZE:
				received_stabilize = !get_messages<typename Stabilize::defs::i_stabilize>(mbs).empty();
				if (received_stabilize) {
					// Get the most recent hover criteria input (found at the back of the vector of inputs)
					hover_criteria = get_messages<typename Stabilize::defs::i_stabilize>(mbs).back();
					state.stabilization_time_prev = seconds_to_time<TIME>(hover_criteria.timeTol);
					state.current_state = States::REQUEST_AIRCRAFT_STATE;
				}
				break;
			case States::GET_AIRCRAFT_STATE:
				received_aircraft_state = !get_messages<typename Stabilize::defs::i_aircraft_state>(mbs).empty();
				if (received_aircraft_state) {
					aircraft_state = get_messages<typename Stabilize::defs::i_aircraft_state>(mbs)[0];
					state.current_state = States::INIT_HOVER;
				}
				break;
			case States::CHECK_STATE:
				received_aircraft_state = !get_messages<typename Stabilize::defs::i_aircraft_state>(mbs).empty();
				if (received_aircraft_state) {
					aircraft_state = get_messages<typename Stabilize::defs::i_aircraft_state>(mbs)[0];
					state.in_tolerance = calculate_hover_criteria_met(aircraft_state);
					if (!state.in_tolerance) {
						state.stabilization_time_prev = seconds_to_time<TIME>(hover_criteria.timeTol);
					} else {
						state.stabilization_time_prev = state.stabilization_time_prev - (polling_rate + e);
						state.time_tolerance_met = (state.stabilization_time_prev <= TIME("00:00:00:000"));
					}
					state.current_state = States::STABILIZING;
				}
				break;
			default:
				break;
		}
	}

	// confluence transition;
	// Used to call set call order
	void confluence_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {
		bool received_cancel_hover = !get_messages<typename Stabilize::defs::i_cancel_hover>(mbs).empty();

		if (received_cancel_hover) {
			external_transition(TIME(), std::move(mbs));
		} else {
			internal_transition();
		}
	}

	// output function
	// Nothing to output
	[[nodiscard]] typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		vector<bool> message_out;
		vector<message_fcc_command_t> message_fcc_out;
		vector<message_update_gcs_t> message_gcs_out;

		switch (state.current_state) {
			case States::REQUEST_AIRCRAFT_STATE:
				message_out.push_back(true);
				get_messages<typename Stabilize::defs::o_request_aircraft_state>(bags) = message_out;
				break;
			case States::INIT_HOVER:
			{
				message_fcc_command_t mfc = message_fcc_command_t();
				mfc.reposition(
						aircraft_state.gps_time,
						hover_criteria.desiredLat * (1E7),
						hover_criteria.desiredLon * (1E7),
						hover_criteria.desiredAltMSL * FT_TO_METERS
						);
				message_fcc_out.push_back(mfc);
				get_messages<typename Stabilize::defs::o_fcc_command_hover>(bags) = message_fcc_out;
			}
			break;
			case States::STABILIZING:
				if (state.time_tolerance_met && state.in_tolerance) {
					message_update_gcs_t temp_gcs_update("Came to hover!", Mav_Severities_E::MAV_SEVERITY_INFO);
					message_out.push_back(true);
					message_gcs_out.push_back(temp_gcs_update);
					get_messages<typename Stabilize::defs::o_hover_criteria_met>(bags) = message_out;
					get_messages<typename Stabilize::defs::o_update_gcs>(bags) = message_gcs_out;
				} else {
					message_out.push_back(true);
					get_messages<typename Stabilize::defs::o_request_aircraft_state>(bags) = message_out;
				}
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
			case States::WAIT_STABILIZE:
			case States::GET_AIRCRAFT_STATE:
			case States::CHECK_STATE:
				next_internal = numeric_limits<TIME>::infinity();
				break;
			case States::REQUEST_AIRCRAFT_STATE:
			case States::INIT_HOVER:
			case States::HOVER:
				next_internal = TIME(TA_ZERO);
				break;
			case States::STABILIZING:
				next_internal = polling_rate;
				break;
			default:
				assert(false && "Unhandled state time advance.");
		}
		return next_internal;
	}

	friend ostringstream& operator<<(ostringstream& os, const typename Stabilize<TIME>::state_type& i) {
        #ifdef DEBUG_MODELS
        os << (string("State: ") + enumToString(i.current_state) + i.failures + "-") << i.stabilization_time_prev << string("\n");
        #else
        os << (string("State: ") + enumToString(i.current_state) + string("\n"));
        #endif
		return os;
	}

	// Stub implementation for now so we can always hover.
	bool calculate_hover_criteria_met(message_aircraft_state_t i_state) {

		if (abs(i_state.alt_MSL - hover_criteria.desiredAltMSL) >= hover_criteria.vertDistTolFt) {
            #ifdef DEBUG_MODELS
            state.failures = "-FAILED-ALT";
            #endif
			return false;
		}
		//If the heading is negative wrap back into 0-360
		while (i_state.hdg_Deg < 0.0) {
			i_state.hdg_Deg += 360;
		}
		if (!isnan(hover_criteria.desiredHdgDeg) && abs(i_state.hdg_Deg - hover_criteria.desiredHdgDeg) >= hover_criteria.hdgToleranceDeg) {
            #ifdef DEBUG_MODELS
            state.failures = "-FAILED-HDG";
            #endif
			return false;
		}
		if (abs(i_state.vel_Kts) >= hover_criteria.velTolKts) {
            #ifdef DEBUG_MODELS
            state.failures = "-FAILED-VEL";
            #endif
			return false;
		}

		//Radius of the earth in meters (WGS-84).
		// const float R = 6378137.0;

		// double i_x = R * cos(i_state.lat) * cos(i_state.lon);
		// double i_y = R * cos(i_state.lat) * sin(i_state.lon);
		// double i_z = R * sin(i_state.lat);

		// double goal_x = R * cos(hover_criteria.desiredLat) * cos(hover_criteria.desiredLon);
		// double goal_y = R * cos(hover_criteria.desiredLat) * sin(hover_criteria.desiredLon);
		// double goal_z = R * sin(hover_criteria.desiredLat);

		// double distance_m = sqrt(pow((i_x - goal_x), 2) + pow((i_y - goal_y), 2) + pow((i_z - goal_z), 2));

		float dist_xy_m, dist_z_m;
		get_distance_to_point_global_wgs84(
			i_state.lat, i_state.lon, i_state.alt_MSL,
			hover_criteria.desiredLat, hover_criteria.desiredLon, hover_criteria.desiredAltMSL,
			&dist_xy_m, &dist_z_m);

		if ((dist_xy_m * METERS_TO_FT) >= hover_criteria.horDistTolFt) {
            #ifdef DEBUG_MODELS
            state.failures = string("-FAILED-DIS-") + std::to_string(dist_xy_m * METERS_TO_FT);
            #endif
			return false;
		}

        #ifdef DEBUG_MODELS
        state.failures = string("");
        #endif
		return true;
	}

private:
	message_hover_criteria_t hover_criteria;
	message_aircraft_state_t aircraft_state;
	TIME polling_rate;

	void reset_state() {
		state.stabilization_time_prev = TIME("00:00:000");
		state.in_tolerance = false;
		state.time_tolerance_met = false;
	}
};

#endif // STABILIZE_HPP
