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

#include "mavNRC/geo.h"
#include "enum_string_conversion.hpp"
#include "time_conversion.hpp"
#include "Constants.hpp"

using namespace cadmium;
using namespace std;

// Input and output port definition
struct Stabilize_defs {
	struct i_aircraft_state : public in_port<message_aircraft_state_t> {};
	struct i_cancel_hover : public in_port<bool> {};
	struct i_gps_time : public in_port<double> {};
	struct i_stabilize : public in_port<message_hover_criteria_t> {};

	struct o_fcc_command_hover : public out_port<message_fcc_command_t> {};
	struct o_hover_criteria_met : public out_port<bool> {};
	struct o_request_aircraft_state : public out_port<bool> {};
	struct o_request_gps_time : public out_port<bool> {};
};

// Atomic Model
template<typename TIME> class Stabilize {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(REQUEST_GPS_TIME)
		(GET_GPS_TIME)
		(INIT_HOVER)
		(STABILIZING)
		(CHECK_STATE)
		(HOVER)
	)

		// Create a tuple of input ports (required for the simulator)
		using input_ports = tuple<
		typename Stabilize_defs::i_aircraft_state,
		typename Stabilize_defs::i_cancel_hover,
		typename Stabilize_defs::i_gps_time,
		typename Stabilize_defs::i_stabilize
		>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
		typename Stabilize_defs::o_fcc_command_hover,
		typename Stabilize_defs::o_hover_criteria_met,
		typename Stabilize_defs::o_request_aircraft_state,
		typename Stabilize_defs::o_request_gps_time
	>;

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct state_type {
		States current_state;
		bool in_tolerance;
		bool time_tolerance_met;
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
		gps_time = 0.0;
		hover_criteria = message_hover_criteria_t();
		aircraft_state = message_aircraft_state_t();
	}

	// Default constructor
	explicit Stabilize(TIME polling_rate) : polling_rate(polling_rate) {
		state.current_state = States::IDLE;
		state.in_tolerance = false;
		state.time_tolerance_met = false;
		gps_time = 0.0;
		hover_criteria = message_hover_criteria_t();
		aircraft_state = message_aircraft_state_t();
	}

	// Constructor with initial state parameter for debugging or partial execution startup.
	explicit Stabilize(States initial_state) {
		state.current_state = initial_state;
		state.in_tolerance = false;
		state.time_tolerance_met = false;
		polling_rate = TIME("00:00:00:100");
		gps_time = 0.0;
		hover_criteria = message_hover_criteria_t();
		aircraft_state = message_aircraft_state_t();
	}

	// Constructor with initial state parameter for debugging or partial execution startup.
	Stabilize(TIME polling_rate, States initial_state) : polling_rate(polling_rate) {
		state.current_state = initial_state;
		state.in_tolerance = false;
		state.time_tolerance_met = false;
		gps_time = 0.0;
		hover_criteria = message_hover_criteria_t();
		aircraft_state = message_aircraft_state_t();
	}

	// Internal transitions
	// These are transitions occurring from internal inputs
	// There are no internal transitions
	// (required for the simulator)
	void internal_transition() {
		switch (state.current_state) {
			case States::REQUEST_GPS_TIME:
				state.current_state = States::GET_GPS_TIME;
				break;
			case States::INIT_HOVER:
				state.current_state = States::STABILIZING;
				break;
			case States::STABILIZING:
				if (state.in_tolerance && state.time_tolerance_met) {
					state.current_state = States::HOVER;
				}
				else {
					state.current_state = States::CHECK_STATE;
				}
				break;
			case States::HOVER:
				state.current_state = States::IDLE;
				break;
			default:
				break;
		}
	}

	// External transitions
	// These are transitions occurring from external inputs
	// (required for the simulator)
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		bool received_gps_time;
		bool received_cancel_hover = !get_messages<typename Stabilize_defs::i_cancel_hover>(mbs).empty();

		if (received_cancel_hover) {
			state.current_state = States::IDLE;
			aircraft_state = message_aircraft_state_t();
			hover_criteria = message_hover_criteria_t();
		} else {
			switch (state.current_state) {
				case States::IDLE:
					if (!get_messages<typename Stabilize_defs::i_stabilize>(mbs).empty()) {
						state.current_state = States::REQUEST_GPS_TIME;
						hover_criteria = get_messages<typename Stabilize_defs::i_stabilize>(mbs)[0];
						stabilization_time_prev = seconds_to_time<TIME>(hover_criteria.timeTol);
					}
					break;
				case States::GET_GPS_TIME:
					received_gps_time = !get_messages<typename Stabilize_defs::i_gps_time>(mbs).empty();
					if (received_gps_time) {
						gps_time = get_messages<typename Stabilize_defs::i_gps_time>(mbs)[0];
						state.current_state = States::INIT_HOVER;
					}
					break;
				case States::CHECK_STATE:
					if (!get_messages<typename Stabilize_defs::i_aircraft_state>(mbs).empty()) {
						state.current_state = States::STABILIZING;
						aircraft_state = get_messages<typename Stabilize_defs::i_aircraft_state>(mbs)[0];
						state.in_tolerance = calculate_hover_criteria_met(aircraft_state);
						if (!state.in_tolerance) {
							stabilization_time_prev = seconds_to_time<TIME>(hover_criteria.timeTol);
						} else {
							stabilization_time_prev = stabilization_time_prev - e;
							state.time_tolerance_met = (stabilization_time_prev <= TIME("00:00:00:000"));
						}
					}
					break;
				default:
					break;
			}
		}
	}

	// confluence transition
	// Used to call set call order
	void confluence_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {
		bool received_cancel_hover = !get_messages<typename Stabilize_defs::i_cancel_hover>(mbs).empty();

		if (received_cancel_hover) {
			external_transition(TIME(), move(mbs));
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

		switch (state.current_state) {
			case States::REQUEST_GPS_TIME:
				message_out.push_back(true);
				get_messages<typename Stabilize_defs::o_request_gps_time>(bags) = message_out;
			case States::INIT_HOVER:
			{
				message_fcc_command_t mfc = message_fcc_command_t();
				mfc.reposition(
						gps_time,
						hover_criteria.desiredLat * (1E7),
						hover_criteria.desiredLon * (1E7),
						hover_criteria.desiredAltMSL
						);
				message_fcc_out.push_back(mfc);
				get_messages<typename Stabilize_defs::o_fcc_command_hover>(bags) = message_fcc_out;
			}
			break;
			case States::STABILIZING:
				if (state.in_tolerance && state.time_tolerance_met) {
					message_out.push_back(true);
					get_messages<typename Stabilize_defs::o_hover_criteria_met>(bags) = message_out;
				}
				else {
					message_out.push_back(false);
					get_messages<typename Stabilize_defs::o_request_aircraft_state>(bags) = message_out;
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
			case States::GET_GPS_TIME:
			case States::CHECK_STATE:
				next_internal = numeric_limits<TIME>::infinity();
				break;
			case States::REQUEST_GPS_TIME:
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
		os << (string("State: ") + enumToString(i.current_state) + i.failures + string("\n"));
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
		if (abs(i_state.hdg_Deg - hover_criteria.desiredHdgDeg) >= hover_criteria.hdgToleranceDeg && !isnan(hover_criteria.desiredHdgDeg)) {
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
	double gps_time;
	TIME stabilization_time_prev;
	TIME polling_rate;
};

#endif // STABILIZE_HPP
