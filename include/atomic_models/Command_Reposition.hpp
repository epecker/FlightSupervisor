/**
 *	\brief		An atomic model representing the Command Reposition model.
 *	\details	This header file define the Command Reposition model as
				an atomic model for use in the Cadmium DEVS
				simulation software.
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef COMMAND_REPOSITION_HPP
#define COMMAND_REPOSITION_HPP

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

#include <limits> // Used to set the time advance to infinity
#include <cassert> // Used to check values and stop the simulation
#include <string>

 // Includes the macro DEFINE_ENUM_WITH_STRING_CONVERSIONS
#include "enum_string_conversion.hpp"
#include "Constants.hpp"
#include "mavNRC/geo.h"

// Data structures that are used in message transport
#include "message_structures/message_aircraft_state_t.hpp"
#include "message_structures/message_hover_criteria_t.hpp"
#include "message_structures/message_landing_point_t.hpp"
#include "message_structures/message_fcc_command_t.hpp"

using namespace cadmium;
using namespace std;

// Input and output port definitions
struct Command_Reposition_defs {
	struct i_aircraft_state : public in_port<message_aircraft_state_t> {};
	struct i_hover_criteria_met : public in_port<bool> {};
	struct i_pilot_handover : public in_port<message_landing_point_t> {};
	struct i_pilot_takeover : public in_port<bool> {};
	struct i_request_reposition : public in_port<message_landing_point_t> {};

	struct o_cancel_hover : public out_port<bool> {};
	struct o_fcc_command_velocity : public out_port<message_fcc_command_t> {};
	struct o_lp_criteria_met : public out_port<message_landing_point_t> {};
	struct o_request_aircraft_state : public out_port<bool> {};
	struct o_stabilize : public out_port<message_hover_criteria_t> {};
};

// Atomic Model
template<typename TIME> class Command_Reposition {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(REQUEST_STATE)
		(GET_STATE)
		(COMMAND_VEL)
		(COMMAND_HOVER)
		(STABILIZING)
		(LP_CRITERIA_MET)
		(LANDING)
		(CANCEL_HOVER)
		(TIMER_EXPIRED)
		(PILOT_CONTROL)
	);

	// Create a tuple of input ports (required for the simulator)
	using input_ports = tuple<
		typename Command_Reposition_defs::i_aircraft_state,
		typename Command_Reposition_defs::i_hover_criteria_met,
		typename Command_Reposition_defs::i_pilot_handover,
		typename Command_Reposition_defs::i_pilot_takeover,
		typename Command_Reposition_defs::i_request_reposition
	>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
		typename Command_Reposition_defs::o_cancel_hover,
		typename Command_Reposition_defs::o_fcc_command_velocity,
		typename Command_Reposition_defs::o_lp_criteria_met,
		typename Command_Reposition_defs::o_request_aircraft_state,
		typename Command_Reposition_defs::o_stabilize
	>;

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct state_type {
		States current_state;
	};
	state_type state;

	// Default constructor
	Command_Reposition() {
		state.current_state = States::IDLE;
		aircraft_state = message_aircraft_state_t();
		landing_point = message_landing_point_t();
	}

	// Constructor with initial state parameter for debugging or partial execution startup.
	explicit Command_Reposition(States initial_state) {
		state.current_state = initial_state;
		aircraft_state = message_aircraft_state_t();
		landing_point = message_landing_point_t();
	}

	// Internal transitions
	// These are transitions occurring from internal inputs
	// (required for the simulator)
	void internal_transition() {
		switch (state.current_state) {
			case States::REQUEST_STATE:
				state.current_state = States::GET_STATE;
				break;
			case States::COMMAND_VEL:
				state.current_state = States::COMMAND_HOVER;
				break;
			case States::COMMAND_HOVER:
				state.current_state = States::STABILIZING;
				break;
			case States::LP_CRITERIA_MET:
				state.current_state = States::LANDING;
				break;
			case States::CANCEL_HOVER:
				state.current_state = States::REQUEST_STATE;
				break;
			default:
				break;
		}
	}

	// External transitions
	// These are transitions occurring from external inputs
	// (required for the simulator)
	void external_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {
		bool received_aircraft_state;
		bool received_hover_criteria_met;
		bool received_pilot_handover;
		bool received_pilot_takeover;
		bool received_request_reposition;

		received_pilot_handover = !get_messages<typename Command_Reposition_defs::i_pilot_handover>(mbs).empty();
		received_pilot_takeover = !get_messages<typename Command_Reposition_defs::i_pilot_takeover>(mbs).empty();

		if (received_pilot_takeover) {
			state.current_state = States::PILOT_CONTROL;
		} else if (received_pilot_handover) {
			state.current_state = States::TIMER_EXPIRED;
		} else {
			switch (state.current_state) {
				case States::IDLE:
					received_request_reposition = !get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs).empty();

					if (received_request_reposition) {
						vector<message_landing_point_t> new_landing_points = get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs);
						landing_point = new_landing_points[0]; // set the new Landing 
						state.current_state = States::REQUEST_STATE;
					}
					break;
				case States::GET_STATE:
					received_aircraft_state = !get_messages<typename Command_Reposition_defs::i_aircraft_state>(mbs).empty();

					if (received_aircraft_state) {
						vector<message_aircraft_state_t> new_aircraft_state = get_messages<typename Command_Reposition_defs::i_aircraft_state>(mbs);
						aircraft_state = new_aircraft_state[0];
						state.current_state = States::COMMAND_VEL;
					}
					break;
				case States::COMMAND_VEL:
					received_request_reposition = !get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs).empty();

					if (received_request_reposition) {
						vector<message_landing_point_t> new_landing_points = get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs);
						landing_point = new_landing_points[0]; // set the new Landing 
						state.current_state = States::REQUEST_STATE;
					}
					break;
				case States::COMMAND_HOVER:
					received_request_reposition = !get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs).empty();

					if (received_request_reposition) {
						vector<message_landing_point_t> new_landing_points = get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs);
						landing_point = new_landing_points[0]; // set the new Landing 
						state.current_state = States::REQUEST_STATE;
					}
					break;
				case States::STABILIZING:
					received_hover_criteria_met = !get_messages<typename Command_Reposition_defs::i_hover_criteria_met>(mbs).empty();
					received_request_reposition = !get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs).empty();

					if (received_request_reposition) {
						vector<message_landing_point_t> new_landing_points = get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs);
						landing_point = new_landing_points[0]; // set the new Landing 
						state.current_state = States::CANCEL_HOVER;
					} else if (received_hover_criteria_met) {
						state.current_state = States::LP_CRITERIA_MET;
					}
					break;
				case States::LP_CRITERIA_MET:
					received_request_reposition = !get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs).empty();

					if (received_request_reposition) {
						vector<message_landing_point_t> new_landing_points = get_messages<typename Command_Reposition_defs::i_request_reposition>(mbs);
						landing_point = new_landing_points[0]; // set the new Landing 
						state.current_state = States::CANCEL_HOVER;
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
		external_transition(TIME(), move(mbs));
	}

	// output function
	[[nodiscard]] typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		vector<bool> bag_port_out;
		vector<message_landing_point_t> bag_port_LP_out;
		vector<message_fcc_command_t> bag_port_fcc_out;
		vector<message_hover_criteria_t> bag_port_hover_out;

		switch (state.current_state) {
			case States::REQUEST_STATE:
				bag_port_out.push_back(true);
				get_messages<typename Command_Reposition_defs::o_request_aircraft_state>(bags) = bag_port_out;
				break;
			case States::COMMAND_VEL:
			{
				message_fcc_command_t mfc = message_fcc_command_t();
				float distance, altitude;
				get_distance_to_point_global_wgs84(aircraft_state.lat, aircraft_state.lon, aircraft_state.alt_MSL, landing_point.lat, landing_point.lon, landing_point.alt * METERS_TO_FT, &distance, &altitude);
				float velocity = distance / (REPO_TIMER - 2.0f);
				if (velocity > MAX_REPO_VEL * KTS_TO_MPS) {
					velocity = MAX_REPO_VEL * KTS_TO_MPS;
				}
				mfc.change_velocity(velocity, aircraft_state.gps_time);
				bag_port_fcc_out.push_back(mfc);
				get_messages<typename Command_Reposition_defs::o_fcc_command_velocity>(bags) = bag_port_fcc_out;
			}
				break;
			case States::COMMAND_HOVER:
			{
				message_hover_criteria_t mhc = message_hover_criteria_t(
					landing_point.lat,
					landing_point.lon,
					landing_point.alt,
					landing_point.hdg,
					DEFAULT_LAND_CRITERIA_HOR_DIST,
					DEFAULT_LAND_CRITERIA_VERT_DIST,
					DEFAULT_LAND_CRITERIA_VEL,
					DEFAULT_LAND_CRITERIA_HDG,
					DEFAULT_LAND_CRITERIA_TIME,
					-1,
					0,
					0
				);
				bag_port_hover_out.push_back(mhc);
				get_messages<typename Command_Reposition_defs::o_stabilize>(bags) = bag_port_hover_out;
			}
				break;
			case States::CANCEL_HOVER:
				bag_port_out.push_back(true);
				get_messages<typename Command_Reposition_defs::o_cancel_hover>(bags) = bag_port_out;
				break;
			case States::LP_CRITERIA_MET:
				bag_port_LP_out.emplace_back(message_landing_point_t());
				get_messages<typename Command_Reposition_defs::o_lp_criteria_met>(bags) = bag_port_LP_out;
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
			case States::GET_STATE:
			case States::STABILIZING:
			case States::LANDING:
			case States::TIMER_EXPIRED:
			case States::PILOT_CONTROL:
				next_internal = numeric_limits<TIME>::infinity();
				break;
			case States::REQUEST_STATE:
			case States::COMMAND_VEL:
			case States::COMMAND_HOVER:
			case States::LP_CRITERIA_MET:
			case States::CANCEL_HOVER:
				next_internal = TIME(TA_ZERO);
				break;
			default:
				assert(false && "Unhandled state time advance.");
		}
		return next_internal;
	}

	friend ostringstream& operator<<(ostringstream& os, const typename Command_Reposition<TIME>::state_type& i) {
		os << (string("State: ") + enumToString(i.current_state) + string("\n"));
		return os;
	}

private:
	message_landing_point_t landing_point;
	message_aircraft_state_t aircraft_state;
};

#endif // COMMAND_REPOSITION_HPP
