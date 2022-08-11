/**
 *	\brief		An atomic model representing the Mission_Initialization model.
 *	\details	This header file defines the Mission_Initialization model as
				an atomic model for use in the Cadmium DEVS. It handles
				the startup behaviour of the aircraft.
				simulation software.
 *	\author		Tanner Trautrim
 */

#ifndef  MISSION_INITIALIZATION_HPP
#define  MISSION_INITIALIZATION_HPP

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

#include <limits>
#include <string>

#include "message_structures/message_aircraft_state_t.hpp"
#include "message_structures/message_start_supervisor_t.hpp"
#include "message_structures/message_update_gcs_t.hpp"

#include "enum_string_conversion.hpp"
#include "Constants.hpp"

using namespace cadmium;

template<typename TIME>
class Mission_Initialization {
public:
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(MISSION_STATUS)
		(RESUME_MISSION)
		(CHECK_AUTONOMY)
		(CHECK_PERCEPTION_SYSTEM)
		(OUTPUT_PERCEPTION_STATUS)
		(REQUEST_AIRCRAFT_STATE)
		(CHECK_AIRCRAFT_STATE)
		(OUTPUT_TAKEOFF_POSITION)
		(REQUIRE_MONITORING)
		(START_MISSION)
	);

	// Input and output port definitions
	struct defs {
		struct i_aircraft_state : public in_port<message_aircraft_state_t> {};
		struct i_perception_status : public in_port<bool> {};
		struct i_start_supervisor : public in_port<message_start_supervisor_t> {};

		struct o_request_perception_status : public out_port<bool> {};
		struct o_request_aircraft_state : public out_port<bool> {};
		struct o_set_mission_monitor_status : public out_port<uint8_t> {};
		struct o_start_mission : public out_port<bool> {};
		struct o_update_gcs : public out_port<message_update_gcs_t> {};
	};

	// Create a tuple of input ports (required for the simulator)
	using input_ports = tuple<
			typename Mission_Initialization::defs::i_aircraft_state,
			typename Mission_Initialization::defs::i_perception_status,
			typename Mission_Initialization::defs::i_start_supervisor
			>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
			typename Mission_Initialization::defs::o_request_perception_status,
			typename Mission_Initialization::defs::o_request_aircraft_state,
			typename Mission_Initialization::defs::o_set_mission_monitor_status,
			typename Mission_Initialization::defs::o_start_mission,
			typename Mission_Initialization::defs::o_update_gcs
			>;

	// Tracks the state of the model
	struct state_type {
		States current_state;
	};
	state_type state;

	bool mission_started;
	bool autonomy_armed;
	bool perception_healthy;
	double aircraft_height;

	Mission_Initialization() {
		state.current_state = States::IDLE;
		mission_started = false;
		autonomy_armed = false;
		perception_healthy = false;
		aircraft_height = 0.0;
	}

	explicit Mission_Initialization(States initial_state) {
		state.current_state = initial_state;
		mission_started = false;
		autonomy_armed = false;
		perception_healthy = false;
		aircraft_height = 0.0;
	}

	// Internal transitions (required for the simulator)
	void internal_transition() {
		switch (state.current_state) {
			case States::MISSION_STATUS:
				state.current_state = mission_started ? States::RESUME_MISSION: States::CHECK_AUTONOMY;
				break;
			case States::RESUME_MISSION:
				state.current_state = States::IDLE;
				break;
			case States::CHECK_AUTONOMY:
				state.current_state = state.current_state = autonomy_armed ? States::CHECK_PERCEPTION_SYSTEM: States::IDLE;
				break;
			case States::OUTPUT_PERCEPTION_STATUS:
				state.current_state = States::REQUEST_AIRCRAFT_STATE;
				break;
			case States::REQUEST_AIRCRAFT_STATE:
				state.current_state = States::CHECK_AIRCRAFT_STATE;
				break;
			case States::OUTPUT_TAKEOFF_POSITION:
				state.current_state = States::START_MISSION;
				break;
			case States::START_MISSION:
				state.current_state = States::IDLE;
				break;
			default:
				return;
		}
	}

	// External transitions (required for the simulator)
	void external_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {
		bool received_start_supervisor;
		bool received_perception_status;
		bool received_aircraft_state;

		switch (state.current_state) {
			case States::IDLE:
				received_start_supervisor = !get_messages<typename Mission_Initialization::defs::i_start_supervisor>(mbs).empty();
				if (received_start_supervisor) {
					vector<message_start_supervisor_t> new_mission_data = get_messages<typename Mission_Initialization::defs::i_start_supervisor>(mbs);
					// Get the most recent start supervisor input (found at the back of the vector of inputs) 
					message_start_supervisor_t latest_start_supervisor = new_mission_data.back();
					mission_started = latest_start_supervisor.mission_started;
					autonomy_armed = latest_start_supervisor.autonomy_armed;
					state.current_state = States::MISSION_STATUS;
				}
				break;
			case States::CHECK_PERCEPTION_SYSTEM:
				received_perception_status = !get_messages<typename Mission_Initialization::defs::i_perception_status>(mbs).empty();
				if (received_perception_status) {
					vector<bool> perception_status = get_messages<typename Mission_Initialization::defs::i_perception_status>(mbs);
					perception_healthy = perception_status[0];
					state.current_state = States::OUTPUT_PERCEPTION_STATUS;
				}
				break;
			case States::CHECK_AIRCRAFT_STATE:
				received_aircraft_state = !get_messages<typename Mission_Initialization::defs::i_aircraft_state>(mbs).empty();

				if (received_aircraft_state) {
					vector<message_aircraft_state_t> new_aircraft_state = get_messages<typename Mission_Initialization::defs::i_aircraft_state>(mbs);
					aircraft_height = new_aircraft_state[0].alt_AGL;
					state.current_state = States::OUTPUT_TAKEOFF_POSITION;
				}
				break;
			default:
				break;
		}
	}

	// Confluence transition sets the internal/external precedence
	// Triggered when a message is received at the same time as an internal transition.
	// (required for the simulator)
	void confluence_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), move(mbs));
	}

	// Creates output messages (required for the simulator)
	[[nodiscard]]
	typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		vector<bool> bool_port_out;
		vector<message_update_gcs_t> gcs_messages;
		vector<uint8_t> mission_monitor_messages;

		switch (state.current_state) {
			case States::CHECK_AUTONOMY:
				{
					if (autonomy_armed) {
						bool_port_out.push_back(true);
						get_messages<typename Mission_Initialization::defs::o_request_perception_status>(bags) = bool_port_out;
					}
				}
				break;
			case States::OUTPUT_PERCEPTION_STATUS:
				{
					message_update_gcs_t temp_gcs_update;
					if (perception_healthy) {
						temp_gcs_update.text = "The perceptions system is ready for operation!";
					} else {
						temp_gcs_update.text = "The perception system is not operational!";
					}
					temp_gcs_update.severity = Mav_Severities_E::MAV_SEVERITY_ALERT;
					gcs_messages.emplace_back(temp_gcs_update);
					get_messages<typename Mission_Initialization::defs::o_update_gcs>(bags) = gcs_messages;
				}
				break;
			case States::REQUEST_AIRCRAFT_STATE:
				{
					bool_port_out.push_back(true);
					get_messages<typename Mission_Initialization::defs::o_request_aircraft_state>(bags) = bool_port_out;
				}
				break;
			case States::OUTPUT_TAKEOFF_POSITION:
				{
					mission_monitor_messages.emplace_back(1);
					get_messages<typename Mission_Initialization::defs::o_set_mission_monitor_status>(bags) = mission_monitor_messages;

					if (aircraft_height > 10.0) {
						message_update_gcs_t temp_gcs_update;
						temp_gcs_update.text = "Starting Mission in air!";
						temp_gcs_update.severity = Mav_Severities_E::MAV_SEVERITY_ALERT;
						gcs_messages.emplace_back(temp_gcs_update);
						get_messages<typename Mission_Initialization::defs::o_update_gcs>(bags) = gcs_messages;
					}
				}
				break;
			case States::START_MISSION:
				{
					bool_port_out.push_back(true);
					get_messages<typename Mission_Initialization::defs::o_start_mission>(bags) = bool_port_out;
				}
				break;
			default:
				break;
		}

		return bags;
	}

	// Time advance sets the wait time of the current state (required for the simulator)
	TIME time_advance() const {
		TIME next_internal;
		switch (state.current_state) {
			case States::IDLE:
			case States::CHECK_PERCEPTION_SYSTEM:
			case States::CHECK_AIRCRAFT_STATE:
				next_internal = numeric_limits<TIME>::infinity();
				break;
			case States::MISSION_STATUS:
			case States::RESUME_MISSION:
			case States::CHECK_AUTONOMY:
			case States::OUTPUT_PERCEPTION_STATUS:
			case States::REQUEST_AIRCRAFT_STATE:
			case States::OUTPUT_TAKEOFF_POSITION:
			case States::START_MISSION:
				next_internal = TIME(TA_ZERO);
				break;
			default:
				assert(false && "Unhandled time advance");
				break;
		}
		return next_internal;
	}

	// Used for logging outputs the state's name. (required for the simulator)
	friend ostringstream& operator<<(ostringstream& os, const typename Mission_Initialization<TIME>::state_type& i) {
		os << (string("State: ") + enumToString(i.current_state) + string("\n"));
		return os;
	}
};

#endif // MISSION_INITIALIZATION_HPP
