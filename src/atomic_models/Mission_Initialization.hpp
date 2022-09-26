/**
 * 	\file		Mission_Initialization.hpp
 *	\brief		Definition of the Mission Initialization atomic model.
 *	\details	This header file defines the Mission Initialization atomic model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor at the beginning
				of the mission when the autonomy system is being initialized.
 *	\image		html atomic_models/mission_initialization.png
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef  MISSION_INITIALIZATION_HPP
#define  MISSION_INITIALIZATION_HPP

// Messages structures
#include "../message_structures/message_aircraft_state_t.hpp"
#include "../message_structures/message_start_supervisor_t.hpp"
#include "../message_structures/message_update_gcs_t.hpp"

// Utility functions
#include "../enum_string_conversion.hpp"
#include "../Constants.hpp"

// Cadmium Simulator Headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

// System Libraries
#include <limits> // Used to set the time advance to infinity
#include <string>

/**
 * 	\class		Mission_Initialization
 *	\brief		Definition of the Mission Initialization atomic model.
 *	\details	This class defines the Mission Initialization atomic model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor at the beginning
				of the mission when the autonomy system is being initialized.
 *	\image		html atomic_models/mission_initialization.png
 */
template<typename TIME>
class Mission_Initialization {
public:
	/**
	 *	\par	States
	 * 	Declaration of the states of the atomic model.
	 */
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

	/**
	 *	\brief	For definition of the input and output ports see:
	 *	\ref 	Mission_Initialization_input_ports "Input Ports" and
	 *	\ref 	Mission_Initialization_output_ports "Output Ports"
	 * 	\note 	All input and output ports must be listed in this struct.
	 */
	struct defs {
		struct i_aircraft_state : public cadmium::in_port<message_aircraft_state_t> {};
		struct i_perception_status : public cadmium::in_port<bool> {};
		struct i_start_supervisor : public cadmium::in_port<message_start_supervisor_t> {};

		struct o_request_perception_status : public cadmium::out_port<bool> {};
		struct o_request_aircraft_state : public cadmium::out_port<bool> {};
		struct o_set_mission_monitor_status : public cadmium::out_port<uint8_t> {};
		struct o_start_mission : public cadmium::out_port<int> {};
		struct o_update_gcs : public cadmium::out_port<message_update_gcs_t> {};
	};

	/**
	 * 	\anchor	Mission_Initialization_input_ports
	 *	\par	Input Ports
	 * 	Definition of the input ports for the model.
	 * 	\param 	i_aircraft_state 	Port for receiving the current state of the aircraft.
	 * 	\param	i_perception_status	Port for receiving the status of the perception system.
	 * 	\param 	i_start_mission 	Port for receiving signal to start the supervisor.
	 */
	using input_ports = std::tuple<
		typename defs::i_aircraft_state,
		typename defs::i_perception_status,
		typename defs::i_start_supervisor
	>;

	/**
	 *	\anchor	Mission_Initialization_output_ports
	 * 	\par 	Output Ports
	 * 	Definition of the output ports for the model.
	 * 	\param	o_request_perception_status		Port for requesting the current state of the perception system.
	 * 	\param	o_request_aircraft_state 		Port for requesting the current aircraft state.
	 * 	\param	o_set_mission_monitor_status 	Port for telling the mission monitor to stop monitoring mission progress.
	 *	\param	o_start_mission					Port for sending a notification that the mission has started.
	 * 	\param	o_update_gcs 					Port for sending updates to the GCS.
	 */
	using output_ports = std::tuple<
		typename defs::o_request_perception_status,
		typename defs::o_request_aircraft_state,
		typename defs::o_set_mission_monitor_status,
		typename defs::o_start_mission,
		typename defs::o_update_gcs
	>;

	/**
	 *	\anchor	Handle_Waypoint_state_type
	 *	\par	State
	 * 	Definition of the states of the atomic model.
	 * 	\param 	current_state 	Current state of atomic model.
	 */
	struct state_type {
		States current_state;
	} state;

	/**
	 * \brief 	Default constructor for the model.
	 */
	Mission_Initialization() {
		state.current_state = States::IDLE;
        mission_data = message_start_supervisor_t();
		perception_healthy = false;
		aircraft_height = 0.0;
	}

	/**
	 * \brief 	Constructor for the model with initial state parameter
	 * 			for debugging or partial execution startup.
	 * \param	initial_state	States initial state of the model.
	 */
	explicit Mission_Initialization(States initial_state) {
		state.current_state = initial_state;
        mission_data = message_start_supervisor_t();
		perception_healthy = false;
		aircraft_height = 0.0;
	}

	/// Internal transitions of the model
	void internal_transition() {
		switch (state.current_state) {
			case States::MISSION_STATUS:
				state.current_state = mission_data.mission_started ? States::RESUME_MISSION: States::CHECK_AUTONOMY;
				break;
			case States::RESUME_MISSION:
				state.current_state = States::IDLE;
				break;
			case States::CHECK_AUTONOMY:
				state.current_state = state.current_state = mission_data.autonomy_armed ? States::CHECK_PERCEPTION_SYSTEM: States::IDLE;
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

	/// External transitions of the model
	void external_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		switch (state.current_state) {
			case States::IDLE: {
				bool received_start_supervisor = !cadmium::get_messages<typename defs::i_start_supervisor>(mbs).empty();
				if (received_start_supervisor) {
					// Get the most recent start supervisor input (found at the back of the vector of inputs)
					mission_data = cadmium::get_messages<typename defs::i_start_supervisor>(mbs).back();
					state.current_state = States::MISSION_STATUS;
				}
				break;
			}
			case States::CHECK_PERCEPTION_SYSTEM: {
				bool received_perception_status = !cadmium::get_messages<typename defs::i_perception_status>(mbs).empty();
				if (received_perception_status) {
					std::vector<bool> perception_status = cadmium::get_messages<typename defs::i_perception_status>(mbs);
					perception_healthy = perception_status[0];
					state.current_state = States::OUTPUT_PERCEPTION_STATUS;
				}
				break;
			}
			case States::CHECK_AIRCRAFT_STATE: {
				bool received_aircraft_state = !cadmium::get_messages<typename defs::i_aircraft_state>(mbs).empty();
				if (received_aircraft_state) {
					std::vector<message_aircraft_state_t> new_aircraft_state = cadmium::get_messages<typename defs::i_aircraft_state>(mbs);
					aircraft_height = new_aircraft_state[0].alt_AGL;
					state.current_state = States::OUTPUT_TAKEOFF_POSITION;
				}
				break;
			}
			default:
				break;
		}
	}

	/// Function used to decide precedence between internal and external transitions when both are scheduled simultaneously.
	void confluence_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), std::move(mbs));
	}

	/// Function for generating output from the model before internal transitions.
	[[nodiscard]] typename cadmium::make_message_bags<output_ports>::type output() const {
		typename cadmium::make_message_bags<output_ports>::type bags;

		switch (state.current_state) {
			case States::CHECK_AUTONOMY: {
				if (mission_data.autonomy_armed) {
					cadmium::get_messages<typename defs::o_request_perception_status>(bags).emplace_back(true);
				}
				break;
			}
			case States::OUTPUT_PERCEPTION_STATUS: {
				std::string update_text;
				if (perception_healthy) {
					update_text = "The perceptions system is ready for operation!";
				} else {
					update_text = "The perception system is not operational!";
				}
				cadmium::get_messages<typename defs::o_update_gcs>(bags).emplace_back(
						update_text,
						Mav_Severities_E::MAV_SEVERITY_ALERT
				);
				break;
			}
			case States::REQUEST_AIRCRAFT_STATE: {
				cadmium::get_messages<typename defs::o_request_aircraft_state>(bags).emplace_back(true);
				break;
			}
			case States::OUTPUT_TAKEOFF_POSITION: {
				cadmium::get_messages<typename defs::o_set_mission_monitor_status>(bags).emplace_back(1);

				if (aircraft_height > 10.0) {
					cadmium::get_messages<typename defs::o_update_gcs>(bags).emplace_back(
							"Starting Mission in air!",
							Mav_Severities_E::MAV_SEVERITY_ALERT
					);
				}
				break;
			}
			case States::START_MISSION: {
				cadmium::get_messages<typename defs::o_start_mission>(bags).push_back(mission_data.mission_number);
				break;
			}
			default:
				break;
		}

		return bags;
	}

	/// Function to declare the time advance value for each state of the model.
	TIME time_advance() const {
		TIME next_internal;
		switch (state.current_state) {
			case States::IDLE:
			case States::CHECK_PERCEPTION_SYSTEM:
			case States::CHECK_AIRCRAFT_STATE:
				next_internal = std::numeric_limits<TIME>::infinity();
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

	/**
	 *  \brief 		Operator for defining how the model state will be represented as a string.
	 * 	\warning 	Prepended "State: " is required for log parsing, do not remove.
	 */
	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Mission_Initialization<TIME>::state_type& i) {
		os << (std::string("State: ") + enumToString(i.current_state) + std::string("\n"));
		return os;
	}

private:
    /// Variable for storing the startup data about the mission.
    message_start_supervisor_t mission_data;
	/// Variable for storing whether the perception system is healthy or not.
	bool perception_healthy;
	/// Variable for storing the height of the aircraft in ft AGL to determine if the aircraft is starting on the ground.
	double aircraft_height;
};

#endif // MISSION_INITIALIZATION_HPP
