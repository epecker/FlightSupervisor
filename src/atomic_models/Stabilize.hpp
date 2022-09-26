/**
 * 	\file		Stabilize.hpp
 *	\brief		Definition of the Stabilize atomic model.
 *	\details	This header file defines the Stabilize atomic model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor when
				stabilizing at a given hover criteria.
 *	\image		html atomic_models/stabilize.png
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef STABILIZE_HPP
#define STABILIZE_HPP

// Messages structures
#include "../message_structures/message_hover_criteria_t.hpp"
#include "../message_structures/message_aircraft_state_t.hpp"
#include "../message_structures/message_fcc_command_t.hpp"
#include "../message_structures/message_update_gcs_t.hpp"

// Utility functions
#include "../enum_string_conversion.hpp"
#include "../time_conversion.hpp"
#include <mavNRC/geo.h>
#include "../Constants.hpp"

// Cadmium Simulator Headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

// System Libraries
#include <limits> // Used to set the time advance to infinity
#include <cassert>
#include <string>

/**
 * 	\class		Stabilize
 *	\brief		Definition of the Stabilize atomic model.
 *	\details	This class defines the Stabilize atomic model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor when
				stabilizing at a given hover criteria.
 *	\image		html atomic_models/stabilize.png
 */
template<typename TIME>
class Stabilize {
public:
	/**
	 *	\par	States
	 * 	Declaration of the states of the atomic model.
	 */
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

	/**
	 *	\brief	For definition of the input and output ports see:
	 *	\ref 	Stabilize_input_ports "Input Ports" and
	 *	\ref 	Stabilize_output_ports "Output Ports"
	 * 	\note 	All input and output ports must be listed in this struct.
	 */
	struct defs {
		struct i_aircraft_state : public cadmium::in_port<message_aircraft_state_t> {};
		struct i_cancel_hover : public cadmium::in_port<bool> {};
		struct i_stabilize : public cadmium::in_port<message_hover_criteria_t> {};
		struct i_start_mission : public cadmium::in_port<int> {};

		struct o_fcc_command_hover : public cadmium::out_port<message_fcc_command_t> {};
		struct o_hover_criteria_met : public cadmium::out_port<bool> {};
		struct o_request_aircraft_state : public cadmium::out_port<bool> {};
		struct o_update_gcs : public cadmium::out_port<message_update_gcs_t> {};
	};

	/**
	 * 	\anchor	Stabilize_input_ports
	 *	\par	Input Ports
	 * 	Definition of the input ports for the model.
	 * 	\param 	i_aircraft_state	Port for receiving the current state of the aircraft.
	 * 	\param	i_cancel_hover		Port for receiving signal indicating that the current attempt to hover should be aborted.
	 * 	\param	i_stabilize			Port for receiving hover criteria to attempt to hover at.
	 * 	\param 	i_start_mission 	Port for receiving signal indicating the mission has started.
	 */
	using input_ports = std::tuple<
		typename defs::i_aircraft_state,
		typename defs::i_cancel_hover,
		typename defs::i_stabilize,
		typename defs::i_start_mission
    >;

	/**
	 *	\anchor	Stabilize_output_ports
	 * 	\par 	Output Ports
	 * 	Definition of the output ports for the model.
	 * 	\param	o_fcc_command_hover			Port for sending hover commands to the FCC.
	 * 	\param	o_hover_criteria_met		Port for sending notification that the helicopter is now hovering at the specified hover criteria.
	 * 	\param	o_request_aircraft_state	Port for requesting the current aircraft state.
	 * 	\param	o_update_gcs 				Port for sending updates to the GCS.
	 */
	using output_ports = std::tuple<
		typename defs::o_fcc_command_hover,
		typename defs::o_hover_criteria_met,
		typename defs::o_request_aircraft_state,
		typename defs::o_update_gcs
	>;

	/**
	 *	\struct	state_type
	 * 	\brief 	Definition of the states of the atomic model.
	 * 	\param 	current_state 			Current state of atomic model.
	 *	\param	in_tolerance			Boolean for whether the helicopter is currently within the specified tolerance of the hover criteria.
	 *	\param	time_tolerance_met		Boolean for whether the time tolerance of the hover criteria has been met.
	 *	\param	stabilization_time_prev	Remaining time left of the time tolerance before the hover criteria is considered met.
	 */
	struct state_type {
		States current_state;
		bool in_tolerance;
		bool time_tolerance_met;
		TIME stabilization_time_prev;
        #ifdef DEBUG_MODELS
        std::string failures;
        #endif
	} state;

	/**
	 * \brief 	Default constructor for the model.
	 */
	Stabilize() {
		state.current_state = States::IDLE;
		state.in_tolerance = false;
		state.time_tolerance_met = false;
		polling_rate = TIME("00:00:00:100");
		state.stabilization_time_prev = TIME("00:00:00:000");
		hover_criteria = message_hover_criteria_t();
		aircraft_state = message_aircraft_state_t();
	}

	/**
	 * \brief 	Constructor for the model with parameter for how fast the aircraft state should be polled when checking the hover criteria.
	 * \param	polling_rate	TIME rate at which the aircraft state should be polled.
	 */
	explicit Stabilize(TIME polling_rate) {
		state.current_state = States::IDLE;
		state.in_tolerance = false;
		state.time_tolerance_met = false;
		this->polling_rate = polling_rate;
		state.stabilization_time_prev = TIME("00:00:00:000");
		hover_criteria = message_hover_criteria_t();
		aircraft_state = message_aircraft_state_t();
	}

	/**
	 * \brief 	Constructor for the model with initial state parameter
	 * 			for debugging or partial execution startup.
	 * \param	initial_state	States initial state of the model.
	 */
	explicit Stabilize(States initial_state) {
		state.current_state = initial_state;
		state.in_tolerance = false;
		state.time_tolerance_met = false;
		polling_rate = TIME("00:00:00:100");
		state.stabilization_time_prev = TIME("00:00:00:000");
		hover_criteria = message_hover_criteria_t();
		aircraft_state = message_aircraft_state_t();
	}

	/**
	 * \brief 	Constructor for the model with initial state parameter
	 * 			for debugging or partial execution startup.
	 * \param	polling_rate	TIME rate at which the aircraft state should be polled.
	 * \param	initial_state	States initial state of the model.
	 */
	Stabilize(TIME polling_rate, States initial_state) {
		state.current_state = initial_state;
		state.in_tolerance = false;
		state.time_tolerance_met = false;
		this->polling_rate = polling_rate;
		state.stabilization_time_prev = TIME("00:00:00:000");
		hover_criteria = message_hover_criteria_t();
		aircraft_state = message_aircraft_state_t();
	}

	/// Internal transitions of the model
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

	/// External transitions of the model
	void external_transition(TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		bool received_cancel_hover = !cadmium::get_messages<typename defs::i_cancel_hover>(mbs).empty();
		bool received_start_mission = !cadmium::get_messages<typename defs::i_start_mission>(mbs).empty();
		if (received_cancel_hover || received_start_mission) {
			reset_state();
            state.current_state = States::WAIT_STABILIZE;
			return;
		}

		switch (state.current_state) {
			case States::WAIT_STABILIZE: {
				bool received_stabilize = !cadmium::get_messages<typename defs::i_stabilize>(mbs).empty();
				if (received_stabilize) {
					// Get the most recent hover criteria input (found at the back of the vector of inputs)
					hover_criteria = cadmium::get_messages<typename defs::i_stabilize>(mbs).back();
					state.stabilization_time_prev = seconds_to_time<TIME>(hover_criteria.timeTol);
					state.current_state = States::REQUEST_AIRCRAFT_STATE;
				}
				break;
			}
			case States::GET_AIRCRAFT_STATE: {
				bool received_aircraft_state = !cadmium::get_messages<typename defs::i_aircraft_state>(mbs).empty();
				if (received_aircraft_state) {
					aircraft_state = cadmium::get_messages<typename defs::i_aircraft_state>(mbs)[0];
					state.current_state = States::INIT_HOVER;
				}
				break;
			}
			case States::CHECK_STATE: {
				bool received_aircraft_state = !cadmium::get_messages<typename defs::i_aircraft_state>(mbs).empty();
				if (received_aircraft_state) {
					aircraft_state = cadmium::get_messages<typename defs::i_aircraft_state>(mbs)[0];
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
			}
			default:
				break;
		}
	}

	/// Function used to decide precedence between internal and external transitions when both are scheduled simultaneously.
	void confluence_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		bool received_cancel_hover = !cadmium::get_messages<typename defs::i_cancel_hover>(mbs).empty();

		if (received_cancel_hover) {
			external_transition(TIME(), std::move(mbs));
		} else {
			internal_transition();
		}
	}

	/// Function for generating output from the model before internal transitions.
	[[nodiscard]] typename cadmium::make_message_bags<output_ports>::type output() const {
		typename cadmium::make_message_bags<output_ports>::type bags;

		switch (state.current_state) {
			case States::REQUEST_AIRCRAFT_STATE:
				cadmium::get_messages<typename defs::o_request_aircraft_state>(bags).emplace_back(true);
				break;
			case States::INIT_HOVER: {
				message_fcc_command_t mfc = message_fcc_command_t();
				mfc.reposition(
						aircraft_state.gps_time,
						hover_criteria.desiredLat * (1E7),
						hover_criteria.desiredLon * (1E7),
						hover_criteria.desiredAltMSL * FT_TO_METERS
				);
				cadmium::get_messages<typename defs::o_fcc_command_hover>(bags).push_back(mfc);
				break;
			}
			case States::STABILIZING:
				if (state.time_tolerance_met && state.in_tolerance) {
					cadmium::get_messages<typename defs::o_hover_criteria_met>(bags).emplace_back(true);
					cadmium::get_messages<typename defs::o_update_gcs>(bags).emplace_back(
							"Came to hover!",
							Mav_Severities_E::MAV_SEVERITY_INFO
					);
				} else {
					cadmium::get_messages<typename defs::o_request_aircraft_state>(bags).emplace_back(true);
				}
				break;
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
			case States::WAIT_STABILIZE:
			case States::GET_AIRCRAFT_STATE:
			case States::CHECK_STATE:
				next_internal = std::numeric_limits<TIME>::infinity();
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

	/**
	 *  \brief 		Operator for defining how the model state will be represented as a string.
	 * 	\warning 	Prepended "State: " is required for log parsing, do not remove.
	 */
	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Stabilize<TIME>::state_type& i) {
        #ifdef DEBUG_MODELS
        os << (string("State: ") + enumToString(i.current_state) + i.failures + "-") << i.stabilization_time_prev << string("\n");
        #else
        os << (std::string("State: ") + enumToString(i.current_state) + std::string("\n"));
        #endif
		return os;
	}

private:
	/// Variable for storing the hover criteria that the helicopter will hover at.
	message_hover_criteria_t hover_criteria;
	/// Variable for storing aircraft state when checking if the tolerance of the hover criteria has been met.
	message_aircraft_state_t aircraft_state;
	/// Variable for storing the rate at which the aircraft state should be polled.
	TIME polling_rate;

    /// Function for resetting the state of the model.
	void reset_state() {
		state.stabilization_time_prev = TIME("00:00:000");
		state.in_tolerance = false;
		state.time_tolerance_met = false;
	}

	/// @brief 	Function calculate_hover_criteria_met is used to check if a given aircraft state is within the current hover criteria.
	/// @param 	i_state message_aircraft_state_t current state of the aircraft.
	/// @return	true if the helicopter is within the criteria, false otherwise.
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
};

#endif // STABILIZE_HPP
