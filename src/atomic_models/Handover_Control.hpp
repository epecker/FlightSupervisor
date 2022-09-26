/**
 * 	\file		Handover_Control.hpp
 *	\brief		Definition of the Handover Control atomic model.
 *	\details	This header file defines the Handover Control atomic model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor when
				control of the aircraft should be handed over to the pilot.
 *	\image		html atomic_models/handover_control.png
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef HANDOVER_CTRL_HPP
#define HANDOVER_CTRL_HPP

// Messages structures
#include "../message_structures/message_landing_point_t.hpp"
#include "../message_structures/message_hover_criteria_t.hpp"

// Utility functions
#include "../enum_string_conversion.hpp"
#include "../Constants.hpp"

// Cadmium Simulator Headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

// System Libraries
#include <limits> // Used to set the time advance to infinity
#include <cassert>
#include <string>

/**
 * 	\file		Handover_Control.hpp
 *	\brief		Definition of the Handover Control atomic model.
 *	\details	This class defines the Handover Control atomic model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor when
				control of the aircraft should be handed over to the pilot.
 *	\image		html atomic_models/handover_control.png
*/
template<typename TIME>
class Handover_Control {
public:
	/**
	 *	\par	States
	 * 	Declaration of the states of the atomic model.
	 */
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(WAIT_PILOT_HANDOVER)
		(HOVER)
		(STABILIZING)
		(NOTIFY_PILOT)
		(WAIT_FOR_PILOT)
		(YIELD_CONTROL)
		(PILOT_CONTROL)
	);

	/**
	 *	\brief	For definition of the input and output ports see:
	 *	\ref 	Handover_Control_input_ports "Input Ports" and
	 *	\ref 	Handover_Control_output_ports "Output Ports"
	 * 	\note 	All input and output ports must be listed in this struct.
	 */
    struct defs {
        struct i_hover_criteria_met : public cadmium::in_port<bool> {};
        struct i_pilot_handover : public cadmium::in_port<message_landing_point_t> {};
        struct i_pilot_takeover : public cadmium::in_port<bool> {};
        struct i_start_mission : public cadmium::in_port<int> {};

        struct o_notify_pilot : public cadmium::out_port<bool> {};
        struct o_control_yielded : public cadmium::out_port<bool> {};
        struct o_stabilize : public cadmium::out_port<message_hover_criteria_t> {};
    };

	/**
	 * 	\anchor	Handover_Control_input_ports
	 *	\par	Input Ports
	 * 	Definition of the input ports for the model.
	 * 	\param 	i_hover_criteria_met 	Port for receiving updates on whether the previously commanded hover was achieved.
	 * 	\param 	i_pilot_handover 		Port for receiving signal indicating control should be handed over to the pilot.
	 * 	\param 	i_pilot_takeover 		Port for receiving signal indicating that the pilot has taken control from the supervisor.
	 * 	\param 	i_start_mission 		Port for receiving signal indicating the mission has started.
	 */
	using input_ports = std::tuple<
		typename defs::i_hover_criteria_met,
		typename defs::i_pilot_handover,
		typename defs::i_pilot_takeover,
		typename defs::i_start_mission
	>;

	/**
	 *	\anchor	Handover_Control_output_ports
	 * 	\par 	Output Ports
	 * 	Definition of the output ports for the model.
	 * 	\param	o_notify_pilot		Port for notifying the pilot that they should take control of the aircraft.
	 * 	\param	o_control_yielded	Port for sending an acknowledgement that the supervisor has relinquished control of the aircraft.
	 * 	\param	o_stabilize 		Port for requesting the helicopter hover at a specific location.
	 */
	using output_ports = std::tuple<
		typename defs::o_notify_pilot,
		typename defs::o_control_yielded,
		typename defs::o_stabilize
	>;

	/**
	 *	\anchor	Handover_Control_state_type
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
	Handover_Control() {
		state.current_state = States::IDLE;
		hover_location = message_landing_point_t();
	}

	/**
	 * \brief 	Constructor for the model with initial state parameter
	 * 			for debugging or partial execution startup.
	 * \param	initial_state	States initial state of the model.
	 */
	explicit Handover_Control(States initial_state) {
		state.current_state = initial_state;
		hover_location = message_landing_point_t();
	}

	/// Internal transitions of the model
	void internal_transition() {
		switch (state.current_state) {
			case States::HOVER:
				state.current_state = States::STABILIZING;
				break;
			case States::NOTIFY_PILOT:
				state.current_state = States::WAIT_FOR_PILOT;
				break;
			case States::YIELD_CONTROL:
				state.current_state = States::PILOT_CONTROL;
				break;
			default:
				break;
		}
	}

	/// External transitions of the model
	void external_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {

        bool received_pilot_takeover = !cadmium::get_messages<typename defs::i_pilot_takeover>(mbs).empty();
        if (received_pilot_takeover && state.current_state != States::WAIT_FOR_PILOT) {
            state.current_state = States::PILOT_CONTROL;
            return;
        }

        bool received_start_mission = !cadmium::get_messages<typename defs::i_start_mission>(mbs).empty();
        if (received_start_mission) {
            state.current_state = States::WAIT_PILOT_HANDOVER;
            return;
        }

		switch (state.current_state) {
			case States::WAIT_PILOT_HANDOVER: {
				bool received_pilot_handover = !cadmium::get_messages<typename defs::i_pilot_handover>(mbs).empty();
				if (received_pilot_handover) {
					// Set the hover location to the newest input (found at the back of the vector of inputs)
					hover_location = cadmium::get_messages<typename defs::i_pilot_handover>(mbs).back();
					state.current_state = States::HOVER;
				}
				break;
			}
			case States::STABILIZING: {
				bool received_hover_crit_met = !cadmium::get_messages<typename defs::i_hover_criteria_met>(mbs).empty();
				if (received_hover_crit_met) {
					state.current_state = States::NOTIFY_PILOT;
				}
				break;
			}
			case States::WAIT_FOR_PILOT:
				if (received_pilot_takeover) {
					state.current_state = States::YIELD_CONTROL;
				}
				break;
			default:
				break;
		}
	}

	/// Function used to decide precedence between internal and external transitions when both are scheduled simultaneously.
	void confluence_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		bool received_pilot_takeover = !cadmium::get_messages<typename defs::i_pilot_takeover>(mbs).empty();

		if (received_pilot_takeover) {
			external_transition(TIME(), std::move(mbs));
			internal_transition();
		} else {
			internal_transition();
			external_transition(TIME(), std::move(mbs));
		}
	}

	/// Function for generating output from the model before internal transitions.
	[[nodiscard]] typename cadmium::make_message_bags<output_ports>::type output() const {
		typename cadmium::make_message_bags<output_ports>::type bags;

		switch (state.current_state) {
			case States::HOVER:
				// Send a hover criteria message
				cadmium::get_messages<typename defs::o_stabilize>(bags).emplace_back(
						hover_location.lat,
						hover_location.lon,
						hover_location.alt,
						nanf(""),
						DEFAULT_LAND_CRITERIA_HOR_DIST,
						DEFAULT_LAND_CRITERIA_VERT_DIST,
						DEFAULT_LAND_CRITERIA_VEL,
						DEFAULT_LAND_CRITERIA_HDG,
						DEFAULT_LAND_CRITERIA_TIME,
						0,
						0,
						0
				);
				break;
			case States::NOTIFY_PILOT:
				cadmium::get_messages<typename defs::o_notify_pilot>(bags).emplace_back(true);
				break;
			case States::YIELD_CONTROL:
				cadmium::get_messages<typename defs::o_control_yielded>(bags).emplace_back(true);
				break;
			default:
				break;
		}

		return bags;
	}

	/// Function to declare the time advance value for each state of the model.
	TIME time_advance() const {
		switch (state.current_state) {
			case States::IDLE:
			case States::WAIT_PILOT_HANDOVER:
            case States::STABILIZING:
            case States::WAIT_FOR_PILOT:
            case States::PILOT_CONTROL:
				return std::numeric_limits<TIME>::infinity();
			case States::HOVER:
			case States::NOTIFY_PILOT:
			case States::YIELD_CONTROL:
				return TIME(TA_ZERO);
			default:
				assert(false && "Unhandled state time advance.");
		}
	}

	/**
	 *  \brief 		Operator for defining how the model state will be represented as a string.
	 * 	\warning 	Prepended "State: " is required for log parsing, do not remove.
	 */
	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Handover_Control<TIME>::state_type& i) {
		os << (std::string("State: ") + enumToString(i.current_state) + std::string("\n"));
		return os;
	}

private:
	/// Variable for storing the location at which the helicopter should hover at.
	message_landing_point_t hover_location;

};

#endif // HANDOVER_CTRL_HPP
