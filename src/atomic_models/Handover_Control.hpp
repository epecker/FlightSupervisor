/**
 * 	\file		Handover_Control.hpp
 *	\brief		Definition of the Handover Control atomic model.
 *	\details	This header file defines the Handover Control atomic model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor when 
				control of the aircraft should be handed over to the pilot.
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
*/
template<typename TIME> 
class Handover_Control {
public:
	/**
	 *	\enum	States
	 * 	\brief	Declaration of the states of the atomic model.
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
	 * \struct	defs
	 * \brief 	Declaration of the ports for the model.
	 * \see		input_ports
	 * \see 	output_ports
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
	 *	\struct	input_ports
	 * 	\brief 	Defintion of the input ports for the model.
	 * 	\var 	i_hover_criteria_met 	[input] Port for receiving updates on whether the previously commanded hover was achieved.
	 * 	\var 	i_pilot_handover 		[input] Port for receiving signal indicating control should be handed over to the pilot.
	 * 	\var 	i_pilot_takeover 		[input] Port for receiving signal indicating that the pilot has taken control from the supervisor.
	 * 	\var 	i_start_mission 		[input] Port for receiving signal indicating the mission has started.
	 */
	using input_ports = std::tuple<
		typename defs::i_hover_criteria_met,
		typename defs::i_pilot_handover,
		typename defs::i_pilot_takeover,
		typename defs::i_start_mission
	>;

	/**
	 *	\struct	output_ports
	 * 	\brief 	Defintion of the output ports for the model.
	 * 	\var	o_notify_pilot		[output] Port for notifying the pilot that they should take control of the aircraft.
	 * 	\var	o_control_yielded	[output] Port for sending an acknowledgement that the supervisor has relinquished control of the aircraft.
	 * 	\var	o_stabilize 		[output] Port for requesting the helicopter hover at a specific location.
	 */
	using output_ports = std::tuple<
		typename defs::o_notify_pilot,
		typename defs::o_control_yielded,
		typename defs::o_stabilize
	>;

	/**
	 *	\struct	state_type
	 * 	\brief 	Defintion of the states of the atomic model.
	 * 	\var 	current_state 	Current state of atomic model.
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

        bool received_pilot_handover;
		bool received_hover_crit_met;
		switch (state.current_state) {
			case States::WAIT_PILOT_HANDOVER:
				received_pilot_handover = !cadmium::get_messages<typename defs::i_pilot_handover>(mbs).empty();
				if (received_pilot_handover) {
					// Set the hover location to the newest input (found at the back of the vector of inputs)
					hover_location = cadmium::get_messages<typename defs::i_pilot_handover>(mbs).back();
					state.current_state = States::HOVER;
				}
				break;
			case States::STABILIZING:
				received_hover_crit_met = !cadmium::get_messages<typename defs::i_hover_criteria_met>(mbs).empty();
                if (received_hover_crit_met) {
					state.current_state = States::NOTIFY_PILOT;
				}
				break;
			case States::WAIT_FOR_PILOT:
				received_pilot_takeover = !cadmium::get_messages<typename defs::i_pilot_takeover>(mbs).empty();
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

	/// Function for generating output from the model after internal transitions.
	[[nodiscard]] typename cadmium::make_message_bags<output_ports>::type output() const {
		typename cadmium::make_message_bags<output_ports>::type bags;
		std::vector<bool> bag_port_out;
		std::vector<message_hover_criteria_t> bag_port_hover_out;
		message_hover_criteria_t hover_criteria;

		switch (state.current_state) {
			case States::HOVER:
				hover_criteria = message_hover_criteria_t(
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
				bag_port_hover_out.push_back(hover_criteria);
				cadmium::get_messages<typename defs::o_stabilize>(bags) = bag_port_hover_out;
				break;
			case States::NOTIFY_PILOT:
				bag_port_out.push_back(true);
				cadmium::get_messages<typename defs::o_notify_pilot>(bags) = bag_port_out;
				break;
			case States::YIELD_CONTROL:
				bag_port_out.push_back(true);
				cadmium::get_messages<typename defs::o_control_yielded>(bags) = bag_port_out;
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
	/// Varible for storing the location at which the helicopter should hover at.
	message_landing_point_t hover_location;

};

#endif // HANDOVER_CTRL_HPP
