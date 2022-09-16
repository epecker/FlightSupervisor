/**
 * 	\file 		Handle_Waypoint.hpp
 *	\brief		Definition of the Handle Waypoint atomic model.
 *	\details	This header file defines the Handle Waypoint atomic model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor when
				a waypoint is met while on-route.
 *	\image		html atomic_models/handle_waypoint.png
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef HANDLE_WAYPOINT_HPP
#define HANDLE_WAYPOINT_HPP

// Messages structures
#include "../message_structures/message_fcc_command_t.hpp"

// Utility functions
#include "../enum_string_conversion.hpp"
#include "../Constants.hpp"

// Cadmium Simulator Headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

// System Libraries
#include <limits>
#include <string>

/**
 * 	\class 		Handle_Waypoint
 *	\brief		Definition of the Handle Waypoint atomic model.
 *	\details	This class defines the Handle Waypoint atomic model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor when
				a waypoint is met while on-route.
 *	\image		html atomic_models/handle_waypoint.png
 */
template<typename TIME>
class Handle_Waypoint {
public:
	/**
	 *	\par		States
	 * 	Declaration of the states of the atomic model.
	 */
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(WAIT_FOR_WAYPOINT)
		(PILOT_TAKEOVER)
		(UPDATE_FCC)
	);

	/**
	 *	\brief	For definition of the input and output ports see:
	 *	\ref 	Handle_Waypoint_input_ports "Input Ports" and
	 *	\ref 	Handle_Waypoint_output_ports "Output Ports"
	 * 	\note 	All input and output ports must be listed in this struct.
	 */
	struct defs {
		struct i_pilot_takeover : public cadmium::out_port<bool> {};
		struct i_start_mission : public cadmium::out_port<int> {};
		struct i_waypoint : public cadmium::out_port<message_fcc_command_t> {};

		struct o_fcc_waypoint_update : public cadmium::out_port<message_fcc_command_t> {};
	};

	/**
	 * 	\anchor	Handle_Waypoint_input_ports
	 *	\par	Input Ports
	 * 	Definition of the input ports for the model.
	 * 	\param	i_pilot_takeover	Port for receiving signal indicating that the pilot has taken control from the supervisor.
	 * 	\param	i_start_mission		Port for receiving signal indicating the mission has started.
	 * 	\param	i_waypoint			Port for receiving new waypoints during the on-route phase.
	 */
	using input_ports = std::tuple<
		typename defs::i_pilot_takeover,
		typename defs::i_start_mission,
		typename defs::i_waypoint
	>;

	/**
	 *	\anchor	Handle_Waypoint_output_ports
	 * 	\par 	Output Ports
	 * 	Definition of the output ports for the model.
	 * 	\param	o_fcc_waypoint_update	Port for sending waypoint commands to the FCC.
	 */
	using output_ports = std::tuple<
		typename defs::o_fcc_waypoint_update
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
	Handle_Waypoint() {
		state.current_state = States::IDLE;
	}

	/**
	 * \brief 	Constructor for the model with initial state parameter
	 * 			for debugging or partial execution startup.
	 * \param	initial_state	States initial state of the model.
	 */
	explicit Handle_Waypoint(States initial_state) {
		state.current_state = initial_state;
	}

	/// Internal transitions of the model
	void internal_transition() {
		switch (state.current_state) {
			case States::UPDATE_FCC:
				state.current_state = States::WAIT_FOR_WAYPOINT;
				break;
			default:
				return;
		}
	}

	/// External transitions of the model
	void external_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		bool received_pilot_takeover;
		bool received_start_mission;
		bool received_waypoint;

		received_pilot_takeover = !cadmium::get_messages<typename defs::i_pilot_takeover>(mbs).empty();
		if (received_pilot_takeover) {
			state.current_state = States::PILOT_TAKEOVER;
			return;
		}

		switch (state.current_state) {
			case States::IDLE:
				received_start_mission = !cadmium::get_messages<typename defs::i_start_mission>(mbs).empty();
				if (received_start_mission) {
					state.current_state = States::WAIT_FOR_WAYPOINT;
				}
				break;
			case States::WAIT_FOR_WAYPOINT:
				received_waypoint = !cadmium::get_messages<typename defs::i_waypoint>(mbs).empty();
				if (received_waypoint) {
					next_waypoint = cadmium::get_messages<typename defs::i_waypoint>(mbs);
					state.current_state = States::UPDATE_FCC;
				}
				break;
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

		if (state.current_state == States::UPDATE_FCC) {
            for (message_fcc_command_t & waypoint : next_waypoint) {
                waypoint.set_supervisor_status(Control_Mode_E::MAV_COMMAND);
            }
			cadmium::get_messages<typename defs::o_fcc_waypoint_update>(bags) = next_waypoint;
		}

		return bags;
	}

	/// Function to declare the time advance value for each state of the model.
	TIME time_advance() const {
		TIME next_internal;
		switch (state.current_state) {
			case States::IDLE:
			case States::WAIT_FOR_WAYPOINT:
			case States::PILOT_TAKEOVER:
				next_internal = std::numeric_limits<TIME>::infinity();
				break;
			case States::UPDATE_FCC:
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
	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Handle_Waypoint<TIME>::state_type& i) {
		os << (std::string("State: ") + enumToString(i.current_state) + std::string("\n"));
		return os;
	}

private:
    /// Variable for storing the next waypoints for forwarding as FCC commands.
    mutable std::vector<message_fcc_command_t> next_waypoint;
};

#endif // HANDLE_WAYPOINT_HPP
