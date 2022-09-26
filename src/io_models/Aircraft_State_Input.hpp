/**
 * 	\file		Aircraft_State_Input.hpp
 *	\brief		Definition of the Aircraft State Input atomic model.
 *	\details	This header file defines the Aircraft State Input atomic model for use in the Cadmium DEVS
				simulation software. The model connects to shared memory and outputs the aircraft state.
 *	\image		html io_models/aircraft_state_input.png
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifdef RT_LINUX
#ifndef AIRCRAFT_STATE_INPUT_HPP
#define AIRCRAFT_STATE_INPUT_HPP

// Message structures
#include "../message_structures/message_aircraft_state_t.hpp"

// Utility functions
#include "../enum_string_conversion.hpp"

// Cadmium Simulator Headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

// Shared Memory Model
#include <sharedmemorymodel/SharedMemoryModel.h>

// System libraries
#include <cassert>
#include <string>

/**
 *	\class		Aircraft_State_Input
 *	\brief		Definition of the Aircraft State Input atomic model.
 *	\details	This class defines the Aircraft State Input atomic model for use in the Cadmium DEVS
				simulation software. The model connects to shared memory and outputs the aircraft state.
 *	\image		html io_models/aircraft_state_input.png
 */
template<typename TIME>
class Aircraft_State_Input {
public:
	/**
	 *	\par	States
	 * 	Declaration of the states of the atomic model.
	 */
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(SEND)
	);

	/**
	 *	\brief	For definition of the input and output ports see:
	*	\ref 	Aircraft_State_Input_input_ports "Input Ports" and
	*	\ref 	Aircraft_State_Input_output_ports "Output Ports"
	* 	\note 	All input and output ports must be listed in this struct.
	*/
	struct defs {
		struct o_message : public cadmium::out_port<message_aircraft_state_t> { };
		struct i_request : public cadmium::in_port<bool> { };
	};

	/**
	 * 	\anchor	Aircraft_State_Input_input_ports
	 *	\par	Input Ports
	 * 	Definition of the input ports for the model.
	 * 	\param 	i_request	Port for receiving a request to get an aircraft state.
	 */
	using input_ports = std::tuple<typename defs::i_request>;

	/**
	 *	\anchor	Aircraft_State_Input_output_ports
	 * 	\par 	Output Ports
	 * 	Definition of the output ports for the model.
	 * 	\param	o_message	Port for sending aircraft state messages.
	 */
	using output_ports = std::tuple<typename defs::o_message>;

	/**
	 *	\anchor	Aircraft_State_Input_state_type
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
	Aircraft_State_Input() {
		//Initialise the current state
		state.current_state = States::IDLE;

		model = SharedMemoryModel();
		model.connectSharedMem();
		if (!model.isConnected()) {
			throw(std::runtime_error("Could not connect to shared memory."));
		}
	}

	/**
	 * \brief 	Destructor for disconnecting from shared memory.
	 */
	~Aircraft_State_Input() {
		model.disconnectSharedMem();
	}

	/// Internal transitions of the model
	void internal_transition() {
		switch (state.current_state)
		{
			case States::SEND:
				state.current_state = States::IDLE;
				break;
			default:
				break;
		}
	}

	/// External transitions of the model
	void external_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		bool received_request = !cadmium::get_messages<typename defs::i_request>(mbs).empty();
		if (received_request) {
			state.current_state = States::SEND;
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

		if (state.current_state == States::SEND) {
			cadmium::get_messages<typename defs::o_message>(bags).emplace_back(
					model.sharedMemoryStruct->hg1700.time,
					model.sharedMemoryStruct->hg1700.lat,
					model.sharedMemoryStruct->hg1700.lng,
					model.sharedMemoryStruct->hg1700.mixedhgt,
					model.sharedMemoryStruct->hg1700.alt,
					model.sharedMemoryStruct->hg1700.hdg,
					sqrt(pow(model.sharedMemoryStruct->hg1700.ve, 2) + pow(model.sharedMemoryStruct->hg1700.vn, 2))
			);
		}
		return bags;
	}

	/// Function to declare the time advance value for each state of the model.
	TIME time_advance() const {
		switch (state.current_state) {
			case States::IDLE:
				return std::numeric_limits<TIME>::infinity();
			case States::SEND:
				return TIME(TA_ZERO);
			default:
				assert(false && "Unhandled state time advance.");
		}
	}

	/**
	 *  \brief 		Operator for defining how the model state will be represented as a string.
	 * 	\warning 	Prepended "State: " is required for log parsing, do not remove.
	 */
	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Aircraft_State_Input<TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state);
		return os;
	}

private:
	// Variable used for shared memory management and access
	SharedMemoryModel model;
};

#endif // AIRCRAFT_STATE_INPUT_HPP
#endif // RT_LINUX
