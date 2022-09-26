/**
 * 	\file		Polling_Condition_Input.hpp
 *	\brief		Definition of the Polling Condition Input atomic model.
 *	\details	This header file defines the Polling Condition Input atomic model for use in the Cadmium DEVS
				simulation software. The model polls shared memory when requested.
 *	\image		html io_models/polling_condition_input.png
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef POLLING_CONDITION_INPUT_HPP
#define POLLING_CONDITION_INPUT_HPP

// Utility functions
#include "../enum_string_conversion.hpp"

// Constants
#include "../Constants.hpp"

// RT-Cadmium
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

// Shared Memory Model
#include <sharedmemorymodel/SharedMemoryModel.h>

// System libraries
#include <iostream>
#include <string>
#include <vector>

/**
 *	\class		Polling_Condition_Input
 *	\brief		Definition of the Polling Condition Input atomic model.
 *	\details	This class defines the Polling Condition Input atomic model for use in the Cadmium DEVS
				simulation software. The model polls shared memory when requested.
 *	\image		html io_models/polling_condition_input.png
 */
template<typename START_TYPE, typename QUIT_TYPE, typename TIME>
class Polling_Condition_Input {
public:
	/**
	 *	\par	States
	 * 	Declaration of the states of the atomic model.
	 */
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(POLL)
	);

	/**
	 *	\brief	For definition of the input and output ports see:
	 *	\ref 	Polling_Condition_Input_input_ports "Input Ports" and
	 *	\ref 	Polling_Condition_Input_output_ports "Output Ports"
	 * 	\note 	All input and output ports must be listed in this struct.
	 */
	struct defs {
		struct i_quit : public cadmium::in_port<QUIT_TYPE> { };
		struct i_start : public cadmium::in_port<START_TYPE> { };
		struct o_message : public cadmium::out_port<bool> { };
	};

	/**
	 * 	\anchor	Polling_Condition_Input_input_ports
	 *	\par	Input Ports
	 * 	Definition of the input ports for the model.
	 * 	\param 	i_quit	Port for to stop polling the memory location.
	 * 	\param 	i_start	Port for to start polling the memory location.
	 */
	using input_ports = std::tuple<
			typename defs::i_quit,
			typename defs::i_start
	>;

	/**
	 *	\anchor	Polling_Condition_Input_output_ports
	 * 	\par 	Output Ports
	 * 	Definition of the output ports for the model.
	 * 	\param	o_message	Port for outputting messages.
	 */
	using output_ports = std::tuple<typename defs::o_message>;

	/**
	 *	\anchor	Polling_Condition_Input_state_type
	 *	\par	State
	 * 	Definition of the states of the atomic model.
	 * 	\param 	current_state 	Current state of atomic model.
	 * 	\param 	condition_met 	Flag to notify that it is time to poll.
	 */
	struct state_type {
		States current_state;
		bool condition_met;
	} state;

	/**
	 * \brief 	Default constructor for the model.
	 */
	Polling_Condition_Input() {
		state.current_state = States::IDLE;
		state.condition_met = false;

		//Set the rate at which the shared memory segment will be polled.
		polling_rate = TIME("00:00:00:100");
	}

	/**
	 * \brief 	Constructor for the model with initial state parameter
	 * 			for debugging or partial execution startup.
	 * \param	rate	Polling frequency.
	 */
	explicit Polling_Condition_Input(TIME rate) : polling_rate(rate) {
		state.current_state = States::IDLE;
		state.condition_met = false;
	}

	/// Internal transitions of the model
	void internal_transition() {
		if (state.condition_met) {
			state.current_state = States::IDLE;
			state.condition_met = false;
		}
		else if (state.current_state == States::POLL && check_condition()) {
			state.condition_met = true;
		}
	}

	/// External transitions of the model
	void external_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		if (cadmium::get_messages<typename defs::i_quit>(mbs).size() >= 1) {
			state.current_state = States::IDLE;
		}
		else if (cadmium::get_messages<typename defs::i_start>(mbs).size() >= 1) {
			state.current_state = States::POLL;
		}
	}

	/// Function used to decide precedence between internal and external transitions when both are scheduled simultaneously.
	void confluence_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		external_transition(TIME(), std::move(mbs));
		internal_transition();
	}

	/// Function for generating output from the model before internal transitions.
	typename cadmium::make_message_bags<output_ports>::type output() const {
		typename cadmium::make_message_bags<output_ports>::type bags;
		if (state.current_state == States::POLL && state.condition_met) {
			cadmium::get_messages<typename defs::o_message>(bags).emplace_back(true);
		}
		return bags;
	}

	/// Function to declare the time advance value for each state of the model.
	TIME time_advance() const {
		switch (state.current_state) {
			case States::IDLE:
				return std::numeric_limits<TIME>::infinity();
			case States::POLL:
				if (state.condition_met) {
					return TIME(TA_ZERO);
				}
				else {
					return polling_rate;
				}
			default:
				return TIME(TA_ZERO);
		}
	}

	/**
	 *  \brief 		Operator for defining how the model state will be represented as a string.
	 * 	\warning 	Prepended "State: " is required for log parsing, do not remove.
	 */
	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Polling_Condition_Input<START_TYPE, QUIT_TYPE, TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state) << "-" << (i.condition_met ? "MET" : "NOT_MET");
		return os;
	}

protected:
	/// Used to setup the shared memory.
	virtual bool setup() {return false;};
	/// Used to verify the shared memory is accessible.
	virtual bool check_condition() {return false;};
	TIME polling_rate;
};

/**
 *	\class		Polling_Condition_Input_Test
 *	\brief		Definition of the Polling Condition Input atomic model.
 *	\details	This class defines the Polling Condition Input atomic model for use in the Cadmium DEVS
				simulation software. The model is only used for testing.
 */
template<typename TIME>
class Polling_Condition_Input_Test : public Polling_Condition_Input<bool, bool, TIME> {
public:
    Polling_Condition_Input_Test() = default;
    explicit Polling_Condition_Input_Test(TIME rate) : Polling_Condition_Input<bool, bool, TIME>(rate) {
		this->polling_rate = rate;
		number_polls = 0;

		if (!setup()) {
			throw(std::runtime_error("Could not set up polling condition input test."));
		}
	}

	bool setup() {
		number_polls = 0;
		return true;
	}

	bool check_condition() {
		number_polls++;
		return (number_polls == 10);
	}

private:
	int number_polls{};
};

/**
 *	\class		Polling_Condition_Input
 *	\brief		Definition of the Polling Condition Input atomic model.
 *	\details	This class defines the Polling Condition Input atomic model for use in the Cadmium DEVS
				simulation software. The model polls shared memory for aircraft height when requested.
 */
template<typename TIME>
class Polling_Condition_Input_Landing_Achieved : public Polling_Condition_Input<message_fcc_command_t, bool, TIME> {
public:
    Polling_Condition_Input_Landing_Achieved() = default;
    Polling_Condition_Input_Landing_Achieved(TIME rate, float landing_height_ft) :
		Polling_Condition_Input<message_fcc_command_t, bool, TIME>(rate),
		landing_height_ft(landing_height_ft) {
		if (!setup()) {
			throw(std::runtime_error("Could not set up polling condition input for landing achieved."));
		}
	}

	~Polling_Condition_Input_Landing_Achieved() {
		model.disconnectSharedMem();
	}

	bool setup() {
		model = SharedMemoryModel();
		model.connectSharedMem();
		return model.isConnected();
	}

	bool check_condition() {
		return (model.sharedMemoryStruct->hg1700.mixedhgt < landing_height_ft);
	}

private:
	SharedMemoryModel model;
	float landing_height_ft{};
};

/**
 *	\class		Polling_Condition_Input
 *	\brief		Definition of the Polling Condition Input atomic model.
 *	\details	This class defines the Polling Condition Input atomic model for use in the Cadmium DEVS
				simulation software. The model polls shared memory for safety status when requested.
 */
template<typename TIME>
class Polling_Condition_Input_Pilot_Takeover : public Polling_Condition_Input<message_start_supervisor_t, bool, TIME> {
public:
    Polling_Condition_Input_Pilot_Takeover() = default;
    explicit Polling_Condition_Input_Pilot_Takeover(TIME rate) :
		Polling_Condition_Input<message_start_supervisor_t, bool, TIME>(rate) {
		if (!setup()) {
			throw(std::runtime_error("Could not set up polling condition input for pilot takeover."));
		}
	}

	~Polling_Condition_Input_Pilot_Takeover() {
		model.disconnectSharedMem();
	}

	bool setup() {
		engaged = ((1 << 0) | (1 << 1));
		model = SharedMemoryModel();
		model.connectSharedMem();
		return model.isConnected();
	}

	bool check_condition() {
		// Store the status bits
		uint32_t status = model.sharedMemoryStruct->hmu_safety.safety_status;
		// Clear all but the FCC engaged bits
		status &= engaged;
		// If both FCC engaged bits are set, return true
		return (status != engaged);
	}

private:
	SharedMemoryModel model;
	uint32_t engaged{};
};

#endif /* POLLING_CONDITION_INPUT_HPP */
