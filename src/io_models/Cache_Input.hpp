/**
 * 	\file		Cache_Input.hpp
 *	\brief		Definition of the Cache Input atomic model.
 *	\details	This header file defines the Cache Input atomic model for use in the Cadmium DEVS
				simulation software. The model receives inputs and caches them for later access.
 *	\image		html io_models/cache_input.png
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef CACHE_INPUT_HPP
#define CACHE_INPUT_HPP

// Utility functions
#include "../enum_string_conversion.hpp"

// Constants
#include "../Constants.hpp"

// Cadmium Simulator Headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

// System libraries
#include <iostream>
#include <string>

/**
 *	\class		Cache_Input
 *	\brief		Definition of the Cache Input atomic model.
 *	\details	This class defines the Cache Input atomic model for use in the Cadmium DEVS
				simulation software. The model receives inputs and caches them for later access.
 *	\image		html io_models/cache_input.png
 */
template<typename MSG, typename TIME>
class Cache_Input {
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
	*	\ref 	Cache_Input_input_ports "Input Ports" and
	*	\ref 	Cache_Input_output_ports "Output Ports"
	* 	\note 	All input and output ports must be listed in this struct.
	*/
	struct defs {
		struct i_new_input   	: public cadmium::in_port<MSG> { };
		struct i_get_input   	: public cadmium::in_port<bool> { };
		struct o_cached_input	: public cadmium::out_port<MSG> { };
	};

	/**
	 * 	\anchor	Cache_Input_input_ports
	 *	\par	Input Ports
	 * 	Definition of the input ports for the model.
	 * 	\param 	i_new_input	Port for receiving a new message to cache.
	 * 	\param 	i_get_input	Port for requesting a cached message be sent.
	 */
	using input_ports=std::tuple<
			typename defs::i_new_input,
			typename defs::i_get_input
	>;

	/**
	 *	\anchor	Cache_Input_output_ports
	 * 	\par 	Output Ports
	 * 	Definition of the output ports for the model.
	 * 	\param	o_cached_input	Port for outputting cached messages.
	 */
	using output_ports=std::tuple<
			typename defs::o_cached_input
	>;

	/**
	 *	\anchor	Cache_Input_state_type
	 *	\par	State
	 * 	Definition of the states of the atomic model.
	 * 	\param 	current_state 	Current state of atomic model.
	 * 	\param 	cached_input 	Message to store for future retrieval.
	 */
	struct state_type{
		States current_state;
		MSG cached_input;
	} state;

	/**
	 * \brief 	Default constructor for the model.
	 */
    Cache_Input() {
        state.current_state = States::IDLE;
		state.cached_input = MSG();
    }

	/**
	 * \brief 	Constructor for the model with initial state parameter
	 * 			for debugging or partial execution startup.
	 * \param	initial_state	States initial state of the model.
	 */
    explicit Cache_Input(MSG initial_cached_input) {
        state.current_state = States::IDLE;
		state.cached_input = initial_cached_input;
    }

	/// Internal transitions of the model
    void internal_transition() {
        if (state.current_state == States::SEND) {
            state.current_state = States::IDLE;
        }
    }

	/// External transitions of the model
    void external_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
        bool new_input = !cadmium::get_messages<typename defs::i_new_input>(mbs).empty();
        bool get_input = !cadmium::get_messages<typename defs::i_get_input>(mbs).empty();

		switch (state.current_state) {
			case States::IDLE:
				if (new_input){
					state.current_state = States::IDLE;
					// Cache the most recent input (found at the back of the vector of inputs)
					state.cached_input = cadmium::get_messages<typename defs::i_new_input>(mbs).back();
				}
				if (get_input) state.current_state = States::SEND;
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

        switch(state.current_state) {
            case States::SEND:
				cadmium::get_messages<typename defs::o_cached_input>(bags).push_back(state.cached_input);
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
                return std::numeric_limits<TIME>::infinity();
            default:
                return TIME(TA_ZERO);
        }
    }

	/**
	 *  \brief 		Operator for defining how the model state will be represented as a string.
	 * 	\warning 	Prepended "State: " is required for log parsing, do not remove.
	 */
    friend std::ostringstream& operator<<(std::ostringstream& os, const typename Cache_Input<MSG, TIME>::state_type& i) {
        os << "State: " << enumToString(i.current_state) + std::string("\n");
        return os;
    }
};

/**
 *	\class		Cache_Input_Boolean
 *	\brief		Definition of the Cache Input atomic model.
 *	\details	This class defines the Cache Input atomic model for use in the Cadmium DEVS
				simulation software. The model receives boolean inputs and caches them for later access.
 */
template<typename T>
class Cache_Input_Boolean : public Cache_Input<bool, T> {
public:
	Cache_Input_Boolean() = default;
	explicit Cache_Input_Boolean(bool initial_cached_input) : Cache_Input<bool, T>(initial_cached_input) {};
};

#endif /* CACHE_INPUT_HPP */
