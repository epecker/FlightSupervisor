/**
 *	\brief		An atomic model for caching inputs for later access.
 *	\details	This header file defines an input atomic model for use in the
                Cadmium DEVS simulation software.  
 *	\author		James Horner
 */

#ifndef CACHE_INPUT_HPP
#define CACHE_INPUT_HPP

// System libraries
#include <iostream>
#include <assert.h>
#include <mutex>
#include <string>
#include <chrono>

// RT-Cadmium
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <cadmium/modeling/dynamic_model.hpp>

#include "enum_string_conversion.hpp"
#include "Constants.hpp"

using namespace cadmium;
using namespace std;

// Input and output port definitions
template<typename MSG> struct Cache_Input_defs{
    struct i_new_input   	: public in_port<MSG> { };
    struct i_get_input   	: public in_port<bool> { };
    struct o_cached_input	: public out_port<MSG> { };
};

// Atomic model
template<typename MSG, typename TIME>
class Cache_Input {

// Private members.
private:
    
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(SEND)
	);

    // Default constructor
    Cache_Input() {
        state.current_state = States::IDLE;
		state.cached_input = MSG();
    }

    // Constructor with polling rate parameter
    Cache_Input(MSG initial_cached_input) {
        state.current_state = States::IDLE;
		state.cached_input = initial_cached_input;
    }

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
    struct state_type{
        States current_state;
		MSG cached_input;
    };
    state_type state;

	// Create a tuple of input ports (required for the simulator)
    using input_ports=std::tuple<
		typename Cache_Input_defs<MSG>::i_new_input,
		typename Cache_Input_defs<MSG>::i_get_input
	>;
 
    // Create a tuple of output ports (required for the simulator)
    using output_ports=std::tuple<
		typename Cache_Input_defs<MSG>::o_cached_input
	>;

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
    void internal_transition() {
        if (state.current_state == States::SEND) {
            state.current_state = States::IDLE;
        }
    }

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
    void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
        bool new_input = get_messages<typename Cache_Input_defs<MSG>::i_new_input>(mbs).size() >= 1;
        bool get_input = get_messages<typename Cache_Input_defs<MSG>::i_get_input>(mbs).size() >= 1;
        if (state.current_state == States::IDLE) {
            if (new_input){
                state.current_state = States::IDLE;
                state.cached_input = get_messages<typename Cache_Input_defs<MSG>::i_new_input>(mbs)[0];
            }
			if (get_input) {
				state.current_state = States::SEND;
			}
        }
    }

	// Confluence transition
	// Used to call set call precedence
    void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
        internal_transition();
        external_transition(TIME(), std::move(mbs));
    }

    // Output function
    typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		vector<MSG> bag_port_message;

        switch(state.current_state) {
            case States::SEND:
				bag_port_message.push_back(state.cached_input);
				get_messages<typename Cache_Input_defs<MSG>::o_cached_input>(bags) = bag_port_message;
		        break;
            default:
                break;
        }
        return bags;
    }

	// Time advance
	// Used to set the internal time of the current state
    TIME time_advance() const {
        switch (state.current_state) {
            case States::IDLE:
                return std::numeric_limits<TIME>::infinity();
            default:
                return TIME(TA_ZERO);
        }
    }

    friend std::ostringstream& operator<<(std::ostringstream& os, const typename Cache_Input<MSG, TIME>::state_type& i) {
        os << "State: " << enumToString(i.current_state) + string("\n");
        return os;
    }
};

#endif /* CACHE_INPUT_HPP */
