/**
 *	\brief		An atomic model for SEND output from the simulation using shared memory.
 *	\details	This header file defines a shared memory output atomic model for use
 				in the RT-Cadmium DEVS simulation software.
 *	\author		James Horner
 */

#ifndef SHARED_MEMORY_OUTPUT_HPP
#define SHARED_MEMORY_OUTPUT_HPP

// System libraries
#include <iostream>
#include <assert.h>
#include <thread>
#include <mutex>
#include <string>
#include <chrono>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

// RT-Cadmium
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <cadmium/modeling/dynamic_model.hpp>

// Message Structures

// Includes
#include "enum_string_conversion.hpp"
#include "Constants.hpp"

using namespace cadmium;
using namespace std;
using namespace boost::interprocess;

// Input and output port definitions
template<typename MSG> struct Shared_Memory_Output_defs{
    struct i_message   : public in_port<MSG> { };
};

// Atomic model
template<typename MSG, typename TIME>
class Shared_Memory_Output {

	// Private members.
private:
	MSG message_to_send;
    MSG *message_structure;
    
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(SEND)
	);

    // Default constructor
    Shared_Memory_Output() {
        state.current_state = States::IDLE;

		//Open and map the shared memory segment.
		shared_memory_object shdmem = {open_or_create, string(DEFAULT_SHARED_MEMORY_NAME).c_str(), read_write, sizeof(MSG)};
		shdmem.truncate(sizeof(MSG));
		mapped_region region{shdmem, read_write};
		message_structure = static_cast<MSG*>(region.get_address());
    }

    // Constructor with name parameter
    Shared_Memory_Output(string name) {
        state.current_state = States::IDLE;

		//Open and map the shared memory segment.
		shared_memory_object shdmem = {open_or_create, name.c_str(), read_write, sizeof(MSG)};
		shdmem.truncate(sizeof(MSG));
		mapped_region region{shdmem, read_write};
		message_structure = static_cast<MSG*>(region.get_address());
    }

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
    struct state_type{
        States current_state;
    };
    state_type state;

	// Create a tuple of input ports (required for the simulator)
    using input_ports=std::tuple<typename Shared_Memory_Output_defs<MSG>::i_message>;
 
    // Create a tuple of output ports (required for the simulator)
    using output_ports=std::tuple<>;

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
        bool have_message = get_messages<typename Shared_Memory_Output_defs<MSG>::i_message>(mbs).size() >= 1;
        if (state.current_state == States::IDLE) {
            if (have_message) {
                state.current_state = States::SEND;
                message_to_send = get_messages<typename Shared_Memory_Output_defs<MSG>::i_message>(mbs)[0];
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
        if (state.current_state == States::SEND) {
			*message_structure = message_to_send;
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
                return TIME("00:00:00:000");
        }
    }

    friend std::ostringstream& operator<<(std::ostringstream& os, const typename Shared_Memory_Output<MSG, TIME>::state_type& i) {
        os << "State: " << enumToString(i.current_state);
        return os;
    }
};

#endif /* SHARED_MEMORY_OUTPUT_HPP */
