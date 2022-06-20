/**
 *	\brief		An atomic model for receiving input into the simulation from shared memory.
 *	\details	This header file defines a polling shared memory input atomic model for use
 				in the RT-Cadmium DEVS simulation software.
 *	\author		James Horner
 */

#ifndef SHARED_MEMORY_INPUT_HPP
#define SHARED_MEMORY_INPUT_HPP

 // System libraries
#include <iostream>
#include <assert.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#include <exception>

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

#ifdef RT_LINUX

using namespace cadmium;
using namespace std;
using namespace boost::interprocess;

// Input and output port definitions
template<typename MSG> struct Shared_Memory_Input_defs {
	struct o_message : public out_port<MSG> { };
	struct i_quit : public in_port<bool> { };
};

// Atomic model
template<typename MSG, typename TIME>
class Shared_Memory_Input {

	// Private members.
private:
	TIME polling_rate;
	MSG *message;
	
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(POLL)
		(SEND)
	);

	// Default constructor
	Shared_Memory_Input() {
		//Initialise the current state
		state.current_state = States::POLL;

		//Set the rate at which the shared memory segement will be polled.
		polling_rate = TIME("00:00:00:100");
		
		//Open and map the shared memory segment.
		// shared_memory_object shdmem{open_or_create, DEFAULT_SHARED_MEMORY_NAME, read_only, sizeof(MSG)};
		// shdmem.truncate(sizeof(MSG));
		// mapped_region region{shdmem, read_only};
		// message = static_cast<MSG*>(region.get_address());
#ifndef WIN32
        //Get an ID for the shared memory segement with the name.
        int hMapFile = shm_open(DEFAULT_SHARED_MEMORY_NAME, O_CREAT | O_RDWR, 0666); //Create file descriptor for shared mem. Create the mem if it doesn't already exist. Set permissions to 666

        //If the ID is invalid,
        if (hMapFile < 0) {
            //Could not create the shared memory file descriptor via shm_open()
            //Print an error message and return failure.
            throw(std::runtime_error("Couldn't connect to shared memory."));
        }

        //Try to map the shared memory segement to a shared_data_t structure.
        ftruncate(hMapFile, sizeof(MSG));
        message = (MSG*)mmap(NULL, sizeof(MSG), PROT_READ | PROT_WRITE, MAP_SHARED, hMapFile, 0);

        //If the mapping is unsuccessful,
        if (message == MAP_FAILED) {
            //Print an error message, and return failure.
            throw(std::runtime_error("Couldn't map view of file."));
        }
#else 
        //Windows is unsupported at this time.
        throw(std::runtime_error("Windows is unsupported at this time for this model."))
#endif
	}

	// Constructor with polling rate and name parameters
	Shared_Memory_Input(TIME rate, string name) {
		//Initialise the current state
		state.current_state = States::POLL;

		//Set the rate at which the shared memory segement will be polled.
		polling_rate = rate;

		//Open and map the shared memory segment.
		// shared_memory_object shdmem{open_or_create, name, read_only, sizeof(MSG)};
		// shdmem.truncate(sizeof(MSG));
		// mapped_region region{shdmem, read_only};
		// message = static_cast<MSG*>(region.get_address());

#ifndef WIN32
        //Get an ID for the shared memory segement with the name.
        int hMapFile = shm_open(name.c_str(), O_CREAT | O_RDWR, 0666); //Create file descriptor for shared mem. Create the mem if it doesn't already exist. Set permissions to 666

        //If the ID is invalid,
        if (hMapFile < 0) {
            //Could not create the shared memory file descriptor via shm_open()
            //Print an error message and return failure.
            throw(std::runtime_error("Couldn't connect to shared memory."));
        }

        //Try to map the shared memory segement to a shared_data_t structure.
        ftruncate(hMapFile, sizeof(MSG));
        message = (MSG*)mmap(NULL, sizeof(MSG), PROT_READ | PROT_WRITE, MAP_SHARED, hMapFile, 0);

        //If the mapping is unsuccessful,
        if (message == MAP_FAILED) {
            //Print an error message, and return failure.
            throw(std::runtime_error("Couldn't map view of file."));
        }
#else 
        //Windows is unsupported at this time.
        throw(std::runtime_error("Windows is unsupported at this time for this model."))
#endif
	}

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct state_type {
		States current_state;
	};
	state_type state;

	// Create a tuple of input ports (required for the simulator)
	using input_ports = std::tuple<typename Shared_Memory_Input_defs<MSG>::i_quit>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = std::tuple<typename Shared_Memory_Input_defs<MSG>::o_message>;

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
	void internal_transition() {
		switch (state.current_state)
		{
			case States::POLL:
				state.current_state = States::SEND;
				break;

			case States::SEND:
				state.current_state = States::POLL;
				break;

			default:
				break;
		}
	}

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		if (get_messages<typename Shared_Memory_Input_defs<MSG>::i_quit>(mbs).size() >= 1) {
			state.current_state = States::IDLE;
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

		if (state.current_state == States::SEND) {
			bag_port_message.push_back(*message);
			get_messages<typename Shared_Memory_Input_defs<MSG>::o_message>(bags) = bag_port_message;
		}
		return bags;
	}

	// Time advance
	// Used to set the internal time of the current state
	TIME time_advance() const {
		switch (state.current_state) {
			case States::IDLE:
				return std::numeric_limits<TIME>::infinity();
			case States::POLL:
				return polling_rate;
			case States::SEND:
				return TIME("00:00:00:000");
			default:
				return TIME("00:00:00:000");
		}
	}

	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Shared_Memory_Input<MSG, TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state);
		return os;
	}
};

#endif /* RT_LINUX */
#endif /* SHARED_MEMORY_INPUT_HPP */
