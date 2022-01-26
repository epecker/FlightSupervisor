/**
 *	\brief		An atomic model for receiving command line input from the user.
 *	\details	This header file defines an asynchronous input atomic model 
                for use in the RT-Cadmium DEVS simulation software. This atomic 
                model is intended to be used as a demonstration of asynchronous 
                input techniques using RT-Cadmium.
 *	\author		James Horner
 */

#ifndef USER_INPUT_HPP
#define USER_INPUT_HPP

// System libraries
#include <iostream>
#include <assert.h>
#include <thread>
#include <mutex>
#include <string>
#include <chrono>
#include <sstream>

// RT-Cadmium
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <cadmium/modeling/dynamic_model.hpp>

#include "../enum_string_conversion.hpp"

#ifdef RT_LINUX

using namespace cadmium;
using namespace std;

void get_input(mutex *lock, string *input);

// Input and output port definitions
template<typename MSG> struct Command_Line_Input_defs{
    struct out  : public out_port<MSG> { };
};

// Atomic model
template<typename MSG, typename TIME>
class Command_Line_Input {

// Private members for thread management.
private:
    string *input;
    TIME polling_rate;

public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(INPUT)
	);

    // Default constructor
    Command_Line_Input() {
        //Just use the other constructor with 100ms polling
        Command_Line_Input(TIME("00:00:00:100"));
    }

    // Constructor with polling rate parameter
    Command_Line_Input(TIME rate) {
        //Initialise the current state
        state.current_state = States::INPUT;

        //Create the mutex and user input variable
        state.input_mutex = new mutex();
        input = new string();
        polling_rate = rate;

        //Start the user input thread.
        thread(get_input, state.input_mutex, input).detach();
    }

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
    struct state_type{
        States current_state;
        mutex *input_mutex; 
    };
    state_type state;

	// Create a tuple of input ports (required for the simulator)
    using input_ports=std::tuple<>;
 
    // Create a tuple of output ports (required for the simulator)
    using output_ports=std::tuple<typename Command_Line_Input_defs<MSG>::out>;

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
    void internal_transition() {
        if (state.current_state == States::INPUT) {
            //If the thread has finished receiving input, change state.
            if (state.input_mutex->try_lock()) {
                if (input->compare("q") == 0) {
                    state.current_state = States::IDLE;
                }
                else {
                    //Start the user input thread.
                    thread(get_input, state.input_mutex, input).detach();
                }
                state.input_mutex->unlock();
            }
        }
    }

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
    void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
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
        if(state.current_state == States::INPUT) {
            //If the thread has finished receiving input, send the string as output.
            if(state.input_mutex->try_lock()) {
                if (input->compare("q") != 0) {
                    MSG message;
                    try {
                        stringstream ss(*input);
                        ss >> message;
                        cout << "Output sent: " << message << endl;
                        get_messages<typename Command_Line_Input_defs<MSG>::out>(bags).push_back(message);
                    }
                    catch(const string exception) {
                        cout << "Error parsing text input into message struct: " << exception << endl;
                    }
                }
                state.input_mutex->unlock();
            } 
        }
        return bags;
    }

	// Time advance
	// Used to set the internal time of the current state
    TIME time_advance() const {
        switch (state.current_state) {
            case States::IDLE:
                return std::numeric_limits<TIME>::infinity();
            case States::INPUT:
                return polling_rate;
            default:
                return TIME("00:00:00:000");
        }
    }

    friend std::ostringstream& operator<<(std::ostringstream& os, const typename Command_Line_Input<MSG, TIME>::state_type& i) {
        bool is_unlocked = i.input_mutex->try_lock();
        if (is_unlocked) {
            i.input_mutex->unlock();
        }
        os << "State: " << enumToString(i.current_state) << "-" << (is_unlocked ? "UNLOCKED" : "LOCKED");
        return os;
    }
};

// Function used to retrieve user input in a thread.
void get_input(mutex *lock, string *input) {
    lock->lock();
    cout << "Please enter any input (enter q to quit): ";
    getline(cin, *input);
    lock->unlock();
}

#endif /* RT_LINUX */

#endif /* USER_INPUT_HPP */
