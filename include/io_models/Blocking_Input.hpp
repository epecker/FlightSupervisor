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

// RT-Cadmium
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <cadmium/modeling/dynamic_model.hpp>

#include "enum_string_conversion.hpp"

#ifdef RT_LINUX

using namespace cadmium;
using namespace std;

chrono::time_point<chrono::high_resolution_clock> start;
void get_user_input(mutex *lock, string *input);

// Input and output port definitions
struct User_Input_defs{
    struct in   : public in_port<bool> { };
    struct out  : public out_port<string> { };
};

// Atomic model
template<typename TIME>
class User_Input {

// Private members for thread management.
private:
    string *user_input;
    TIME polling_rate;

public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(INPUT)
	);

    // Default constructor
    User_Input() {
        //Just use the other constructor with 100ms polling
        User_Input(TIME("00:00:00:100"));
    }

    // Constructor with polling rate parameter
    User_Input(TIME rate) {
        //Initialise the current state
        state.current_state = States::IDLE;

        //Create the mutex and user input variable
        state.user_input_mutex = new mutex();
        user_input = new string();
        polling_rate = rate;

		start = chrono::high_resolution_clock::now(); //to measure simulation execution time
    }

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
    struct state_type{
        States current_state;
        mutex *user_input_mutex; 
    };
    state_type state;

	// Create a tuple of input ports (required for the simulator)
    using input_ports=std::tuple<typename User_Input_defs::in>;
 
    // Create a tuple of output ports (required for the simulator)
    using output_ports=std::tuple<typename User_Input_defs::out>;

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
    void internal_transition() {
        if (state.current_state == States::INPUT) {
            //If the thread has finished receiving input, change state.
            if (state.user_input_mutex->try_lock()) {
                state.current_state = States::IDLE;
                state.user_input_mutex->unlock();
            }
        }
    }

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
    void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
        bool start = get_messages<typename User_Input_defs::in>(mbs).size() >= 1;
        if (state.current_state == States::IDLE) {
            if (start){
                state.current_state = States::INPUT;
                //Start the user input thread on changing to the input state.
                thread(get_user_input, state.user_input_mutex, user_input).detach();
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
        if(state.current_state == States::INPUT) {
            //If the thread has finished receiving input, send the string as output.
            if(state.user_input_mutex->try_lock()) {
                get_messages<typename User_Input_defs::out>(bags).push_back(*user_input);
                auto elapsed = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1>>>(chrono::high_resolution_clock::now() - start).count();
                cout << "(" << elapsed << ") Output sent: " << *user_input << endl;
                state.user_input_mutex->unlock();
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

    friend std::ostringstream& operator<<(std::ostringstream& os, const typename User_Input<TIME>::state_type& i) {
        bool is_unlocked = i.user_input_mutex->try_lock();
        if (is_unlocked) {
            i.user_input_mutex->unlock();
        }
        os << "State: " << enumToString(i.current_state) << "-" << (is_unlocked ? "UNLOCKED" : "LOCKED");
        return os;
    }
};

// Function used to retrieve user input in a thread.
void get_user_input(mutex *lock, string *input) {
    lock->lock();
    auto elapsed = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1>>>(chrono::high_resolution_clock::now() - start).count();
    cout << "(" << elapsed << ") Please enter any input: ";
    cin >> *input;
    lock->unlock();
}

#endif /* RT_LINUX */

#endif /* USER_INPUT_HPP */
