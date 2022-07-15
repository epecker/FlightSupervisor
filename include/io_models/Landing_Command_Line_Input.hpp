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

//Messages structures
#include "message_structures/message_landing_point_t.hpp"
#include "message_structures/message_aircraft_state_t.hpp"

#ifdef RT_LINUX

using namespace cadmium;
using namespace std;

void get_input(mutex *lock, string *input);

// Input and output port definitions
struct landing_Command_Line_Input_defs{
	struct o_landing_achieved : public out_port<bool> {};
	struct o_aircraft_state : public out_port<message_aircraft_state_t> {};
	struct o_pilot_takeover : public out_port<bool> {};
	struct o_LP_recv : public out_port<message_landing_point_t> {};
	struct o_PLP_ach : public out_port<message_landing_point_t> {};
};

// Atomic model
template<typename TIME>
class Landing_Command_Line_Input {

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
    Landing_Command_Line_Input() {
        //Just use the other constructor with 100ms polling
        Landing_Command_Line_Input(TIME("00:00:00:100"));
    }

    // Constructor with polling rate parameter
    Landing_Command_Line_Input(TIME rate) {
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
    using output_ports=std::tuple<
        typename landing_Command_Line_Input_defs::o_landing_achieved,
        typename landing_Command_Line_Input_defs::o_aircraft_state,
        typename landing_Command_Line_Input_defs::o_pilot_takeover,
        typename landing_Command_Line_Input_defs::o_LP_recv,
        typename landing_Command_Line_Input_defs::o_PLP_ach
    >;

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
                    int port;
                    try {
                        stringstream ss(*input);
                        ss >> port;
                        //cout << "Port: " << port << "Input: " << *input << endl;
                        if (port == 0) {
                            bool value;
                            ss >> value;
                            get_messages<typename landing_Command_Line_Input_defs::o_landing_achieved>(bags).push_back(value);
                        }
                        else if (port == 1) {
                            message_aircraft_state_t value;
                            ss >> value;
                            get_messages<typename landing_Command_Line_Input_defs::o_aircraft_state>(bags).push_back(value);
                        }
                        else if (port == 2) {
                            bool value;
                            ss >> value;
                            get_messages<typename landing_Command_Line_Input_defs::o_pilot_takeover>(bags).push_back(value);
                        }
                        else if (port == 3) {
                            message_landing_point_t value;
                            ss >> value;
                            get_messages<typename landing_Command_Line_Input_defs::o_LP_recv>(bags).push_back(value);
                        }
                        else if (port == 4) {
                            message_landing_point_t value;
                            ss >> value;
                            get_messages<typename landing_Command_Line_Input_defs::o_PLP_ach>(bags).push_back(value);
                        }
                        else {
                            cout << "Invalid port number: " << port << endl;
                        }
                    }
                    catch(const string exception) {
                        cout << "Error parsing text input into port and message struct: " << exception << endl;
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

    friend std::ostringstream& operator<<(std::ostringstream& os, const typename Landing_Command_Line_Input<TIME>::state_type& i) {
        bool is_unlocked = i.input_mutex->try_lock();
        if (is_unlocked) {
            i.input_mutex->unlock();
        }
        os << "State: " << enumToString(i.current_state) << "-" << (is_unlocked ? "UNLOCKED" : "LOCKED") << endl;
        return os;
    }
};

// Function used to retrieve user input in a thread.
void get_input(mutex *lock, string *input) {
    lock->lock();
    cout << "Please enter a port name and value:" << endl;
    cout << "\tLanding Achieved: \t0 value(boolean)" << endl;
    cout << "\tAircraft State: \t1 lat(float) lon(double) alt_AGL(double) alt_MSL(double) hdg(float) vel(float)" << endl;
    cout << "\tPilot Takeover: \t2 value(boolean)" << endl;
    cout << "\tLP Received: \t\t3 id(int) lat(double) lon(double) alt(double) hdg(double_" << endl;
    cout << "\tPLP Achieved: \t\t4 id(int) lat(double) lon(double) alt(double) hdg(double)" << endl;
    cout << "Enter q to quit" << endl;
    cout << ">>> ";
    getline(cin, *input);
    lock->unlock();
}

#endif /* RT_LINUX */

#endif /* USER_INPUT_HPP */
