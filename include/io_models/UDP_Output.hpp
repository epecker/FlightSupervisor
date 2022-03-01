/**
 *	\brief		An atomic model for sending UDP packets to an address and port.
 *	\details	This header file defines a output atomic model for use in the
                Cadmium DEVS simulation software.  
 *	\author		James Horner
 */

#ifndef UDP_OUTPUT_HPP
#define UDP_OUTPUT_HPP

// System libraries
#include <iostream>
#include <assert.h>
#include <thread>
#include <mutex>
#include <string>
#include <chrono>

#include <boost/asio.hpp>

// RT-Cadmium
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <cadmium/modeling/dynamic_model.hpp>

#include "enum_string_conversion.hpp"
#include "Constants.hpp"

using namespace cadmium;
using namespace std;
using namespace boost;



// Input and output port definitions
template<typename MSG> struct UDP_Output_defs{
    struct i_message   : public in_port<MSG> { };
};

// Atomic model
template<typename MSG, typename TIME>
class UDP_Output {

// Private members for thread management.
private:
    asio::ip::udp::endpoint network_endpoint;
    MSG message;
    
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(SENDING)
	);

    // Default constructor
    UDP_Output() {
        //Just use the other constructor with 100ms polling
        UDP_Output(TIME("00:00:00:100"));
    }

    // Constructor with polling rate parameter
    UDP_Output(string address, string port) {
        state.current_state = States::IDLE;
        unsigned short port_num = (unsigned short) strtoul(port.c_str(), NULL, 0);
        network_endpoint = asio::ip::udp::endpoint(asio::ip::address::from_string(address), port_num);
    }

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
    struct state_type{
        States current_state;
    };
    state_type state;

	// Create a tuple of input ports (required for the simulator)
    using input_ports=std::tuple<typename UDP_Output_defs<MSG>::i_message>;
 
    // Create a tuple of output ports (required for the simulator)
    using output_ports=std::tuple<>;

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
    void internal_transition() {
        if (state.current_state == States::SENDING) {
            state.current_state = States::IDLE;
        }
    }

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
    void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
        bool have_message = get_messages<typename UDP_Output_defs<MSG>::i_message>(mbs).size() >= 1;
        if (state.current_state == States::IDLE) {
            if (have_message){
                state.current_state = States::SENDING;
                message = get_messages<typename UDP_Output_defs<MSG>::i_message>(mbs)[0];
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
        asio::io_service io_service;
        asio::ip::udp::socket socket(io_service);
        boost::system::error_code err;
        char data[sizeof(MSG)];
        switch(state.current_state) {
            case States::SENDING:
                memcpy(data, &message, sizeof(data)); // Convert back to MSG: MSG recv = MSG(); memcpy(recv, data, sizeof(data));
                socket.open(asio::ip::udp::v4());
                socket.send_to(asio::buffer(data), network_endpoint, 0, err);
                socket.close();
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
                return TIME("00:00:00:000");
        }
    }

    friend std::ostringstream& operator<<(std::ostringstream& os, const typename UDP_Output<MSG, TIME>::state_type& i) {
        os << "State: " << enumToString(i.current_state) + string("\n");
        return os;
    }
};

#endif /* UDP_OUTPUT_HPP */
