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

using namespace cadmium;
using namespace std;
using namespace boost::asio::ip;

// Input and output port definitions
template<typename MSG> struct UDP_Output_defs{
    struct i_message   : public in_port<MSG> { };
};

// Atomic model
template<typename MSG, typename TIME>
class UDP_Output {

// Private members for thread management.
private:
    MSG message;
    
    string destination_address;
    int destination_port;

    udp::socket m_socket; 
    udp::resolver::results_type m_endpoint;

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
        //Initialise the current state
        state.current_state = States::IDLE;

        boost::asio::io_context io_context;

        udp::resolver resolver(io_context);
        m_endpoint = resolver.resolve(udp::v4(), address, port);

        m_socket = udp::socket(io_context, m_endpoint.begin());


        // udp::resolver resolver(io_service);

        // socket = socket(io_service);
        // socket.open(udp::v4());
        // socket.set_option(socket_base::broadcast(true));

        // addrinfo hints;
        // memset(&hints, 0, sizeof(hints));
        // hints.ai_family = AF_UNSPEC;
        // hints.ai_socktype = SOCK_DGRAM;
        // hints.ai_protocol = IPPROTO_UDP;
        // int r(getaddrinfo(address.c_str(), string(destination_port).c_str(), &hints, &address_info));
        // if(r != 0 || address_info == NULL)
        // {
        //     throw udp_client_server_runtime_error(("invalid address or port: \"" + destination_address + ":" + destination_port + "\"").c_str());
        // }
        // socket = socket(address_info->ai_family, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
        // if(socket == -1)
        // {
        //     freeaddrinfo(address_info);
        //     throw udp_client_server_runtime_error(("could not create socket for: \"" + destination_address + ":" + destination_port + "\"").c_str());
        // }
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
        if(state.current_state == States::SENDING) {
            string message_string;
            message_string  = "test";
            // sendto(socket, message_string.c_str, message_string.size(), 0, address_info->ai_addr, address_info->aiaddrlen);
            //socket.send_to(boost::asio::buffer(message_string), receiver_endpoint);
            boost::system::error_code err;
            m_socket.send_to(boost::asio::buffer(message), m_endpoint.begin(), 0, err);
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
