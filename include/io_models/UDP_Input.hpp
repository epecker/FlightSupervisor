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

#include <boost/asio.hpp>

// RT-Cadmium
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <cadmium/modeling/dynamic_model.hpp>

#include "enum_string_conversion.hpp"
#include "Constants.hpp"

#ifdef RT_LINUX

using namespace cadmium;
using namespace std;

// Input and output port definitions
template<typename MSG> struct UDP_Input_defs{
    struct o_message  : public out_port<MSG> { };
    struct i_quit  : public in_port<bool> { };
};

// Atomic model
template<typename MSG, typename TIME>
class UDP_Input {

// Private members for thread management.
private:
    mutable std::vector<MSG> message;
    TIME polling_rate;
    asio::ip::udp::endpoint network_endpoint;
    asio::io_service io_service;
    asio::ip::udp::socket socket{io_service};
    bool temp;
    char recv_buffer[MAX_SER_BUFFER_CHARS];

public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(INPUT)
	);

    // Default constructor
    UDP_Input() {
        //Initialise the current state
        state.current_state = States::INPUT;

        //Create the mutex and user input variable
        state.input_mutex = new std::mutex();
        polling_rate = TIME("00:00:00:100");
        unsigned short port_num = (unsigned short) MAVLINK_OVER_UDP_PORT;
        network_endpoint = asio::ip::udp::endpoint(asio::ip::address::from_string(PEREGRINE_IP), port_num);
        //Start the user input thread.
        std::thread(&UDP_Input::receive_packet_thread, this).detach();
    }

    // Constructor with polling rate parameter
    UDP_Input(TIME rate, string address, string port) {
        //Initialise the current state
        state.current_state = States::INPUT;

        //Create the mutex and user input variable
        state.input_mutex = new std::mutex();
        polling_rate = rate;
        unsigned short port_num = (unsigned short) strtoul(port.c_str(), NULL, 0);
        network_endpoint = asio::ip::udp::endpoint(asio::ip::address::from_string(address), port_num);

        //Start the user input thread.
        std::thread(&UDP_Input::receive_packet_thread, this).detach();
    }

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
    struct state_type{
        States current_state;
        std::mutex * input_mutex; 
    };
    state_type state;

	// Create a tuple of input ports (required for the simulator)
    using input_ports=std::tuple<typename UDP_Input_defs<MSG>::i_quit>;
 
    // Create a tuple of output ports (required for the simulator)
    using output_ports=std::tuple<typename UDP_Input_defs<MSG>::o_message>;

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
    void internal_transition() {
        if (state.current_state == States::INPUT) {
            //If the thread has finished receiving input, change state.
            if (state.input_mutex->try_lock()) {
                //Restart the user input thread.
                std::thread(&UDP_Input::receive_packet_thread, this).detach();
                state.input_mutex->unlock();
            }
        }
    }

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
    void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
        if(get_messages<typename UDP_Input_defs<MSG>::i_quit>(mbs).size() >= 1) {
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
        vector<MSG> message_out;
    
        if(state.current_state == States::INPUT) {
            //If the thread has finished receiving input, send the string as output.
            if(state.input_mutex->try_lock()) {
                message_out.push_back(message.back());
                message.pop_back();
                get_messages<typename UDP_Input_defs<MSG>::o_message>(bags) = message_out;
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

    void receive_packet_thread() {
        state.input_mutex->lock();
        socket.open(asio::ip::udp::v4());
        socket.bind(network_endpoint);
        socket.async_receive_from(
            asio::buffer(recv_buffer),
            network_endpoint,
            bind(
                &UDP_Input::receive_packet, 
                this, 
                boost::asio::placeholders::error, 
                boost::asio::placeholders::bytes_transferred));
        io_service.run();
        state.input_mutex->unlock();
    }

    void receive_packet(const boost::system::error_code& error, size_t bytes_transferred) {
        if (error) return;
        MSG recv = MSG();
        memcpy(&recv, &recv_buffer, bytes_transferred);
        message.insert(message.begin(), recv);
    }

    friend std::ostringstream& operator<<(std::ostringstream& os, const typename UDP_Input<MSG, TIME>::state_type& i) {
        bool is_unlocked = i.input_mutex->try_lock();
        if (is_unlocked) {
            i.input_mutex->unlock();
        }
        os << "State: " << enumToString(i.current_state) << "-" << (is_unlocked ? "UNLOCKED" : "LOCKED");
        return os;
    }
};

#endif /* RT_LINUX */
#endif /* USER_INPUT_HPP */
