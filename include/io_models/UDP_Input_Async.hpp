/**
 *	\brief		An atomic model for receiving UDP packets on a specified port.
 *	\details	This header file defines an asynchronous input atomic model
				(without polling) for use in the RT-Cadmium DEVS simulation 
				software. 
 *	\author		James Horner
 */

#ifndef UDP_INPUT_ASYNC_HPP
#define UDP_INPUT_ASYNC_HPP

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

// Message Structures
#include "message_structures/message_command_ack_t.hpp"

// Includes
#include "enum_string_conversion.hpp"
#include "Constants.hpp"

#ifdef RT_LINUX

using namespace cadmium;
using namespace std;

// Input and output port definitions
template<typename MSG>
struct UDP_Input_Async_defs {
	struct o_message : public out_port<MSG> { };
	struct i_quit : public in_port<bool> { };
};

// Atomic model
template<typename MSG, typename TIME>
class UDP_Input_Async {

	// Private members for thread management.
private:
	cadmium::dynamic::modeling::AsyncEventSubject* _sub;
	
	boost::asio::ip::udp::endpoint network_endpoint;
	boost::asio::ip::udp::endpoint remote_endpoint;
	boost::asio::io_service io_service;
	boost::asio::ip::udp::socket socket{ io_service };
	bool send_ack;
	char recv_buffer[MAX_SER_BUFFER_CHARS];
	int notifies;
	
	// Mutex for thread synchronization using unique locks.
	mutable std::mutex input_mutex;

public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(INPUT)
	);

	//Parameters to be overwriten when instantiating the atomic model
	//Callback_Tester* tester;

	// Default Constructor
	UDP_Input_Async() {
		//Initialise the current state
		state.current_state = States::INPUT;
		state.has_messages = !state.message.empty();

		//Create the network endpoint using a default address and port.
		send_ack = false;
		unsigned short port_num = (unsigned short)MAVLINK_OVER_UDP_PORT;
		network_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), port_num);

		//Start the user input thread.
		std::thread(&UDP_Input_Async::receive_packet_thread, this).detach();
	}

	//Constructor with address and port as well as whether acknowledgements are required.
	UDP_Input_Async(cadmium::dynamic::modeling::AsyncEventSubject* sub, bool ack_required, string port) {
		//Initialise the current state
		state.current_state = States::INPUT;
		state.has_messages = !state.message.empty();
		_sub = sub;

		//Create the network endpoint using the supplied address and port.
		send_ack = ack_required;
		unsigned short port_num = (unsigned short)strtoul(port.c_str(), NULL, 0);
		network_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), port_num);

		//Start the user input thread.
		std::thread(&UDP_Input_Async::receive_packet_thread, this).detach();
	}

	// Destructor for the class that stops the packet receipt thread
	~UDP_Input_Async() {
		//Before exiting stop the Boost IO service to interupt the receipt handler.
		io_service.stop();
		std::terminate(child_thread);
		if (socket.is_open()) 
			socket.close();
	}

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct state_type {
		States current_state;
		bool has_messages;
		mutable std::vector<MSG> message;
	};
	state_type state;

	// Create a tuple of input ports (required for the simulator)
	using input_ports = std::tuple<typename UDP_Input_Async_defs<MSG>::i_quit>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = std::tuple<typename UDP_Input_Async_defs<MSG>::o_message>;

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
	void internal_transition() {
		std::unique_lock<std::mutex> mutexLock(input_mutex, std::defer_lock);
		//If the thread has finished receiving input, change state if there are messages.
		if (mutexLock.try_lock()) {
			state.has_messages = !state.message.empty();
		}
	}

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		std::unique_lock<std::mutex> mutexLock(input_mutex, std::defer_lock);
		//If the thread has finished receiving input, change state if there are messages.
		if (mutexLock.try_lock()) {
			state.has_messages = !state.message.empty();
		}

		if (get_messages<typename UDP_Input_Async_defs<MSG>::i_quit>(mbs).size() >= 1) {
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
		std::unique_lock<std::mutex> mutexLock(input_mutex, std::defer_lock);
		if (state.current_state == States::INPUT) {
			//If the lock is free and there are messages, send the messages.
			if (!state.message.empty() && mutexLock.try_lock()) {
				for (auto msg : state.message) {
					message_out.push_back(msg);
				}
				state.message.clear();
				get_messages<typename UDP_Input_Async_defs<MSG>::o_message>(bags) = message_out;
			}
		}
		return bags;
	}

	// Time advance
	// Used to set the internal time of the current state
	TIME time_advance() const {
		if (!state.message.empty() && state.current_state == States::INPUT) {
			return TIME("00:00:00:00");
		}

		return std::numeric_limits<TIME>::infinity();
	}

	// Child thread for receiving UDP packets
	void receive_packet_thread() {
		//Open and bind the socket using Boost.
		socket.open(boost::asio::ip::udp::v4());
		socket.bind(network_endpoint);

		//While the model is not passivated,
		while (state.current_state != States::IDLE) {
			//Reset the io service then asynchronously receive a packet and 
			//use the handler to add it to the state.message vector.
			io_service.reset();
			socket.async_receive_from(
				boost::asio::buffer(recv_buffer),
				remote_endpoint,
				bind(
				&UDP_Input_Async::receive_packet,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));

			//Receive one packet then loop.
			io_service.run_one();
		}
		//Once done, close the socket.
		socket.close();
	}

	//state.message handler that is called on UDP packet receipt.
	void receive_packet(const boost::system::error_code& error, size_t bytes_transferred) {
		//Aquire the unique lock for the state.message vector.
		std::unique_lock<std::mutex> mutexLock(input_mutex);
		if (error) return;

		//Add the state.message to the vector.
		MSG recv = MSG();
		memcpy(&recv, &recv_buffer, bytes_transferred);
		state.message.insert(state.message.begin(), recv);

		//If an ack is required,
		if (send_ack) {
			//Construct the ack state.message and associated data array.
			message_command_ack_t ack_message(MAV_CMD_DEFAULT, MAV_RESULT_ACCEPTED, 0, 0, 0, 0);
			boost::system::error_code ack_err;
			char ack_data[sizeof(message_command_ack_t)];
			memcpy(ack_data, &ack_message, sizeof(ack_data));

			//Send the ack to the origin of the packet.
			socket.send_to(boost::asio::buffer(ack_data), remote_endpoint, 0, ack_err);
		}
		notifies++;
		_sub->notify();
	}

	friend std::ostringstream& operator<<(std::ostringstream& os, const typename UDP_Input_Async<MSG, TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state) << "-" << (i.has_messages ? "MESSAGES" : "NO_MESSAGES");
		return os;
	}
};

#endif // RT_LINUX
#endif // UDP_INPUT_ASYNC_HPP