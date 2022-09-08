/**
 *	\brief		An atomic model for receiving command line input from the user.
 *	\details	This header file defines an asynchronous input atomic model
				for use in the RT-Cadmium DEVS simulation software. This atomic
				model is intended to be used as a demonstration of asynchronous
				input techniques using RT-Cadmium.
 *	\author		James Horner
 */

#ifndef UDP_INPUT_HPP
#define UDP_INPUT_HPP

 // System libraries
#include <iostream>
#include <assert.h>
#include <thread>
#include <mutex>
#include <string>
#include <chrono>
#include <sstream>
#include <csignal>

#include <boost/asio.hpp>

// RT-Cadmium
#include "cadmium/engine/pdevs_dynamic_runner.hpp"
#include "cadmium/modeling/ports.hpp"
#include "cadmium/modeling/message_bag.hpp"
#include "cadmium/modeling/dynamic_model.hpp"

// Message Structures
#include "../message_structures/message_command_ack_t.hpp"

// Includes
#include "../enum_string_conversion.hpp"
#include "../Constants.hpp"

#ifdef RT_LINUX

// Input and output port definitions
template<typename MSG> struct UDP_Input_defs {
	struct o_message : public cadmium::out_port<MSG> { };
	struct i_quit : public cadmium::in_port<bool> { };
};

// Atomic model
template<typename MSG, typename TIME>
class UDP_Input {

	// Private members for thread management.
private:
	// Mutex for thread synchronization using unique locks.
	mutable std::mutex input_mutex;
	mutable std::vector<MSG> message;

	// Networking members
	boost::asio::ip::udp::endpoint endpoint_local;
	boost::asio::ip::udp::endpoint endpoint_remote;
	boost::asio::io_service io_service;
	boost::asio::ip::udp::socket socket{ io_service };
	char recv_buffer[MAX_SER_BUFFER_CHARS];

	// Const Members
	bool send_ack;
	TIME polling_rate;

	// Global member for thread sync
	bool stop;

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
		state.has_messages = false;

		polling_rate = TIME("00:00:00:100");
		send_ack = false;
		stop = false;

		//Create the network endpoint using a default address and port.
		unsigned short port_num = (unsigned short)MAVLINK_OVER_UDP_PORT;
		endpoint_local = boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), port_num);

		//Set the interupts for the program to stop the child thread.
		// std::signal(SIGINT, UDP_Input::handle_signal);
		// std::signal(SIGTERM, UDP_Input::handle_signal);

		//Start the user input thread.
		std::thread(&UDP_Input::receive_packet_thread, this).detach();
	}

	// Constructor with polling rate parameter
	UDP_Input(TIME rate, bool ack_required, std::string port) {
		//Initialise the current state
		state.current_state = States::INPUT;
		state.has_messages = false;

		polling_rate = rate;
		send_ack = ack_required;
		stop = false;

		//Create the network endpoint using the supplied address and port.
		unsigned short port_num = (unsigned short)strtoul(port.c_str(), NULL, 0);
		endpoint_local = boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), port_num);

		//Set the interupts for the program to stop the child thread.
		// std::signal(SIGINT, UDP_Input::handle_signal);
		// std::signal(SIGTERM, UDP_Input::handle_signal);

		//Start the user input thread.
		std::thread(&UDP_Input::receive_packet_thread, this).detach();
	}

	// Destructor for the class that shuts down gracefully
	~UDP_Input() {
		shutdown();
	}

	// Handler for signals.
    void handle_signal(int signal_number) {
		shutdown();
	}

	// Member for shutting down the thread and IO services gracefully.
	void shutdown() {
		//Before exiting stop the Boost IO service to interupt the receipt handler.
		stop = true;
		io_service.stop();
		if (socket.is_open())
			socket.close();
	}

	// This is used to track the state of the atomic model.
	// (required for the simulator)
	struct state_type {
		States current_state;
		bool has_messages;
	};
	state_type state;

	// Create a tuple of input ports (required for the simulator)
	using input_ports = std::tuple<typename UDP_Input_defs<MSG>::i_quit>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = std::tuple<typename UDP_Input_defs<MSG>::o_message>;

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
	void internal_transition() {
		if (state.current_state == States::INPUT) {
			std::unique_lock<std::mutex> mutexLock(input_mutex, std::defer_lock);
			//If the thread has finished receiving input, change state if there are messages.
			if (mutexLock.try_lock()) {
				state.has_messages = !message.empty();
			}
		}
	}

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
	void external_transition(TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		if (cadmium::get_messages<typename UDP_Input_defs<MSG>::i_quit>(mbs).size() >= 1) {
			state.current_state = States::IDLE;
		}
	}

	// Confluence transition
	// Used to call set call precedence
	void confluence_transition(TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), std::move(mbs));
	}

	// Output function
	typename cadmium::make_message_bags<output_ports>::type output() const {
		typename cadmium::make_message_bags<output_ports>::type bags;
		std::vector<MSG> message_out;

		if (state.current_state == States::INPUT) {
			//If the lock is free and there are messages, send the messages.
			std::unique_lock<std::mutex> mutexLock(input_mutex, std::defer_lock);
			if (state.has_messages && mutexLock.try_lock()) {
				for (auto msg : message) {
					message_out.push_back(msg);
				}
				message.clear();
			 cadmium::get_messages<typename UDP_Input_defs<MSG>::o_message>(bags) = message_out;
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
				if (state.has_messages) {
					return TIME(TA_ZERO);
				} else {
					return polling_rate;
				}
			default:
				return TIME(TA_ZERO);
		}
	}

	// Child thread for receiving UDP packets
	void receive_packet_thread() {
		//Open and bind the socket using Boost.
		socket.open(boost::asio::ip::udp::v4());
		socket.bind(endpoint_local);

		//While the model is not passivated,
		while (state.current_state != States::IDLE && !stop) {
			//Reset the io service then asynchronously receive a packet and
			//use the handler to add it to the message vector.
			io_service.restart();
			socket.async_receive_from(
				boost::asio::buffer(recv_buffer),
				endpoint_remote,
				bind(
					&UDP_Input::receive_packet,
					this,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred));

			//Receive one packet then loop.
			io_service.run_one();
		}
		//Once done, close the socket.
		socket.close();
	}

	//Message handler that is called on UDP packet receipt.
	void receive_packet(const boost::system::error_code& error, size_t bytes_transferred) {
		//Aquire the unique lock for the message vector.
		std::unique_lock<std::mutex> mutexLock(input_mutex);
		if (error) return;

		//Add the message to the vector.
		MSG recv = MSG();
		memcpy(&recv, &recv_buffer, bytes_transferred);
		message.insert(message.begin(), recv);

		//If an ack is required,
		if (send_ack) {
			//Construct the ack message and associated data array.
			message_command_ack_t ack_message(MAV_CMD_DEFAULT, MAV_RESULT_ACCEPTED, 0, 0, 0, 0);
			boost::system::error_code ack_err;
			char ack_data[sizeof(message_command_ack_t)];
			memcpy(ack_data, &ack_message, sizeof(ack_data));

			//Send the ack to the origin of the packet.
			socket.send_to(boost::asio::buffer(ack_data), endpoint_remote, 0, ack_err);
		}
	}

	friend std::ostringstream& operator<<(std::ostringstream& os, const typename UDP_Input<MSG, TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state) << "-" << (i.has_messages ? "MESSAGES" : "NO_MESSAGES");
		return os;
	}
};

#endif /* RT_LINUX */
#endif /* UDP_INPUT_HPP */
