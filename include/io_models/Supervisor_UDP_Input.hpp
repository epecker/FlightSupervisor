/**
 *	\brief		An atomic model for receiving command line input from the user.
 *	\details	This header file defines an asynchronous input atomic model
				for use in the RT-Cadmium DEVS simulation software. This atomic
				model is intended to be used as a demonstration of asynchronous
				input techniques using RT-Cadmium.
 *	\author		James Horner
 */

#ifndef SUPERVISOR_UDP_INPUT_HPP
#define SUPERVISOR_UDP_INPUT_HPP

 // System libraries
#include <iostream>
#include <assert.h>
#include <thread>
#include <mutex>
#include <string>
#include <string.h>
#include <chrono>
#include <sstream>
#include <csignal>

// RT-Cadmium
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <cadmium/modeling/dynamic_model.hpp>

// RUDP
#include "RUDP/src/ConnectionController.hpp"

// Message Structures
#include "message_structures/message_landing_point_t.hpp"
#include "message_structures/message_start_supervisor_t.hpp"

// Includes
#include "enum_string_conversion.hpp"
#include "Constants.hpp"
#include "component_macros.h"

#define RT_LINUX
#ifdef RT_LINUX

using namespace cadmium;
using namespace std;

// Input and output port definitions
struct Supervisor_UDP_Input_defs {
	struct o_start_supervisor 	: public out_port<message_start_supervisor_t> { };
	struct o_perception_status	: public out_port<bool> { };
	struct o_waypoint 			: public out_port<message_landing_point_t> { };
	struct o_lp_recv			: public out_port<message_landing_point_t> { };
	struct o_plp_ach 			: public out_port<message_landing_point_t> { };
	struct i_quit 				: public in_port<bool> { };
};

// Atomic model
template<typename MSG, typename TIME>
class Supervisor_UDP_Input {

	// Private members for thread management.
private:
	// Mutex for thread synchronization using unique locks.
	mutable mutex input_mutex;
	mutable vector<message_start_supervisor_t> message_start_supervisor;
	mutable vector<bool> message_perception_status;
	mutable vector<message_landing_point_t> message_waypoint;
	mutable vector<message_landing_point_t> message_lp_recv;
	mutable vector<message_landing_point_t> message_plp_ach;

	// Networking members
	rudp::Connection * connection;
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
	Supervisor_UDP_Input() {
		//Initialise the current state
		state.current_state = States::INPUT;
		state.has_messages = false;

		polling_rate = TIME("00:00:00:100");
		send_ack = false;
		stop = false;

		//Create the network endpoint using a default address and port.
		int connection_number = rudp::ConnectionController::getInstance()->addConnection(DEFAULT_TIMEOUT_MS);
		connection = rudp::ConnectionController::getInstance()->getConnection(connection_number);
		connection->setEndpointLocal(2300);

		//Set the interupts for the program to stop the child thread.
		// signal(SIGINT, Supervisor_UDP_Input::handle_signal);
		// signal(SIGTERM, Supervisor_UDP_Input::handle_signal);

		//Start the user input thread.
		thread(&Supervisor_UDP_Input::receive_packet_thread, this).detach();
	}

	// Constructor with polling rate parameter
	Supervisor_UDP_Input(TIME rate, unsigned short port) {
		//Initialise the current state
		state.current_state = States::INPUT;
		state.has_messages = false;

		polling_rate = rate;
		send_ack = ack_required;
		stop = false;

		//Create the network endpoint using the supplied address and port.
		int connection_number = rudp::ConnectionController::getInstance()->addConnection(DEFAULT_TIMEOUT_MS);
		connection = rudp::ConnectionController::getInstance()->getConnection(connection_number);
		connection->setEndpointLocal(port);

		//Set the interupts for the program to stop the child thread.
		// signal(SIGINT, Supervisor_UDP_Input::handle_signal);
		// signal(SIGTERM, Supervisor_UDP_Input::handle_signal);

		//Start the user input thread.
		thread(&Supervisor_UDP_Input::receive_packet_thread, this).detach();
	}

	// Destructor for the class that shuts down gracefully
	~Supervisor_UDP_Input() {
		shutdown();
	}

	// Handler for signals.
	static void handle_signal(int signal_number) {
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
	using input_ports = tuple<typename Supervisor_UDP_Input_defs<MSG>::i_quit>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<typename Supervisor_UDP_Input_defs<MSG>::o_message>;

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
	void internal_transition() {
		if (state.current_state == States::INPUT) {
			unique_lock<mutex> mutexLock(input_mutex, defer_lock);
			//If the thread has finished receiving input, change state if there are messages.
			if (mutexLock.try_lock()) {
				state.has_messages = !message.empty();
			}
		}
	}

	// External transitions
	// These are transitions occuring from external inputs
	// (required for the simulator)
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		if (get_messages<typename Supervisor_UDP_Input_defs<MSG>::i_quit>(mbs).size() >= 1) {
			state.current_state = States::IDLE;
		}
	}

	// Confluence transition
	// Used to call set call precedence
	void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), move(mbs));
	}

	// Output function
	typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		vector<message_start_supervisor_t> start_supervisor_out;
		vector<bool> perception_status_out;
		vector<message_landing_point_t> waypoint_out;
		vector<message_landing_point_t> lp_recv_out;
		vector<message_landing_point_t> plp_ach_out;

		if (state.current_state == States::INPUT) {
			//If the lock is free and there are messages, send the messages.
			unique_lock<mutex> mutexLock(input_mutex, defer_lock);
			if (state.has_messages && mutexLock.try_lock()) {
				for (auto msg : message_start_supervisor) {
					start_supervisor_out.push_back(msg);
				}
				message_start_supervisor.clear();
				get_messages<typename Supervisor_UDP_Input_defs<MSG>::o_message>(bags) = start_supervisor_out;

				for (auto msg : message_perception_status) {
					perception_status_out.push_back(msg);
				}
				message_perception_status.clear();
				get_messages<typename Supervisor_UDP_Input_defs<MSG>::o_message>(bags) = perception_status_out;
				
				for (auto msg : message_waypoint) {
					waypoint_out.push_back(msg);
				}
				message_waypoint.clear();
				get_messages<typename Supervisor_UDP_Input_defs<MSG>::o_message>(bags) = waypoint_out;

				for (auto msg : message_lp_recv) {
					lp_recv_out.push_back(msg);
				}
				message_lp_recv.clear();
				get_messages<typename Supervisor_UDP_Input_defs<MSG>::o_message>(bags) = lp_recv_out;
								
				for (auto msg : message_plp_ach) {
					plp_ach_out.push_back(msg);
				}
				message_plp_ach.clear();
				get_messages<typename Supervisor_UDP_Input_defs<MSG>::o_message>(bags) = plp_ach_out;
			}
		}
		return bags;
	}

	// Time advance
	// Used to set the internal time of the current state
	TIME time_advance() const {
		switch (state.current_state) {
			case States::IDLE:
				return numeric_limits<TIME>::infinity();
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
		//While the model is not passivated,
		while (state.current_state != States::IDLE && !stop) {
			string sender_address;
			int sender_port;
			int bytes_received = connection->receive(recv_buffer, MAX_SER_BUFFER_CHARS, &sender_address, &sender_port);
			 
			bool error_occured = false;
			uint8_t sysid;
			uint8_t compid;
			uint8_t sigid;
			vector<char>char_cache;
			int offset = 0;
			int data_length = bytes_received - 3 * sizeof(uint8_t);

			if (!error_occured) {
				// Parse the bytes of the received system ID.
				char_cache = vector<char>(sizeof(sysid));
				for (int i = 0; i < sizeof(sysid); i++)
				{
					char_cache[i] = recv_buffer[i + offset];
				}
				offset += sizeof(sysid);
				if (memcpy_s(&sysid, sizeof(sysid), char_cache.data(), sizeof(sysid)))
				{
					string error_message = "[Supervisor UDP Input] Error copying system ID from " + sender_address + ":" + to_string(sender_port) + "\n";
					cout << error_message;
					error_occured = true;
				}
			}

			if (!error_occured) {
				// Parse the bytes of the received component ID.
				char_cache = vector<char>(sizeof(compid));
				for (int i = 0; i < sizeof(compid); i++)
				{
					char_cache[i] = recv_buffer[i + offset];
				}
				offset += sizeof(compid);
				if (memcpy_s(&compid, sizeof(compid), char_cache.data(), sizeof(compid)))
				{
					string error_message = "[Supervisor UDP Input] Error copying component ID from " + sender_address + ":" + to_string(sender_port) + "\n";
					cout << error_message;
					error_occured = true;
				}
			}

			if (!error_occured) {
				// Parse the bytes of the received signal ID.
				char_cache = vector<char>(sizeof(sigid));
				for (int i = 0; i < sizeof(sigid); i++)
				{
					char_cache[i] = recv_buffer[i + offset];
				}
				offset += sizeof(sigid);
				if (memcpy_s(&sigid, sizeof(sigid), char_cache.data(), sizeof(sigid)))
				{
					string error_message = "[Supervisor UDP Input] Error copying signal ID from " + sender_address + ":" + to_string(sender_port) + "\n";
					cout << error_message;
					error_occured = true;
				}
			}

			if (!error_occured) {
				// Parse the bytes of the payload.
				char_cache = vector<char>(data_length);
				for (int i = 0; i < data_length; i++)
				{
					char_cache[i] = recv_buffer[i + offset];
				}
				offset += sizeof(sigid);

				message_start_supervisor_t temp_start_supervisor;
				bool temp_bool;
				message_landing_point_t temp_landing_point;
				unique_lock<mutex> mutexLock(input_mutex);
				if (sysid == SUPERVISOR_SIG_ID_PLP_ACHIEVED && compid == COMP_ID_MISSION_MANAGER) {
					memcpy_s(&temp_landing_point, sizeof(temp_landing_point), char_cache.data(), data_length);
					message_plp_ach.push_back(temp_landing_point);
					state.has_messages = true;
				}
				if (sysid == SUPERVISOR_SIG_ID_WAYPOINT && compid == COMP_ID_MISSION_MANAGER) {
					memcpy_s(&temp_landing_point, sizeof(temp_landing_point), char_cache.data(), data_length);
					message_waypoint.push_back(temp_landing_point);
					state.has_messages = true;
				}
				if (sysid ==  SUPERVISOR_SIG_ID_START_SUPERVISOR && compid == COMP_ID_MISSION_MANAGER) {
					memcpy_s(&temp_start_supervisor, sizeof(temp_start_supervisor), char_cache.data(), data_length);
					message_start_supervisor.push_back(temp_start_supervisor);
					state.has_messages = true;
				}
				if (sysid == SUPERVISOR_SIG_ID_PERCEPTION_STATUS && compid == COMP_ID_PERCEPTION_SYSTEM) {
					memcpy_s(&temp_bool, sizeof(temp_bool), char_cache.data(), data_length);
					message_perception_status.push_back(temp_bool);
					state.has_messages = true;
				}
				if (sysid == SUPERVISOR_SIG_ID_LP_RECEIVE && compid == COMP_ID_PERCEPTION_SYSTEM) {
					memcpy_s(&temp_landing_point, sizeof(temp_landing_point), char_cache.data(), data_length);
					message_lp_recv.push_back(temp_landing_point);
					state.has_messages = true;
				}
			}
		}
	}

	friend ostringstream& operator<<(ostringstream& os, const typename Supervisor_UDP_Input<MSG, TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state) << "-" << (i.has_messages ? "MESSAGES" : "NO_MESSAGES");
		return os;
	}
};

#endif /* RT_LINUX */
#endif /* SUPERVISOR_UDP_INPUT_HPP */
