/**
 *	\brief		An atomic model for receiving command line input from the user.
 *	\details	This header file defines an asynchronous input atomic model
				for use in the RT-Cadmium DEVS simulation software. This atomic
				model is intended to be used as a demonstration of asynchronous
				input techniques using RT-Cadmium.
 *	\author		James Horner
 *	\author		Tanner Trautrim
 */

#ifndef SUPERVISOR_UDP_INPUT_HPP
#define SUPERVISOR_UDP_INPUT_HPP

 // System libraries
#include <cassert>
#include <chrono>
#include <csignal>
#include <iostream>
#include <mutex>
#include <string>
#include <cstring>
#include <sstream>
#include <thread>

// RT-Cadmium
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <cadmium/modeling/dynamic_model.hpp>

// RUDP
#include "RUDP/src/ConnectionController.hpp"

// Message Structures
#include "message_structures/message_landing_point_t.hpp"
#include "message_structures/message_fcc_command_t.hpp"
#include "message_structures/message_start_supervisor_t.hpp"

// Includes
#include "enum_string_conversion.hpp"
#include "Constants.hpp"
#include "component_macros.h"

#ifdef RT_LINUX

using namespace cadmium;
using namespace std;

// Input and output port definitions
struct Supervisor_UDP_Input_defs {
	struct o_start_supervisor 	: public out_port<message_start_supervisor_t> { };
	struct o_perception_status	: public out_port<bool> { };
	struct o_waypoint 			: public out_port<message_fcc_command_t> { };
	struct o_lp_recv			: public out_port<message_landing_point_t> { };
	struct o_plp_ach 			: public out_port<message_landing_point_t> { };
	struct i_quit 				: public in_port<bool> { };
};

template<typename TIME>
class Supervisor_UDP_Input {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(INPUT)
	);

	// Default constructor
	Supervisor_UDP_Input() {
		state.current_state = States::INPUT;
		state.has_messages = false;
		polling_rate = TIME("00:00:00:100");
		stop = false;

		//Create the network endpoint
		connection_number = rudp::ConnectionController::addConnection(DEFAULT_TIMEOUT_MS);
		connection = rudp::ConnectionController::getConnection(connection_number);
		connection->setEndpointLocal(2300);

		//Start the user input thread.
		thread(&Supervisor_UDP_Input::receive_packet_thread, this).detach();
	}

	// Constructor with polling rate parameter
	Supervisor_UDP_Input(TIME rate, unsigned short port) {
		state.current_state = States::INPUT;
		state.has_messages = false;
		polling_rate = rate;
		stop = false;

		//Create the network endpoint
		connection_number = rudp::ConnectionController::addConnection(DEFAULT_TIMEOUT_MS);
		connection = rudp::ConnectionController::getConnection(connection_number);
		connection->setEndpointLocal(port);

		//Start the user input thread.
		thread(&Supervisor_UDP_Input::receive_packet_thread, this).detach();
	}

	// Destructor
	~Supervisor_UDP_Input() {
		stop = true;
		rudp::ConnectionController::removeConnection(connection_number);
	}

	// This is used to track the state of the atomic model. 
	// (required for the simulator)
	struct state_type {
		States current_state;
		bool has_messages;
	} state;

	// Create a tuple of input ports (required for the simulator)
	using input_ports = tuple<typename Supervisor_UDP_Input_defs::i_quit>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = tuple<
		typename Supervisor_UDP_Input_defs::o_start_supervisor,
		typename Supervisor_UDP_Input_defs::o_perception_status,
		typename Supervisor_UDP_Input_defs::o_waypoint,
		typename Supervisor_UDP_Input_defs::o_lp_recv,
		typename Supervisor_UDP_Input_defs::o_plp_ach
	>;

	// Internal transitions
	// These are transitions occuring from internal inputs
	// (required for the simulator)
	void internal_transition() {
		if (state.current_state == States::INPUT) {
			unique_lock<mutex> mutexLock(input_mutex, defer_lock);
			//If the thread has finished receiving input, change state if there are messages.
			if (mutexLock.try_lock()) {
				state.has_messages = (
					!message_start_supervisor.empty() ||
					!message_perception_status.empty() ||
					!message_waypoint.empty() ||
					!message_lp_recv.empty() ||
					!message_plp_ach.empty()
				);
			}
		}
	}

	// External transitions
	// These are transitions occurring from external inputs
	// (required for the simulator)
	void external_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {
        bool received_quit = !get_messages<typename Supervisor_UDP_Input_defs::i_quit>(mbs).empty();
		if (received_quit) {
			state.current_state = States::IDLE;
		}
	}

	// Confluence transition
	// Used to call set call precedent
	void confluence_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), mbs);
	}

	// Output function
	[[nodiscard]] typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;

		if (state.current_state == States::INPUT) {
			//If the lock is free and there are messages, send the messages.
			unique_lock<mutex> mutexLock(input_mutex, defer_lock);
			if (state.has_messages && mutexLock.try_lock()) {
				if (message_start_supervisor.size() > 0) {
					get_messages<typename Supervisor_UDP_Input_defs::o_start_supervisor>(bags) = message_start_supervisor;
					message_start_supervisor.clear();
				}

				if (message_perception_status.size() > 0) {
					get_messages<typename Supervisor_UDP_Input_defs::o_perception_status>(bags) = message_perception_status;
					message_perception_status.clear();
				}
				
				if (message_waypoint.size() > 0) {
					get_messages<typename Supervisor_UDP_Input_defs::o_waypoint>(bags) = message_waypoint;
					message_waypoint.clear();
				}

				if (message_lp_recv.size() > 0) {
					get_messages<typename Supervisor_UDP_Input_defs::o_lp_recv>(bags) = message_lp_recv;
					message_lp_recv.clear();
				}
								
				if (message_plp_ach.size() > 0) {
					get_messages<typename Supervisor_UDP_Input_defs::o_plp_ach>(bags) = message_plp_ach;
					message_plp_ach.clear();
				}
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
			char* sender_address = new char[IPV4_ADDRESS_LENGTH_BYTES];
			int sender_port;
			int bytes_received = connection->receive(recv_buffer, MAX_SER_BUFFER_CHARS, sender_address, &sender_port);
			 
			bool error_occured = false;
			uint8_t sysid;
			uint8_t compid;
			uint8_t sigid;
			vector<byte>byte_cache;
			int offset = 0;
			int data_length = bytes_received - 3 * (int)sizeof(uint8_t);

            // Parse the bytes of the received system ID.
            byte_cache = vector<byte>(sizeof(sysid));
            for (int i = 0; i < sizeof(sysid); i++)
            {
                byte_cache[i] = (byte)recv_buffer[i + offset];
            }
            offset += sizeof(sysid);
            if (!std::memcpy(&sysid, byte_cache.data(), sizeof(sysid))) {
                string error_message = string("[Supervisor UDP Input] Error copying system ID from ") + string(sender_address) + string(":") + to_string(sender_port) + string("\n");
                cout << error_message;
                error_occured = true;
            }

			if (!error_occured) {
				// Parse the bytes of the received component ID.
				byte_cache = vector<byte>(sizeof(compid));
				for (int i = 0; i < sizeof(compid); i++)
				{
					byte_cache[i] = (byte)recv_buffer[i + offset];
				}
				offset += sizeof(compid);
				if (!std::memcpy(&compid, byte_cache.data(), sizeof(compid))) {
					string error_message = string("[Supervisor UDP Input] Error copying component ID from ") + string(sender_address) + string(":") + to_string(sender_port) + string("\n");
					cout << error_message;
					error_occured = true;
				}
			}

			if (!error_occured) {
				// Parse the bytes of the received signal ID.
				byte_cache = vector<byte>(sizeof(sigid));
				for (int i = 0; i < sizeof(sigid); i++)
				{
					byte_cache[i] = (byte)recv_buffer[i + offset];
				}
				offset += sizeof(sigid);
				if (!std::memcpy(&sigid, byte_cache.data(), sizeof(sigid))) {
					string error_message = string("[Supervisor UDP Input] Error copying signal ID from ") + string(sender_address) + string(":") + to_string(sender_port) + string("\n");
					cout << error_message;
					error_occured = true;
				}
			}

			if (!error_occured) {
				// Parse the bytes of the payload.
				byte_cache = vector<byte>(data_length);
				for (int i = 0; i < data_length; i++)
				{
					byte_cache[i] = (byte)recv_buffer[i + offset];
				}

				message_start_supervisor_t temp_start_supervisor;
				bool temp_bool;
				message_landing_point_t temp_landing_point;
				message_fcc_command_t temp_fcc_command_waypoint;
				unique_lock<mutex> mutexLock(input_mutex);

                // std::cout << "SYS: " << sysid << "\tCOMP: " << compid << "\tSIG: " << sigid << std::endl;
                if (sigid == SUPERVISOR_SIG_ID_PLP_ACHIEVED && compid == COMP_ID_MISSION_MANAGER) {
                    std::memcpy(&temp_landing_point, byte_cache.data(), sizeof(temp_landing_point));
                    message_plp_ach.push_back(temp_landing_point);
                    state.has_messages = true;
                }
                if (sigid == SUPERVISOR_SIG_ID_WAYPOINT && compid == COMP_ID_MISSION_MANAGER) {
                    std::memcpy(&temp_fcc_command_waypoint, byte_cache.data(), sizeof(temp_fcc_command_waypoint));
                    message_waypoint.push_back(temp_fcc_command_waypoint);
                    state.has_messages = true;
                }
                if (sigid ==  SUPERVISOR_SIG_ID_START_SUPERVISOR && compid == COMP_ID_MISSION_MANAGER) {
                    std::memcpy(&temp_start_supervisor, byte_cache.data(), sizeof(temp_start_supervisor));
                    message_start_supervisor.push_back(temp_start_supervisor);
                    state.has_messages = true;
                }
                if (sigid == SUPERVISOR_SIG_ID_PERCEPTION_STATUS && compid == COMP_ID_PERCEPTION_SYSTEM) {
                    std::memcpy(&temp_bool, byte_cache.data(), sizeof(temp_bool));
                    message_perception_status.push_back(temp_bool);
                    state.has_messages = true;
                }
                if (sigid == SUPERVISOR_SIG_ID_LP_RECEIVE && compid == COMP_ID_PERCEPTION_SYSTEM) {
                    std::memcpy(&temp_landing_point, byte_cache.data(), sizeof(temp_landing_point));
                    message_lp_recv.push_back(temp_landing_point);
                    state.has_messages = true;
                }
			}
		}
	}

	friend ostringstream& operator<<(ostringstream& os, const typename Supervisor_UDP_Input<TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state) << "-" << (i.has_messages ? "MESSAGES" : "NO_MESSAGES");
		return os;
	}


    // Private members for thread management.
private:
    // Mutex for thread synchronization using unique locks.
    mutable mutex input_mutex;
    mutable vector<message_start_supervisor_t> message_start_supervisor;
    mutable vector<bool> message_perception_status;
    mutable vector<message_fcc_command_t> message_waypoint;
    mutable vector<message_landing_point_t> message_lp_recv;
    mutable vector<message_landing_point_t> message_plp_ach;

    // Networking members
    rudp::Connection * connection;
    int connection_number;
    char recv_buffer[MAX_SER_BUFFER_CHARS]{};

    // Const Members
    TIME polling_rate;

    // Global member for thread sync
    bool stop;
};

#endif /* RT_LINUX */
#endif /* SUPERVISOR_UDP_INPUT_HPP */
