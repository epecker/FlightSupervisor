/**
 * 	\file		Supervisor_UDP_Input.hpp
 *	\brief		Definition of the Supervisor UDP Input atomic model.
 *	\details	This header file defines the Supervisor UDP Input atomic model for use in the Cadmium DEVS
				simulation software. Supervisor UDP Input is an atomic model for receiving UDP packets via RUDP and
				forwarding them as Cadmium events into the Supervisor.
 *	\image		html io_models/supervisor_udp_input.png
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef SUPERVISOR_UDP_INPUT_HPP
#define SUPERVISOR_UDP_INPUT_HPP

// Messages structures
#include "../message_structures/message_landing_point_t.hpp"
#include "../message_structures/message_fcc_command_t.hpp"
#include "../message_structures/message_start_supervisor_t.hpp"

// Utility functions
#include "../enum_string_conversion.hpp"
#include "../Constants.hpp"
#include "../component_macros.h"

// RUDP Library
#include <RUDP/src/ConnectionController.hpp>

// Cadmium Simulator Headers
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <cadmium/modeling/dynamic_model.hpp>

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

// Guard to ensure that the model is run in real-time.
#ifdef RT_LINUX

/**
 * 	\class		Supervisor_UDP_Input
 *	\brief		Definition of the Supervisor UDP Input atomic model.
 *	\details	This class defines the Supervisor UDP Input atomic model for use in the Cadmium DEVS
				simulation software. RUDP Output is an atomic model for receiving UDP packets via RUDP and
				forwarding them as Cadmium events into the Supervisor. See the
				\ref Supervisor_UDP_Input_child_thread "child thread" implementation for more details.
 *	\image		html io_models/supervisor_udp_input.png
 */
template<typename TIME>
class Supervisor_UDP_Input {
public:
	/**
	 *	\par	States
	 * 	Declaration of the states of the atomic model.
	 */
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(INPUT)
	);

	/**
	 *	\brief	For definition of the input and output ports see:
	 *	\ref 	Supervisor_UDP_Input_input_ports "Input Ports" and
	 *	\ref 	Supervisor_UDP_Input_output_ports "Output Ports"
	 * 	\note 	All input and output ports must be listed in this struct.
	 */
	struct defs {
		struct o_start_supervisor 	: public cadmium::out_port<message_start_supervisor_t> { };
		struct o_perception_status	: public cadmium::out_port<bool> { };
		struct o_waypoint 			: public cadmium::out_port<message_fcc_command_t> { };
		struct o_lp_recv			: public cadmium::out_port<message_landing_point_t> { };
		struct o_plp_ach 			: public cadmium::out_port<message_landing_point_t> { };
		struct i_quit 				: public cadmium::in_port<bool> { };
	};

	/**
	 * 	\anchor	Supervisor_UDP_Input_input_ports
	 *	\par	Input Ports
	 * 	Defintion of the input ports for the model.
	 *	\param	i_quit	Port for receiving signal indicating that the model should stop listening for packets and passivate.
	 */
	using input_ports = std::tuple<typename defs::i_quit>;

	/**
	 *	\anchor	Supervisor_UDP_Input_output_ports
	 * 	\par 	Output Ports
	 * 	Defintion of the output ports for the model.
	 * 	\param 	o_start_supervisor	Port to forward any start supervisor messages to the Supervisor.
	 * 	\param 	o_perception_status	Port to forward any perception status messages to the Supervisor.
	 * 	\param 	o_waypoint			Port to forward any waypoint messages to the Supervisor.
	 * 	\param 	o_lp_recv			Port to forward any landing point messages to the Supervisor.
	 * 	\param 	o_plp_ach			Port to forward any planned landing point messages to the Supervisor.
	 */
	using output_ports = std::tuple<
		typename defs::o_start_supervisor,
		typename defs::o_perception_status,
		typename defs::o_waypoint,
		typename defs::o_lp_recv,
		typename defs::o_plp_ach
	>;

	/**
	 *	\anchor	Supervisor_UDP_Input_state_type
	 *	\par	State
	 * 	Defintion of the states of the atomic model.
	 * 	\param 	current_state 	Current state of atomic model.
	 * 	\param	has_messages	State variable indicating if any packets have been received since the last poll.
	 */
	struct state_type {
		States current_state;
		bool has_messages;
	} state;

	/**
	 * \brief 	Default constructor for the model.
	 */
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
		std::thread(&Supervisor_UDP_Input::receive_packet_thread, this).detach();
	}

	/**
	 * \brief 	Constructor for the model with port that model should listen on and rate at which message queue should be polled.
	 * \param	rate	TIME rate at which the message queue should be polled.
	 * \param	port	unsigned short port number that the model should listen on.
	 */
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
		std::thread(&Supervisor_UDP_Input::receive_packet_thread, this).detach();
	}

	/**
	 * \brief 	Destructor for the model.
	 */
	~Supervisor_UDP_Input() {
		stop = true;
		rudp::ConnectionController::removeConnection(connection_number);
	}

	/// Internal transitions of the model
	void internal_transition() {
		if (state.current_state == States::INPUT) {
			std::unique_lock<std::mutex> mutexLock(input_mutex, std::defer_lock);
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

	/// External transitions of the model
	void external_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
        bool received_quit = !cadmium::get_messages<typename defs::i_quit>(mbs).empty();
		if (received_quit) {
			state.current_state = States::IDLE;
		}
	}

	/// Function used to decide precedence between internal and external transitions when both are scheduled simultaneously.
	void confluence_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), mbs);
	}

	/// Function for generating output from the model before internal transitions.
	[[nodiscard]] typename cadmium::make_message_bags<output_ports>::type output() const {
		typename cadmium::make_message_bags<output_ports>::type bags;

		if (state.current_state == States::INPUT) {
			//If the lock is free and there are messages, send the messages.
			std::unique_lock<std::mutex> mutexLock(input_mutex, std::defer_lock);
			if (state.has_messages && mutexLock.try_lock()) {
				if (message_start_supervisor.size() > 0) {
				 cadmium::get_messages<typename defs::o_start_supervisor>(bags) = message_start_supervisor;
					message_start_supervisor.clear();
				}

				if (message_perception_status.size() > 0) {
				 cadmium::get_messages<typename defs::o_perception_status>(bags) = message_perception_status;
					message_perception_status.clear();
				}

				if (message_waypoint.size() > 0) {
				 cadmium::get_messages<typename defs::o_waypoint>(bags) = message_waypoint;
					message_waypoint.clear();
				}

				if (message_lp_recv.size() > 0) {
				 cadmium::get_messages<typename defs::o_lp_recv>(bags) = message_lp_recv;
					message_lp_recv.clear();
				}

				if (message_plp_ach.size() > 0) {
				 cadmium::get_messages<typename defs::o_plp_ach>(bags) = message_plp_ach;
					message_plp_ach.clear();
				}
			}
		}
		return bags;
	}

	/// Function to declare the time advance value for each state of the model.
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

	/**
	 *  \brief 		Operator for defining how the model state will be represented as a string.
	 * 	\warning 	Prepended "State: " is required for log parsing, do not remove.
	 */
	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Supervisor_UDP_Input<TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state) << "-" << (i.has_messages ? "MESSAGES" : "NO_MESSAGES");
		return os;
	}

private:
    /// Variable for the mutex for thread synchronization using unique locks.
    mutable std::mutex input_mutex;
    /// Queues to store the start supervisor packets that have been received by the child thread until they can be forwarded.
    mutable std::vector<message_start_supervisor_t> message_start_supervisor;
	/// Queues to store the perception_status packets that have been received by the child thread until they can be forwarded.
    mutable std::vector<bool> message_perception_status;
	/// Queues to store the waypoint packets that have been received by the child thread until they can be forwarded.
    mutable std::vector<message_fcc_command_t> message_waypoint;
	/// Queues to store the landing point packets that have been received by the child thread until they can be forwarded.
    mutable std::vector<message_landing_point_t> message_lp_recv;
	/// Queues to store the planned landing point packets that have been received by the child thread until they can be forwarded.
    mutable std::vector<message_landing_point_t> message_plp_ach;

    /// Variable to store a pointer to the RUDP connection that will be used to receive the packets.
    rudp::Connection * connection;
	/// Variable to store the number of the RUDP connection that will be used to receive the packets.
    int connection_number;
    /// Buffer to hold the bytes received by RUDP until they can be parsed.
    char recv_buffer[MAX_SER_BUFFER_CHARS]{};

    /// Variable for rate at which the message queues should be polled by the model.
    TIME polling_rate;

    /// Variable for thread synchronization.
    bool stop;

	/**
	 * 	\anchor		Supervisor_UDP_Input_child_thread
	 *	\brief		Function receive_packet_thread is used as a child thread for receiving UDP packets via RUDP.
	 * 	\details	The thread is started in the model constructor and constantly receives packets via RUDP until
	 * 				the model is passivated. The packets received by the thread are expected to follow a predefined
	 * 				protocol in order to be parsed and forwarded onto the Supervisor.
	 *	\par		Interface Details
	 * 				The model only accepts RUDP packets sent with a Mavlink-like structure:
	 * 				[uint8_t System ID][uint8_t Component ID][uint8_t Signal ID][Payload]
	 *				The combination of the System, Component, and Signal IDs are used to infer the datatype of
	 *				the payload and direct the cast payload to the correct port of the Supervisor. The Component and
	 * 				Signal IDs can be found in the component_macros header file. The to direct a packet to a port
	 * 				configure the IDs as follows:
	 *	\param		start_supervisor	System ID: Helicopter (1),	Component ID: Mission Manager 	(3),	Signal ID: start_supervisor 	(1),	Payload: message_start_supervisor_t
	 *	\param 		perception_status	System ID: Helicopter (1),	Component ID: Perception System	(2),	Signal ID: perception_status	(2),	Payload: bool
	 *	\param 		waypoint			System ID: Helicopter (1),	Component ID: Mission Manager 	(3),	Signal ID: waypoint 			(3),	Payload: message_fcc_command_t
	 *	\param 		lp_recv				System ID: Helicopter (1),	Component ID: Perception System	(2),	Signal ID: lp_recv 				(4),	Payload: message_landing_point_t
	 *	\param 		plp_ach				System ID: Helicopter (1),	Component ID: Mission Manager	(3),	Signal ID: plp_ach 				(5),	Payload: message_landing_point_t
	 */
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
			std::vector<std::byte>byte_cache;
			int offset = 0;
			int data_length = bytes_received - 3 * (int)sizeof(uint8_t);

            // Parse the bytes of the received system ID.
            byte_cache = std::vector<std::byte>(sizeof(sysid));
            for (int i = 0; i < sizeof(sysid); i++)
            {
                byte_cache[i] = (std::byte)recv_buffer[i + offset];
            }
            offset += sizeof(sysid);
            if (!std::memcpy(&sysid, byte_cache.data(), sizeof(sysid))) {
                std::string error_message = std::string("[Supervisor UDP Input] Error copying system ID from ") + std::string(sender_address) + std::string(":") + std::to_string(sender_port) + std::string("\n");
				std::cout << error_message;
                error_occured = true;
            }

			if (!error_occured) {
				// Parse the bytes of the received component ID.
				byte_cache = std::vector<std::byte>(sizeof(compid));
				for (int i = 0; i < sizeof(compid); i++)
				{
					byte_cache[i] = (std::byte)recv_buffer[i + offset];
				}
				offset += sizeof(compid);
				if (!std::memcpy(&compid, byte_cache.data(), sizeof(compid))) {
					std::string error_message = std::string("[Supervisor UDP Input] Error copying component ID from ") + std::string(sender_address) + std::string(":") + std::to_string(sender_port) + std::string("\n");
					std::cout << error_message;
					error_occured = true;
				}
			}

			if (!error_occured) {
				// Parse the bytes of the received signal ID.
				byte_cache = std::vector<std::byte>(sizeof(sigid));
				for (int i = 0; i < sizeof(sigid); i++)
				{
					byte_cache[i] = (std::byte)recv_buffer[i + offset];
				}
				offset += sizeof(sigid);
				if (!std::memcpy(&sigid, byte_cache.data(), sizeof(sigid))) {
					std::string error_message = std::string("[Supervisor UDP Input] Error copying signal ID from ") + std::string(sender_address) + std::string(":") + std::to_string(sender_port) + std::string("\n");
					std::cout << error_message;
					error_occured = true;
				}
			}

			if (!error_occured) {
				// Parse the bytes of the payload.
				byte_cache = std::vector<std::byte>(data_length);
				for (int i = 0; i < data_length; i++)
				{
					byte_cache[i] = (std::byte)recv_buffer[i + offset];
				}

				message_start_supervisor_t temp_start_supervisor;
				bool temp_bool;
				message_landing_point_t temp_landing_point;
				message_fcc_command_t temp_fcc_command_waypoint;
				std::unique_lock<std::mutex> mutexLock(input_mutex);

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
};

#endif /* RT_LINUX */
#endif /* SUPERVISOR_UDP_INPUT_HPP */
