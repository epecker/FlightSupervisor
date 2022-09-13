/**
 * 	\file		UDP_Input_Async.hpp
 *	\brief		Definition of the UDP Input Asynchronous atomic model.
 *	\details	This header file defines the UDP Input Asynchronous atomic model for use in the Cadmium DEVS
				simulation software. UDP Input Asynchronous is an atomic model for receiving UDP packets and
				forwarding them as Cadmium events.
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef UDP_INPUT_ASYNC_HPP
#define UDP_INPUT_ASYNC_HPP

// Message structures
#include "../message_structures/message_command_ack_t.hpp"

// Utility functions
#include "../enum_string_conversion.hpp"
#include "../Constants.hpp"

// Cadmium Simulator Headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

// Boost Libraries
#include <boost/asio.hpp>

// System Libraries
#include <thread>
#include <mutex>
#include <string>

// Guard to ensure that the model is run in real-time.
#ifdef RT_LINUX

/**
 * 	\class		UDP_Input_Async
 *	\brief		Definition of the UDP Input Asynchronous atomic model.
 *	\details	This class defines the UDP Input Asynchronous atomic model for use in the Cadmium DEVS
				simulation software. UDP Input Asynchronous is an atomic model for receiving UDP packets and
				forwarding them as Cadmium events.
 *	\tparam		MSG Template parameter for the message type.
 */
template<typename MSG, typename TIME>
class UDP_Input_Async {
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
	 *	\ref 	UDP_Input_Async_input_ports "Input Ports" and
	 *	\ref 	UDP_Input_Async_output_ports "Output Ports"
	 * 	\note 	All input and output ports must be listed in this struct.
	 */
	struct defs {
		struct o_message : public cadmium::out_port<MSG> { };
		struct i_quit : public cadmium::in_port<bool> { };
	};

	/**
	 * 	\anchor	UDP_Input_Async_input_ports
	 *	\par	Input Ports
	 * 	Definition of the input ports for the model.
	 *	\param	i_quit	Port for receiving signal indicating that the model should stop listening for packets and passivate.
	 */
	using input_ports = std::tuple<typename defs::i_quit>;

	/**
	 *	\anchor	UDP_Input_Async_output_ports
	 * 	\par 	Output Ports
	 * 	Definition of the output ports for the model.
	 * 	\param	o_message	Port to forward any packets received.
	 */
	using output_ports = std::tuple<typename defs::o_message>;

	/**
	 *	\anchor	UDP_Input_Async_state_type
	 *	\par	State
	 * 	Definition of the states of the atomic model.
	 * 	\param 	current_state 	Current state of atomic model.
	 * 	\param	has_messages	State variable indicating if any packets have been received since the last poll.
	 * 	\param	message			Queue of messages that have been received by the child thread.
	 */
	struct state_type {
		States current_state;
		bool has_messages;
		mutable std::vector<MSG> message;
	};
	state_type state;

	/**
	 * \brief 	Default constructor for the model.
	 */
	UDP_Input_Async() {
		//Initialise the current state
		state.current_state = States::INPUT;
		state.has_messages = !state.message.empty();

		send_ack = false;
		stop = false;

		//Create the network endpoint using a default address and port.
		unsigned short port_num = MAVLINK_OVER_UDP_PORT;
		network_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), port_num);

		// Set the interrupts for the program to stop the child thread.
		// std::signal(SIGINT, UDP_Input_Async::handle_signal);
		// std::signal(SIGTERM, UDP_Input_Async::handle_signal);

		//Start the user input thread.
		std::thread(&UDP_Input_Async::receive_packet_thread, this).detach();
	}

	/**
	 * 	\brief 	Constructor for the model with port that model should listen on and whether an acknowledgement should be returned back.
	 * 	\param	ack_required	bool true for if an acknowledgement should be sent back to the sender on packet receipt.
	 * 	\param	port			unsigned short port number that the model should listen on.
	 */
	UDP_Input_Async(cadmium::dynamic::modeling::AsyncEventSubject* sub, bool ack_required, const std::string& port) {
		//Initialise the current state
		state.current_state = States::INPUT;
		state.has_messages = !state.message.empty();
		_sub = sub;

		send_ack = ack_required;
		stop = false;

		//Create the network endpoint using the supplied address and port.
		unsigned short port_num = strtoul(port.c_str(), nullptr, 0);
		network_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), port_num);

		// Set the interrupts for the program to stop the child thread.
		// std::signal(SIGINT, UDP_Input_Async::handle_signal);
		// std::signal(SIGTERM, UDP_Input_Async::handle_signal);

		//Start the user input thread.
		std::thread(&UDP_Input_Async::receive_packet_thread, this).detach();
	}

	/**
	 * \brief 	Destructor for the model.
	 */
	~UDP_Input_Async() {
		shutdown();
	}

	/// Handler for signals.
	void handle_signal(int signal_number) {
		shutdown();
	}

	/// Member for shutting down the thread and IO services gracefully.
	void shutdown() {
		//Before exiting stop the Boost IO service to interrupt the receipt handler.
		stop = true;
		io_service.stop();
		if (socket.is_open())
			socket.close();
	}

	/// Internal transitions of the model
	void internal_transition() {
		std::unique_lock<std::mutex> mutexLock(input_mutex, std::defer_lock);
		//If the thread has finished receiving input, change state if there are messages.
		if (mutexLock.try_lock()) {
			state.has_messages = !state.message.empty();
		}
	}

	/// External transitions of the model
	void external_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		std::unique_lock<std::mutex> mutexLock(input_mutex, std::defer_lock);
		//If the thread has finished receiving input, change state if there are messages.
		if (mutexLock.try_lock()) {
			state.has_messages = !state.message.empty();
		}

		if (cadmium::get_messages<typename defs::i_quit>(mbs).size() >= 1) {
			state.current_state = States::IDLE;
		}
	}

	/// Function used to decide precedence between internal and external transitions when both are scheduled simultaneously.
	void confluence_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), std::move(mbs));
	}

	/// Function for generating output from the model before internal transitions.
	typename cadmium::make_message_bags<output_ports>::type output() const {
		typename cadmium::make_message_bags<output_ports>::type bags;
		std::vector<MSG> message_out;
		std::unique_lock<std::mutex> mutexLock(input_mutex, std::defer_lock);
		if (state.current_state == States::INPUT) {
			//If the lock is free and there are messages, send the messages.
			if (!state.message.empty() && mutexLock.try_lock()) {
				for (auto msg : state.message) {
					message_out.push_back(msg);
				}
				state.message.clear();
			 cadmium::get_messages<typename defs::o_message>(bags) = message_out;
			}
		}
		return bags;
	}

	/// Function to declare the time advance value for each state of the model.
	TIME time_advance() const {
		if (!state.message.empty() && state.current_state == States::INPUT) {
			return TIME(TA_ZERO);
		}

		return std::numeric_limits<TIME>::infinity();
	}

	/**
	 *  \brief 		Operator for defining how the model state will be represented as a string.
	 * 	\warning 	Prepended "State: " is required for log parsing, do not remove.
	 */
	friend std::ostringstream& operator<<(std::ostringstream& os, const typename UDP_Input_Async<MSG, TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state) << "-" << (i.has_messages ? "MESSAGES" : "NO_MESSAGES");
		return os;
	}

private:
	/// Variable to store a pointer to the asynchronous event subject used by the simulator for asynchronous interrupts.
	cadmium::dynamic::modeling::AsyncEventSubject* _sub{};
    /// Variable for the mutex for thread synchronization using unique locks.
	mutable std::mutex input_mutex;
	/// Stores multiple messages
	mutable std::vector<MSG> message;
	/// Variable to store the endpoint that the model for listen for packets on.
	boost::asio::ip::udp::endpoint network_endpoint;
	/// Variable to store the origin endpoint of the packet that was just received.
	boost::asio::ip::udp::endpoint remote_endpoint;
	/// Variable to store the Boost IO service.
	boost::asio::io_service io_service;
	/// Variable to store the socket that the child thread will listen on.
	boost::asio::ip::udp::socket socket{ io_service };
	/// Buffer to hold the bytes received by RUDP until they can be parsed.
	char recv_buffer[MAX_SER_BUFFER_CHARS]{};

	/// Variable to indicate if an acknowledgement should be sent back to the sender on packet receipt.
	bool send_ack;

    /// Variable for thread synchronization.
	bool stop;

	/**
	 * 	\anchor		UDP_Input_Async_child_thread
	 *	\brief		Function receive_packet_thread is used as a child thread for receiving UDP packets.
	 *	\details	The thread is started in the model constructor and constantly receives packets until
	 * 				the model is passivated. The packets are received then added to a queue to be sent
	 * 				as Cadmium events.
	 */
	void receive_packet_thread() {
		//Open and bind the socket using Boost.
		socket.open(boost::asio::ip::udp::v4());
		socket.bind(network_endpoint);

		//While the model is not passivated,
		while (state.current_state != States::IDLE && !stop) {
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

	/// Handler that is called on UDP packet receipt by Boost.
	void receive_packet(const boost::system::error_code& error, size_t bytes_transferred) {
		// Acquire the unique lock for the state.message vector.
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
		_sub->notify();
	}
};

#endif // RT_LINUX
#endif // UDP_INPUT_ASYNC_HPP
