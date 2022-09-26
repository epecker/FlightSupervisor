/**
 * 	\file		UDP_Input.hpp
 *	\brief		Definition of the UDP Input atomic model.
 *	\details	This header file defines the UDP Input atomic model for use in the Cadmium DEVS
				simulation software. UDP Input is an atomic model for receiving UDP packets and
				forwarding them as Cadmium events.
 *	\image		html io_models/udp_input.png
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef UDP_INPUT_HPP
#define UDP_INPUT_HPP

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
#include <csignal>

// Guard to ensure that the model is run in real-time.
#ifdef RT_LINUX

/**
 * 	\class		UDP_Input
 *	\brief		Definition of the UDP Input atomic model.
 *	\details	This class defines the UDP Input atomic model for use in the Cadmium DEVS
				simulation software. UDP Input is an atomic model for receiving UDP packets and
				forwarding them as Cadmium events.
 *	\image		html io_models/udp_input.png
 *	\tparam		MSG Template parameter for the message type.
 */
template<typename MSG, typename TIME>
class UDP_Input {
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
	 *	\ref 	UDP_Input_input_ports "Input Ports" and
	 *	\ref 	UDP_Input_output_ports "Output Ports"
	 * 	\note 	All input and output ports must be listed in this struct.
	 */
	struct defs {
		struct o_message : public cadmium::out_port<MSG> { };
		struct i_quit : public cadmium::in_port<bool> { };
	};

	/**
	 * 	\anchor	UDP_Input_input_ports
	 *	\par	Input Ports
	 * 	Definition of the input ports for the model.
	 *	\param	i_quit	Port for receiving signal indicating that the model should stop listening for packets and passivate.
	 */
	using input_ports = std::tuple<typename defs::i_quit>;

	/**
	 *	\anchor	UDP_Input_output_ports
	 * 	\par 	Output Ports
	 * 	Definition of the output ports for the model.
	 * 	\param	o_message	Port to forward any packets received.
	 */
	using output_ports = std::tuple<typename defs::o_message>;

	/**
	 *	\anchor	UDP_Input_state_type
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
	UDP_Input() {
		//Initialise the current state
		state.current_state = States::INPUT;
		state.has_messages = false;

		polling_rate = TIME("00:00:00:100");
		send_ack = false;
		stop = false;

		//Create the network endpoint using a default address and port.
		unsigned short port_num = MAVLINK_OVER_UDP_PORT;
		endpoint_local = boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), port_num);

		//Set the interrupts for the program to stop the child thread.
		// std::signal(SIGINT, UDP_Input::handle_signal);
		// std::signal(SIGTERM, UDP_Input::handle_signal);

		//Start the user input thread.
		std::thread(&UDP_Input::receive_packet_thread, this).detach();
	}

	/**
	 * 	\brief 	Constructor for the model with port that model should listen on and whether an acknowledgement should be returned back.
	 * \param	rate			TIME rate at which the message queue should be polled.
	 * 	\param	ack_required	bool true for if an acknowledgement should be sent back to the sender on packet receipt.
	 * 	\param	port			unsigned short port number that the model should listen on.
	 */
	UDP_Input(TIME rate, bool ack_required, const std::string& port) {
		//Initialise the current state
		state.current_state = States::INPUT;
		state.has_messages = false;

		polling_rate = rate;
		send_ack = ack_required;
		stop = false;

		//Create the network endpoint using the supplied address and port.
		auto port_num = (unsigned short)strtoul(port.c_str(), nullptr, 0);
		endpoint_local = boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), port_num);

		//Set the interrupts for the program to stop the child thread.
		// std::signal(SIGINT, UDP_Input::handle_signal);
		// std::signal(SIGTERM, UDP_Input::handle_signal);

		//Start the user input thread.
		std::thread(&UDP_Input::receive_packet_thread, this).detach();
	}

	/**
	 * \brief 	Destructor for the model.
	 */
	~UDP_Input() {
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
		if (state.current_state == States::INPUT) {
			std::unique_lock<std::mutex> mutexLock(input_mutex, std::defer_lock);
			//If the thread has finished receiving input, change state if there are messages.
			if (mutexLock.try_lock()) {
				state.has_messages = !message.empty();
			}
		}
	}

	/// External transitions of the model
	void external_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
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
		switch (state.current_state) {
			case States::INPUT:
				std::unique_lock<std::mutex> mutexLock(input_mutex, std::defer_lock);
				if (state.has_messages && mutexLock.try_lock()) {
					for (auto msg: state.message) {
						cadmium::get_messages<typename defs::o_message>(bags).push_back(msg);
					}
					state.message.clear();
				}
				break;
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
	friend std::ostringstream& operator<<(std::ostringstream& os, const typename UDP_Input<MSG, TIME>::state_type& i) {
		os << "State: " << enumToString(i.current_state) << "-" << (i.has_messages ? "MESSAGES" : "NO_MESSAGES");
		return os;
	}

private:
    /// Variable for the mutex for thread synchronization using unique locks.
	mutable std::mutex input_mutex;
	/// Stores multiple messages
	mutable std::vector<MSG> message;
	/// Variable to store the endpoint that the model for listen for packets on.
	boost::asio::ip::udp::endpoint endpoint_local;
	/// Variable to store the origin endpoint of the packet that was just received.
	boost::asio::ip::udp::endpoint endpoint_remote;
	/// Variable to store the Boost IO service.
	boost::asio::io_service io_service;
	/// Variable to store the socket that the child thread will listen on.
	boost::asio::ip::udp::socket socket{ io_service };
	/// Buffer to hold the bytes received by RUDP until they can be parsed.
	char recv_buffer[MAX_SER_BUFFER_CHARS]{};

	/// Variable to indicate if an acknowledgement should be sent back to the sender on packet receipt.
	bool send_ack;
    /// Variable for rate at which the message queues should be polled by the model.
	TIME polling_rate;

    /// Variable for thread synchronization.
	bool stop;

	/**
	 * 	\anchor		UDP_Input_child_thread
	 *	\brief		Function receive_packet_thread is used as a child thread for receiving UDP packets.
	 *	\details	The thread is started in the model constructor and constantly receives packets until
	 * 				the model is passivated. The packets are received then added to a queue to be sent
	 * 				as Cadmium events.
	 */
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

	/// Handler that is called on UDP packet receipt by Boost.
	void receive_packet(const boost::system::error_code& error, size_t bytes_transferred) {
		// Acquire the unique lock for the message vector.
		std::unique_lock<std::mutex> mutexLock(input_mutex);
		if (error) return;

		//Add the message to the vector.
		MSG recv = MSG();
		memcpy(&recv, &recv_buffer, bytes_transferred);
		state.message.insert(message.begin(), recv);

		// If an ack is required,
		if (send_ack) {
			// Construct the ack message and associated data array.
			message_command_ack_t ack_message(MAV_CMD_DEFAULT, MAV_RESULT_ACCEPTED, 0, 0, 0, 0);
			boost::system::error_code ack_err;
			char ack_data[sizeof(message_command_ack_t)];
			memcpy(ack_data, &ack_message, sizeof(ack_data));

			// Send the ack to the origin of the packet.
			socket.send_to(boost::asio::buffer(ack_data), endpoint_remote, 0, ack_err);
		}
	}
};

#endif // RT_LINUX
#endif // UDP_INPUT_HPP
