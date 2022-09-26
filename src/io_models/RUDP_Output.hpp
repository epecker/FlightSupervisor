/**
 * 	\file		RUDP_Output.hpp
 *	\brief		Definition of the RUDP Output atomic model.
 *	\details	This header file defines the RUDP Output atomic model for use in the Cadmium DEVS
				simulation software. RUDP Output is an atomic model for sending packets using
				RUDP (Reliable-UDP) to an address and port.
 *	\image		html io_models/udp_output.png
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef RUDP_OUTPUT_HPP
#define RUDP_OUTPUT_HPP

// Utility functions
#include "../enum_string_conversion.hpp"
#include "../Constants.hpp"

// Cadmium Simulator Headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

// RUDP Library
#include <RUDP/src/ConnectionController.hpp>

/**
 * 	\class		RUDP_Output
 *	\brief		Definition of the RUDP Output atomic model.
 *	\details	This class defines the RUDP Output atomic model for use in the Cadmium DEVS
				simulation software. RUDP Output is an atomic model for sending packets using
				RUDP (Reliable-UDP) to an address and port.
 *	\image		html io_models/udp_output.png
 */
template<typename TIME>
class RUDP_Output {
public:
	/**
	 *	\par	States
	 * 	Declaration of the states of the atomic model.
	 */
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(SENDING)
	);

	/**
	 *	\brief	For definition of the input and output ports see:
	 *	\ref 	RUDP_Output_input_ports "Input Ports" and
	 *	\ref 	RUDP_Output_output_ports "Output Ports"
	 * 	\note 	All input and output ports must be listed in this struct.
	 */
	struct defs{
	    struct i_message : public cadmium::in_port<std::vector<char>> { };
	};

	/**
	 * 	\anchor	RUDP_Output_input_ports
	 *	\par	Input Ports
	 * 	Definition of the input ports for the model.
	 * 	\param	i_message	Port for receiving byte vectors to send to a predefined address and port.
	 */
    using input_ports=std::tuple<typename defs::i_message>;

	/**
	 *	\anchor	RUDP_Output_output_ports
	 * 	\par 	Output Ports
	 * 	Definition of the output ports for the model.
	 */
    using output_ports=std::tuple<>;

	/**
	 *	\anchor	RUDP_Output_state_type
	 *	\par	State
	 * 	Definition of the states of the atomic model.
	 * 	\param 	current_state 	Current state of atomic model.
	 * 	\param	messages		Queue of byte vectors to send to the predefined address and port.
	 */
    struct state_type{
        States current_state;
		std::vector<std::vector<char>> messages;
    };
    state_type state;

	/**
	 * \brief 	Default constructor for the model.
	 */
    RUDP_Output() {
        state.current_state = States::IDLE;
        int connection_number = rudp::ConnectionController::addConnection(DEFAULT_TIMEOUT_MS);
		connection = rudp::ConnectionController::getConnection(connection_number);
		try {
			connection->setEndpointRemote(PEREGRINE_IP, MAVLINK_OVER_UDP_PORT);
			connection->setSendRetriesLimit(10);
		}
		catch (std::runtime_error& error) {
			std::cout << error.what();
			assert(false);
		}
    }

	/**
	 * \brief 	Constructor for the model with destination of the packets and RUDP configuration.
	 * \param	address			String IP version 4 address of the receiver.
	 * \param	port			unsigned short port number of the receiver.
	 * \param	timeout_ms		int timeout in milliseconds after which the packet will be retransmitted.
	 * \param	retries_limit	int maximum number of retries to send the packet, if no acknowledgement is received.
	 */
    RUDP_Output(const std::string& address, unsigned short port, int timeout_ms, int retries_limit) {
        state.current_state = States::IDLE;
        int connection_number = rudp::ConnectionController::addConnection(timeout_ms);
		connection = rudp::ConnectionController::getConnection(connection_number);
		try {
			connection->setEndpointRemote(address, port);
			connection->setSendRetriesLimit(retries_limit);
		}
		catch (std::runtime_error& error) {
			std::cout << error.what();
			assert(false);
		}
    }

	/// Internal transitions of the model
    void internal_transition() {
        if (state.current_state == States::SENDING) {
            state.current_state = States::IDLE;
			state.messages.clear();
        }
    }

	/// External transitions of the model
    void external_transition(TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		if (cadmium::get_messages<typename defs::i_message>(mbs).size() >= 1){
			state.current_state = States::SENDING;
			for (std::vector<char> m : cadmium::get_messages<typename defs::i_message>(mbs)) {
				state.messages.push_back(m);
			}
		}
    }

	/// Function used to decide precedence between internal and external transitions when both are scheduled simultaneously.
    void confluence_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
        internal_transition();
        external_transition(TIME(), std::move(mbs));
    }

	/// Function for generating output from the model before internal transitions.
    [[nodiscard]] typename cadmium::make_message_bags<output_ports>::type output() const {
		typename cadmium::make_message_bags<output_ports>::type bags;

        switch(state.current_state) {
            case States::SENDING:
                send_packets();
		        break;
            default:
                break;
        }
        return bags;
    }

	/// Function to declare the time advance value for each state of the model.
    TIME time_advance() const {
        switch (state.current_state) {
            case States::IDLE:
                return std::numeric_limits<TIME>::infinity();
            case States::SENDING:
                return TIME(TA_ZERO);
            default:
                assert(false && "Unhandled time advance in RUDP_Output.hpp");
        }
    }

	/**
	 *  \brief 		Operator for defining how the model state will be represented as a string.
	 * 	\warning 	Prepended "State: " is required for log parsing, do not remove.
	 */
    friend std::ostringstream& operator<<(std::ostringstream& os, const typename RUDP_Output::state_type& i) {
        os << "State: " << enumToString(i.current_state) + std::string("\n");
        return os;
    }

protected:
	/// Variable for storing a reference to the RUDP connection that the packets will be sent via.
	rudp::Connection *connection;

private:
    /// Function send_packets is used to send all the packets in the message queue to the destination via the RUDP connection.
    void send_packets() const {
        for (std::vector<char> m : state.messages) {
			try {
            	connection->send(m.data(), m.size());
			}
            catch (std::runtime_error& error) {
                std::cout << "[RUDP Output] (ERROR) Error sending packet using RUDP Output model: " << error.what() << std::endl;
            }
        }
    }
};

#endif /* RUDP_OUTPUT_HPP */
