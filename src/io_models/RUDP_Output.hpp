/**
 *	\brief		An atomic model for sending UDP packets over RUDP to an address and port.
 *	\details	This header file defines a output atomic model for use in the
                Cadmium DEVS simulation software.
 *	\author		James Horner
 *	\author		Tanner Trautrim
 */

#ifndef RUDP_OUTPUT_HPP
#define RUDP_OUTPUT_HPP

#include "RUDP/src/ConnectionController.hpp"

// RT-Cadmium
#include "cadmium/modeling/ports.hpp"
#include "cadmium/modeling/message_bag.hpp"

#include "../enum_string_conversion.hpp"
#include "../Constants.hpp"

// Atomic model
template<typename TIME>
class RUDP_Output {
protected:
	rudp::Connection *connection;

public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(SENDING)
	);

	// Input and output port definitions
	struct defs{
	    struct i_message : public cadmium::in_port<std::vector<char>> { };
	};

    // Default constructor
    RUDP_Output() {
        state.current_state = States::IDLE;
        int connection_number = rudp::ConnectionController::addConnection(DEFAULT_TIMEOUT_MS);
		connection = rudp::ConnectionController::getConnection(connection_number);
		try {
			connection->setEndpointRemote(PEREGRINE_IP, MAVLINK_OVER_UDP_PORT);
			connection->setSendRetriesLimit(10);
		}
		catch (std::runtime_error error) {
			assert(false && error.what());
		}
    }

    // Constructor with polling rate parameter
    RUDP_Output(const std::string& address, unsigned short port, int timeout_ms, int retries_limit) {
        state.current_state = States::IDLE;
        int connection_number = rudp::ConnectionController::addConnection(timeout_ms);
		connection = rudp::ConnectionController::getConnection(connection_number);
		try {
			connection->setEndpointRemote(address, port);
			connection->setSendRetriesLimit(retries_limit);
		}
		catch (std::runtime_error error) {
			assert(false && error.what());
		}
    }

	// This is used to track the state of the atomic model.
	// (required for the simulator)
    struct state_type{
        States current_state;
		std::vector<std::vector<char>> messages;
    };
    state_type state;

	// Create a tuple of input ports (required for the simulator)
    using input_ports=std::tuple<typename RUDP_Output::defs::i_message>;

    // Create a tuple of output ports (required for the simulator)
    using output_ports=std::tuple<>;

	// Internal transitions
	// These are transitions occurring from internal inputs
	// (required for the simulator)
    void internal_transition() {
        if (state.current_state == States::SENDING) {
            state.current_state = States::IDLE;
			state.messages.clear();
        }
    }

	// External transitions
	// These are transitions occurring from external inputs
	// (required for the simulator)
    void external_transition(TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		if (cadmium::get_messages<typename RUDP_Output::defs::i_message>(mbs).size() >= 1){
			state.current_state = States::SENDING;
			for (std::vector<char> m : cadmium::get_messages<typename RUDP_Output::defs::i_message>(mbs)) {
				state.messages.push_back(m);
			}
		}
    }

	// Confluence transition
	// Used to call set call precedent
    void confluence_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
        internal_transition();
        external_transition(TIME(), std::move(mbs));
    }

    // Output function
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

	// Time advance
	// Used to set the internal time of the current state
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

    friend std::ostringstream& operator<<(std::ostringstream& os, const typename RUDP_Output::state_type& i) {
        os << "State: " << enumToString(i.current_state) + std::string("\n");
        return os;
    }

private:
    void send_packets() const {
        for (std::vector<char> m : state.messages) {
			try {
            	connection->send(m.data(), m.size());
			}
            catch (std::runtime_error error) {
                std::cout << "[RUDP Output] (ERROR) Error sending packet using RUDP Output model: " << error.what() << std::endl;
            }
        }
    }
};

#endif /* RUDP_OUTPUT_HPP */
