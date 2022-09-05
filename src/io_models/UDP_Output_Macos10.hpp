/**
 *	\brief		An atomic model for sending UDP packets to an address and port.
 *	\details	This header file defines a output atomic model for use in the
                Cadmium DEVS simulation software.
 *	\author		Tanner Trautrim
 */

#ifndef UDP_OUTPUT_HPP
#define UDP_OUTPUT_HPP

// RT-Cadmium
#include "cadmium/modeling/ports.hpp"
#include "cadmium/modeling/message_bag.hpp"

#include "../enum_string_conversion.hpp"
#include "../Constants.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <cassert>
#include <iostream>

using namespace cadmium;

// Atomic model
template<typename TIME>
class UDP_Output_Macos10 {
public:
	// Used to keep track of the states
	// (not required for the simulator)
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
		(IDLE)
		(SENDING)
	);

	// Input and output port definitions
	struct defs{
	struct i_message : public in_port<std::vector<char>> { };
	};

    // Default constructor
	UDP_Output_Macos10() {
        state.current_state = States::IDLE;
		dest_addr.sin_family = AF_INET;
		dest_addr.sin_port = htons(MAVLINK_OVER_UDP_PORT);
		dest_addr.sin_addr.s_addr = inet_addr(PEREGRINE_IP);
		server_sock = socket(AF_INET, SOCK_DGRAM, 0);
    }

    // Constructor with polling rate parameter
	UDP_Output_Macos10(const std::string& address, unsigned short port) {
        state.current_state = States::IDLE;
		dest_addr.sin_family = AF_INET;
		dest_addr.sin_port = htons(port);
		dest_addr.sin_addr.s_addr = inet_addr(address.c_str());
		server_sock = socket(AF_INET, SOCK_DGRAM, 0);
    }

	// This is used to track the state of the atomic model.
	// (required for the simulator)
    struct state_type{
        States current_state;
		std::vector<std::vector<char>> messages;
    } state;

	// Create a tuple of input ports (required for the simulator)
    using input_ports=std::tuple<typename UDP_Output_Macos10::defs::i_message>;

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
    void external_transition(TIME e, [[maybe_unused]] typename make_message_bags<input_ports>::type mbs) {
		if (get_messages<typename UDP_Output_Macos10::defs::i_message>(mbs).size() >= 1){
			state.current_state = States::SENDING;
			for (std::vector<char> m : get_messages<typename UDP_Output_Macos10::defs::i_message>(mbs)) {
				state.messages.push_back(m);
			}
		}
    }

	// Confluence transition
	// Used to call set call precedent
    void confluence_transition([[maybe_unused]] TIME e, typename make_message_bags<input_ports>::type mbs) {
        internal_transition();
        external_transition(TIME(), std::move(mbs));
    }

    // Output function
    [[nodiscard]] typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;

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
                assert(false && "Unhandled time advance in UDP_Output.hpp");
        }
    }

    friend std::ostringstream& operator<<(std::ostringstream& os, const typename UDP_Output_Macos10::state_type& i) {
        os << (std::string("State: ") + enumToString(i.current_state) + std::string("\n"));
        return os;
    }

private:
	sockaddr_in dest_addr{};
	int server_sock{};

    void send_packets() const {
        for (std::vector<char> m : state.messages) {
			ssize_t sent = sendto(server_sock, m.data(), m.size(), 0, (struct sockaddr*) &dest_addr, sizeof(dest_addr));
            if (sent < 0) {
                std::cout << "[UDP Output] (ERROR) Error sending packet using UDP Output model " << std::endl;
            }
        }
    }
};

#endif /* UDP_OUTPUT_HPP */
