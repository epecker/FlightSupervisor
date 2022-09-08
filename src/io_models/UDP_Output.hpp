/**
 *	\brief		An atomic model for sending UDP packets to an address and port.
 *	\details	This header file defines a output atomic model for use in the
                Cadmium DEVS simulation software.
 *	\author		James Horner
 *	\author		Tanner Trautrim
 */

#ifndef UDP_OUTPUT_HPP
#define UDP_OUTPUT_HPP

#include <boost/asio.hpp>

// RT-Cadmium
#include "cadmium/modeling/ports.hpp"
#include "cadmium/modeling/message_bag.hpp"

#include "../enum_string_conversion.hpp"
#include "../Constants.hpp"

// Atomic model
template<typename TIME>
class UDP_Output {
protected:
	bool broadcast;
    boost::asio::ip::udp::endpoint network_endpoint;

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
    UDP_Output() {
        state.current_state = States::IDLE;
		broadcast = true;
        network_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::broadcast(), MAVLINK_OVER_UDP_PORT);
    }

    // Constructor with polling rate parameter
    UDP_Output(const std::string& address, unsigned short port, bool broadcast) {
        state.current_state = States::IDLE;
		this->broadcast = broadcast;
		if (broadcast) {
        	network_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::broadcast(), port);
		}
		else {
        	network_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(address), port);
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
    using input_ports=std::tuple<typename UDP_Output::defs::i_message>;

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
		if (cadmium::get_messages<typename UDP_Output::defs::i_message>(mbs).size() >= 1){
			state.current_state = States::SENDING;
			for (std::vector<char> m : cadmium::get_messages<typename UDP_Output::defs::i_message>(mbs)) {
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
                assert(false && "Unhandled time advance in UDP_Output.hpp");
        }
    }

    friend std::ostringstream& operator<<(std::ostringstream& os, const typename UDP_Output::state_type& i) {
        os << "State: " << enumToString(i.current_state) + std::string("\n");
        return os;
    }

private:
    void send_packets() const {
        boost::asio::io_service io_service;
        boost::asio::ip::udp::socket socket(io_service);
        boost::system::error_code err;

		socket.open(boost::asio::ip::udp::v4(), err);
		if (err) {
			std::cout << "[UDP Output] (ERROR) Error opening socket in UDP Output model: " << err.message() << std::endl;
		}
		if (broadcast) {
			socket.set_option(boost::asio::socket_base::broadcast(true), err);
			if (err) {
				std::cout << "[UDP Output] (ERROR) Error setting socket option in UDP Output model: " << err.message() << std::endl;
			}
		}
        for (std::vector<char> m : state.messages) {
            socket.send_to(boost::asio::buffer(m.data(), m.size()), network_endpoint, 0, err);
			if (err) {
                std::cout << "[UDP Output] (ERROR) Error sending packet using UDP Output model: " << err.message() << std::endl;
            }
        }
		socket.close();
    }
};

#endif /* UDP_OUTPUT_HPP */
