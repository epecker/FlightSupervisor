/**
 *	\brief		A generic atomic model used for creating packets.
 *	\details	This header file defines a packet builder which takes in
				any signal and outputs a generic packet. This atomic model is for use
				in the Cadmium DEVS simulation software.
 *	\author		Tanner Trautrim
 */

#ifndef PACKET_BUILDER_HPP
#define PACKET_BUILDER_HPP

#include <limits>

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

#include "message_structures/message_update_gcs_t.hpp"
#include "message_structures/message_fcc_command_t.hpp"

#include "enum_string_conversion.hpp"
#include "component_macros.h"
#include "Constants.hpp"

template<typename TYPE, typename TIME>
class Packet_Builder {
public:
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
										(IDLE)
										(GENERATE_PACKET)
	);

	// Input and output port definitions
	struct defs {
		struct i_data : public cadmium::in_port<TYPE> {};
		struct o_packet: public cadmium::out_port<std::array<char, sizeof(TYPE)>> {};
	};

	// Create a tuple of input ports (required for the simulator)
	using input_ports = std::tuple<
			typename Packet_Builder::defs::i_data
	>;

	// Create a tuple of output ports (required for the simulator)
	using output_ports = std::tuple<
			typename Packet_Builder::defs::o_packet
	>;

	// Tracks the state of the model
	struct state_type {
		States current_state;
	} state;

	Packet_Builder() {
		state.current_state = States::IDLE;
		data = TYPE();
		packet_sequence = 0;
	}

	explicit Packet_Builder(States initial_state) {
		state.current_state = initial_state;
		data = TYPE();
		packet_sequence = 0;
	}

	void internal_transition() {
		switch (state.current_state) {
			case States::GENERATE_PACKET:
				state.current_state = States::IDLE;
				packet_sequence++;
				break;
			default:
				break;
		}
	}

	void external_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		bool received_data = !cadmium::get_messages<typename Packet_Builder::defs::i_data>(mbs).empty();
		switch (state.current_state) {
			case States::IDLE:
				if (received_data){
					data = cadmium::get_messages<typename Packet_Builder::defs::i_data>(mbs)[0];
					state.current_state = States::GENERATE_PACKET;
				}
				break;
			default:
				break;
		}
	}

	// Confluence transition sets the internal/external precedence
	// Triggered when a message is received at the same time as an internal transition.
	// (required for the simulator)
	void confluence_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), move(mbs));
	}

	[[nodiscard]] typename cadmium::make_message_bags<output_ports>::type output() const {
		typename cadmium::make_message_bags<output_ports>::type bags;
		vector<std::array<char, sizeof(TYPE)>> packets;

		switch (state.current_state) {
			case States::GENERATE_PACKET:
				packets.push_back(generate_packet());
				cadmium::get_messages<typename Packet_Builder::defs::o_packet>(bags) = packets;
				break;
			default:
				break;
		}
		return bags;
	}

	TIME time_advance() const {
		TIME next_internal;
		switch (state.current_state) {
			case States::IDLE:
				next_internal = numeric_limits<TIME>::infinity();
				break;
			case States::GENERATE_PACKET:
				next_internal = TIME(TA_ZERO);
				break;
			default:
				assert(false && "Unhandled time advance");
		}
		return next_internal;
	}

	// Used for logging outputs the state's name. (required for the simulator)
	friend ostringstream& operator<<(ostringstream& os, const typename Packet_Builder<TYPE, TIME>::state_type& i) {
		os << (string("State: ") + enumToString(i.current_state) + string("\n"));
		return os;
	}

protected:
	TYPE data;
	uint8_t packet_sequence;

	virtual std::array<char, sizeof(TYPE)> generate_packet() const {
		std::string e = "The type \"" + std::string(typeid(TYPE).name()) + "\" is not a supported type";
		assert(false && e.c_str());
	}
};

template<typename TIME>
class Packet_Builder_FCC : public Packet_Builder<message_fcc_command_t, TIME> {
public:
	Packet_Builder_FCC() = default;
	explicit Packet_Builder_FCC(typename Packet_Builder<message_fcc_command_t, TIME>::States initial_state) : Packet_Builder<message_fcc_command_t, TIME>(initial_state){};

	[[nodiscard]] std::array<char, sizeof(message_fcc_command_t)> generate_packet() const {
		std::array<char, sizeof(message_fcc_command_t)> packet = {};
		std::memcpy(packet.data(), &data, sizeof(message_fcc_command_t));
		return packet;
	}
private:
	message_fcc_command_t data;
};

#endif // PACKET_BUILDER_HPP
