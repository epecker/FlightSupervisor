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
#include "message_structures/message_boss_mission_update_t.hpp"

#include "mavNRC/endian.hpp"
#include "enum_string_conversion.hpp"
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
    struct o_packet: public cadmium::out_port<std::vector<char>> {};
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
                    preprocess_data();
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
		external_transition(TIME(), std::move(mbs));
	}

	[[nodiscard]] typename cadmium::make_message_bags<output_ports>::type output() const {
		typename cadmium::make_message_bags<output_ports>::type bags;
		std::vector<std::vector<char>> packets;
        vector<char> v;

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
                assert(false && "Unhandled time advance in Packet_Builder.hpp");
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

private:
    virtual void preprocess_data() {}

    [[nodiscard]] virtual std::vector<char> generate_packet() const {
        std::vector<char> packet(sizeof(this->data));
        std::memcpy(packet.data(), (char *)&this->data, sizeof(this->data));
        return packet;
    }
};

/**
 * \brief Packet_Builder_Boss creates packets for use in output models
 * \details Packet_Builder_Boss uses the default implementation of
 *          generate_packet from Packet builder.
 */
template<typename TIME>
class Packet_Builder_Boss : public Packet_Builder<message_boss_mission_update_t, TIME> {
	using TYPE = message_boss_mission_update_t;

public:
	Packet_Builder_Boss() = default;
	explicit Packet_Builder_Boss(typename Packet_Builder<TYPE, TIME>::States initial_state) : Packet_Builder<TYPE, TIME>(initial_state){};
};

/**
 * \brief Packet_Builder_Fcc creates packets for use in output models
 * \details Packet_Builder_Fcc requires the bytes be swapped using a
 *          the functions specified in MavNRC endian.c.
 */
template<typename TIME>
class Packet_Builder_Fcc : public Packet_Builder<message_fcc_command_t, TIME> {
    using TYPE = message_fcc_command_t;

public:
    Packet_Builder_Fcc() = default;
    explicit Packet_Builder_Fcc(typename Packet_Builder<TYPE, TIME>::States initial_state) : Packet_Builder<TYPE, TIME>(initial_state){};

private:
    void preprocess_data() {
        Struct_ntohl((void *)&this->data, sizeof(this->data));
        Swap_Double(&this->data.supervisor_gps_time);
    }

    [[nodiscard]] std::vector<char> generate_packet() const {
        std::vector<char> packet(sizeof(this->data));
        std::memcpy(packet.data(), (char *)&this->data, sizeof(this->data));
        return packet;
    }
};

/**
 * \brief Packet_Builder_GCS creates packets for use in output models
 * \details Packet_Builder_GCS creates packets in the same style as mavlink.
 * 			This allows the packets to be sent systems using the mavlink protocol.
 */
template<typename TIME>
class Packet_Builder_GCS : public Packet_Builder<message_update_gcs_t, TIME> {
public:
	using TYPE = message_update_gcs_t;

	Packet_Builder_GCS() = default;
	explicit Packet_Builder_GCS(typename Packet_Builder<TYPE, TIME>::States initial_state) : Packet_Builder<TYPE, TIME>(initial_state){};

private:
	struct mavlink_message_t {
		uint8_t magic;
		uint8_t len;
		uint8_t incompat_flags;
		uint8_t compat_flags;
		uint8_t seq;
		uint8_t sysid;
		uint8_t compid;
		uint32_t msgid:24;
		uint64_t payload64[(MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7)/8];
	};

	struct mavlink_statustext_t {
		uint8_t severity;
		char text[50];
		uint16_t id;
		uint8_t chunk_seq;
	};

	void crc_accumulate_CUSTOM(uint8_t data, uint16_t *crcAccum) const
	{
		uint8_t tmp;

		tmp = data ^ (uint8_t)(*crcAccum &0xff);
		tmp ^= (tmp<<4);
		*crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
	}

	void crc(uint16_t *crcAccum, const uint8_t *header, uint16_t header_length, const uint8_t *buffer, uint16_t buffer_length, const uint8_t static_crc) const
	{
		while (header_length--) {
			crc_accumulate_CUSTOM(*header++, crcAccum);
		}
		while (buffer_length--) {
			crc_accumulate_CUSTOM(*buffer++, crcAccum);
		}
		crc_accumulate_CUSTOM(static_crc, crcAccum);
	}

	void create_packet(uint8_t *buf, mavlink_message_t *msg) const
	{
		buf[0] = msg->magic;
		buf[1] = msg->len;
		buf[2] = msg->incompat_flags;
		buf[3] = msg->compat_flags;
		buf[4] = msg->seq;
		buf[5] = msg->sysid;
		buf[6] = msg->compid;
		buf[7] = msg->msgid & 0xFF;
		buf[8] = (msg->msgid >> 8) & 0xFF;
		buf[9] = (msg->msgid >> 16) & 0xFF;
		memcpy(&buf[10], (const char *)&msg->payload64, msg->len);

		uint16_t checksum = 0xffff;
		crc(&checksum, &buf[1], MAVLINK_CORE_HEADER_LEN, (const uint8_t *)&msg->payload64, msg->len, MAVLINK_MSG_ID_STATUSTEXT_CRC);
		buf[MAVLINK_CORE_HEADER_LEN + msg->len + 1] = (uint8_t)(checksum & 0xFF);
		buf[MAVLINK_CORE_HEADER_LEN + msg->len + 2] = (uint8_t)(checksum >> 8);
	}

	void create_message(mavlink_message_t * msg) const {
		mavlink_statustext_t status_text{};
		status_text.severity = this->data.severity;
		status_text.id = 0;
		status_text.chunk_seq = 0;
		memcpy(status_text.text, this->data.text.c_str(), sizeof(char)*50);
		memcpy((char *)msg->payload64, &status_text, MAVLINK_MSG_ID_STATUSTEXT_LEN);
		msg->msgid = MAVLINK_MSG_ID_STATUSTEXT;
		msg->magic = MAVLINK_STX;
		msg->len = strlen((char *)msg->payload64);
		msg->sysid = MY_MAV_SYS_ID;
		msg->compid = MY_MAV_COMP_ID;
		msg->incompat_flags = 0;
		msg->compat_flags = 0;
		msg->seq = this->packet_sequence;
	}

	[[nodiscard]] std::vector<char> generate_packet() const {
		mavlink_message_t msg{};
		create_message(&msg);

        std::vector<char> packet(MAVLINK_CORE_HEADER_LEN + msg.len + 3); // 3 = checksum(2 bytes) + magic(1 byte)
		create_packet((uint8_t *)packet.data(), &msg);
		
		return packet;
	}
};

#endif // PACKET_BUILDER_HPP
