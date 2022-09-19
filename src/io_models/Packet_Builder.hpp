/**
 * 	\file		Packet_Builder.hpp
 *	\brief		Definition of the Packet Builder atomic model.
 *	\details	This header file defines the Packet Builder atomic model for use in the Cadmium DEVS
				simulation software. This model converts input messages into char buffers.
 *	\image		html io_models/packet_builder.png
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef PACKET_BUILDER_HPP
#define PACKET_BUILDER_HPP

// Messages structures
#include "../message_structures/message_update_gcs_t.hpp"
#include "../message_structures/message_fcc_command_t.hpp"
#include "../message_structures/message_boss_mission_update_t.hpp"
#include "../message_structures/message_landing_point_t.hpp"

// Utility Functions
#include "../enum_string_conversion.hpp"
#include <mavNRC/endian.hpp>

// Constants
#include "../Constants.hpp"

// Cadmium Simulator Headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>

// Boost Headers
#include <boost/container/vector.hpp>

// System libraries
#include <limits>
#include <cstring>
#include <cassert>

/**
 *	\class		Packet_Builder
 *	\brief		Definition of the Packet Builder atomic model.
 *	\details	This class defines the Packet Builder atomic model for use in the Cadmium DEVS
				simulation software. This model converts input messages into char buffers.
 *	\image		html io_models/packet_builder.png
 */
template<typename TYPE, typename TIME>
class Packet_Builder {
public:
	/**
	 *	\par	States
	 * 	Declaration of the states of the atomic model.
	 */
	DEFINE_ENUM_WITH_STRING_CONVERSIONS(States,
										(IDLE)
										(GENERATE_PACKET)
	);

	/**
	 *	\brief	For definition of the input and output ports see:
	 *	\ref 	RUDP_Output_input_ports "Input Ports" and
	 *	\ref 	RUDP_Output_output_ports "Output Ports"
	 * 	\note 	All input and output ports must be listed in this struct.
	 */
	struct defs {
		struct i_data : public cadmium::in_port<TYPE> {};
		struct o_packet: public cadmium::out_port<std::vector<char>> {};
	};

	/**
	 * 	\anchor	Packet_Builder_input_ports
	 *	\par	Input Ports
	 * 	Definition of the input ports for the model.
	 * 	\param 	i_data	Port for receiving data to be converted into a packet.
	 */
	using input_ports = std::tuple<
			typename defs::i_data
	>;

	/**
	 *	\anchor	Packet_Builder_output_ports
	 * 	\par 	Output Ports
	 * 	Definition of the output ports for the model.
	 * 	\param	o_packet	Port for outputting the packet.
	 */
	using output_ports = std::tuple<
			typename defs::o_packet
	>;

	/**
	 *	\anchor	Packet_Builder_state_type
	 *	\par	State
	 * 	Definition of the states of the atomic model.
	 * 	\param 	current_state 	Current state of atomic model.
	 */
	struct state_type {
		States current_state;
	} state;

	/**
	 * \brief 	Default constructor for the model.
	 */
	Packet_Builder() {
		state.current_state = States::IDLE;
		packet_sequence = 0;
	}

	/**
	 * \brief 	Constructor for the model with initial state parameter
	 * 			for debugging or partial execution startup.
	 * \param	initial_state	States initial state of the model.
	 */
	explicit Packet_Builder(States initial_state) {
		state.current_state = initial_state;
		packet_sequence = 0;
	}

	/// Internal transitions of the model
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

	/// External transitions of the model
	void external_transition([[maybe_unused]] TIME e, typename cadmium::make_message_bags<input_ports>::type mbs) {
		bool received_data = !cadmium::get_messages<typename Packet_Builder::defs::i_data>(mbs).empty();

		switch (state.current_state) {
			case States::IDLE:
				if (received_data){
                    std::vector<TYPE> temp = cadmium::get_messages<typename Packet_Builder::defs::i_data>(mbs);
                    for (int i = 0; i < temp.size(); i++) {
                        data.push_back(temp[i]);
                        preprocess_data(&data[i]);
                    }
					state.current_state = States::GENERATE_PACKET;
				}
				break;
			default:
				break;
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
		std::vector<std::vector<char>> packets;
        std::vector<char> v;

		switch (state.current_state) {
			case States::GENERATE_PACKET:
                for (int i = 0; i < data.size(); i++) {
                    packets.push_back(generate_packet(&data[i]));
                }
                cadmium::get_messages<typename Packet_Builder::defs::o_packet>(bags) = packets;
                data.clear();
				break;
			default:
				break;
		}
		return bags;
	}

	/// Function to declare the time advance value for each state of the model.
	TIME time_advance() const {
		TIME next_internal;
		switch (state.current_state) {
			case States::IDLE:
				next_internal = std::numeric_limits<TIME>::infinity();
				break;
			case States::GENERATE_PACKET:
				next_internal = TIME(TA_ZERO);
				break;
			default:
                assert(false && "Unhandled time advance in Packet_Builder.hpp");
        }
		return next_internal;
	}

	/**
	 *  \brief 		Operator for defining how the model state will be represented as a string.
	 * 	\warning 	Prepended "State: " is required for log parsing, do not remove.
	 */
	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Packet_Builder<TYPE, TIME>::state_type& i) {
		os << (std::string("State: ") + enumToString(i.current_state) + std::string("\n"));
		return os;
	}

protected:
	/// Stores the data to be converted into a packet
    mutable boost::container::vector<TYPE> data;
	/// Number of packets processed
	uint8_t packet_sequence;

private:
	/// Defined if the data needs to be processed before being converted into a packet.
    virtual void preprocess_data(TYPE * data_point) {}

	/// Packet generator - can be overloaded to allow for different packet types.
    [[nodiscard]] virtual std::vector<char> generate_packet(TYPE * data_point) const {
        std::vector<char> packet(sizeof(*data_point));
        std::memcpy(packet.data(), (char *)data_point, sizeof(*data_point));
        return packet;
    }
};

/**
 *	\class		Packet_Builder_Boss
 *	\brief		Definition of the Packet Builder Boss atomic model.
 *	\details	This class defines the Packet Builder Boss atomic model for use in the Cadmium DEVS
				simulation software. This model converts message_fcc_command_t messages into char buffers.
 */
template<typename TIME>
class Packet_Builder_Boss : public Packet_Builder<message_boss_mission_update_t, TIME> {
public:
	Packet_Builder_Boss() = default;
};

/**
 *	\class		Packet_Builder_Bool
 *	\brief		Definition of the Packet Builder Bool atomic model.
 *	\details	This class defines the Packet Builder Bool atomic model for use in the Cadmium DEVS
				simulation software. This model converts message_fcc_command_t messages into char buffers.
 */
template<typename TIME>
class Packet_Builder_Bool : public Packet_Builder<bool, TIME> {
public:
    Packet_Builder_Bool() = default;
    explicit Packet_Builder_Bool(uint8_t signal_id) {
        this->state.current_state = Packet_Builder_Bool::States::IDLE;
        this->packet_sequence = 0;
        this->signal_id = signal_id;
    }

    [[nodiscard]] virtual std::vector<char> generate_packet(bool * data_point) const {
        std::vector<char> packet(sizeof(*data_point) + 1);
        packet[0] = signal_id;
        std::memcpy(&packet[1], (char *)data_point, sizeof(*data_point));
        return packet;
    }
private:
    uint8_t signal_id{};
};

/**
 *	\class		Packet_Builder_Uint8
 *	\brief		Definition of the Packet Builder Uint8 atomic model.
 *	\details	This class defines the Packet Builder Uint8 atomic model for use in the Cadmium DEVS
				simulation software. This model converts message_fcc_command_t messages into char buffers.
 */
template<typename TIME>
class Packet_Builder_Uint8 : public Packet_Builder<uint8_t, TIME> {
public:
    Packet_Builder_Uint8() = default;
    explicit Packet_Builder_Uint8(uint8_t signal_id) {
        this->state.current_state = Packet_Builder_Uint8::States::IDLE;
        this->packet_sequence = 0;
        this->signal_id = signal_id;
    }

    [[nodiscard]] virtual std::vector<char> generate_packet(uint8_t * data_point) const {
        std::vector<char> packet(sizeof(*data_point) + 1);
        packet[0] = signal_id;
        std::memcpy(&packet[1], (char *)data_point, sizeof(*data_point));
        return packet;
    }
private:
    uint8_t signal_id{};
};

/**
 *	\class		Packet_Builder_Int
 *	\brief		Definition of the Packet Builder Int atomic model.
 *	\details	This class defines the Packet Builder Int atomic model for use in the Cadmium DEVS
				simulation software. This model converts message_fcc_command_t messages into char buffers.
 */
template<typename TIME>
class Packet_Builder_Int : public Packet_Builder<int, TIME> {
public:
    Packet_Builder_Int() = default;
    explicit Packet_Builder_Int(uint8_t signal_id) {
        this->state.current_state = Packet_Builder_Int::States::IDLE;
        this->packet_sequence = 0;
        this->signal_id = signal_id;
    }

    [[nodiscard]] virtual std::vector<char> generate_packet(int * data_point) const {
        std::vector<char> packet(sizeof(*data_point) + 1);
        packet[0] = signal_id;
        std::memcpy(&packet[1], (char *)data_point, sizeof(*data_point));
        return packet;
    }
private:
    uint8_t signal_id{};
};

/**
 *	\class		Packet_Builder_Landing_Point
 *	\brief		Definition of the Packet Builder Landing Point atomic model.
 *	\details	This class defines the Packet Builder Landing Point atomic model for use in the Cadmium DEVS
				simulation software. This model converts message_landing_point_t messages into char buffers.
 */
template<typename TIME>
class Packet_Builder_Landing_Point : public Packet_Builder<message_landing_point_t, TIME> {
public:
    Packet_Builder_Landing_Point() = default;

    [[nodiscard]] virtual std::vector<char> generate_packet(message_landing_point_t * data_point) const {
        std::vector<char> packet(sizeof(*data_point) + 1);
        packet[0] = SIG_ID_LANDING_POINT;
        std::memcpy(&packet[1], (char *)data_point, sizeof(*data_point));
        return packet;
    }
};

/**
 *	\class		Packet_Builder_Fcc
 *	\brief		Definition of the Packet Builder Fcc atomic model.
 *	\details	This class defines the Packet Builder Fcc atomic model for use in the Cadmium DEVS
				simulation software. This model converts message_fcc_command_t messages into char buffers.
 */
template<typename TIME>
class Packet_Builder_Fcc : public Packet_Builder<message_fcc_command_t, TIME> {
public:
    Packet_Builder_Fcc() = default;

private:
    void preprocess_data(message_fcc_command_t * data_point) {
        Struct_ntohl((void *)data_point, sizeof(*data_point));
        Swap_Double(&data_point->supervisor_gps_time);
    }
};

/**
 *	\class		Packet_Builder_GCS
 *	\brief		Definition of the Packet Builder GCS atomic model.
 *	\details	This class defines the Packet Builder GCS atomic model for use in the Cadmium DEVS
				simulation software. This model converts message_update_gcs_t messages into char buffers.
 */
template<typename TIME>
class Packet_Builder_GCS : public Packet_Builder<message_update_gcs_t, TIME> {
public:
	Packet_Builder_GCS() = default;

private:
	/// Mavlink compatible structure
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

	/// Mavlink compatible structure
	struct mavlink_statustext_t {
		uint8_t severity;
		char text[50];
		uint16_t id;
		uint8_t chunk_seq;
	};

	/// Helper that generates the crc checksum
	void crc_accumulate_CUSTOM(uint8_t data, uint16_t *crcAccum) const
	{
		uint8_t tmp;

		tmp = data ^ (uint8_t)(*crcAccum &0xff);
		tmp ^= (tmp<<4);
		*crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
	}

	/// Generates the crc checksum
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

	/// Helper that fills the packet buffer
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
	    std::memcpy(&buf[10], (const char *)&msg->payload64, msg->len);

		uint16_t checksum = 0xffff;
		crc(&checksum, &buf[1], MAVLINK_CORE_HEADER_LEN, (const uint8_t *)&msg->payload64, msg->len, MAVLINK_MSG_ID_STATUSTEXT_CRC);
		buf[MAVLINK_CORE_HEADER_LEN + msg->len + 1] = (uint8_t)(checksum & 0xFF);
		buf[MAVLINK_CORE_HEADER_LEN + msg->len + 2] = (uint8_t)(checksum >> 8);
	}

	/// Used to craft mavlink packages
	void create_message(mavlink_message_t * msg, message_update_gcs_t * data_point) const {
		mavlink_statustext_t status_text{};
		status_text.severity = data_point->severity;
		status_text.id = 0;
		status_text.chunk_seq = 0;
	    std::memcpy(status_text.text, data_point->text.c_str(), sizeof(char) * 50);
	    std::memcpy((char *)msg->payload64, &status_text, MAVLINK_MSG_ID_STATUSTEXT_LEN);
		msg->msgid = MAVLINK_MSG_ID_STATUSTEXT;
		msg->magic = MAVLINK_STX;
		msg->len = strlen((char *)msg->payload64);
		msg->sysid = MY_MAV_SYS_ID;
		msg->compid = MY_MAV_COMP_ID;
		msg->incompat_flags = 0;
		msg->compat_flags = 0;
		msg->seq = this->packet_sequence;
	}

	[[nodiscard]] std::vector<char> generate_packet(message_update_gcs_t * data_point) const {
		mavlink_message_t msg{};
		create_message(&msg, data_point);

        std::vector<char> packet(MAVLINK_CORE_HEADER_LEN + msg.len + 3); // 3 = checksum(2 bytes) + magic(1 byte)
		create_packet((uint8_t *)packet.data(), &msg);

		return packet;
	}
};

#endif // PACKET_BUILDER_HPP
