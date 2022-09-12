#ifndef MESSAGE_COMMAND_ACK_T_HPP
#define MESSAGE_COMMAND_ACK_T_HPP

#include <iostream>

/*******************************************/
/**************** Message_t ****************/
/*******************************************/
struct message_command_ack_t{
	uint16_t	command;
	uint8_t		result;
	uint8_t		progress;
	uint32_t	result_param2;
	uint8_t		target_system;
	uint8_t		target_component;

	message_command_ack_t()
		:command(0), result(4), progress(0), result_param2(0), target_system(0), target_component(0) {}
	message_command_ack_t(uint16_t i_command, uint8_t i_result, uint8_t i_progress, uint32_t i_result_param2, uint8_t i_target_system, uint8_t i_target_component)
		:command(i_command), result(i_result), progress(i_progress), result_param2(i_result_param2), target_system(i_target_system), target_component(i_target_component) {}
};

/***************************************************/
/************* Output stream ************************/
/***************************************************/

std::ostream& operator<<(std::ostream& os, const message_command_ack_t& msg) {
	os << msg.command << " " << msg.result << " " << msg.progress << " " << msg.result_param2 << " " << msg.target_system << " " << msg.target_component;
	return os;
}

/***************************************************/
/************* Input stream ************************/
/***************************************************/

std::istream& operator>> (std::istream& is, message_command_ack_t& msg) {
	is >> msg.command;
	is >> msg.result;
	is >> msg.progress;
	is >> msg.result_param2;
	is >> msg.target_system;
	is >> msg.target_component;
	return is;
}

#endif // MESSAGE_COMMAND_ACK_T_HPP
