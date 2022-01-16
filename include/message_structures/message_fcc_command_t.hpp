#ifndef FCC_COMMAND_HPP
#define FCC_COMMAND_HPP

#include <stdint.h>

struct message_fcc_command_t {
	float param1;				/*<  PARAM1, see MAV_CMD enum*/
	float param2;				/*<  PARAM2, see MAV_CMD enum*/
	float param3;				/*<  PARAM3, see MAV_CMD enum*/
	float param4;				/*<  PARAM4, see MAV_CMD enum*/
	int32_t x;					/*<  PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7*/
	int32_t y;					/*<  PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7*/
	float z;					/*<  PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.*/
	uint16_t seq;				/*<  Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4).*/
	uint16_t command;			/*<  The scheduled action for the waypoint.*/
	uint8_t target_system;		/*<  System ID*/
	uint8_t target_component;	/*<  Component ID*/
	uint8_t frame;				/*<  The coordinate system of the waypoint.*/
	uint8_t current;			/*<  false:0, true:1*/
	uint8_t autocontinue;		/*<  Autocontinue to next waypoint*/
	uint8_t mission_type;		/*<  Mission type.*/

	message_fcc_command_t() :
		param1(0.0),
		param2(0.0),
		param3(0.0),
		param4(0.0),
		x(0),
		y(0),
		z(0.0),
		seq(0),
		command(0),
		target_system(0),
		target_component(0),
		frame(0),
		current(0),
		autocontinue(0),
		mission_type(0) {}

	message_fcc_command_t(
		float i_param1,
		float i_param2,
		float i_param3,
		float i_param4,
		int32_t i_x,
		int32_t i_y,
		float i_z,
		uint16_t i_seq,
		uint16_t i_command,
		uint8_t i_target_system,
		uint8_t i_target_component,
		uint8_t i_frame,
		uint8_t i_current,
		uint8_t i_autocontinue,
		uint8_t i_mission_type
	) :
		param1(i_param1),
		param2(i_param2),
		param3(i_param3),
		param4(i_param4),
		x(i_x),
		y(i_y),
		z(i_z),
		seq(i_seq),
		command(i_command),
		target_system(i_target_system),
		target_component(i_target_component),
		frame(i_frame),
		current(i_current),
		autocontinue(i_autocontinue),
		mission_type(i_mission_type) {}
};

/***************************************************/
/************* Output stream ***********************/
/***************************************************/

ostream& operator<<(ostream& os, const message_fcc_command_t& msg) {
	os << msg.param1 << " "
		<< msg.param2 << " "
		<< msg.param3 << " "
		<< msg.param4 << " "
		<< msg.x << " "
		<< msg.y << " "
		<< msg.z << " "
		<< msg.seq << " "
		<< msg.command << " "
		<< (int)msg.target_system << " "		// Casted to to int to work with logger
		<< (int)msg.target_component << " "		// Casted to to int to work with logger
		<< (int)msg.frame << " "				// Casted to to int to work with logger
		<< (int)msg.current << " "				// Casted to to int to work with logger
		<< (int)msg.autocontinue << " "			// Casted to to int to work with logger
		<< (int)msg.mission_type;				// Casted to to int to work with logger
	return os;
}

/***************************************************/
/************* Input stream ************************/
/***************************************************/

istream& operator>> (istream& is, message_fcc_command_t& msg) {
	is >> msg.param1
		>> msg.param2
		>> msg.param3
		>> msg.param4
		>> msg.x
		>> msg.y
		>> msg.z
		>> msg.seq
		>> msg.command
		>> msg.target_system
		>> msg.target_component
		>> msg.frame
		>> msg.current
		>> msg.autocontinue
		>> msg.mission_type;
	return is;
}

#endif