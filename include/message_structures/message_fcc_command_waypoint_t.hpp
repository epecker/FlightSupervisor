#ifndef MESSAGE_FCC_COMMAND_TO_REPLACE_ORIG_T_HPP
#define MESSAGE_FCC_COMMAND_TO_REPLACE_ORIG_T_HPP

#include <cstdint>

typedef enum
{
	LANDING_REQUESTED,
	TAKEOFF_REQUESTED,
	TRAJECTORY_CONTROL,
	DAA_CONTROL,
	MAV_COMMAND
} control_mode_e;

struct message_fcc_command_waypoint_t {
	double supervisor_gps_time;
	uint32_t supervisor_status;
	uint16_t command;
	float param1;
	float param2;
	float param3;
	float param4;
	int32_t latitude;
	int32_t longitude;
	float altitude_msl;

	message_fcc_command_waypoint_t() :
			supervisor_gps_time(0.0),
			supervisor_status(0),
			command(0),
			param1(0.0),
			param2(0.0),
			param3(0.0),
			param4(0.0),
			latitude(0),
			longitude(0),
			altitude_msl(0.0) {}

	message_fcc_command_waypoint_t(
			double i_supervisor_gps_time,
			uint32_t i_supervisor_status,
			uint16_t i_command,
			float i_param1,
			float i_param2,
			float i_param3,
			float i_param4,
			int32_t i_latitude,
			int32_t i_longitude,
			float i_altitude_msl
	) :
			supervisor_gps_time(i_supervisor_gps_time),
			supervisor_status(i_supervisor_status),
			command(i_command),
			param1(i_param1),
			param2(i_param2),
			param3(i_param3),
			param4(i_param4),
			latitude(i_latitude),
			longitude(i_longitude),
			altitude_msl(i_altitude_msl) {}

	void set_supervisor_status(control_mode_e new_mode) {
		supervisor_status = 0;
		supervisor_status |= (1 << 0); // Supervisor is ready to operate

		switch (new_mode) {
			case LANDING_REQUESTED:
				supervisor_status = (1 << 1);
				break;
			case TAKEOFF_REQUESTED:
				supervisor_status |= (1 << 2);
				break;
			case TRAJECTORY_CONTROL:
				supervisor_status |= (1 << 3);
				break;
			case DAA_CONTROL:
				supervisor_status |= (1 << 4);
				break;
			case MAV_COMMAND:
				supervisor_status |= (1 << 5);
				break;
			default:
				supervisor_status = 0;
		}
	}
};

/***************************************************/
/************* Output stream ***********************/
/***************************************************/

ostream& operator<<(ostream& os, const message_fcc_command_waypoint_t& msg) {
	os << msg.supervisor_gps_time << " "
	   << msg.supervisor_status << " "
	   << msg.command << " "
	   << msg.param1 << " "
	   << msg.param2 << " "
	   << msg.param3 << " "
	   << msg.param4 << " "
	   << msg.latitude << " "
	   << msg.longitude << " "
	   << msg.altitude_msl << " ";
	return os;
}

/***************************************************/
/************* Input stream ************************/
/***************************************************/

istream& operator>> (istream& is, message_fcc_command_waypoint_t& msg) {
	is >> msg.supervisor_gps_time
	   >> msg.supervisor_status
	   >> msg.command
	   >> msg.param1
	   >> msg.param2
	   >> msg.param3
	   >> msg.param4
	   >> msg.latitude
	   >> msg.longitude
	   >> msg.altitude_msl;
	return is;
}

#endif // MESSAGE_FCC_COMMAND_TO_REPLACE_ORIG_T_HPP
