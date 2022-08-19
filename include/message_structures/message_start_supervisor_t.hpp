#ifndef MESSAGE_START_SUPERVISOR_T_HPP
#define MESSAGE_START_SUPERVISOR_T_HPP

#include <iostream>

#pragma pack(push, 4)
struct message_start_supervisor_t {
	bool autonomy_armed;
	bool mission_started;
    int mission_number;

	message_start_supervisor_t() :
			autonomy_armed(false),
			mission_started(false),
            mission_number(0) {}

	message_start_supervisor_t(
			bool i_autonomy_armed,
			bool i_mission_started,
            int i_mission_number
			)
			: autonomy_armed(i_autonomy_armed),
			  mission_started(i_mission_started),
              mission_number(i_mission_number){}
};
#pragma pack(pop)

/***************************************************/
/************* Output stream ************************/
/***************************************************/

ostream &operator<<(ostream &os, const message_start_supervisor_t &msg) {
	os << msg.autonomy_armed << " "
	   << msg.mission_started << " "
       << msg.mission_number << " ";
	return os;
}

/***************************************************/
/************* Input stream ************************/
/***************************************************/

istream &operator>>(istream &is, message_start_supervisor_t &msg) {
	is >> msg.autonomy_armed;
	is >> msg.mission_started;
	is >> msg.mission_number;
	return is;
}

#endif // MESSAGE_START_SUPERVISOR_T_HPP
