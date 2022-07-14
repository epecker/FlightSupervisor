#ifndef MESSAGE_START_SUPERVISOR_T_HPP
#define MESSAGE_START_SUPERVISOR_T_HPP

#include <iostream>

/*******************************************/
/**************** Message_t ****************/
/*******************************************/
struct message_start_supervisor_t {
	bool autonomy_armed;
	bool mission_started;

	message_start_supervisor_t() :
			autonomy_armed(false),
			mission_started(false) {}

	message_start_supervisor_t(
			bool i_autonomy_armed,
			bool i_mission_started
			)
			: autonomy_armed(i_autonomy_armed),
			  mission_started(i_mission_started){}
};

/***************************************************/
/************* Output stream ************************/
/***************************************************/

ostream &operator<<(ostream &os, const message_start_supervisor_t &msg) {
	os << msg.autonomy_armed << " "
	   << msg.mission_started;
	return os;
}

/***************************************************/
/************* Input stream ************************/
/***************************************************/

istream &operator>>(istream &is, message_start_supervisor_t &msg) {
	is >> msg.autonomy_armed;
	is >> msg.mission_started;
	return is;
}

#endif // MESSAGE_START_SUPERVISOR_T_HPP
