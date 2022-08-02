#ifndef MESSAGE_UPDATE_GCS_T_HPP
#define MESSAGE_UPDATE_GCS_T_HPP

#include <cstdint>
#include <string>
#include <iostream>

enum Mav_Severities_E {
	MAV_SEVERITY_ALERT = 1,
	MAV_SEVERITY_INFO = 6
};

struct message_update_gcs_t {
	std::string text;
	int severity;

	message_update_gcs_t(): severity(0) {}
	message_update_gcs_t(std::string text, uint8_t severity): text(std::move(text)), severity(severity) {}
};

/***************************************************/
/************* Output stream ***********************/
/***************************************************/

std::ostream& operator<<(std::ostream& os, const message_update_gcs_t& msg) {
	os << msg.text << " "
	   << msg.severity << " ";
	return os;
}

/***************************************************/
/************* Input stream ************************/
/***************************************************/

std::istream& operator>> (std::istream& is, message_update_gcs_t& msg) {
	is >> msg.text
	   >> msg.severity;
	return is;
}

#endif // MESSAGE_UPDATE_GCS_T_HPP
