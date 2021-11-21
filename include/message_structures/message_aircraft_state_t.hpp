#ifndef AIRCRAFT_STATE_MESSAGE_HPP
#define AIRCRAFT_STATE_MESSAGE_HPP

#include <iostream>

using namespace std;

/*******************************************/
/**************** Message_t ****************/
/*******************************************/
struct message_aircraft_state_t {
	double  lat;		            // Decimal degrees
	double  lon;		            // Decimal degrees
	float   alt_MSL;	            // Ft
	float   hdg_Deg;	            // 0360 True hdg
	double  vel_Kts;			        // Kts, tolerance for velocity  - horizontal

	message_aircraft_state_t() :
		lat(0.0),
		lon(0.0),
		alt_MSL(0.0),
		hdg_Deg(0.0),
		vel_Kts(0.0) 
	{}

	message_aircraft_state_t(
		double  i_lat,
		double  i_lon,
		float   i_alt_MSL,
		float   i_hdg_Deg,
		double  i_vel_Kts
	)
		: lat(i_lat),
		lon(i_lon),
		alt_MSL(i_alt_MSL),
		hdg_Deg(i_hdg_Deg),
		vel_Kts(i_vel_Kts) {}
};

/***************************************************/
/************* Output stream ************************/
/***************************************************/

ostream& operator<<(ostream& os, const message_aircraft_state_t& msg) {
	os << msg.lat << " "
		<< msg.lon << " "
		<< msg.alt_MSL << " "
		<< msg.hdg_Deg << " "
		<< msg.vel_Kts;
	return os;
}

/***************************************************/
/************* Input stream ************************/
/***************************************************/

istream& operator>> (istream& is, message_aircraft_state_t& msg) {
	is >> msg.lat;
	is >> msg.lon;
	is >> msg.alt_MSL;
	is >> msg.hdg_Deg;
	is >> msg.vel_Kts;
	return is;
}



#endif // AIRCRAFT_STATE_MESSAGE_HPP