#ifndef AIRCRAFT_STATE_MESSAGE_HPP
#define AIRCRAFT_STATE_MESSAGE_HPP

#include <iostream>

using namespace std;

/*******************************************/
/**************** Message_t ****************/
/*******************************************/
struct message_aircraft_state_t {
	double  gps_time;
	double  lat;		            // Decimal degrees
	double  lon;		            // Decimal degrees
	float	alt_AGL;				// Ft
	float   alt_MSL;	            // Ft
	float   hdg_Deg;	            // 0360 True hdg
	double  vel_Kts;			        // Kts, tolerance for velocity  - horizontal

	message_aircraft_state_t() :
		gps_time(0.0),
		lat(0.0),
		lon(0.0),
		alt_AGL(0.0),
		alt_MSL(0.0),
		hdg_Deg(0.0),
		vel_Kts(0.0)
	{}

	message_aircraft_state_t(
		double  gps_time,
		double  lat,
		double  lon,
		float   alt_AGL,
		float   alt_MSL,
		float   hdg_Deg,
		double  vel_Kts
	)
		: gps_time(gps_time),
		lat(lat),
		lon(lon),
		alt_AGL(alt_AGL),
		alt_MSL(alt_MSL),
		hdg_Deg(hdg_Deg),
		vel_Kts(vel_Kts) {}
};

/***************************************************/
/************* Output stream ************************/
/***************************************************/

ostream& operator<<(ostream& os, const message_aircraft_state_t& msg) {
    os  << std::fixed << std::setprecision(2)
        << msg.gps_time << " "
        << std::fixed << std::setprecision(7)
        << msg.lat << " "
        << msg.lon << " "
        << std::fixed << std::setprecision(2)
        << msg.alt_AGL << " "
        << msg.alt_MSL << " "
        << msg.hdg_Deg << " "
        << msg.vel_Kts;
	return os;
}

/***************************************************/
/************* Input stream ************************/
/***************************************************/

istream& operator>> (istream& is, message_aircraft_state_t& msg) {
	is >> msg.gps_time;
	is >> msg.lat;
	is >> msg.lon;
	is >> msg.alt_AGL;
	is >> msg.alt_MSL;
	is >> msg.hdg_Deg;
	is >> msg.vel_Kts;
	return is;
}

#endif // AIRCRAFT_STATE_MESSAGE_HPP
