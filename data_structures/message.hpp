#ifndef BOOST_SIMULATION_MESSAGE_HPP
#define BOOST_SIMULATION_MESSAGE_HPP

#include <iostream>
#include <math.h>

using namespace std;

/*******************************************/
/**************** Message_t ****************/
/*******************************************/
struct Message_t {
	int		id;
	double	lat;
	double	lon;
	double	alt;

	Message_t() : id(0), lat(0.0), lon(0.0), alt(0.0) {}
	Message_t(int i_id, double i_lat, double i_lon, double i_alt) {
		id = i_id;
		lat = i_lat;
		lon = i_lon;
		alt = i_alt;
	}

	double separation(Message_t i_lp) {
		//Radius of the earth in meters.
		float R = 6371000;

		double my_x = R * cos(lat) * cos(lon);
		double my_y = R * cos(lat) * sin(lon);
		double my_z = R * sin(lat);

		double i_x = R * cos(i_lp.lat) * cos(i_lp.lon);
		double i_y = R * cos(i_lp.lat) * sin(i_lp.lon);
		double i_z = R * sin(i_lp.lat);

		return sqrt(pow((i_x - my_x), 2) + pow((i_y - my_y), 2) + pow((i_z - my_z), 2));
	};
};

/***************************************************/
/************* Output stream ************************/
/***************************************************/

ostream& operator<<(ostream& os, const Message_t& msg) {
	os << msg.id << " " << msg.lat << " " << msg.lon << " " << msg.alt;
	return os;
}

/***************************************************/
/************* Input stream ************************/
/***************************************************/

istream& operator>> (istream& is, Message_t& msg) {
	is >> msg.id;
	is >> msg.lat;
	is >> msg.lon;
	is >> msg.alt;
	return is;
}

#endif // BOOST_SIMULATION_MESSAGE_HPP