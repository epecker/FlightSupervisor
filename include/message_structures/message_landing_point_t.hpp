#ifndef BOOST_SIMULATION_PLP_MESSAGE_HPP
#define BOOST_SIMULATION_PLP_MESSAGE_HPP

#include <iostream>

using namespace std;

/*******************************************/
/**************** Message_t ****************/
/*******************************************/
struct message_landing_point_t{
	int		id;
	double	lat;
	double	lon;
	double	alt;

	message_landing_point_t()
		:id(0), lat(0), lon(0), alt(0) {}
	message_landing_point_t(int i_id, double i_lat, double i_lon, double i_alt)
		:id(i_id), lat(i_lat), lon(i_lon), alt(i_alt){}
};

/***************************************************/
/************* Output stream ************************/
/***************************************************/

ostream& operator<<(ostream& os, const message_landing_point_t& msg) {
	os << msg.id << " " << msg.lat << " " << msg.lon << " " << msg.alt;
	return os;
}

/***************************************************/
/************* Input stream ************************/
/***************************************************/

istream& operator>> (istream& is, message_landing_point_t& msg) {
	is >> msg.id;
	is >> msg.lat;
	is >> msg.lon;
	is >> msg.alt;
	return is;
}

#endif // BOOST_SIMULATION_PLP_MESSAGE_HPP