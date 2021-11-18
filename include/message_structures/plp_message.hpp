#ifndef BOOST_SIMULATION_PLP_MESSAGE_HPP
#define BOOST_SIMULATION_PLP_MESSAGE_HPP

#include <assert.h>
#include <iostream>
#include <string>
#include <math.h>
#include <fstream>

using namespace std;

/*******************************************/
/**************** Message_t ****************/
/*******************************************/
struct PLPMessage_t{
  PLPMessage_t(){}
  PLPMessage_t(int i_id, double i_lat, double i_lon, double i_alt)
   :id(i_id), lat(i_lat), lon(i_lon), alt(i_alt){}

  	int		id;
  	double	lat;
	double	lon;
	double	alt;

};

/***************************************************/
/************* Output stream ************************/
/***************************************************/

ostream& operator<<(ostream& os, const PLPMessage_t& msg) {
	os << msg.id << " " << msg.lat << " " << msg.lon << " " << msg.alt;
	return os;
}

/***************************************************/
/************* Input stream ************************/
/***************************************************/

istream& operator>> (istream& is, PLPMessage_t& msg) {
	is >> msg.id;
	is >> msg.lat;
	is >> msg.lon;
	is >> msg.alt;
	return is;
}

#endif // BOOST_SIMULATION_PLP_MESSAGE_HPP