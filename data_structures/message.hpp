#ifndef BOOST_SIMULATION_MESSAGE_HPP
#define BOOST_SIMULATION_MESSAGE_HPP

#include <assert.h>
#include <iostream>
#include <string>
#include <math.h>

using namespace std;

/*******************************************/
/**************** Message_t ****************/
/*******************************************/
struct Message_t{
  Message_t(){}
  Message_t(int i_id, double i_lat, double i_lon, double i_alt)
   :id(i_id), lat(i_lat), lon(i_lon), alt(i_alt){}

  	int		id;
  	double	lat;
	double	lon;
	double	alt;

	double separation(Message_t i_lp) {
		//Radius of the earth in meters.
		float R = 6371000;

		double my_x = R * cos(lat) * cos(lon);
		double my_y = R * cos(lat) * sin(lon);
		double my_z = R * sin(lat);

		double i_x = R * cos(i_lp.lat) * cos(i_lp.lon);
		double i_y = R * cos(i_lp.lat) * sin(i_lp.lon);
		double i_z = R * sin(i_lp.lat);

		return sqrt(pow((i_x - my_x), 2) + pow((i_y - my_y), 2) + pow((i_z - my_z), 2))
	};
};

istream& operator>> (istream& is, Message_t& msg);

ostream& operator<<(ostream& os, const Message_t& msg);


#endif // BOOST_SIMULATION_MESSAGE_HPP