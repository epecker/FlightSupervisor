#include <math.h> 
#include <assert.h>
#include <iostream>
#include <fstream>
#include <string>

#include "hover_criteria_message.hpp"

/***************************************************/
/************* Output stream ************************/
/***************************************************/

ostream& operator<<(ostream& os, const HoverCriteriaMessage_t& msg) {
	os	<< msg.desiredLat		<< " " 
		<< msg.desiredLon		<< " " 
		<< msg.desiredAltMSL	<< " " 
		<< msg.desiredHdgDeg;
	return os;
}

/***************************************************/
/************* Input stream ************************/
/***************************************************/

istream& operator>> (istream& is, HoverCriteriaMessage_t& msg) {
	is >> msg.desiredLat;
	is >> msg.desiredLon;
	is >> msg.desiredAltMSL;
	is >> msg.desiredHdgDeg;
	return is;
}


