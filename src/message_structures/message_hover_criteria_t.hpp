#ifndef HOVER_CRITERIA_MESSAGE_HPP
#define HOVER_CRITERIA_MESSAGE_HPP

#include <iostream>

/*******************************************/
/**************** Message_t ****************/
/*******************************************/
struct message_hover_criteria_t {
	double  desiredLat;		            // Decimal degrees
	double  desiredLon;		            // Decimal degrees
	float   desiredAltMSL;	            // Ft
	float   desiredHdgDeg;	            // 0360 True hdg
	double  horDistTolFt;		        // Ft, tolerance within meeting criteria horizontally
	double  vertDistTolFt;		        // Ft, tolerance vertically to meet hover criteria
	double  velTolKts;			        // Kts, tolerance for velocity  - horizontal
	double  hdgToleranceDeg;            // Heading tolerance
	double  timeTol;			        // Seconds, time it supposed to hover to say that criteria is met
	double  timeCritFirstMet;           // System time at which criteria was first met, to start counting.
	double  hoverCompleted;             // Flag that hover is completed
	int     manCtrlRequiredAfterCritMet;// Allows supervisor to flag display system that MAN CTRL is needed

	message_hover_criteria_t() :
		desiredLat(0.0),
		desiredLon(0.0),
		desiredAltMSL(0.0),
		desiredHdgDeg(0.0),
		horDistTolFt(0.0),
		vertDistTolFt(0.0),
		velTolKts(0.0),
		hdgToleranceDeg(0.0),
		timeTol(0.0),
		timeCritFirstMet(0.0),
		hoverCompleted(0.0),
		manCtrlRequiredAfterCritMet(0) {}

	message_hover_criteria_t(
		double  i_desiredLat,
		double  i_desiredLon,
		float   i_desiredAltMSL,
		float   i_desiredHdgDeg,
		double  i_horDistTolFt,
		double  i_vertDistTolFt,
		double  i_velTolKts,
		double  i_hdgToleranceDeg,
		double  i_timeTol,
		double  i_timeCritFirstMet,
		double  i_hoverCompleted,
		int     i_manCtrlRequiredAfterCritMet
	)
		: desiredLat(i_desiredLat),
		desiredLon(i_desiredLon),
		desiredAltMSL(i_desiredAltMSL),
		desiredHdgDeg(i_desiredHdgDeg),
		horDistTolFt(i_horDistTolFt),
		vertDistTolFt(i_vertDistTolFt),
		velTolKts(i_velTolKts),
		hdgToleranceDeg(i_hdgToleranceDeg),
		timeTol(i_timeTol),
		timeCritFirstMet(i_timeCritFirstMet),
		hoverCompleted(i_hoverCompleted),
		manCtrlRequiredAfterCritMet(i_manCtrlRequiredAfterCritMet) {}
};

/***************************************************/
/************* Output stream ************************/
/***************************************************/

std::ostream& operator<<(std::ostream& os, const message_hover_criteria_t& msg) {
    os  << std::fixed << std::setprecision(7)
        << msg.desiredLat << " "
        << msg.desiredLon << " "
        << std::fixed << std::setprecision(2)
        << msg.desiredAltMSL << " "
        << msg.desiredHdgDeg << " "
        << msg.horDistTolFt << " "
        << msg.vertDistTolFt << " "
        << msg.velTolKts << " "
        << msg.hdgToleranceDeg << " "
        << msg.timeTol << " "
        << msg.timeCritFirstMet << " "
        << msg.hoverCompleted << " "
        << msg.manCtrlRequiredAfterCritMet << " ";
	return os;
}

/***************************************************/
/************* Input stream ************************/
/***************************************************/

std::istream& operator>> (std::istream& is, message_hover_criteria_t& msg) {
	is >> msg.desiredLat;
	is >> msg.desiredLon;
	is >> msg.desiredAltMSL;
	is >> msg.desiredHdgDeg;
	is >> msg.horDistTolFt;
	is >> msg.vertDistTolFt;
	is >> msg.velTolKts;
	is >> msg.hdgToleranceDeg;
	is >> msg.timeTol;
	is >> msg.timeCritFirstMet;
	is >> msg.hoverCompleted;
	is >> msg.manCtrlRequiredAfterCritMet;
	return is;
}

#endif // HOVER_CRITERIA_MESSAGE_HPP
