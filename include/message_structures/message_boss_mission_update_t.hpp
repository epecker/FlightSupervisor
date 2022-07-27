#ifndef MESSAGE_BOSS_MISSION_UPDATE_T_HPP
#define MESSAGE_BOSS_MISSION_UPDATE_T_HPP

#include <iostream>
#include "Constants.hpp"

/*******************************************/
/**************** Message_t ****************/
/*******************************************/
struct message_boss_mission_update_t {
	int lpNo; // What landing point we are on within the current mission (if 0 means no LP data yet)
	double lpLat; // lat lon coordinates of the LP
	double lpLon;
	int missionNo; // The number of the current mission we are running...saved in mavNrc processMissionItemInt()
	int missionItemNo;
	int isMissionStarted; // 1 = yes, 0 = no
	int isLandingLeg; // 1 = yes, true, 0 = not the landing leg
	double lat;  // WGS 84 Latitude of the Item
	double lon;  // WGS 84 longitude of the item
	float  alt;  // MSL Alt of the item in meters
	float  yaw;  // Yaw in deg
	float  speed; // Speed in m/s
	float  horzAcceptRadiusM; // horizontal Acceptance radius in meters
	float  vertAcceptRadiusM; // vertical acceptance radius in meters
	int	previewLength; // the actual # of waypoints in the preview (must be less than WPT_PREVIEW_LENGTH
	double latNext[WPT_PREVIEW_LENGTH]; // latitude of next (up to 3) waypoints for path drawing
	double lonNext[WPT_PREVIEW_LENGTH]; // longitude of next (up to 3) waypoints
	char description[10]; // Text description of the item (leave all text to the end

	message_boss_mission_update_t() :
		lpNo(0),
		lpLat(0),
		lpLon(0),
		missionNo(0),
		missionItemNo(0),
		isMissionStarted(0),
		isLandingLeg(0),
		lat(0),
		lon(0),
		alt(0),
		yaw(0),
		speed(0),
		horzAcceptRadiusM(0),
		vertAcceptRadiusM(0),
		previewLength(0),
		description("")
		{
			for (int i = 0; i < WPT_PREVIEW_LENGTH; i++) {
				latNext[i] = 0;
				lonNext[i] = 0;
			}
		}

	message_boss_mission_update_t(
		int i_lpNo,
		double i_lpLat,
		double i_lpLon,
		int i_missionNo,
		int i_missionItemNo,
		int i_isMissionStarted,
		int i_isLandingLeg,
		double i_lat,
		double i_lon,
		float i_alt,
		float i_yaw,
		float i_speed,
		float i_horzAcceptRadiusM,
		float i_vertAcceptRadiusM,
		int	i_previewLength,
		double i_latNext[WPT_PREVIEW_LENGTH],
		double i_lonNext[WPT_PREVIEW_LENGTH],
		char i_description[10]
	) : lpNo(i_lpNo),
		lpLat(i_lpLat),
		lpLon(i_lpLon),
		missionNo(i_missionNo),
		missionItemNo(i_missionItemNo),
		isMissionStarted(i_isMissionStarted),
		isLandingLeg(i_isLandingLeg),
		lat(i_lat),
		lon(i_lon),
		alt(i_alt),
		yaw(i_yaw),
		speed(i_speed),
		horzAcceptRadiusM(i_horzAcceptRadiusM),
		vertAcceptRadiusM(i_vertAcceptRadiusM),
		previewLength(i_previewLength)		
	{
		for (int i = 0; i < WPT_PREVIEW_LENGTH; i++) {
			latNext[i] = i_latNext[i];
			lonNext[i] = i_lonNext[i];
		}
		strcpy(description, i_description);
	}
};

/***************************************************/
/************* Output stream ************************/
/***************************************************/

ostream &operator<<(ostream &os, const message_boss_mission_update_t &msg) {
	os << msg.lpNo << " "
		<< msg.lpLat << " "
		<< msg.lpLon << " "
		<< msg.missionNo << " "
		<< msg.missionItemNo << " "
		<< msg.isMissionStarted << " "
		<< msg.isLandingLeg << " "
		<< msg.lat << " "
		<< msg.lon << " "
		<< msg.alt << " "
		<< msg.yaw << " "
		<< msg.speed << " "
		<< msg.horzAcceptRadiusM << " "
		<< msg.vertAcceptRadiusM << " "
		<< msg.previewLength << " ";
	for (int i = 0; i < WPT_PREVIEW_LENGTH; i++) {
		os << msg.latNext[i] << " "
			<< msg.lonNext[i] << " ";
	}
	os << msg.description;
	return os;
}

/***************************************************/
/************* Input stream ************************/
/***************************************************/

istream &operator>>(istream &is, message_boss_mission_update_t &msg) {
	is >> msg.lpNo;
	is >> msg.lpLat;
	is >> msg.lpLon;
	is >> msg.missionNo;
	is >> msg.missionItemNo;
	is >> msg.isMissionStarted;
	is >> msg.isLandingLeg;
	is >> msg.lat;
	is >> msg.lon;
	is >> msg.alt;
	is >> msg.yaw;
	is >> msg.speed;
	is >> msg.horzAcceptRadiusM;
	is >> msg.vertAcceptRadiusM;
	is >> msg.previewLength;
	for (int i = 0; i < WPT_PREVIEW_LENGTH; i++) {
		is >> msg.latNext[i];
		is >> msg.lonNext[i];
	}
	is >> msg.description;
	return is;
}

#endif // MESSAGE_BOSS_MISSION_UPDATE_T_HPP
