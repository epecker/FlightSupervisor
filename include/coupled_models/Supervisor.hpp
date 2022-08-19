/**
 *	\brief		A coupled model representing the total of the Supervisor behaviour.
 *	\details	This header file define the Supervisor model as a coupled model for use
 				in the Cadmium DEVS simulation software.
 *	\author		James Horner
 */

#ifndef SUPERVISOR_HPP
#define SUPERVISOR_HPP

//Cadmium Simulator headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>

//Time class header
#include <NDTime.hpp>

//Messages structures
#include "message_structures/message_aircraft_state_t.hpp"
#include "message_structures/message_fcc_command_t.hpp"
#include "message_structures/message_landing_point_t.hpp"
#include "message_structures/message_start_supervisor_t.hpp"
#include "message_structures/message_boss_mission_update_t.hpp"
#include "message_structures/message_update_gcs_t.hpp"

//Atomic model headers
#include "coupled_models/Takeoff.hpp"
#include "coupled_models/On_Route.hpp"
#include "coupled_models/Landing.hpp"

using namespace cadmium;

using TIME = NDTime;

/***** Define input port for coupled models *****/
struct Supervisor_defs {
	/* Inputs ================================================================ */
	/* Takeoff Inputs **********************************************************/
	struct i_aircraft_state : public in_port<message_aircraft_state_t> {};
	struct i_perception_status : public in_port<bool> {};
	struct i_start_supervisor : public in_port<message_start_supervisor_t> {};

	/* On Route Inputs *********************************************************/
	struct i_waypoint : public in_port<message_fcc_command_t> {};

	/* Landing Inputs **********************************************************/
	struct i_landing_achieved : public in_port<bool> {};
	struct i_LP_recv : public in_port<message_landing_point_t> {};
	struct i_pilot_takeover : public in_port<bool> {};
	struct i_PLP_ach : public in_port<message_landing_point_t> {};

	/* Outputs =============================================================== */
	/* Takeoff Outputs *********************************************************/
	struct o_request_aircraft_state : public out_port<bool> {};
	struct o_set_mission_monitor_status : public out_port<uint8_t> {};
	struct o_start_mission : public out_port<int> {};
	struct o_update_gcs : public out_port<message_update_gcs_t> {};

	/* On Route Outputs ********************************************************/
	struct o_fcc_waypoint_update : public out_port<message_fcc_command_t> {};

	/* Landing Outputs ********************************************************/
	struct o_control_yielded : public out_port<bool> {};
	struct o_fcc_command_hover : public out_port<message_fcc_command_t> {};
	struct o_fcc_command_land : public out_port<message_fcc_command_t> {};
	struct o_fcc_command_orbit : public out_port<message_fcc_command_t> {};
	struct o_fcc_command_velocity : public out_port<message_fcc_command_t> {};
	struct o_LP_expired : public out_port<message_landing_point_t> {};
	struct o_LP_new : public out_port<message_landing_point_t> {};
	struct o_mission_complete : public out_port<bool> {};
	struct o_notify_pilot : public out_port<bool> {};
	struct o_update_boss : public out_port<message_boss_mission_update_t> {};
	struct o_update_mission_item : public out_port<bool> {};
};

class Supervisor {
public:
	/**
	* Instantiate the coupled models.
	*/
	Takeoff takeoff_instance = Takeoff();
	On_Route on_route_instance = On_Route();
	Landing landing_instance = Landing();
	shared_ptr<dynamic::modeling::coupled<TIME>> takeoff = make_shared<dynamic::modeling::coupled<TIME>>("takeoff", takeoff_instance.submodels, takeoff_instance.iports, takeoff_instance.oports, takeoff_instance.eics, takeoff_instance.eocs, takeoff_instance.ics);
	shared_ptr<dynamic::modeling::coupled<TIME>> on_route = make_shared<dynamic::modeling::coupled<TIME>>("on_route", on_route_instance.submodels, on_route_instance.iports, on_route_instance.oports, on_route_instance.eics, on_route_instance.eocs, on_route_instance.ics);
	shared_ptr<dynamic::modeling::coupled<TIME>> landing = make_shared<dynamic::modeling::coupled<TIME>>("landing", landing_instance.submodels, landing_instance.iports, landing_instance.oports, landing_instance.eics, landing_instance.eocs, landing_instance.ics);

	//Define the inputs takeoff_instance the Landing Point Reposition coupled model.
	dynamic::modeling::Ports iports = {
		typeid(Supervisor_defs::i_aircraft_state),
		typeid(Supervisor_defs::i_perception_status),
		typeid(Supervisor_defs::i_start_supervisor),

		typeid(Supervisor_defs::i_waypoint),

		typeid(Supervisor_defs::i_landing_achieved),
		typeid(Supervisor_defs::i_LP_recv),
		typeid(Supervisor_defs::i_pilot_takeover),
		typeid(Supervisor_defs::i_PLP_ach)
	};

	//Define the outputs of the Landing Point Reposition coupled model.
	dynamic::modeling::Ports oports = {
		typeid(Supervisor_defs::o_request_aircraft_state),
		typeid(Supervisor_defs::o_set_mission_monitor_status),
		typeid(Supervisor_defs::o_start_mission),
		typeid(Supervisor_defs::o_update_gcs),

		typeid(Supervisor_defs::o_fcc_waypoint_update),

		typeid(Supervisor_defs::o_control_yielded),
		typeid(Supervisor_defs::o_fcc_command_hover),
		typeid(Supervisor_defs::o_fcc_command_land),
		typeid(Supervisor_defs::o_fcc_command_orbit),
		typeid(Supervisor_defs::o_fcc_command_velocity),
		typeid(Supervisor_defs::o_LP_expired),
		typeid(Supervisor_defs::o_LP_new),
		typeid(Supervisor_defs::o_mission_complete),
		typeid(Supervisor_defs::o_notify_pilot),
		typeid(Supervisor_defs::o_update_boss),
		typeid(Supervisor_defs::o_update_mission_item)
	};

	//Define the sub-models that make up the Landing Point Reposition coupled model.
	dynamic::modeling::Models submodels = {
		takeoff,
		on_route,
		landing
	};

	//Define the external takeoff_instance internal couplings.
	dynamic::modeling::EICs eics = {
		/* Takeoff Inputs **********************************************************/
		dynamic::translate::make_EIC<Supervisor_defs::i_aircraft_state, Takeoff::defs::i_aircraft_state>("takeoff"),
		dynamic::translate::make_EIC<Supervisor_defs::i_perception_status, Takeoff::defs::i_perception_status>("takeoff"),
		dynamic::translate::make_EIC<Supervisor_defs::i_start_supervisor, Takeoff::defs::i_start_supervisor>("takeoff"),

		/* On Route Inputs *********************************************************/
		dynamic::translate::make_EIC<Supervisor_defs::i_waypoint, On_Route_defs::i_waypoint>("on_route"),
		dynamic::translate::make_EIC<Supervisor_defs::i_pilot_takeover, On_Route_defs::i_pilot_takeover>("on_route"),

		/* Landing Inputs **********************************************************/
		dynamic::translate::make_EIC<Supervisor_defs::i_aircraft_state, Landing::defs::i_aircraft_state>("landing"),
		dynamic::translate::make_EIC<Supervisor_defs::i_landing_achieved, Landing::defs::i_landing_achieved>("landing"),
		dynamic::translate::make_EIC<Supervisor_defs::i_LP_recv, Landing::defs::i_LP_recv>("landing"),
		dynamic::translate::make_EIC<Supervisor_defs::i_pilot_takeover, Landing::defs::i_pilot_takeover>("landing"),
		dynamic::translate::make_EIC<Supervisor_defs::i_PLP_ach, Landing::defs::i_PLP_ach>("landing")
	};

	//Define the internal takeoff_instance external couplings.
	dynamic::modeling::EOCs eocs = {
		/* Takeoff Outputs *********************************************************/
        dynamic::translate::make_EOC<Takeoff::defs::o_request_aircraft_state, Supervisor_defs::o_request_aircraft_state>("takeoff"),
        dynamic::translate::make_EOC<Takeoff::defs::o_set_mission_monitor_status, Supervisor_defs::o_set_mission_monitor_status>("takeoff"),
        dynamic::translate::make_EOC<Takeoff::defs::o_update_gcs, Supervisor_defs::o_update_gcs>("takeoff"),
        dynamic::translate::make_EOC<Takeoff::defs::o_start_mission, Supervisor_defs::o_start_mission>("takeoff"),

		/* On Route Outputs ********************************************************/
		dynamic::translate::make_EOC<On_Route_defs::o_fcc_waypoint_update, Supervisor_defs::o_fcc_waypoint_update>("on_route"),

		/* Landing Outputs *********************************************************/
		dynamic::translate::make_EOC<Landing::defs::o_request_aircraft_state, Supervisor_defs::o_request_aircraft_state>("landing"),
		dynamic::translate::make_EOC<Landing::defs::o_control_yielded, Supervisor_defs::o_control_yielded>("landing"),
		dynamic::translate::make_EOC<Landing::defs::o_fcc_command_hover, Supervisor_defs::o_fcc_command_hover>("landing"),
		dynamic::translate::make_EOC<Landing::defs::o_fcc_command_land, Supervisor_defs::o_fcc_command_land>("landing"),
		dynamic::translate::make_EOC<Landing::defs::o_fcc_command_orbit, Supervisor_defs::o_fcc_command_orbit>("landing"),
		dynamic::translate::make_EOC<Landing::defs::o_fcc_command_velocity, Supervisor_defs::o_fcc_command_velocity>("landing"),
		dynamic::translate::make_EOC<Landing::defs::o_LP_expired, Supervisor_defs::o_LP_expired>("landing"),
		dynamic::translate::make_EOC<Landing::defs::o_LP_new, Supervisor_defs::o_LP_new>("landing"),
		dynamic::translate::make_EOC<Landing::defs::o_mission_complete, Supervisor_defs::o_mission_complete>("landing"),
		dynamic::translate::make_EOC<Landing::defs::o_notify_pilot, Supervisor_defs::o_notify_pilot>("landing"),
		dynamic::translate::make_EOC<Landing::defs::o_update_boss, Supervisor_defs::o_update_boss>("landing"),
		dynamic::translate::make_EOC<Landing::defs::o_update_gcs, Supervisor_defs::o_update_gcs>("landing"),
		dynamic::translate::make_EOC<Landing::defs::o_set_mission_monitor_status, Supervisor_defs::o_set_mission_monitor_status>("landing"),
		dynamic::translate::make_EOC<Landing::defs::o_update_mission_item, Supervisor_defs::o_update_mission_item>("landing")
	};

	//Define the internal takeoff_instance internal couplings.
	dynamic::modeling::ICs ics = {
		dynamic::translate::make_IC<Takeoff::defs::o_start_mission, On_Route_defs::i_start_mission>("takeoff", "on_route"),
		dynamic::translate::make_IC<Takeoff::defs::o_start_mission, Landing::defs::i_start_mission>("takeoff", "landing")
	};
};

#endif // SUPERVISOR_HPP
