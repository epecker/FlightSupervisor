/**
 *	\brief		A coupled model representing the total of the Supervisor behaviour.
 *	\details	This header file define the Supervisor model as a coupled model for use
 				in the Cadmium DEVS simulation software.
 *	\author		James Horner
 */

#ifndef SUPERVISOR_HPP
#define SUPERVISOR_HPP

//Cadmium Simulator headers
#include "cadmium/modeling/ports.hpp"
#include "cadmium/modeling/dynamic_model_translator.hpp"

//Time class header
#include "NDTime.hpp"

//Messages structures
#include "../message_structures/message_aircraft_state_t.hpp"
#include "../message_structures/message_fcc_command_t.hpp"
#include "../message_structures/message_landing_point_t.hpp"
#include "../message_structures/message_start_supervisor_t.hpp"
#include "../message_structures/message_boss_mission_update_t.hpp"
#include "../message_structures/message_update_gcs_t.hpp"

//Atomic model headers
#include "Takeoff.hpp"
#include "On_Route.hpp"
#include "Landing.hpp"

using TIME = NDTime;

/***** Define input port for coupled models *****/
struct Supervisor_defs {
	/* Inputs ================================================================ */
	/* Takeoff Inputs **********************************************************/
	struct i_aircraft_state : public cadmium::in_port<message_aircraft_state_t> {};
	struct i_perception_status : public cadmium::in_port<bool> {};
	struct i_start_supervisor : public cadmium::in_port<message_start_supervisor_t> {};

	/* On Route Inputs *********************************************************/
	struct i_waypoint : public cadmium::in_port<message_fcc_command_t> {};

	/* Landing Inputs **********************************************************/
	struct i_landing_achieved : public cadmium::in_port<bool> {};
	struct i_LP_recv : public cadmium::in_port<message_landing_point_t> {};
	struct i_pilot_takeover : public cadmium::in_port<bool> {};
	struct i_PLP_ach : public cadmium::in_port<message_landing_point_t> {};

	/* Outputs =============================================================== */
	/* Takeoff Outputs *********************************************************/
	struct o_request_aircraft_state : public cadmium::out_port<bool> {};
	struct o_set_mission_monitor_status : public cadmium::out_port<uint8_t> {};
	struct o_start_mission : public cadmium::out_port<int> {};
	struct o_update_gcs : public cadmium::out_port<message_update_gcs_t> {};

	/* On Route Outputs ********************************************************/
	struct o_fcc_waypoint_update : public cadmium::out_port<message_fcc_command_t> {};

	/* Landing Outputs ********************************************************/
	struct o_control_yielded : public cadmium::out_port<bool> {};
	struct o_fcc_command_hover : public cadmium::out_port<message_fcc_command_t> {};
	struct o_fcc_command_land : public cadmium::out_port<message_fcc_command_t> {};
	struct o_fcc_command_orbit : public cadmium::out_port<message_fcc_command_t> {};
	struct o_fcc_command_velocity : public cadmium::out_port<message_fcc_command_t> {};
	struct o_LP_expired : public cadmium::out_port<message_landing_point_t> {};
	struct o_LP_new : public cadmium::out_port<message_landing_point_t> {};
	struct o_mission_complete : public cadmium::out_port<bool> {};
	struct o_notify_pilot : public cadmium::out_port<bool> {};
	struct o_update_boss : public cadmium::out_port<message_boss_mission_update_t> {};
	struct o_update_mission_item : public cadmium::out_port<bool> {};
};

class Supervisor {
public:
	/**
	* Instantiate the coupled models.
	*/
	Takeoff takeoff_instance = Takeoff();
	On_Route on_route_instance = On_Route();
	Landing landing_instance = Landing();
	std::shared_ptr<cadmium::dynamic::modeling::coupled<TIME>> takeoff = std::make_shared<cadmium::dynamic::modeling::coupled<TIME>>("takeoff", takeoff_instance.submodels, takeoff_instance.iports, takeoff_instance.oports, takeoff_instance.eics, takeoff_instance.eocs, takeoff_instance.ics);
	std::shared_ptr<cadmium::dynamic::modeling::coupled<TIME>> on_route = std::make_shared<cadmium::dynamic::modeling::coupled<TIME>>("on_route", on_route_instance.submodels, on_route_instance.iports, on_route_instance.oports, on_route_instance.eics, on_route_instance.eocs, on_route_instance.ics);
	std::shared_ptr<cadmium::dynamic::modeling::coupled<TIME>> landing = std::make_shared<cadmium::dynamic::modeling::coupled<TIME>>("landing", landing_instance.submodels, landing_instance.iports, landing_instance.oports, landing_instance.eics, landing_instance.eocs, landing_instance.ics);

	//Define the inputs takeoff_instance the Landing Point Reposition coupled model.
 	cadmium::dynamic::modeling::Ports iports = {
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
 	cadmium::dynamic::modeling::Ports oports = {
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
 	cadmium::dynamic::modeling::Models submodels = {
		takeoff,
		on_route,
		landing
	};

	//Define the external takeoff_instance internal couplings.
 	cadmium::dynamic::modeling::EICs eics = {
		/* Takeoff Inputs **********************************************************/
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_aircraft_state, Takeoff::defs::i_aircraft_state>("takeoff"),
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_perception_status, Takeoff::defs::i_perception_status>("takeoff"),
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_start_supervisor, Takeoff::defs::i_start_supervisor>("takeoff"),

		/* On Route Inputs *********************************************************/
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_waypoint, On_Route_defs::i_waypoint>("on_route"),
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_pilot_takeover, On_Route_defs::i_pilot_takeover>("on_route"),

		/* Landing Inputs **********************************************************/
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_aircraft_state, Landing::defs::i_aircraft_state>("landing"),
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_landing_achieved, Landing::defs::i_landing_achieved>("landing"),
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_LP_recv, Landing::defs::i_LP_recv>("landing"),
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_pilot_takeover, Landing::defs::i_pilot_takeover>("landing"),
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_PLP_ach, Landing::defs::i_PLP_ach>("landing")
	};

	//Define the internal takeoff_instance external couplings.
 	cadmium::dynamic::modeling::EOCs eocs = {
		/* Takeoff Outputs *********************************************************/
        cadmium::dynamic::translate::make_EOC<Takeoff::defs::o_request_aircraft_state, Supervisor_defs::o_request_aircraft_state>("takeoff"),
        cadmium::dynamic::translate::make_EOC<Takeoff::defs::o_set_mission_monitor_status, Supervisor_defs::o_set_mission_monitor_status>("takeoff"),
        cadmium::dynamic::translate::make_EOC<Takeoff::defs::o_update_gcs, Supervisor_defs::o_update_gcs>("takeoff"),
        cadmium::dynamic::translate::make_EOC<Takeoff::defs::o_start_mission, Supervisor_defs::o_start_mission>("takeoff"),

		/* On Route Outputs ********************************************************/
		cadmium::dynamic::translate::make_EOC<On_Route_defs::o_fcc_waypoint_update, Supervisor_defs::o_fcc_waypoint_update>("on_route"),

		/* Landing Outputs *********************************************************/
		cadmium::dynamic::translate::make_EOC<Landing::defs::o_request_aircraft_state, Supervisor_defs::o_request_aircraft_state>("landing"),
		cadmium::dynamic::translate::make_EOC<Landing::defs::o_control_yielded, Supervisor_defs::o_control_yielded>("landing"),
		cadmium::dynamic::translate::make_EOC<Landing::defs::o_fcc_command_hover, Supervisor_defs::o_fcc_command_hover>("landing"),
		cadmium::dynamic::translate::make_EOC<Landing::defs::o_fcc_command_land, Supervisor_defs::o_fcc_command_land>("landing"),
		cadmium::dynamic::translate::make_EOC<Landing::defs::o_fcc_command_orbit, Supervisor_defs::o_fcc_command_orbit>("landing"),
		cadmium::dynamic::translate::make_EOC<Landing::defs::o_fcc_command_velocity, Supervisor_defs::o_fcc_command_velocity>("landing"),
		cadmium::dynamic::translate::make_EOC<Landing::defs::o_LP_expired, Supervisor_defs::o_LP_expired>("landing"),
		cadmium::dynamic::translate::make_EOC<Landing::defs::o_LP_new, Supervisor_defs::o_LP_new>("landing"),
		cadmium::dynamic::translate::make_EOC<Landing::defs::o_mission_complete, Supervisor_defs::o_mission_complete>("landing"),
		cadmium::dynamic::translate::make_EOC<Landing::defs::o_notify_pilot, Supervisor_defs::o_notify_pilot>("landing"),
		cadmium::dynamic::translate::make_EOC<Landing::defs::o_update_boss, Supervisor_defs::o_update_boss>("landing"),
		cadmium::dynamic::translate::make_EOC<Landing::defs::o_update_gcs, Supervisor_defs::o_update_gcs>("landing"),
		cadmium::dynamic::translate::make_EOC<Landing::defs::o_set_mission_monitor_status, Supervisor_defs::o_set_mission_monitor_status>("landing"),
		cadmium::dynamic::translate::make_EOC<Landing::defs::o_update_mission_item, Supervisor_defs::o_update_mission_item>("landing")
	};

	//Define the internal takeoff_instance internal couplings.
 	cadmium::dynamic::modeling::ICs ics = {
		cadmium::dynamic::translate::make_IC<Takeoff::defs::o_start_mission, On_Route_defs::i_start_mission>("takeoff", "on_route"),
		cadmium::dynamic::translate::make_IC<Takeoff::defs::o_start_mission, Landing::defs::i_start_mission>("takeoff", "landing")
	};
};

#endif // SUPERVISOR_HPP
