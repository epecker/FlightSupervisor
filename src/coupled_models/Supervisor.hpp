/**
 * 	\file		Supervisor.hpp
 *	\brief		Definition of the Supervisor coupled model.
 *	\details	This header file defines the Supervisor coupled model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor.
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef SUPERVISOR_HPP
#define SUPERVISOR_HPP

// Messages structures
#include "../message_structures/message_aircraft_state_t.hpp"
#include "../message_structures/message_boss_mission_update_t.hpp"
#include "../message_structures/message_fcc_command_t.hpp"
#include "../message_structures/message_landing_point_t.hpp"
#include "../message_structures/message_start_supervisor_t.hpp"
#include "../message_structures/message_update_gcs_t.hpp"

// Coupled model headers
#include "Takeoff.hpp"
#include "On_Route.hpp"
#include "Landing.hpp"

// Cadmium Simulator Headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>

// Time Class Header
#include <NDTime.hpp>

/**
 *	\brief	For definition of the input and output ports see:
 *	\ref 	Supervisor_input_ports "Input Ports" and
 *	\ref 	Supervisor_output_ports "Output Ports"
 * 	\note 	All input and output ports must be listed in this struct.
 */
struct Supervisor_defs {
	/***** Define input port for coupled models *****/
	struct i_aircraft_state : public cadmium::in_port<message_aircraft_state_t> {};
	struct i_landing_achieved : public cadmium::in_port<bool> {};
	struct i_LP_recv : public cadmium::in_port<message_landing_point_t> {};
	struct i_perception_status : public cadmium::in_port<bool> {};
	struct i_pilot_takeover : public cadmium::in_port<bool> {};
	struct i_PLP_ach : public cadmium::in_port<message_landing_point_t> {};
	struct i_start_supervisor : public cadmium::in_port<message_start_supervisor_t> {};
	struct i_waypoint : public cadmium::in_port<message_fcc_command_t> {};

	/***** Define output ports for coupled model *****/
	struct o_control_yielded : public cadmium::out_port<bool> {};
	struct o_fcc_command_hover : public cadmium::out_port<message_fcc_command_t> {};
	struct o_fcc_command_land : public cadmium::out_port<message_fcc_command_t> {};
	struct o_fcc_command_orbit : public cadmium::out_port<message_fcc_command_t> {};
	struct o_fcc_command_velocity : public cadmium::out_port<message_fcc_command_t> {};
	struct o_fcc_waypoint_update : public cadmium::out_port<message_fcc_command_t> {};
	struct o_LP_expired : public cadmium::out_port<message_landing_point_t> {};
	struct o_LP_new : public cadmium::out_port<message_landing_point_t> {};
	struct o_mission_complete : public cadmium::out_port<bool> {};
	struct o_notify_pilot : public cadmium::out_port<bool> {};
	struct o_request_aircraft_state : public cadmium::out_port<bool> {};
	struct o_set_mission_monitor_status : public cadmium::out_port<uint8_t> {};
	struct o_start_mission : public cadmium::out_port<int> {};
	struct o_update_boss : public cadmium::out_port<message_boss_mission_update_t> {};
	struct o_update_gcs : public cadmium::out_port<message_update_gcs_t> {};
	struct o_update_mission_item : public cadmium::out_port<bool> {};
};

/**
 * 	\class		Supervisor
 *	\brief		Definition of the Supervisor coupled model.
 *	\details	This class defines the Supervisor coupled model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor.
 */
class Supervisor {
	using TIME = NDTime;

public:
	// Instantiate the Coupled models.
	Takeoff takeoff_instance = Takeoff();
	On_Route on_route_instance = On_Route();
	Landing landing_instance = Landing();

	std::shared_ptr<cadmium::dynamic::modeling::coupled<TIME>> takeoff = std::make_shared<cadmium::dynamic::modeling::coupled<TIME>>("takeoff", takeoff_instance.submodels, takeoff_instance.iports, takeoff_instance.oports, takeoff_instance.eics, takeoff_instance.eocs, takeoff_instance.ics);
	std::shared_ptr<cadmium::dynamic::modeling::coupled<TIME>> on_route = std::make_shared<cadmium::dynamic::modeling::coupled<TIME>>("on_route", on_route_instance.submodels, on_route_instance.iports, on_route_instance.oports, on_route_instance.eics, on_route_instance.eocs, on_route_instance.ics);
	std::shared_ptr<cadmium::dynamic::modeling::coupled<TIME>> landing = std::make_shared<cadmium::dynamic::modeling::coupled<TIME>>("landing", landing_instance.submodels, landing_instance.iports, landing_instance.oports, landing_instance.eics, landing_instance.eocs, landing_instance.ics);

	/**
	 * 	\anchor	Supervisor_input_ports
	 *	\par	Input Ports
	 * 	Definition of the input ports for the model.
	 * 	\param 	i_aircraft_state	Port for receiving the current state of the aircraft.
	 * 	\param 	i_landing_achieved	Port for receiving signal indicating that the aircraft has successfully landed.
	 * 	\param 	i_LP_recv			Port for receiving landing points from the perception system.
	 * 	\param 	i_perception_status	Port for receiving the status of the perception system.
	 * 	\param 	i_pilot_takeover	Port for signal indicating that the pilot has taken control from the supervisor.
	 * 	\param 	i_PLP_ach			Port for receiving signal indicating that the planned landing point has been achieved.
	 * 	\param 	i_start_supervisor	Port for receiving signal indicating the mission has started.
	 * 	\param 	i_waypoint			Port for receiving new waypoints during the on-route phase.
	 */
 	cadmium::dynamic::modeling::Ports iports = {
		typeid(Supervisor_defs::i_aircraft_state),
		typeid(Supervisor_defs::i_landing_achieved),
		typeid(Supervisor_defs::i_LP_recv),
		typeid(Supervisor_defs::i_perception_status),
		typeid(Supervisor_defs::i_pilot_takeover),
		typeid(Supervisor_defs::i_PLP_ach),
		typeid(Supervisor_defs::i_start_supervisor),
		typeid(Supervisor_defs::i_waypoint)
	};

	/**
	 *	\anchor	Supervisor_output_ports
	 * 	\par 	Output Ports
	 * 	Definition of the output ports for the model.
	 * 	\param 	o_control_yielded				Port for sending an acknowledgement that the supervisor has relinquished control of the aircraft.
	 * 	\param 	o_fcc_command_hover				Port for sending hover commands to the FCC.
	 * 	\param 	o_fcc_command_land				Port for sending land commands to the FCC.
	 * 	\param 	o_fcc_command_orbit				Port for orbit commands to the FCC.
	 * 	\param 	o_fcc_command_velocity			Port for sending velocity commands to the FCC.
	 * 	\param 	o_fcc_waypoint_update			Port for sending waypoint commands to the FCC.
	 * 	\param 	o_LP_expired					Port for sending notification that the LP accept timer has expired.
	 * 	\param 	o_LP_new					 	Port for sending new valid landing points.
	 * 	\param 	o_mission_complete				Port for declaring the mission as being complete after landing.
	 * 	\param 	o_notify_pilot					Port for notifying the pilot that they should take control of the aircraft.
	 * 	\param 	o_request_aircraft_state		Port for requesting the current aircraft state.
	 * 	\param 	o_set_mission_monitor_status	Port for telling the mission monitor to stop monitoring mission progress.
	 * 	\param 	o_start_mission					Port for sending a notification that the mission has started.
	 * 	\param 	o_update_boss					Port for sending updates to BOSS.
	 * 	\param 	o_update_gcs					Port for sending updates to the GCS.
	 * 	\param 	o_update_mission_item			Port for updating the mission manager that the last mission item has been reached.
	 */
 	cadmium::dynamic::modeling::Ports oports = {
		typeid(Supervisor_defs::o_control_yielded),
		typeid(Supervisor_defs::o_fcc_command_hover),
		typeid(Supervisor_defs::o_fcc_command_land),
		typeid(Supervisor_defs::o_fcc_command_orbit),
		typeid(Supervisor_defs::o_fcc_command_velocity),
		typeid(Supervisor_defs::o_fcc_waypoint_update),
		typeid(Supervisor_defs::o_LP_expired),
		typeid(Supervisor_defs::o_LP_new),
		typeid(Supervisor_defs::o_mission_complete),
		typeid(Supervisor_defs::o_notify_pilot),
		typeid(Supervisor_defs::o_request_aircraft_state),
		typeid(Supervisor_defs::o_set_mission_monitor_status),
		typeid(Supervisor_defs::o_start_mission),
		typeid(Supervisor_defs::o_update_boss),
		typeid(Supervisor_defs::o_update_gcs),
		typeid(Supervisor_defs::o_update_mission_item)
	};

	/**
	 *	\anchor	Supervisor_submodels
	 * 	\par 	Output Ports
	 * 	Definition of the sub-models that make up the coupled model.
	 * 	\param 	landing		Model for behaviour of the flight in the landing phase.
	 *	\param 	on_route	Model for behaviour of the flight in the on_route phase.
	 *	\param 	takeoff		Model for behaviour of the flight in the takeoff phase.
	 */
 	cadmium::dynamic::modeling::Models submodels = {
		landing,
		on_route,
		takeoff
	};

	/**
	 * 	\par 	External Input Couplings
	 *	Definition of the external to internal couplings for the model.
	 * 	\see 	Supervisor
	 */
 	cadmium::dynamic::modeling::EICs eics = {
		// takeoff
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_aircraft_state, Takeoff::defs::i_aircraft_state>("takeoff"),
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_perception_status, Takeoff::defs::i_perception_status>("takeoff"),
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_start_supervisor, Takeoff::defs::i_start_supervisor>("takeoff"),

		// on_route
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_waypoint, On_Route_defs::i_waypoint>("on_route"),
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_pilot_takeover, On_Route_defs::i_pilot_takeover>("on_route"),

		// landing
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_aircraft_state, Landing::defs::i_aircraft_state>("landing"),
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_landing_achieved, Landing::defs::i_landing_achieved>("landing"),
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_LP_recv, Landing::defs::i_LP_recv>("landing"),
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_pilot_takeover, Landing::defs::i_pilot_takeover>("landing"),
		cadmium::dynamic::translate::make_EIC<Supervisor_defs::i_PLP_ach, Landing::defs::i_PLP_ach>("landing")
	};

	/**
	 * 	\par 	External Output Couplings
	 *	Definition of the internal to external couplings for the model.
	 * 	\see 	Supervisor
	 */
 	cadmium::dynamic::modeling::EOCs eocs = {
		// takeoff
        cadmium::dynamic::translate::make_EOC<Takeoff::defs::o_request_aircraft_state, Supervisor_defs::o_request_aircraft_state>("takeoff"),
        cadmium::dynamic::translate::make_EOC<Takeoff::defs::o_set_mission_monitor_status, Supervisor_defs::o_set_mission_monitor_status>("takeoff"),
        cadmium::dynamic::translate::make_EOC<Takeoff::defs::o_update_gcs, Supervisor_defs::o_update_gcs>("takeoff"),
        cadmium::dynamic::translate::make_EOC<Takeoff::defs::o_start_mission, Supervisor_defs::o_start_mission>("takeoff"),

		// on_route
		cadmium::dynamic::translate::make_EOC<On_Route_defs::o_fcc_waypoint_update, Supervisor_defs::o_fcc_waypoint_update>("on_route"),

		// landing
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

	/**
	 * 	\par 	Internal Couplings
	 * 	Definition of the internal to internal couplings for the model.
	 * 	\see 	Supervisor
	 */
 	cadmium::dynamic::modeling::ICs ics = {
		// on_route
		cadmium::dynamic::translate::make_IC<Takeoff::defs::o_start_mission, On_Route_defs::i_start_mission>("takeoff", "on_route"),

		// landing
		cadmium::dynamic::translate::make_IC<Takeoff::defs::o_start_mission, Landing::defs::i_start_mission>("takeoff", "landing")
	};
};

#endif // SUPERVISOR_HPP
