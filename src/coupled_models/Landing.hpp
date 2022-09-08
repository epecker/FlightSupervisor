/**
 * 	\file		Landing.hpp
 *	\brief		Definition of the Landing coupled model.
 *	\details	This header file defines the Landing coupled model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor in 
				the landing phase of flight.
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef LANDING_HPP
#define LANDING_HPP

// Coupled model headers
#include "LP_Reposition.hpp"

// Atomic model headers
#include "../atomic_models/LP_Manager.hpp"
#include "../atomic_models/Stabilize.hpp"
#include "../atomic_models/Handover_Control.hpp"

// Messages structures
#include "../message_structures/message_aircraft_state_t.hpp"
#include "../message_structures/message_landing_point_t.hpp"
#include "../message_structures/message_fcc_command_t.hpp"
#include "../message_structures/message_boss_mission_update_t.hpp"
#include "../message_structures/message_update_gcs_t.hpp"

// Utility functions
#include "../time_conversion.hpp"

// Project information headers this is created by cmake at generation time!!!!
#include "../SupervisorConfig.hpp"

// Constants
#include "../Constants.hpp"

// Cadmium Simulator Headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>

// Time Class Header
#include <NDTime.hpp>


/**
 * 	\class		Landing 
 *	\brief		Definition of the Landing coupled model.
 *	\details	This class defines the Landing coupled model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor in 
				the landing phase of flight.
 */
class Landing {
	using TIME = NDTime;

public:
	/**
	 * \struct	defs
	 * \brief 	Declaration of the ports for the model.
	 * \see		iports
	 * \see 	oports
	 */
	struct defs {
		/***** Define input port for coupled models *****/
		struct i_aircraft_state : public in_port<message_aircraft_state_t> {};
		struct i_landing_achieved : public in_port<bool> {};
		struct i_LP_recv : public in_port<message_landing_point_t> {};
		struct i_pilot_takeover : public in_port<bool> {};
		struct i_PLP_ach : public in_port<message_landing_point_t> {};
		struct i_start_mission : public in_port<int> {};


		/***** Define output ports for coupled model *****/
		struct o_control_yielded : public out_port<bool> {};
		struct o_fcc_command_hover : public out_port<message_fcc_command_t> {};
		struct o_fcc_command_land : public out_port<message_fcc_command_t> {};
		struct o_fcc_command_orbit : public out_port<message_fcc_command_t> {};
		struct o_fcc_command_velocity : public out_port<message_fcc_command_t> {};
		struct o_LP_expired : public out_port<message_landing_point_t> {};
		struct o_LP_new : public out_port<message_landing_point_t> {};
		struct o_mission_complete : public out_port<bool> {};
		struct o_notify_pilot : public out_port<bool> {};
		struct o_request_aircraft_state : public out_port<bool> {};
		struct o_set_mission_monitor_status : public out_port<uint8_t> {};
		struct o_update_boss : public out_port<message_boss_mission_update_t> {};
		struct o_update_gcs : public out_port<message_update_gcs_t> {};
		struct o_update_mission_item : public out_port<bool> {};
	};

	// Instantiate the Atomic models.
	shared_ptr<cadmium::dynamic::modeling::model> lp_manager = cadmium::dynamic::translate::make_dynamic_atomic_model<LP_Manager, TIME, TIME, TIME>("lp_manager", seconds_to_time<TIME>(LP_ACCEPT_TIMER), seconds_to_time<TIME>(ORBIT_TIMER));
	shared_ptr<cadmium::dynamic::modeling::model> stabilize = cadmium::dynamic::translate::make_dynamic_atomic_model<Stabilize, TIME>("stabilize");
	shared_ptr<cadmium::dynamic::modeling::model> handover_control = cadmium::dynamic::translate::make_dynamic_atomic_model<Handover_Control, TIME>("handover_control");

	// Instantiate the Coupled models.
	LP_Reposition lpr = LP_Reposition();
	shared_ptr<cadmium::dynamic::modeling::coupled<TIME>> lp_reposition = make_shared<cadmium::dynamic::modeling::coupled<TIME>>("lp_reposition", lpr.submodels, lpr.iports, lpr.oports, lpr.eics, lpr.eocs, lpr.ics);

	/**
	 *	\struct	iports
	 * 	\brief 	Defintion of the input ports for the model.
	 * 	\var 	i_aircraft_state 
	 * 	[input] Port for 
	 * 	\var 	i_landing_achieved 
	 * 	[input] Port for 
	 * 	\var 	i_LP_recv 
	 * 	[input] Port for 
	 * 	\var 	i_pilot_takeover 
	 * 	[input] Port for 
	 * 	\var 	i_PLP_ach 
	 * 	[input] Port for 
	 * 	\var 	i_start_mission 
	 * 	[input] Port for 
	 */
	cadmium::dynamic::modeling::Ports iports = {
		typeid(Landing::defs::i_aircraft_state),
		typeid(Landing::defs::i_landing_achieved),
		typeid(Landing::defs::i_LP_recv),
		typeid(Landing::defs::i_pilot_takeover),
		typeid(Landing::defs::i_PLP_ach),
		typeid(Landing::defs::i_start_mission)
	};

	/**
	 *	\struct	oports
	 * 	\brief 	Defintion of the output ports for the model.
	 * 	\var 	o_control_yielded 
	 * 	[output] Port for
	 * 	\var 	o_fcc_command_hover 
	 * 	[output] Port for
	 * 	\var 	o_fcc_command_land 
	 * 	[output] Port for
	 * 	\var 	o_fcc_command_orbit 
	 * 	[output] Port for
	 * 	\var 	o_fcc_command_velocity 
	 * 	[output] Port for
	 * 	\var 	o_LP_expired 
	 * 	[output] Port for
	 * 	\var 	o_LP_new 
	 * 	[output] Port for
	 * 	\var 	o_mission_complete 
	 * 	[output] Port for
	 * 	\var 	o_notify_pilot 
	 * 	[output] Port for
	 * 	\var 	o_request_aircraft_state 
	 * 	[output] Port for
	 * 	\var 	o_set_mission_monitor_status 
	 * 	[output] Port for
	 * 	\var 	o_update_boss 
	 * 	[output] Port for
	 * 	\var 	o_update_gcs 
	 * 	[output] Port for
	 * 	\var 	o_update_mission_item 
	 * 	[output] Port for
	 */
	cadmium::dynamic::modeling::Ports oports = {
		typeid(Landing::defs::o_control_yielded),
		typeid(Landing::defs::o_fcc_command_hover),
		typeid(Landing::defs::o_fcc_command_land),
		typeid(Landing::defs::o_fcc_command_orbit),
		typeid(Landing::defs::o_fcc_command_velocity),
		typeid(Landing::defs::o_LP_expired),
		typeid(Landing::defs::o_LP_new),
		typeid(Landing::defs::o_mission_complete),
		typeid(Landing::defs::o_notify_pilot),
		typeid(Landing::defs::o_request_aircraft_state),
		typeid(Landing::defs::o_set_mission_monitor_status),
		typeid(Landing::defs::o_update_boss),
		typeid(Landing::defs::o_update_gcs),
		typeid(Landing::defs::o_update_mission_item)
	};

	/**
	 *	\struct	submodels
	 * 	\brief 	Definition of the sub-models that make up the coupled model.
	 * 	\var 	lp_manager 
	 * 	Model for 
	 *	\var 	stabilize 
	 *	Model for 
	 *	\var 	handover_control 
	 *	Model for 
	 *	\var 	lp_reposition 
	 *	Model for 
	 */
	cadmium::dynamic::modeling::Models submodels = {
		lp_manager,
		stabilize,
		handover_control,
		lp_reposition
	};

	/** 
	 * \struct	eics
	 * \brief	Definition of the external to internal couplings for the model.
	 * \see 	Landing
	 */
	cadmium::dynamic::modeling::EICs eics = {
		// lp_manager
		cadmium::dynamic::translate::make_EIC<Landing::defs::i_LP_recv, LP_Manager<TIME>::defs::i_lp_recv>("lp_manager"),
		cadmium::dynamic::translate::make_EIC<Landing::defs::i_PLP_ach, LP_Manager<TIME>::defs::i_plp_ach>("lp_manager"),
		cadmium::dynamic::translate::make_EIC<Landing::defs::i_pilot_takeover, LP_Manager<TIME>::defs::i_pilot_takeover>("lp_manager"),
		cadmium::dynamic::translate::make_EIC<Landing::defs::i_aircraft_state, LP_Manager<TIME>::defs::i_aircraft_state>("lp_manager"),
		cadmium::dynamic::translate::make_EIC<Landing::defs::i_start_mission, LP_Manager<TIME>::defs::i_start_mission>("lp_manager"),

		// lp_reposition
		cadmium::dynamic::translate::make_EIC<Landing::defs::i_landing_achieved, LP_Reposition::defs::i_landing_achieved>("lp_reposition"),
		cadmium::dynamic::translate::make_EIC<Landing::defs::i_aircraft_state, LP_Reposition::defs::i_aircraft_state>("lp_reposition"),
		cadmium::dynamic::translate::make_EIC<Landing::defs::i_pilot_takeover, LP_Reposition::defs::i_pilot_takeover>("lp_reposition"),
		cadmium::dynamic::translate::make_EIC<Landing::defs::i_start_mission, LP_Reposition::defs::i_start_mission>("lp_reposition"),

		// stabilize
		cadmium::dynamic::translate::make_EIC<Landing::defs::i_aircraft_state, Stabilize<TIME>::defs::i_aircraft_state>("stabilize"),
		cadmium::dynamic::translate::make_EIC<Landing::defs::i_start_mission, Stabilize<TIME>::defs::i_start_mission>("stabilize"),

		// handover_control
		cadmium::dynamic::translate::make_EIC<Landing::defs::i_pilot_takeover, Handover_Control<TIME>::defs::i_pilot_takeover>("handover_control"),
		cadmium::dynamic::translate::make_EIC<Landing::defs::i_start_mission, Handover_Control<TIME>::defs::i_start_mission>("handover_control")
	};

	/** 
	 * \struct	eocs
	 * \brief	Definition of the internal to external couplings for the model.
	 * \see 	Landing
	 */
	cadmium::dynamic::modeling::EOCs eocs = {
		// lp_manager
		cadmium::dynamic::translate::make_EOC<LP_Manager<TIME>::defs::o_fcc_command_orbit, Landing::defs::o_fcc_command_orbit>("lp_manager"),
		cadmium::dynamic::translate::make_EOC<LP_Manager<TIME>::defs::o_lp_expired, Landing::defs::o_LP_expired>("lp_manager"),
		cadmium::dynamic::translate::make_EOC<LP_Manager<TIME>::defs::o_lp_new, Landing::defs::o_LP_new>("lp_manager"),
		cadmium::dynamic::translate::make_EOC<LP_Manager<TIME>::defs::o_update_boss, Landing::defs::o_update_boss>("lp_manager"),
		cadmium::dynamic::translate::make_EOC<LP_Manager<TIME>::defs::o_update_gcs, Landing::defs::o_update_gcs>("lp_manager"),
		cadmium::dynamic::translate::make_EOC<LP_Manager<TIME>::defs::o_request_aircraft_state, Landing::defs::o_request_aircraft_state>("lp_manager"),
		cadmium::dynamic::translate::make_EOC<LP_Manager<TIME>::defs::o_set_mission_monitor_status, Landing::defs::o_set_mission_monitor_status>("lp_manager"),

		// lp_reposition
		cadmium::dynamic::translate::make_EOC<LP_Reposition::defs::o_fcc_command_land, Landing::defs::o_fcc_command_land>("lp_reposition"),
		cadmium::dynamic::translate::make_EOC<LP_Reposition::defs::o_fcc_command_velocity, Landing::defs::o_fcc_command_velocity>("lp_reposition"),
		cadmium::dynamic::translate::make_EOC<LP_Reposition::defs::o_mission_complete, Landing::defs::o_mission_complete>("lp_reposition"),
		cadmium::dynamic::translate::make_EOC<LP_Reposition::defs::o_request_aircraft_state, Landing::defs::o_request_aircraft_state>("lp_reposition"),
		cadmium::dynamic::translate::make_EOC<LP_Reposition::defs::o_set_mission_monitor_status, Landing::defs::o_set_mission_monitor_status>("lp_reposition"),
		cadmium::dynamic::translate::make_EOC<LP_Reposition::defs::o_update_boss, Landing::defs::o_update_boss>("lp_reposition"),
		cadmium::dynamic::translate::make_EOC<LP_Reposition::defs::o_update_gcs, Landing::defs::o_update_gcs>("lp_reposition"),
		cadmium::dynamic::translate::make_EOC<LP_Reposition::defs::o_update_mission_item, Landing::defs::o_update_mission_item>("lp_reposition"),

		// handover_control
		cadmium::dynamic::translate::make_EOC<Handover_Control<TIME>::defs::o_control_yielded, Landing::defs::o_control_yielded>("handover_control"),
		cadmium::dynamic::translate::make_EOC<Handover_Control<TIME>::defs::o_notify_pilot, Landing::defs::o_notify_pilot>("handover_control"),

		//stabilize
		cadmium::dynamic::translate::make_EOC<Stabilize<TIME>::defs::o_fcc_command_hover, Landing::defs::o_fcc_command_hover>("stabilize"),
		cadmium::dynamic::translate::make_EOC<Stabilize<TIME>::defs::o_request_aircraft_state, Landing::defs::o_request_aircraft_state>("stabilize"),
		cadmium::dynamic::translate::make_EOC<Stabilize<TIME>::defs::o_update_gcs, Landing::defs::o_update_gcs>("stabilize")
	};

	/** 
	 * \struct	ics
	 * \brief	Definition of the internal to internal couplings for the model.
	 * \see 	Landing
	 */
	cadmium::dynamic::modeling::ICs ics = {
		// lp_manager
		cadmium::dynamic::translate::make_IC<LP_Manager<TIME>::defs::o_lp_new, LP_Reposition::defs::i_lp_new>("lp_manager","lp_reposition"),
		cadmium::dynamic::translate::make_IC<LP_Manager<TIME>::defs::o_pilot_handover, Handover_Control<TIME>::defs::i_pilot_handover>("lp_manager","handover_control"),

		// lp_reposition
		cadmium::dynamic::translate::make_IC<LP_Reposition::defs::o_cancel_hover, Stabilize<TIME>::defs::i_cancel_hover>("lp_reposition","stabilize"),
		cadmium::dynamic::translate::make_IC<LP_Reposition::defs::o_stabilize, Stabilize<TIME>::defs::i_stabilize>("lp_reposition","stabilize"),
		cadmium::dynamic::translate::make_IC<LP_Reposition::defs::o_pilot_handover, Handover_Control<TIME>::defs::i_pilot_handover>("lp_reposition","handover_control"),
		cadmium::dynamic::translate::make_IC<LP_Reposition::defs::o_fcc_command_land, LP_Manager<TIME>::defs::i_fcc_command_land>("lp_reposition","lp_manager"),

		// stabilize
		cadmium::dynamic::translate::make_IC<Stabilize<TIME>::defs::o_hover_criteria_met, Handover_Control<TIME>::defs::i_hover_criteria_met>("stabilize","handover_control"),
		cadmium::dynamic::translate::make_IC<Stabilize<TIME>::defs::o_hover_criteria_met, LP_Reposition::defs::i_hover_criteria_met>("stabilize","lp_reposition"),

		// handover_control
		cadmium::dynamic::translate::make_IC<Handover_Control<TIME>::defs::o_control_yielded, LP_Manager<TIME>::defs::i_control_yielded>("handover_control","lp_manager"),
		cadmium::dynamic::translate::make_IC<Handover_Control<TIME>::defs::o_control_yielded, LP_Reposition::defs::i_control_yielded>("handover_control","lp_reposition"),
		cadmium::dynamic::translate::make_IC<Handover_Control<TIME>::defs::o_stabilize, Stabilize<TIME>::defs::i_stabilize>("handover_control","stabilize")
	};
};

#endif // LANDING_HPP
