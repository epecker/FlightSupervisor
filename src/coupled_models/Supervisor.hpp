/**
 *	\brief		A coupled model representing the Supervisor model.
 *	\details	This header file define the Supervisor model as
				a coupled model for use in the Cadmium DEVS
				simulation software.
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

//Cadmium Simulator headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/logger/common_loggers.hpp>

//Time class header
#include <NDTime.hpp>

//Constants
#include "../../include/Constants.hpp"

//Utility functions
#include "../../include/time_conversion.hpp"

//Messages structures
#include "../../include/message_structures/message_hover_criteria_t.hpp"
#include "../../include/message_structures/message_aircraft_state_t.hpp"
#include "../../include/message_structures/message_mavlink_mission_item_t.hpp"
#include "../../include/message_structures/message_mavlink_mission_item_t.hpp"
#include "../../include/message_structures/message_fcc_command_t.hpp"

//Atomic model headers
#include "../../include/atomic_models/LP_Manager.hpp"
#include "../../include/atomic_models/Stabilize.hpp"
#include "../../include/atomic_models/Handover_Control.hpp"

//Coupled model headers
#include "../../src/coupled_models/LP_Reposition.hpp"

// Project information headers this is created by cmake at generation time!!!!
#include "../../include/SupervisorConfig.hpp"

//C++ headers
#include <chrono>
#include <algorithm>
#include <string>
#include <iostream>
#include <filesystem>

using namespace std;
using namespace cadmium;
using namespace cadmium::basic_models::pdevs;

using TIME = NDTime;

/***** Define input port for coupled models *****/
struct Supervisor_defs {
	struct i_landing_achieved : public in_port<bool> {};
	struct i_aircraft_state : public in_port<message_aircraft_state_t> {};
	struct i_LP_criteria_met : public in_port<message_mavlink_mission_item_t> {};
	struct i_pilot_takeover : public in_port<bool> {};
	struct i_LP_recv : public in_port<message_mavlink_mission_item_t> {};
	struct i_PLP_ach : public in_port<message_mavlink_mission_item_t> {};


	/***** Define output ports for coupled model *****/
	struct o_LP_expired : public out_port<message_mavlink_mission_item_t> {};
	struct o_start_LZE_scan : public out_port<bool> {};
	struct o_mission_complete : public out_port<bool> {};
	struct o_land_requested : public out_port<bool> {};
	struct o_fcc_command_velocity : public out_port<message_fcc_command_t> {};
	struct o_control_yielded : public out_port<bool> {};
	struct o_notify_pilot : public out_port<bool> {};
	struct o_fcc_command_hover : public out_port<message_fcc_command_t> {};
};


/**
* Instantiate the Atomic models.
*/
shared_ptr<dynamic::modeling::model> lp_manager = dynamic::translate::make_dynamic_atomic_model<LP_Manager, TIME, TIME, TIME>("lp_manager", seconds_to_time<TIME>(LP_ACCEPT_TIMER), seconds_to_time<TIME>(ORBIT_TIMER));
shared_ptr<dynamic::modeling::model> stabilize = dynamic::translate::make_dynamic_atomic_model<Stabilize, TIME>("stabilize");
shared_ptr<dynamic::modeling::model> handover_control = dynamic::translate::make_dynamic_atomic_model<Handover_Control, TIME>("handover_control");

shared_ptr<dynamic::modeling::coupled<TIME>> lp_reposition = make_shared<dynamic::modeling::coupled<TIME>>("lp_reposition", submodels_LPReposition, iports_LPReposition, oports_LPReposition, eics_LPReposition, eocs_LPReposition, ics_LPReposition);

//Define the inputs to the Supervisor coupled model.
dynamic::modeling::Ports iports_Supervisor = {
	typeid(Supervisor_defs::i_landing_achieved),
	typeid(Supervisor_defs::i_aircraft_state),
	typeid(Supervisor_defs::i_LP_criteria_met),
	typeid(Supervisor_defs::i_pilot_takeover),
	typeid(Supervisor_defs::i_LP_recv),
	typeid(Supervisor_defs::i_PLP_ach)
};

//Define the outputs of the Supervisor coupled model.
dynamic::modeling::Ports oports_Supervisor = {
	typeid(Supervisor_defs::o_LP_expired),
	typeid(Supervisor_defs::o_start_LZE_scan),
	typeid(Supervisor_defs::o_mission_complete),
	typeid(Supervisor_defs::o_land_requested),
	typeid(Supervisor_defs::o_fcc_command_velocity),
	typeid(Supervisor_defs::o_control_yielded),
	typeid(Supervisor_defs::o_notify_pilot),
	typeid(Supervisor_defs::o_fcc_command_hover)
};

//Define the sub-models that make up the Supervisor coupled model.
dynamic::modeling::Models submodels_Supervisor = {
	lp_manager,
	stabilize,
	handover_control,
	lp_reposition
};

//Define the external to internal couplings for the Supervisor.
dynamic::modeling::EICs eics_Supervisor = {
	// lp_manager
	dynamic::translate::make_EIC<Supervisor_defs::i_LP_recv, LP_Manager_defs::i_lp_recv>("lp_manager"),
	dynamic::translate::make_EIC<Supervisor_defs::i_PLP_ach, LP_Manager_defs::i_plp_ach>("lp_manager"),
	dynamic::translate::make_EIC<Supervisor_defs::i_pilot_takeover, LP_Manager_defs::i_pilot_takeover>("lp_manager"),

	// lp_reposition
	dynamic::translate::make_EIC<Supervisor_defs::i_landing_achieved, LP_Reposition_defs::i_landing_achieved>("lp_reposition"),
	dynamic::translate::make_EIC<Supervisor_defs::i_aircraft_state, LP_Reposition_defs::i_aircraft_state>("lp_reposition"),
	dynamic::translate::make_EIC<Supervisor_defs::i_LP_criteria_met, LP_Reposition_defs::i_LP_criteria_met>("lp_reposition"),
	dynamic::translate::make_EIC<Supervisor_defs::i_pilot_takeover, LP_Reposition_defs::i_pilot_takeover>("lp_reposition"),

	// stabilize
	dynamic::translate::make_EIC<Supervisor_defs::i_aircraft_state, Stabilize_defs::i_aircraft_state>("stabilize"),

	// handover_control
	dynamic::translate::make_EIC<Supervisor_defs::i_pilot_takeover, Handover_Control_defs::i_pilot_takeover>("handover_control")
};

//Define the internal to external couplings for the Supervisor.
dynamic::modeling::EOCs eocs_Supervisor = {
	// lp_manager
	dynamic::translate::make_EOC<LP_Manager_defs::o_lp_expired, Supervisor_defs::o_LP_expired>("lp_manager"),
	dynamic::translate::make_EOC<LP_Manager_defs::o_start_lze_scan, Supervisor_defs::o_start_LZE_scan>("lp_manager"),
	
	// lp_reposition
	dynamic::translate::make_EOC<LP_Reposition_defs::o_mission_complete, Supervisor_defs::o_mission_complete>("lp_reposition"),
	dynamic::translate::make_EOC<LP_Reposition_defs::o_land_requested, Supervisor_defs::o_land_requested>("lp_reposition"),
	dynamic::translate::make_EOC<LP_Reposition_defs::o_fcc_command_velocity, Supervisor_defs::o_fcc_command_velocity>("lp_reposition"),
	
	// handover_control
	dynamic::translate::make_EOC<Handover_Control_defs::o_control_yielded, Supervisor_defs::o_control_yielded>("handover_control"),
	dynamic::translate::make_EOC<Handover_Control_defs::o_notify_pilot, Supervisor_defs::o_notify_pilot>("handover_control"),
	
	//stabilize
	dynamic::translate::make_EOC<Stabilize_defs::o_fcc_command_hover, Supervisor_defs::o_fcc_command_hover>("stabilize")
};

//Define the internal to internal couplings for the Supervisor.
dynamic::modeling::ICs ics_Supervisor = {
	// lp_manager
	dynamic::translate::make_IC<LP_Manager_defs::o_lp_new, LP_Reposition_defs::i_LP_new>("lp_manager","lp_reposition"),
	dynamic::translate::make_IC<LP_Manager_defs::o_pilot_handover, Handover_Control_defs::i_pilot_handover>("lp_manager","handover_control"),
	dynamic::translate::make_IC<LP_Manager_defs::o_stabilize, Stabilize_defs::i_stabilize>("lp_manager","stabilize"),

	// lp_reposition
	dynamic::translate::make_IC<LP_Reposition_defs::o_cancel_hover, Stabilize_defs::i_cancel_hover>("lp_reposition","stabilize"),
	dynamic::translate::make_IC<LP_Reposition_defs::o_stabilize, Stabilize_defs::i_stabilize>("lp_reposition","stabilize"),
	dynamic::translate::make_IC<LP_Reposition_defs::o_pilot_handover, Handover_Control_defs::i_pilot_handover>("lp_reposition","handover_control"),

	// stabilize
	dynamic::translate::make_IC<Stabilize_defs::o_hover_criteria_met, Handover_Control_defs::i_hover_criteria_met>("stabilize","handover_control"),
	dynamic::translate::make_IC<Stabilize_defs::o_hover_criteria_met, LP_Manager_defs::i_hover_criteria_met>("stabilize","lp_manager"),
	dynamic::translate::make_IC<Stabilize_defs::o_hover_criteria_met, LP_Reposition_defs::i_hover_criteria_met>("stabilize","lp_reposition"),

	// handover_control
	dynamic::translate::make_IC<Handover_Control_defs::o_control_yielded, LP_Manager_defs::i_control_yielded>("handover_control","lp_manager"),
	dynamic::translate::make_IC<Handover_Control_defs::o_control_yielded, LP_Reposition_defs::i_control_yielded>("handover_control","lp_reposition"),
	dynamic::translate::make_IC<Handover_Control_defs::o_stabilize, Stabilize_defs::i_stabilize>("handover_control","stabilize")
};
