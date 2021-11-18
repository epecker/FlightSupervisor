/**
 *	\brief		A coupled model representing the LP Reposition model.
 *	\details	This header file define the LP Reposition model as
				A coupled model for use in the Cadmium DEVS
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

//Messages structures
#include "../../include/message_structures/hover_criteria_message.hpp"
#include "../../include/message_structures/aircraft_state_message.hpp"
#include "../../include/message_structures/lp_message.hpp"
#include "../../include/message_structures/fcc_command.hpp"

//Atomic model headers
#include "../../include/atomic_models/Landing_Routine.hpp"
#include "../../include/atomic_models/Reposition_Timer.hpp"
#include "../../include/atomic_models/Command_Reposition.hpp"

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
struct LP_Reposition_defs {
	struct i_landing_achieved : public in_port<bool> {};
	struct i_pilot_takeover : public in_port<bool> {};
	struct i_hover_criteria_met : public in_port<bool> {};
	struct i_aircraft_state : public in_port<AircraftStateMessage_t> {};
	struct i_control_yielded : public in_port<bool> {};
	struct i_LP_new : public in_port<LPMessage_t> {};
	struct i_LP_criteria_met : public in_port<LPMessage_t> {};


	/***** Define output ports for coupled model *****/
	struct o_mission_complete : public out_port<bool> {};
	struct o_land_requested : public out_port<bool> {};
	struct o_stabilize : public out_port<HoverCriteriaMessage_t> {};
	struct o_cancel_hover : public out_port<bool> {};
	struct o_fcc_command_velocity : public out_port<FccCommandMessage_t> {};
	struct o_pilot_handover : public out_port<bool> {};
};


/**
* Instantiate the Atomic models.
*/
shared_ptr<dynamic::modeling::model> landing_routine = dynamic::translate::make_dynamic_atomic_model<Landing_Routine, TIME>("landing_routine");
shared_ptr<dynamic::modeling::model> command_reposition = dynamic::translate::make_dynamic_atomic_model<Command_Reposition, TIME>("command_reposition");
shared_ptr<dynamic::modeling::model> reposition_timer = dynamic::translate::make_dynamic_atomic_model<Reposition_Timer, TIME>("reposition_timer");

//Define the inputs to the Landing Point Reposition coupled model.
dynamic::modeling::Ports iports_LPReposition = {
	typeid(LP_Reposition_defs::i_landing_achieved),
	typeid(LP_Reposition_defs::i_pilot_takeover),
	typeid(LP_Reposition_defs::i_hover_criteria_met),
	typeid(LP_Reposition_defs::i_aircraft_state),
	typeid(LP_Reposition_defs::i_control_yielded),
	typeid(LP_Reposition_defs::i_LP_new),
	typeid(LP_Reposition_defs::i_LP_criteria_met)
};

//Define the outputs of the Landing Point Reposition coupled model.
dynamic::modeling::Ports oports_LPReposition = {
	typeid(LP_Reposition_defs::o_mission_complete),
	typeid(LP_Reposition_defs::o_land_requested),
	typeid(LP_Reposition_defs::o_stabilize),
	typeid(LP_Reposition_defs::o_cancel_hover),
	typeid(LP_Reposition_defs::o_fcc_command_velocity),
	typeid(LP_Reposition_defs::o_pilot_handover)
};

//Define the sub-models that make up the Landing Point Reposition coupled model.
dynamic::modeling::Models submodels_LPReposition = {
	landing_routine,
	command_reposition,
	reposition_timer
};

//Define the external to internal couplings for the Landing Point Reposition model.
dynamic::modeling::EICs eics_LPReposition = {
	dynamic::translate::make_EIC<LP_Reposition_defs::i_landing_achieved, Landing_Routine_defs::i_landing_achieved>("landing_routine"),
	dynamic::translate::make_EIC<LP_Reposition_defs::i_pilot_takeover, Landing_Routine_defs::i_pilot_takeover>("landing_routine"),
	dynamic::translate::make_EIC<LP_Reposition_defs::i_hover_criteria_met, Landing_Routine_defs::i_hover_crit_met>("landing_routine"),

	dynamic::translate::make_EIC<LP_Reposition_defs::i_hover_criteria_met, Command_Reposition_defs::i_hover_criteria_met>("command_reposition"),
	dynamic::translate::make_EIC<LP_Reposition_defs::i_pilot_takeover, Command_Reposition_defs::i_pilot_handover>("command_reposition"),
	dynamic::translate::make_EIC<LP_Reposition_defs::i_aircraft_state, Command_Reposition_defs::i_aircraft_state>("command_reposition"),

	dynamic::translate::make_EIC<LP_Reposition_defs::i_pilot_takeover, Reposition_Timer_defs::i_pilot_takeover>("reposition_timer"),
	dynamic::translate::make_EIC<LP_Reposition_defs::i_control_yielded, Reposition_Timer_defs::i_control_yielded>("reposition_timer"),
	dynamic::translate::make_EIC<LP_Reposition_defs::i_LP_new, Reposition_Timer_defs::i_lp_new>("reposition_timer"),
	dynamic::translate::make_EIC<LP_Reposition_defs::i_LP_criteria_met, Reposition_Timer_defs::i_lp_crit_met>("reposition_timer")
};

//Define the internal to external couplings for the Landing Point Reposition model.
dynamic::modeling::EOCs eocs_LPReposition = {
	dynamic::translate::make_EOC<Landing_Routine_defs::o_mission_complete, LP_Reposition_defs::o_mission_complete>("landing_routine"),
	dynamic::translate::make_EOC<Landing_Routine_defs::o_land_requested, LP_Reposition_defs::o_land_requested>("landing_routine"),
	dynamic::translate::make_EOC<Landing_Routine_defs::o_stabilize, LP_Reposition_defs::o_stabilize>("landing_routine"),
	
	dynamic::translate::make_EOC<Command_Reposition_defs::o_stabilize, LP_Reposition_defs::o_stabilize>("command_reposition"),
	dynamic::translate::make_EOC<Command_Reposition_defs::o_stabilize, LP_Reposition_defs::o_stabilize>("command_reposition"),
	dynamic::translate::make_EOC<Command_Reposition_defs::o_fcc_command_velocity, LP_Reposition_defs::o_fcc_command_velocity>("command_reposition"),
	
	dynamic::translate::make_EOC<Reposition_Timer_defs::o_pilot_handover, LP_Reposition_defs::o_pilot_handover>("reposition_timer")
};

//Define the internal to internal couplings for the Landing Point Reposition model.
dynamic::modeling::ICs ics_LPReposition = {
	dynamic::translate::make_IC<Command_Reposition_defs::o_lp_criteria_met, Reposition_Timer_defs::i_lp_crit_met>("command_reposition", "reposition_timer"),

	dynamic::translate::make_IC<Reposition_Timer_defs::o_land, Landing_Routine_defs::i_land>("reposition_timer", "landing_routine"),
	dynamic::translate::make_IC<Reposition_Timer_defs::o_pilot_handover, Command_Reposition_defs::i_pilot_handover>("reposition_timer", "command_reposition"),
	dynamic::translate::make_IC<Reposition_Timer_defs::o_request_reposition, Command_Reposition_defs::i_request_reposition>("reposition_timer", "command_reposition")
};