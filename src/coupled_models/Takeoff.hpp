/**
 *	\brief		A coupled model representing the takeoff phase of the Supervisor behaviour.
 *	\details	This header file define the Takeoff Phase model as a coupled model for use
 				in the Cadmium DEVS simulation software.
 *	\author		James Horner
 */

#ifndef TAKEOFF_HPP
#define TAKEOFF_HPP

//Cadmium Simulator headers
#include "cadmium/modeling/ports.hpp"
#include "cadmium/modeling/dynamic_model_translator.hpp"

//Time class header
#include "NDTime.hpp"

//Messages structures
#include "../message_structures/message_aircraft_state_t.hpp"
#include "../message_structures/message_start_supervisor_t.hpp"
#include "../message_structures/message_update_gcs_t.hpp"

//Atomic model headers
#include "../atomic_models/Mission_Initialization.hpp"

//IO model headers
#include "../input_models.hpp"

//Project information headers this is created by cmake at generation time!!!!
#include "../SupervisorConfig.hpp"

using namespace cadmium;

using TIME = NDTime;

class Takeoff {
public:
	/***** Define input port for coupled models *****/
	struct defs {
		struct i_aircraft_state : public in_port<message_aircraft_state_t> {};
		struct i_perception_status : public in_port<bool> {};
		struct i_start_supervisor : public in_port<message_start_supervisor_t> {};

		/***** Define output ports for coupled model *****/
		struct o_request_aircraft_state : public out_port<bool> {};
		struct o_set_mission_monitor_status : public out_port<uint8_t> {};
		struct o_start_mission : public out_port<int> {};
		struct o_update_gcs : public out_port<message_update_gcs_t> {};
	};

	/**
	* Instantiate the Atomic models.
	*/
	shared_ptr<dynamic::modeling::model> mission_initialization = dynamic::translate::make_dynamic_atomic_model<Mission_Initialization, TIME>("mission_initialization");
	shared_ptr<dynamic::modeling::model> cache_input = dynamic::translate::make_dynamic_atomic_model<Cache_Input_Boolean, TIME, bool>("cache_input", false);

	//Define the inputs to the Landing Point Reposition coupled model.
	dynamic::modeling::Ports iports = {
		typeid(Takeoff::defs::i_aircraft_state),
		typeid(Takeoff::defs::i_perception_status),
		typeid(Takeoff::defs::i_start_supervisor)
	};

	//Define the outputs of the Landing Point Reposition coupled model.
	dynamic::modeling::Ports oports = {
		typeid(Takeoff::defs::o_request_aircraft_state),
		typeid(Takeoff::defs::o_set_mission_monitor_status),
		typeid(Takeoff::defs::o_start_mission),
		typeid(Takeoff::defs::o_update_gcs)
	};

	//Define the sub-models that make up the Landing Point Reposition coupled model.
	dynamic::modeling::Models submodels = {
		mission_initialization,
		cache_input
	};

	//Define the external to internal couplings for the Landing Point Reposition model.
	dynamic::modeling::EICs eics = {
		dynamic::translate::make_EIC<Takeoff::defs::i_aircraft_state, Mission_Initialization<TIME>::defs::i_aircraft_state>("mission_initialization"),
		dynamic::translate::make_EIC<Takeoff::defs::i_start_supervisor, Mission_Initialization<TIME>::defs::i_start_supervisor>("mission_initialization"),

		dynamic::translate::make_EIC<Takeoff::defs::i_perception_status, Cache_Input_defs<bool>::i_new_input>("cache_input")
	};

	//Define the internal to external couplings for the Landing Point Reposition model.
	dynamic::modeling::EOCs eocs = {
		dynamic::translate::make_EOC<Mission_Initialization<TIME>::defs::o_request_aircraft_state, Takeoff::defs::o_request_aircraft_state>("mission_initialization"),
		dynamic::translate::make_EOC<Mission_Initialization<TIME>::defs::o_set_mission_monitor_status, Takeoff::defs::o_set_mission_monitor_status>("mission_initialization"),
		dynamic::translate::make_EOC<Mission_Initialization<TIME>::defs::o_start_mission, Takeoff::defs::o_start_mission>("mission_initialization"),
		dynamic::translate::make_EOC<Mission_Initialization<TIME>::defs::o_update_gcs, Takeoff::defs::o_update_gcs>("mission_initialization"),

	};

	//Define the internal to internal couplings for the Landing Point Reposition model.
	dynamic::modeling::ICs ics = {
		dynamic::translate::make_IC<Mission_Initialization<TIME>::defs::o_request_perception_status, Cache_Input_defs<bool>::i_get_input>("mission_initialization", "cache_input"),

		dynamic::translate::make_IC<Cache_Input_defs<bool>::o_cached_input, Mission_Initialization<TIME>::defs::i_perception_status>("cache_input", "mission_initialization")
	};
};

#endif // TAKEOFF_HPP
