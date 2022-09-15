/**
 * 	\file		Takeoff.hpp
 *	\brief		Definition of the Takeoff coupled model.
 *	\details	This header file defines the Takeoff coupled model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor in
				the Takeoff phase of flight.
 *	\image		html coupled_models/takeoff.png
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef TAKEOFF_HPP
#define TAKEOFF_HPP

//Messages structures
#include "../message_structures/message_aircraft_state_t.hpp"
#include "../message_structures/message_start_supervisor_t.hpp"
#include "../message_structures/message_update_gcs_t.hpp"

// IO model headers
#include "../io_models/Cache_Input.hpp"

//Atomic model headers
#include "../atomic_models/Mission_Initialization.hpp"

// Cadmium Simulator Headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>

// Time Class Header
#include <NDTime.hpp>

/**
 * 	\class		Takeoff
 *	\brief		Definition of the Takeoff coupled model.
 *	\details	This class defines the Takeoff coupled model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor in
				the Takeoff phase of flight.
 *	\image		html coupled_models/takeoff.png
 */
class Takeoff {
	using TIME = NDTime;

public:
	/**
	 *	\brief	For definition of the input and output ports see:
	 *	\ref 	Takeoff_input_ports "Input Ports" and
	 *	\ref 	Takeoff_output_ports "Output Ports"
	 * 	\note 	All input and output ports must be listed in this struct.
	 */
	struct defs {
		/***** Define input port for coupled models *****/
		struct i_aircraft_state : public cadmium::in_port<message_aircraft_state_t> {};
		struct i_perception_status : public cadmium::in_port<bool> {};
		struct i_start_supervisor : public cadmium::in_port<message_start_supervisor_t> {};

		/***** Define output ports for coupled model *****/
		struct o_request_aircraft_state : public cadmium::out_port<bool> {};
		struct o_set_mission_monitor_status : public cadmium::out_port<uint8_t> {};
		struct o_start_mission : public cadmium::out_port<int> {};
		struct o_update_gcs : public cadmium::out_port<message_update_gcs_t> {};
	};

	// Instantiate the Atomic models.
	std::shared_ptr <cadmium::dynamic::modeling::model> mission_initialization = cadmium::dynamic::translate::make_dynamic_atomic_model<Mission_Initialization, TIME>("mission_initialization");
	std::shared_ptr <cadmium::dynamic::modeling::model> cache_input = cadmium::dynamic::translate::make_dynamic_atomic_model<Cache_Input_Boolean, TIME, bool>("cache_input", false);

	/**
	 * 	\anchor	Takeoff_input_ports
	 *	\par	Input Ports
	 * 	Definition of the input ports for the model.
	 * 	\param 	i_aircraft_state	Port for receiving the current state of the aircraft.
	 * 	\param 	i_perception_status	Port for receiving the status of the perception system.
	 * 	\param 	i_start_supervisor	Port for receiving signal to start the supervisor.
	 */
 	cadmium::dynamic::modeling::Ports iports = {
		typeid(Takeoff::defs::i_aircraft_state),
		typeid(Takeoff::defs::i_perception_status),
		typeid(Takeoff::defs::i_start_supervisor)
	};

	/**
	 *	\anchor	Takeoff_output_ports
	 * 	\par 	Output Ports
	 * 	Definition of the output ports for the model.
	 * 	\param 	o_request_aircraft_state		Port for requesting the current aircraft state.
	 * 	\param 	o_set_mission_monitor_status	Port for telling the mission monitor to start monitoring mission progress.
	 * 	\param 	o_start_mission					Port for sending a notification that the mission has started.
	 * 	\param 	o_update_gcs					Port for sending updates to the GCS.
	 */
 	cadmium::dynamic::modeling::Ports oports = {
		typeid(Takeoff::defs::o_request_aircraft_state),
		typeid(Takeoff::defs::o_set_mission_monitor_status),
		typeid(Takeoff::defs::o_start_mission),
		typeid(Takeoff::defs::o_update_gcs)
	};

	/**
	 *	\anchor	Takeoff_submodels
	 * 	\par 	Output Ports
	 * 	Definition of the sub-models that make up the coupled model.
	 * 	\param 	mission_initialization	Model for starting a mission.
	 *	\param 	cache_input				Model for caches inputs for later use.
	 */
 	cadmium::dynamic::modeling::Models submodels = {
		mission_initialization,
		cache_input
	};

	/**
	 * 	\par 	External Input Couplings
	 *	Definition of the external to internal couplings for the model.
	 * 	\see 	Takeoff
	 */
 	cadmium::dynamic::modeling::EICs eics = {
		// mission_initialization
		cadmium::dynamic::translate::make_EIC<Takeoff::defs::i_aircraft_state, Mission_Initialization<TIME>::defs::i_aircraft_state>("mission_initialization"),
		cadmium::dynamic::translate::make_EIC<Takeoff::defs::i_start_supervisor, Mission_Initialization<TIME>::defs::i_start_supervisor>("mission_initialization"),

		// takeoff to cache_input<bool>
		cadmium::dynamic::translate::make_EIC<Takeoff::defs::i_perception_status, Cache_Input<bool, TIME>::defs::i_new_input>("cache_input")
	};

	/**
	 * 	\par 	External Output Couplings
	 *	Definition of the internal to external couplings for the model.
	 * 	\see 	Takeoff
	 */
 	cadmium::dynamic::modeling::EOCs eocs = {
		// mission_initialization
		cadmium::dynamic::translate::make_EOC<Mission_Initialization<TIME>::defs::o_request_aircraft_state, Takeoff::defs::o_request_aircraft_state>("mission_initialization"),
		cadmium::dynamic::translate::make_EOC<Mission_Initialization<TIME>::defs::o_set_mission_monitor_status, Takeoff::defs::o_set_mission_monitor_status>("mission_initialization"),
		cadmium::dynamic::translate::make_EOC<Mission_Initialization<TIME>::defs::o_start_mission, Takeoff::defs::o_start_mission>("mission_initialization"),
		cadmium::dynamic::translate::make_EOC<Mission_Initialization<TIME>::defs::o_update_gcs, Takeoff::defs::o_update_gcs>("mission_initialization"),
	};

	/**
	 * 	\par 	Internal Couplings
	 * 	Definition of the internal to internal couplings for the model.
	 * 	\see 	Takeoff
	 */
 	cadmium::dynamic::modeling::ICs ics = {
		// mission_initialization
		cadmium::dynamic::translate::make_IC<Mission_Initialization<TIME>::defs::o_request_perception_status, Cache_Input<bool, TIME>::defs::i_get_input>("mission_initialization", "cache_input"),

		// cache_input
		cadmium::dynamic::translate::make_IC<Cache_Input<bool, TIME>::defs::o_cached_input, Mission_Initialization<TIME>::defs::i_perception_status>("cache_input", "mission_initialization")
	};
};

#endif // TAKEOFF_HPP
