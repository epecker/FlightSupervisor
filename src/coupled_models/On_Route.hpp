/**
 * 	\file		On_Route.hpp
 *	\brief		Definition of the On_Route coupled model.
 *	\details	This header file defines the On_Route coupled model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor when
				a waypoint is met while on-route.
 *	\image		html coupled_models/on_route.png
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef ON_ROUTE_HPP
#define ON_ROUTE_HPP

// Messages structures
#include "../message_structures/message_fcc_command_t.hpp"

// Atomic model headers
#include "../atomic_models/Handle_Waypoint.hpp"

// Cadmium Simulator Headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>

// Time Class Header
#include <NDTime.hpp>

/**
 * 	\class		On_Route
 *	\brief		Definition of the On_Route coupled model.
 *	\details	This class defines the On_Route coupled model for use in the Cadmium DEVS
				simulation software. The model represents the behaviour of the Supervisor when
				a waypoint is met while on-route.
 *	\image		html coupled_models/on_route.png
 */
class On_Route {
	using TIME = NDTime;

public:
	/**
	 *	\brief	For definition of the input and output ports see:
	*	\ref 	On_Route_input_ports "Input Ports" and
	*	\ref 	On_Route_output_ports "Output Ports"
	* 	\note 	All input and output ports must be listed in this struct.
	*/
	struct defs {
		/***** Define input port for coupled models *****/
		struct i_pilot_takeover : public cadmium::out_port<bool> {};
		struct i_start_mission : public cadmium::out_port<int> {};
		struct i_waypoint : public cadmium::out_port<message_fcc_command_t> {};

		/***** Define output ports for coupled model *****/
		struct o_fcc_waypoint_update : public cadmium::out_port<message_fcc_command_t> {};
	};

	// Instantiate the Atomic models.
	std::shared_ptr <cadmium::dynamic::modeling::model> handle_waypoint = cadmium::dynamic::translate::make_dynamic_atomic_model<Handle_Waypoint, TIME>("handle_waypoint");

	/**
	 * 	\anchor	On_Route_input_ports
	 *	\par	Input Ports
	 * 	Definition of the input ports for the model.
	 * 	\param 	i_pilot_takeover	Port for signal indicating that the pilot has taken control from the supervisor.
	 * 	\param 	i_start_mission		Port for receiving signal indicating the mission has started.
	 * 	\param 	i_waypoint			Port for receiving new waypoints during the on-route phase.
	 */
 	cadmium::dynamic::modeling::Ports iports = {
			typeid(defs::i_pilot_takeover),
			typeid(defs::i_start_mission),
			typeid(defs::i_waypoint)
	};

	/**
	 *	\anchor	On_Route_output_ports
	 * 	\par 	Output Ports
	 * 	Definition of the output ports for the model.
	 * 	\param 	o_fcc_waypoint_update	Port for sending waypoint commands to the FCC.
	 */
 	cadmium::dynamic::modeling::Ports oports = {
			typeid(defs::o_fcc_waypoint_update)
	};

	/**
	 *	\anchor	On_Route_submodels
	 * 	\par 	Output Ports
	 * 	Definition of the sub-models that make up the coupled model.
	 *	\param 	handle_waypoint		Model for sending waypoint commands to the FCC after a waypoint is met.
	 */
 	cadmium::dynamic::modeling::Models submodels = {
			handle_waypoint
	};

	/**
	 * 	\par 	External Input Couplings
	 *	Definition of the external to internal couplings for the model.
	 * 	\see 	On_Route
	 */
 	cadmium::dynamic::modeling::EICs eics = {
		// handle_waypoint
		cadmium::dynamic::translate::make_EIC<defs::i_pilot_takeover, Handle_Waypoint<TIME>::defs::i_pilot_takeover>("handle_waypoint"),
		cadmium::dynamic::translate::make_EIC<defs::i_start_mission, Handle_Waypoint<TIME>::defs::i_start_mission>("handle_waypoint"),
		cadmium::dynamic::translate::make_EIC<defs::i_waypoint, Handle_Waypoint<TIME>::defs::i_waypoint>("handle_waypoint")
	};

	/**
	 * 	\par 	External Output Couplings
	 *	Definition of the internal to external couplings for the model.
	 * 	\see 	On_Route
	 */
 	cadmium::dynamic::modeling::EOCs eocs = {
		 // handle_waypoint
		 cadmium::dynamic::translate::make_EOC<Handle_Waypoint<TIME>::defs::o_fcc_waypoint_update, defs::o_fcc_waypoint_update>("handle_waypoint"),
	};

	/**
	 * 	\par 	Internal Couplings
	 * 	Definition of the internal to internal couplings for the model.
	 * 	\see 	On_Route
	 */
 	cadmium::dynamic::modeling::ICs ics = {};
};

#endif // ON_ROUTE_HPP
