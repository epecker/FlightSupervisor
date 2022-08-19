/**
 *	\brief		A coupled model representing the On_Route model.
 *	\details	This header file defines the On_Route model as
				a coupled model for use in the Cadmium DEVS
				simulation software. This model handles the
				navigation between waypoints.
 *	\author		Tanner Trautrim
 */

#ifndef ON_ROUTE_HPP
#define ON_ROUTE_HPP

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <NDTime.hpp>

#include <limits>
#include <string>

#include "message_structures/message_fcc_command_t.hpp"

#include "atomic_models/Handle_Waypoint.hpp"

#include "Constants.hpp"

using TIME = NDTime;
using namespace cadmium;

// Input and output port definitions
struct On_Route_defs {
	struct i_pilot_takeover : public out_port<bool> {};
	struct i_start_mission : public out_port<int> {};
	struct i_waypoint : public out_port<message_fcc_command_t> {};

	struct o_fcc_waypoint_update : public out_port<message_fcc_command_t> {};
};

class On_Route {
public:
	/**
	* Instantiate the Atomic models.
	*/
	std::shared_ptr<dynamic::modeling::model> handle_waypoint = dynamic::translate::make_dynamic_atomic_model<Handle_Waypoint, TIME>("handle_waypoint");

	//Define the inputs to the Landing Point Reposition coupled model.
	dynamic::modeling::Ports iports = {
			typeid(On_Route_defs::i_pilot_takeover),
			typeid(On_Route_defs::i_start_mission),
			typeid(On_Route_defs::i_waypoint)
	};

	//Define the outputs of the Landing Point Reposition coupled model.
	dynamic::modeling::Ports oports = {
			typeid(On_Route_defs::o_fcc_waypoint_update)
	};

	//Define the sub-models that make up the Landing Point Reposition coupled model.
	dynamic::modeling::Models submodels = {
			handle_waypoint
	};

	//Define the external to internal couplings for the Landing Point Reposition model.
	dynamic::modeling::EICs eics = {
			dynamic::translate::make_EIC<On_Route_defs::i_pilot_takeover, Handle_Waypoint_defs::i_pilot_takeover>("handle_waypoint"),
			dynamic::translate::make_EIC<On_Route_defs::i_start_mission, Handle_Waypoint_defs::i_start_mission>("handle_waypoint"),
			dynamic::translate::make_EIC<On_Route_defs::i_waypoint, Handle_Waypoint_defs::i_waypoint>("handle_waypoint")
	};

	//Define the internal to external couplings for the Landing Point Reposition model.
	dynamic::modeling::EOCs eocs = {
			dynamic::translate::make_EOC<Handle_Waypoint_defs::o_fcc_waypoint_update, On_Route_defs::o_fcc_waypoint_update>("handle_waypoint"),
	};

	//Define the internal to internal couplings for the Landing Point Reposition model.
	dynamic::modeling::ICs ics = {};
};

#endif // ON_ROUTE_HPP
