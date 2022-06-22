//C++ headers
#include <chrono>
#include <algorithm>
#include <string>
#include <iostream>
#include <filesystem>

//Cadmium Simulator headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/logger/common_loggers.hpp>

//Real-Time Headers
#include <cadmium/basic_model/pdevs/generator.hpp>
#include <cadmium/modeling/coupling.hpp>
#include <cadmium/concept/coupled_model_assert.hpp>

//Time class header
#include <NDTime.hpp>

// Project information headers this is created by cmake at generation time!!!!
#include "SupervisorConfig.hpp"
#include "input_models.hpp" // Input Model Definitions.

//Coupled model headers
#include "coupled_models/Supervisor.hpp"

using namespace std;
using namespace cadmium;

using hclock = std::chrono::high_resolution_clock;
using TIME = NDTime;

// Used for oss_sink_state and oss_sink_messages
ofstream out_messages;
ofstream out_state;
ofstream out_info;

// Define output ports to be used for logging purposes
struct o_LP_expired : public out_port<message_landing_point_t> {};
struct o_start_LZE_scan : public out_port<bool> {};
struct o_mission_complete : public out_port<bool> {};
struct o_land_requested : public out_port<bool> {};
struct o_fcc_command_velocity : public out_port<message_fcc_command_t> {};
struct o_control_yielded : public out_port<bool> {};
struct o_notify_pilot : public out_port<bool> {};
struct o_fcc_command_hover : public out_port<message_fcc_command_t> {};


int main(int argc, char* argv[]) {
	const string o_base_dir = string(PROJECT_DIRECTORY) + string("/test/simulation_results/rt_supervisor");

	string out_directory = o_base_dir;
	string out_messages_file = out_directory + string("/output_messages.txt");
	string out_state_file = out_directory + string("/output_state.txt");
	string out_info_file = out_directory + string("/output_info.txt");

	// Create the output location
	filesystem::create_directories(out_directory.c_str()); // Creates if it does not exist. Does nothing if it does.

	// Instantiate the coupled model to test
	Supervisor supervisor_object = Supervisor();
	shared_ptr<dynamic::modeling::coupled<TIME>> supervisor = make_shared<dynamic::modeling::coupled<TIME>>("supervisor", supervisor_object.submodels, supervisor_object.iports, supervisor_object.oports, supervisor_object.eics, supervisor_object.eocs, supervisor_object.ics);

	// Instantiate the input readers.
	// One for each input
	shared_ptr<dynamic::modeling::model> im_landing_achieved =
		dynamic::translate::make_dynamic_atomic_model<UDP_Input_Boolean, TIME, TIME, bool, string, string>("im_landing_achieved", std::move(TIME("00:00:00:100")), true, std::move("127.0.0.1"), std::move("23001"));
	shared_ptr<dynamic::modeling::model> im_pilot_takeover =
		dynamic::translate::make_dynamic_atomic_model<UDP_Input_Boolean, TIME, TIME, bool, string, string>("im_pilot_takeover", std::move(TIME("00:00:00:100")), true, std::move("127.0.0.1"), std::move("23002"));
	shared_ptr<dynamic::modeling::model> im_lp_recv =
		dynamic::translate::make_dynamic_atomic_model<UDP_Input_Landing_Point, TIME, TIME, bool, string, string>("im_lp_recv", std::move(TIME("00:00:00:100")), true, std::move("127.0.0.1"), std::move("23003"));
	shared_ptr<dynamic::modeling::model> im_plp_ach =
		dynamic::translate::make_dynamic_atomic_model<UDP_Input_Landing_Point, TIME, TIME, bool, string, string>("im_plp_ach", std::move(TIME("00:00:00:100")), true, std::move("127.0.0.1"), std::move("23004"));

	shared_ptr<dynamic::modeling::model> im_aircraft_state =
		dynamic::translate::make_dynamic_atomic_model<Shared_Memory_Input_Aircraft_State, TIME, TIME, string>("im_aircraft_state", std::move(TIME("00:00:00:100")), std::move(DEFAULT_SHARED_MEMORY_NAME));

	// The models to be included in this coupled model 
	// (accepts atomic and coupled models)
	dynamic::modeling::Models submodels_TestDriver = {
		supervisor,
		im_landing_achieved,
		im_aircraft_state,
		im_pilot_takeover,
		im_lp_recv,
		im_plp_ach
	};

	dynamic::modeling::Ports iports_TestDriver = { };

	dynamic::modeling::Ports oports_TestDriver = {
		typeid(o_LP_expired),
		typeid(o_start_LZE_scan),
		typeid(o_mission_complete),
		typeid(o_land_requested),
		typeid(o_fcc_command_velocity),
		typeid(o_control_yielded),
		typeid(o_notify_pilot),
		typeid(o_fcc_command_hover)
	};

	dynamic::modeling::EICs eics_TestDriver = { };

	// The output ports will be used to export in logging
	dynamic::modeling::EOCs eocs_TestDriver = {
		dynamic::translate::make_EOC<Supervisor_defs::o_LP_expired, o_LP_expired>("supervisor"),
		dynamic::translate::make_EOC<Supervisor_defs::o_start_LZE_scan, o_start_LZE_scan>("supervisor"),
		dynamic::translate::make_EOC<Supervisor_defs::o_mission_complete, o_mission_complete>("supervisor"),
		dynamic::translate::make_EOC<Supervisor_defs::o_land_requested, o_land_requested>("supervisor"),
		dynamic::translate::make_EOC<Supervisor_defs::o_fcc_command_velocity, o_fcc_command_velocity>("supervisor"),
		dynamic::translate::make_EOC<Supervisor_defs::o_control_yielded, o_control_yielded>("supervisor"),
		dynamic::translate::make_EOC<Supervisor_defs::o_notify_pilot, o_notify_pilot>("supervisor"),
		dynamic::translate::make_EOC<Supervisor_defs::o_fcc_command_hover, o_fcc_command_hover>("supervisor")
	};

	// This will connect our outputs from our input reader to the file
	dynamic::modeling::ICs ics_TestDriver = {
		dynamic::translate::make_IC<UDP_Input_defs<bool>::out, Supervisor_defs::i_landing_achieved>("im_landing_achieved", "supervisor"),
		dynamic::translate::make_IC<Shared_Memory_Input_defs<message_aircraft_state_t>::out, Supervisor_defs::i_aircraft_state>("im_aircraft_state", "supervisor"),
		dynamic::translate::make_IC<UDP_Input_defs<bool>::out, Supervisor_defs::i_pilot_takeover>("im_pilot_takeover", "supervisor"),
		dynamic::translate::make_IC<UDP_Input_defs<message_landing_point_t>::out, Supervisor_defs::i_LP_recv>("im_lp_recv", "supervisor"),
		dynamic::translate::make_IC<UDP_Input_defs<message_landing_point_t>::out, Supervisor_defs::i_PLP_ach>("im_plp_ach", "supervisor")
	};

	shared_ptr<dynamic::modeling::coupled<TIME>> test_driver = make_shared<dynamic::modeling::coupled<TIME>>(
		"test_driver", submodels_TestDriver, iports_TestDriver, oports_TestDriver, eics_TestDriver, eocs_TestDriver, ics_TestDriver
	);

	/*************** Loggers *******************/
	out_messages = ofstream(out_messages_file);
	struct oss_sink_messages {
		static ostream& sink() {
			return out_messages;
		}
	};

	out_state = ofstream(out_state_file);
	struct oss_sink_state {
		static ostream& sink() {
			return out_state;
		}
	};

	out_info = ofstream(out_info_file);
	struct oss_sink_info {
		static ostream& sink() {
			return out_info;
		}
	};

	using state = logger::logger<logger::logger_state, dynamic::logger::formatter<TIME>, oss_sink_state>;
	using log_messages = logger::logger<logger::logger_messages, dynamic::logger::formatter<TIME>, oss_sink_messages>;
	using global_time_mes = logger::logger<logger::logger_global_time, dynamic::logger::formatter<TIME>, oss_sink_messages>;
	using global_time_sta = logger::logger<logger::logger_global_time, dynamic::logger::formatter<TIME>, oss_sink_state>;
	using info = logger::logger<logger::logger_info, dynamic::logger::formatter<TIME>, oss_sink_info>;
	using logger_supervisor = logger::multilogger<state, log_messages, global_time_mes, global_time_sta, info>;

	auto start = hclock::now(); //to measure simulation execution time

	cadmium::dynamic::engine::runner<NDTime, logger_supervisor> r(test_driver, { TIME("00:00:00:000:000") });
	r.run_until_passivate();

	auto elapsed = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1>>>(hclock::now() - start).count();
	cout << "Simulation took: " << elapsed << " seconds" << endl;

	return 0;
}
