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
#include "io_models/Supervisor_UDP_Input.hpp"
#include "io_models/Aircraft_State_Input.hpp"
#include "io_models/Landing_Achieved_Demand_Input.hpp"
#include "io_models/Pilot_Takeover_Input.hpp"

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

int main(int argc, char* argv[]) {
	const string o_base_dir = string(PROJECT_DIRECTORY) + string("/test/simulation_results/supervisor/0");

	string out_directory = o_base_dir;
	string out_messages_file = out_directory + string("/output_messages.txt");
	string out_state_file = out_directory + string("/output_state.txt");
	string out_info_file = out_directory + string("/output_info.txt");

	// Create the output location
	filesystem::create_directories(out_directory.c_str()); // Creates if it does not exist. Does nothing if it does.

	// Instantiate the coupled model to test
	Supervisor supervisor_instance = Supervisor();
	shared_ptr<dynamic::modeling::coupled<TIME>> supervisor = make_shared<dynamic::modeling::coupled<TIME>>("supervisor", supervisor_instance.submodels, supervisor_instance.iports, supervisor_instance.oports, supervisor_instance.eics, supervisor_instance.eocs, supervisor_instance.ics);

	// Instantiate the input readers.
	// One for each input
	shared_ptr<dynamic::modeling::model> im_udp_interface = dynamic::translate::make_dynamic_atomic_model<Supervisor_UDP_Input, TIME, TIME, unsigned short>("im_udp_interface", std::move(TIME("00:00:00:100")), std::move(23001));

	shared_ptr<dynamic::modeling::model> im_aircraft_state = dynamic::translate::make_dynamic_atomic_model<Aircraft_State_Input, TIME>("im_aircraft_state");

	shared_ptr<dynamic::modeling::model> im_landing_achieved = dynamic::translate::make_dynamic_atomic_model<Landing_Achieved_Demand_Input, TIME, TIME, float>("im_landing_achieved", std::move(TIME("00:00:00:100")), DEFAULT_LAND_CRITERIA_VERT_DIST);

	shared_ptr<dynamic::modeling::model> im_pilot_takeover = dynamic::translate::make_dynamic_atomic_model<Pilot_Takeover_Input, TIME, TIME>("im_pilot_takeover", std::move(TIME("00:00:01:000")));

	// The models to be included in this coupled model 
	// (accepts atomic and coupled models)
	dynamic::modeling::Models submodels_TestDriver = {
		supervisor,
		im_landing_achieved,
		im_aircraft_state,
		im_pilot_takeover,
		im_udp_interface
	};

	dynamic::modeling::Ports iports_TestDriver = { };

	dynamic::modeling::Ports oports_TestDriver = { };

	dynamic::modeling::EICs eics_TestDriver = { };

	// The output ports will be used to export in logging
	dynamic::modeling::EOCs eocs_TestDriver = {	};

	// This will connect our outputs from our input reader to the file
	dynamic::modeling::ICs ics_TestDriver = {
		dynamic::translate::make_IC<Landing_Achieved_Demand_Input_defs::o_message, Supervisor_defs::i_landing_achieved>("im_landing_achieved", "supervisor"),
		dynamic::translate::make_IC<Supervisor_defs::o_land_requested, Landing_Achieved_Demand_Input_defs::i_start>("supervisor", "im_landing_achieved"),
		dynamic::translate::make_IC<Supervisor_defs::o_mission_complete, Landing_Achieved_Demand_Input_defs::i_quit>("supervisor", "im_landing_achieved"),

		dynamic::translate::make_IC<Aircraft_State_Input_defs::o_message, Supervisor_defs::i_aircraft_state>("im_aircraft_state", "supervisor"),
		dynamic::translate::make_IC<Supervisor_defs::o_request_aircraft_state, Aircraft_State_Input_defs::i_request>("supervisor", "im_aircraft_state"),

		dynamic::translate::make_IC<Pilot_Takeover_Input_defs::o_message, Supervisor_defs::i_pilot_takeover>("im_pilot_takeover", "supervisor"),
		dynamic::translate::make_IC<Supervisor_defs::o_mission_complete, Pilot_Takeover_Input_defs::i_quit>("supervisor", "im_pilot_takeover"),

		dynamic::translate::make_IC<Supervisor_UDP_Input_defs::o_lp_recv, Supervisor_defs::i_LP_recv>("im_udp_interface", "supervisor"),
		dynamic::translate::make_IC<Supervisor_UDP_Input_defs::o_plp_ach, Supervisor_defs::i_PLP_ach>("im_udp_interface", "supervisor"),
		dynamic::translate::make_IC<Supervisor_UDP_Input_defs::o_perception_status, Supervisor_defs::i_perception_status>("im_udp_interface", "supervisor"),
		dynamic::translate::make_IC<Supervisor_UDP_Input_defs::o_start_supervisor, Supervisor_defs::i_start_supervisor>("im_udp_interface", "supervisor"),
		dynamic::translate::make_IC<Supervisor_UDP_Input_defs::o_waypoint, Supervisor_defs::i_waypoint>("im_udp_interface", "supervisor"),
		dynamic::translate::make_IC<Supervisor_defs::o_mission_complete, Supervisor_UDP_Input_defs::i_quit>("supervisor", "im_udp_interface")
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

	fflush(NULL);
	string path_to_script = PROJECT_DIRECTORY + string("/test/scripts/simulation_cleanup.py");
	string path_to_simulation_results = PROJECT_DIRECTORY + string("/test/simulation_results");
	if (std::system("python3 --version") == 0) {
		string command = "python3 " + path_to_script + string(" ") + path_to_simulation_results;
		std::system(command.c_str());
	} else if (std::system("python --version") == 0) {
		string command = "python " + path_to_script + string(" ") + path_to_simulation_results;
		std::system(command.c_str());
	} else {
		cout << "\nPython is not installed!\n";
	}

	return 0;
}