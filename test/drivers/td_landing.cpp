//C++ headers
#include <chrono>
#include <string>
#include <iostream>
#include <filesystem>

//Cadmium Simulator headers
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/logger/common_loggers.hpp>

//Time class header
#include <NDTime.hpp>

//Messages structures
#include "message_structures/message_landing_point_t.hpp"

// Project information headers this is created by cmake at generation time!!!!
#include "SupervisorConfig.hpp"
#include "input_readers.hpp" // Input Reader Definitions.

//Coupled model headers
#include "coupled_models/Landing.hpp"

using namespace cadmium;
using namespace cadmium::basic_models::pdevs;
using TIME = NDTime;

int main() {
	int test_set_enumeration = 0;

	const string i_base_dir = string(PROJECT_DIRECTORY) + string("/test/input_data/landing/");
	const string o_base_dir = string(PROJECT_DIRECTORY) + string("/test/simulation_results/landing/");

	do {
		// Input Files
		string input_dir = i_base_dir + to_string(test_set_enumeration);
		string input_file_landing_achieved = input_dir + string("/landing_achieved.txt");
		string input_file_aircraft_state = input_dir + string("/aircraft_state.txt");
		string input_file_pilot_takeover = input_dir + string("/pilot_takeover.txt");
		string input_file_LP_recv = input_dir + string("/LP_recv.txt");
		string input_file_PLP_ach = input_dir + string("/PLP_ach.txt");
		string input_file_start_mission = input_dir + string("/start_mission.txt");

		// Output locations
		string out_directory = o_base_dir + to_string(test_set_enumeration);
		string out_messages_file = out_directory + string("/output_messages.txt");
		string out_state_file = out_directory + string("/output_state.txt");

		if (!filesystem::exists(input_file_landing_achieved) ||
			!filesystem::exists(input_file_aircraft_state) ||
			!filesystem::exists(input_file_pilot_takeover) ||
			!filesystem::exists(input_file_LP_recv) ||
			!filesystem::exists(input_file_PLP_ach) ||
			!filesystem::exists(input_file_start_mission)) {
			printf("One of the input files do not exist\n");
			return 1;
		}

		// Create the output location
		filesystem::create_directories(out_directory.c_str()); // Creates if it does not exist. Does nothing if it does.

		// Instantiate the coupled model to test
		Landing landing = Landing();
		shared_ptr<dynamic::modeling::coupled<TIME>> ldg = make_shared<dynamic::modeling::coupled<TIME>>("landing", landing.submodels, landing.iports, landing.oports, landing.eics, landing.eocs, landing.ics);

		// Instantiate the input readers.
		// One for each input
		shared_ptr<dynamic::modeling::model> ir_landing_achieved =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_landing_achieved", input_file_landing_achieved.c_str());
		shared_ptr<dynamic::modeling::model> ir_aircraft_state =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Aircraft_State, TIME, const char* >("ir_aircraft_state", input_file_aircraft_state.c_str());
		shared_ptr<dynamic::modeling::model> ir_pilot_takeover =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_pilot_takeover", input_file_pilot_takeover.c_str());
		shared_ptr<dynamic::modeling::model> ir_lp_recv =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Mavlink_Mission_Item, TIME, const char* >("ir_lp_recv", input_file_LP_recv.c_str());
		shared_ptr<dynamic::modeling::model> ir_plp_ach =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Mavlink_Mission_Item, TIME, const char* >("ir_plp_ach", input_file_PLP_ach.c_str());
        shared_ptr<dynamic::modeling::model> ir_start_mission =
                dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_start_mission", input_file_start_mission.c_str());


		// The models to be included in this coupled model
		// (accepts atomic and coupled models)
		dynamic::modeling::Models submodels_TestDriver = {
				ldg,
				ir_landing_achieved,
				ir_aircraft_state,
				ir_pilot_takeover,
				ir_lp_recv,
				ir_plp_ach,
                ir_start_mission
		};

		dynamic::modeling::Ports iports_TestDriver = { };

		dynamic::modeling::Ports oports_TestDriver = { };

		dynamic::modeling::EICs eics_TestDriver = {	};

		// The output ports will be used to export in logging
		dynamic::modeling::EOCs eocs_TestDriver = { };

		// This will connect our outputs from our input reader to the file
		dynamic::modeling::ICs ics_TestDriver = {
			dynamic::translate::make_IC<iestream_input_defs<bool>::out, Landing::defs::i_landing_achieved>("ir_landing_achieved", "landing"),
			dynamic::translate::make_IC<iestream_input_defs<message_aircraft_state_t>::out, Landing::defs::i_aircraft_state>("ir_aircraft_state", "landing"),
			dynamic::translate::make_IC<iestream_input_defs<bool>::out, Landing::defs::i_pilot_takeover>("ir_pilot_takeover", "landing"),
			dynamic::translate::make_IC<iestream_input_defs<message_landing_point_t>::out, Landing::defs::i_LP_recv>("ir_lp_recv", "landing"),
			dynamic::translate::make_IC<iestream_input_defs<message_landing_point_t>::out, Landing::defs::i_PLP_ach>("ir_plp_ach", "landing"),
			dynamic::translate::make_IC<iestream_input_defs<bool>::out, Landing::defs::i_start_mission>("ir_start_mission", "landing")
		};

		shared_ptr<dynamic::modeling::coupled<TIME>> test_driver = make_shared<dynamic::modeling::coupled<TIME>>(
			"test_driver", submodels_TestDriver, iports_TestDriver, oports_TestDriver, eics_TestDriver, eocs_TestDriver, ics_TestDriver
		);

		/*************** Loggers *******************/
        static ofstream out_messages;
        static ofstream out_state;

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

		using state = logger::logger<logger::logger_state, dynamic::logger::formatter<TIME>, oss_sink_state>;
		using log_messages = logger::logger<logger::logger_messages, dynamic::logger::formatter<TIME>, oss_sink_messages>;
		using global_time_mes = logger::logger<logger::logger_global_time, dynamic::logger::formatter<TIME>, oss_sink_messages>;
		using global_time_sta = logger::logger<logger::logger_global_time, dynamic::logger::formatter<TIME>, oss_sink_state>;

		using logger_supervisor = logger::multilogger<state, log_messages, global_time_mes, global_time_sta>;

		dynamic::engine::runner<NDTime, logger_supervisor> r(test_driver, { 0 });

		r.run_until_passivate();
		test_set_enumeration++;
	} while (filesystem::exists(i_base_dir + std::to_string(test_set_enumeration)));

	fflush(nullptr);
	string path_to_script = PROJECT_DIRECTORY + string("/test/scripts/simulation_cleanup.py");
	string path_to_simulation_results = PROJECT_DIRECTORY + string("/test/simulation_results");
	if (system("python3 --version") == 0) {
		string command = "python3 " + path_to_script + string(" ") + path_to_simulation_results;
		system(command.c_str());
	} else if (system("python --version") == 0) {
		string command = "python " + path_to_script + string(" ") + path_to_simulation_results;
		system(command.c_str());
	} else {
		cout << "\nPython is not installed!\n";
	}

	return 0;
}
