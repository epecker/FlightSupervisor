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

//Time class header
#include <NDTime.hpp>

//Messages structures
#include "../../include/message_structures/message_mavlink_mission_item_t.hpp"
#include "../../include/message_structures/message_fcc_command_t.hpp"

// Project information headers this is created by cmake at generation time!!!!
#include "../../include/SupervisorConfig.hpp"
//#include "../../include/input_readers.hpp" // Input Reader Definitions.

//Coupled model headers
#include "../../include/coupled_models/Supervisor.hpp"
#include "../../include/io_models/Supervisor_Command_Line_Input.hpp"

using namespace std;
using namespace cadmium;

using hclock = chrono::high_resolution_clock;
using TIME = NDTime;

// Used for oss_sink_state and oss_sink_messages
ofstream out_messages;
ofstream out_state;
ofstream out_info;

// Define output ports to be used for logging purposes
struct o_LP_expired : public out_port<message_mavlink_mission_item_t> {};
struct o_start_LZE_scan : public out_port<bool> {};
struct o_mission_complete : public out_port<bool> {};
struct o_land_requested : public out_port<bool> {};
struct o_fcc_command_velocity : public out_port<message_fcc_command_t> {};
struct o_control_yielded : public out_port<bool> {};
struct o_notify_pilot : public out_port<bool> {};
struct o_fcc_command_hover : public out_port<message_fcc_command_t> {};

/**
* ==========================================================
* MAIN METHOD
* ==========================================================
*/
int main(int argc, char* argv[]) {
	int test_set_enumeration = 0;

	const string i_base_dir = string(PROJECT_DIRECTORY) + string("/test/input_data/supervisor_command_line/");
	const string o_base_dir = string(PROJECT_DIRECTORY) + string("/test/simulation_results/supervisor_command_line/");

	do {
		// Input Files
		string input_dir = i_base_dir + to_string(test_set_enumeration);

		// Output locations
		string out_directory = o_base_dir + to_string(test_set_enumeration);
		string out_messages_file = out_directory + string("/output_messages.txt");
		string out_state_file = out_directory + string("/output_state.txt");
		string out_info_file = out_directory + string("/output_info.txt");

		// Create the output location
		filesystem::create_directories(out_directory.c_str()); // Creates if it does not exist. Does nothing if it does.

		// Instantiate the coupled model to test
		Supervisor supr = Supervisor();
		shared_ptr<dynamic::modeling::coupled<TIME>> supervisor = make_shared<dynamic::modeling::coupled<TIME>>("supervisor", supr.submodels, supr.iports, supr.oports, supr.eics, supr.eocs, supr.ics);

		// Instantiate the atomic model that will drive the coupled model
		shared_ptr<dynamic::modeling::model> supervisor_command_line_input = dynamic::translate::make_dynamic_atomic_model<Supervisor_Command_Line_Input, TIME, TIME>("supervisor_command_line_input", move(TIME("00:00:00:100")));

		// The models to be included in this coupled model 
		// (accepts atomic and coupled models)
		dynamic::modeling::Models submodels_TestDriver = {
			supervisor,
            supervisor_command_line_input
		};

		dynamic::modeling::Ports iports_TestDriver = {	};

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

		dynamic::modeling::EICs eics_TestDriver = {	};

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
			dynamic::translate::make_IC<Supervisor_Command_Line_Input_defs::o_landing_achieved, Supervisor_defs::i_landing_achieved>("supervisor_command_line_input", "supervisor"),
			dynamic::translate::make_IC<Supervisor_Command_Line_Input_defs::o_aircraft_state, Supervisor_defs::i_aircraft_state>("supervisor_command_line_input", "supervisor"),
			dynamic::translate::make_IC<Supervisor_Command_Line_Input_defs::o_pilot_takeover, Supervisor_defs::i_pilot_takeover>("supervisor_command_line_input", "supervisor"),
			dynamic::translate::make_IC<Supervisor_Command_Line_Input_defs::o_LP_recv, Supervisor_defs::i_LP_recv>("supervisor_command_line_input", "supervisor"),
			dynamic::translate::make_IC<Supervisor_Command_Line_Input_defs::o_PLP_ach, Supervisor_defs::i_PLP_ach>("supervisor_command_line_input", "supervisor")
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
		using logger_top = logger::multilogger<state, log_messages, global_time_mes, global_time_sta, info>;

		auto start = hclock::now(); //to measure simulation execution time

		cadmium::dynamic::engine::runner<NDTime, logger_top> r(test_driver, { TIME("00:00:00:000:000") });
		r.run_until_passivate();

		auto elapsed = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1>>>(hclock::now() - start).count();
		cout << "\nSimulation took: " << elapsed << " seconds" << endl;

		test_set_enumeration++;
	} while (filesystem::exists(i_base_dir + std::to_string(test_set_enumeration)));

    filesystem::current_path(PROJECT_DIRECTORY);
    system("pwd");

    string path_to_script = string("test/scripts/");
    string script_name = string("simulation_cleanup.py");
    string python_command = string("python3") + string(" ");
    string command = python_command + path_to_script + script_name;
    
    if(system(command.c_str()) != 0) {
        python_command = string("python") + string(" ");
        command = python_command + path_to_script + script_name;
        if(system(command.c_str()) != 0) {
            return -1;
        }
    }

	return 0;
}