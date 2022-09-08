//C++ headers
#include <chrono>
#include <string>
#include <iostream>
#include <boost/filesystem.hpp>

//Cadmium Simulator headers
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>

//Time class header
#include <NDTime.hpp>

// Project information headers this is created by cmake at generation time!!!!
#include "../../src/SupervisorConfig.hpp"
#include "../../src/input_readers.hpp" // Input Reader Definitions.

//Coupled model headers
#include "../../src/io_models/Polling_Condition_Input.hpp"

using namespace cadmium;

using hclock = std::chrono::high_resolution_clock;
using TIME = NDTime;

int main() {
	int test_set_enumeration = 0;

	const string i_base_dir = string(PROJECT_DIRECTORY) + string("/test/input_data/polling_condition_input_takeover/");
	const string o_base_dir = string(PROJECT_DIRECTORY) + string("/test/simulation_results/polling_condition_input_takeover/");

	do {
		// Input Files
		string input_dir = i_base_dir + to_string(test_set_enumeration);
		string input_file_start = input_dir + string("/start.txt");
		string input_file_quit = input_dir + string("/quit.txt");

		// Output locations
		string out_directory = o_base_dir + to_string(test_set_enumeration);
		string out_messages_file = out_directory + string("/output_messages.txt");
		string out_state_file = out_directory + string("/output_state.txt");
		string out_info_file = out_directory + string("/output_info.txt");

		if (!boost::filesystem::exists(input_file_start) ||
			!boost::filesystem::exists(input_file_quit)) {
			printf("One of the input files do not exist\n");
			return 1;
		}

		// Create the output location
		boost::filesystem::create_directories(out_directory.c_str()); // Creates if it does not exist. Does nothing if it does.

		// Instantiate the atomic model to test
		std::shared_ptr<cadmium::dynamic::modeling::model> polling_condition_input_takeover = cadmium::dynamic::translate::make_dynamic_atomic_model<Polling_Condition_Input_Pilot_Takeover, TIME, TIME>("polling_condition_input_takeover", std::move(TIME("00:00:00:100")));

		// Instantiate the input readers.
		// One for each input
		std::shared_ptr<cadmium::dynamic::modeling::model> ir_start =
			cadmium::dynamic::translate::make_dynamic_atomic_model<Input_Reader_Start_Supervisor, TIME, const char* >("ir_start", input_file_start.c_str());
		std::shared_ptr<cadmium::dynamic::modeling::model> ir_quit =
			cadmium::dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_quit", input_file_quit.c_str());

		// The models to be included in this coupled model
		// (accepts atomic and coupled models)
		cadmium::dynamic::modeling::Models submodels_TestDriver = {
			polling_condition_input_takeover,
            ir_start,
			ir_quit
		};

		cadmium::dynamic::modeling::Ports iports_TestDriver = { };

		cadmium::dynamic::modeling::Ports oports_TestDriver = { };

		cadmium::dynamic::modeling::EICs eics_TestDriver = {	};

		// The output ports will be used to export in logging
		cadmium::dynamic::modeling::EOCs eocs_TestDriver = { };

		// This will connect our outputs from our input reader to the file
		cadmium::dynamic::modeling::ICs ics_TestDriver = {
			cadmium::dynamic::translate::make_IC<cadmium::basic_models::pdevs::iestream_input_defs<message_start_supervisor_t>::out, Polling_Condition_Input_Pilot_Takeover<TIME>::defs::i_start>("ir_start", "polling_condition_input_takeover"),
			cadmium::dynamic::translate::make_IC<cadmium::basic_models::pdevs::iestream_input_defs<bool>::out, Polling_Condition_Input_Pilot_Takeover<TIME>::defs::i_quit>("ir_quit", "polling_condition_input_takeover")
		};

		std::shared_ptr<cadmium::dynamic::modeling::coupled<TIME>> test_driver = std::make_shared<cadmium::dynamic::modeling::coupled<TIME>>(
			"test_driver", submodels_TestDriver, iports_TestDriver, oports_TestDriver, eics_TestDriver, eocs_TestDriver, ics_TestDriver
		);

		/*************** Loggers *******************/
        static ofstream out_messages;
        static ofstream out_state;
        static ofstream out_info;

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

		using state = logger::logger<logger::logger_state, cadmium::dynamic::logger::formatter<TIME>, oss_sink_state>;
		using log_messages = logger::logger<logger::logger_messages, cadmium::dynamic::logger::formatter<TIME>, oss_sink_messages>;
		using global_time_mes = logger::logger<logger::logger_global_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_messages>;
		using global_time_sta = logger::logger<logger::logger_global_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_state>;
		using info = logger::logger<logger::logger_info, cadmium::dynamic::logger::formatter<TIME>, oss_sink_info>;
		using logger_top = cadmium::logger::multilogger<state, log_messages, global_time_mes, global_time_sta, info>;

		auto start = hclock::now(); //to measure simulation execution time

		cadmium::dynamic::engine::runner<NDTime, logger_top> r(test_driver, { TIME("00:00:00:000:000") });
		r.run_until_passivate();

		auto elapsed = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1>>>(hclock::now() - start).count();
		cout << "\nSimulation took: " << elapsed << " seconds" << endl;

		test_set_enumeration++;
	} while (boost::filesystem::exists(i_base_dir + std::to_string(test_set_enumeration)));

	fflush(nullptr);
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
