//C++ headers
#include <chrono>
#include <string>
#include <iostream>
#include <filesystem>

//Cadmium Simulator headers
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>

//Time class header
#include <NDTime.hpp>

#include "SupervisorConfig.hpp" // Created during cmake generation
#include "input_readers.hpp"
#include "io_models/Aircraft_Gps_Time_Input.hpp"

using namespace cadmium;

using hclock = std::chrono::high_resolution_clock;
using TIME = NDTime;

// Define output ports to be used for logging purposes
struct out : public out_port<message_aircraft_state_t> {};

int main() {
	int test_set_enumeration = 0;

	const std::string i_base_dir = std::string(PROJECT_DIRECTORY) + std::string("/test/input_data/aircraft_gps_time_input/");
	const std::string o_base_dir = std::string(PROJECT_DIRECTORY) + std::string("/test/simulation_results/aircraft_gps_time_input/");

	do {
		// Input Files
	 	std::string input_dir = i_base_dir + to_string(test_set_enumeration);
	 	std::string input_file_request = input_dir + std::string("/request.txt");

		// Output locations
	 	std::string out_directory = o_base_dir + to_string(test_set_enumeration);
	 	std::string out_messages_file = out_directory + std::string("/output_messages.txt");
	 	std::string out_state_file = out_directory + std::string("/output_state.txt");
	 	std::string out_info_file = out_directory + std::string("/output_info.txt");

		if (!filesystem::exists(input_file_request)) {
			printf("One of the input files do not exist\n");
			return 1;
		}

		// Creates the output location if it doesn't exist
		filesystem::create_directories(out_directory.c_str());

		// Instantiate the atomic model to test
		std::shared_ptr<dynamic::modeling::model> aircraft_gps_time_input = dynamic::translate::make_dynamic_atomic_model<Aircraft_State_Input, TIME>("aircraft_gps_time_input");

		// Instantiate the input readers.
		// One for each input
		std::shared_ptr<dynamic::modeling::model> ir_request =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_request", input_file_request.c_str());

		// The models to be included in this coupled model
		// (accepts atomic and coupled models)
		dynamic::modeling::Models submodels_TestDriver = {
			aircraft_gps_time_input,
            ir_request
		};

		dynamic::modeling::Ports iports_TestDriver = {	};

		dynamic::modeling::Ports oports_TestDriver = {
            typeid(out)
        };

		dynamic::modeling::EICs eics_TestDriver = {	};

		// The output ports will be used to export in logging
		dynamic::modeling::EOCs eocs_TestDriver = { };

		// This will connect our outputs from our input reader to the file
		dynamic::modeling::ICs ics_TestDriver = {
			dynamic::translate::make_IC<iestream_input_defs<bool>::out, Aircraft_State_Input_defs::i_request>("ir_request", "aircraft_gps_time_input")
		};

		std::shared_ptr<dynamic::modeling::coupled<TIME>> test_driver = std::make_shared<dynamic::modeling::coupled<TIME>>(
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

	fflush(nullptr);
 	std::string path_to_script = PROJECT_DIRECTORY + std::string("/test/scripts/simulation_cleanup.py");
 	std::string path_to_simulation_results = PROJECT_DIRECTORY + std::string("/test/simulation_results");

	if (std::system("python3 --version") == 0) {
	 	std::string command = "python3 " + path_to_script + std::string(" ") + path_to_simulation_results;
		std::system(command.c_str());
	} else if (std::system("python --version") == 0) {
	 	std::string command = "python " + path_to_script + std::string(" ") + path_to_simulation_results;
		std::system(command.c_str());
	} else {
		cout << "\nPython is not installed!\n";
	}

	return 0;
}
