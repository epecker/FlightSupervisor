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

//Messages structures

// Project information headers this is created by cmake at generation time!!!!
#include "SupervisorConfig.hpp"
#include "input_readers.hpp" // Input Reader Definitions.

//Coupled model headers
#include "io_models/Shared_Memory_Output.hpp"

using namespace std;
using namespace cadmium;

using hclock = std::chrono::high_resolution_clock;
using TIME = NDTime;

// Used for oss_sink_state and oss_sink_messages
ofstream out_messages;
ofstream out_state;
ofstream out_info;
template<typename T>
class Shared_Memory_Output_LP : public Shared_Memory_Output<message_landing_point_t, T> {
public:
    Shared_Memory_Output_LP() = default;
    Shared_Memory_Output_LP(string name) : Shared_Memory_Output<message_landing_point_t, T>(name){}
};

int main(int argc, char* argv[]) {
	int test_set_enumeration = 0;

	const string i_base_dir = string(PROJECT_DIRECTORY) + string("/test/input_data/shared_memory_output/");
	const string o_base_dir = string(PROJECT_DIRECTORY) + string("/test/simulation_results/shared_memory_output/");

	do {
		// Input Files
		string input_dir = i_base_dir + to_string(test_set_enumeration);
		string input_file_in = input_dir + string("/message.txt");

		// Output locations
		string out_directory = o_base_dir + to_string(test_set_enumeration);
		string out_messages_file = out_directory + string("/output_messages.txt");
		string out_state_file = out_directory + string("/output_state.txt");
		string out_info_file = out_directory + string("/output_info.txt");

		if (!filesystem::exists(input_file_in)) {
			printf("One of the input files do not exist\n");
			return 1;
		}

		// Create the output location
		filesystem::create_directories(out_directory.c_str()); // Creates if it does not exist. Does nothing if it does.

		// Instantiate the atomic model to test
		std::shared_ptr<dynamic::modeling::model> shared_memory_output = dynamic::translate::make_dynamic_atomic_model<Shared_Memory_Output_LP, TIME, const char*>("shared_memory_output", std::move(DEFAULT_SHARED_MEMORY_NAME));

		// Instantiate the input readers.
		// One for each input
		std::shared_ptr<dynamic::modeling::model> ir_message =
			dynamic::translate::make_dynamic_atomic_model<Input_Reader_Mavlink_Mission_Item, TIME, const char* >("ir_message", std::move(input_file_in.c_str()));

		// The models to be included in this coupled model 
		// (accepts atomic and coupled models)
		dynamic::modeling::Models submodels_TestDriver = {
			shared_memory_output,
			ir_message
		};

		dynamic::modeling::Ports iports_TestDriver = {	};

		dynamic::modeling::Ports oports_TestDriver = {
		};

		dynamic::modeling::EICs eics_TestDriver = {	};

		// The output ports will be used to export in logging
		dynamic::modeling::EOCs eocs_TestDriver = {
		};

		// This will connect our outputs from our input reader to the file
		dynamic::modeling::ICs ics_TestDriver = {
			dynamic::translate::make_IC<iestream_input_defs<message_landing_point_t>::out, Shared_Memory_Output_defs<message_landing_point_t>::i_message>("ir_message", "shared_memory_output")
		};

		std::shared_ptr<dynamic::modeling::coupled<TIME>> test_driver = std::make_shared<dynamic::modeling::coupled<TIME>>(
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
		r.run_until(TIME("00:00:20:000:000"));

		auto elapsed = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1>>>(hclock::now() - start).count();
		cout << "\nSimulation took: " << elapsed << " seconds" << endl;

		test_set_enumeration++;
	} while (filesystem::exists(i_base_dir + std::to_string(test_set_enumeration)));

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
