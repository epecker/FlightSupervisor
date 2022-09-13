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
#include "../../src/io_models/Supervisor_UDP_Input.hpp"

using namespace cadmium;

using hclock = std::chrono::high_resolution_clock;
using TIME = NDTime;

// Used for oss_sink_state and oss_sink_messages
ofstream out_messages;
ofstream out_state;
ofstream out_info;

// Define output ports to be used for logging purposes

int main() {
	int test_set_enumeration = 0;

	const string i_base_dir = string(PROJECT_DIRECTORY) + string("/test/input_data/supervisor_udp_input/");
	const string o_base_dir = string(PROJECT_DIRECTORY) + string("/test/simulation_results/supervisor_udp_input/");

	do {
		// Input Files
		string input_dir = i_base_dir + to_string(test_set_enumeration);
		string input_file_quit = input_dir + string("/quit.txt");

		// Output locations
		string out_directory = o_base_dir + to_string(test_set_enumeration);
		string out_messages_file = out_directory + string("/output_messages.txt");
		string out_state_file = out_directory + string("/output_state.txt");
		string out_info_file = out_directory + string("/output_info.txt");

		if (!boost::filesystem::exists(input_file_quit)) {
			printf("One of the input files do not exist\n");
			return 1;
		}

		// Create the output location
		boost::filesystem::create_directories(out_directory.c_str()); // Creates if it does not exist. Does nothing if it does.

		// Instantiate the atomic model to test
  		unsigned short port = 23000;
		std::shared_ptr<cadmium::dynamic::modeling::model> supervisor_udp_input = cadmium::dynamic::translate::make_dynamic_atomic_model<Supervisor_UDP_Input, TIME, TIME, unsigned short>("supervisor_udp_input", std::move(TIME("00:00:00:100")), std::move(port));

		// Instantiate the input readers.
		// One for each input
		std::shared_ptr<cadmium::dynamic::modeling::model> ir_quit =
			cadmium::dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_quit", input_file_quit.c_str());

		// The models to be included in this coupled model
		// (accepts atomic and coupled models)
		cadmium::dynamic::modeling::Models submodels_TestDriver = {
			supervisor_udp_input,
            ir_quit
		};

		cadmium::dynamic::modeling::Ports iports_TestDriver = { };

		cadmium::dynamic::modeling::Ports oports_TestDriver = { };

		cadmium::dynamic::modeling::EICs eics_TestDriver = {	};

		// The output ports will be used to export in logging
		cadmium::dynamic::modeling::EOCs eocs_TestDriver = { };

		// This will connect our outputs from our input reader to the file
		cadmium::dynamic::modeling::ICs ics_TestDriver = {
			cadmium::dynamic::translate::make_IC<cadmium::basic_models::pdevs::iestream_input_defs<bool>::out, Supervisor_UDP_Input<TIME>::defs::i_quit>("ir_quit", "supervisor_udp_input")
		};

		std::shared_ptr<cadmium::dynamic::modeling::coupled<TIME>> test_driver = std::make_shared<cadmium::dynamic::modeling::coupled<TIME>>(
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

		using state = cadmium::logger::logger<cadmium::logger::logger_state, cadmium::dynamic::logger::formatter<TIME>, oss_sink_state>;
		using log_messages = cadmium::logger::logger<cadmium::logger::logger_messages, cadmium::dynamic::logger::formatter<TIME>, oss_sink_messages>;
		using global_time_mes = cadmium::logger::logger<cadmium::logger::logger_global_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_messages>;
		using global_time_sta = cadmium::logger::logger<cadmium::logger::logger_global_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_state>;
		using info = cadmium::logger::logger<cadmium::logger::logger_info, cadmium::dynamic::logger::formatter<TIME>, oss_sink_info>;
		using logger_top = cadmium::logger::multilogger<state, log_messages, global_time_mes, global_time_sta, info>;

		auto start = hclock::now(); //to measure simulation execution time

		cadmium::dynamic::engine::runner<NDTime, logger_top> r(test_driver, { TIME("00:00:00:000:000") });
		r.run_until_passivate();

		auto elapsed = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1>>>(hclock::now() - start).count();
		cout << "\nSimulation took: " << elapsed << " seconds" << endl;

		test_set_enumeration++;
	} while (boost::filesystem::exists(i_base_dir + std::to_string(test_set_enumeration)));

	return 0;
}
