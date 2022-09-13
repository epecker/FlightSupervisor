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
#include "../../src/io_models/Cache_Input.hpp"

using namespace cadmium;

using hclock = std::chrono::high_resolution_clock;
using TIME = NDTime;

template<typename T>
class Cache_Input_LP : public Cache_Input<message_landing_point_t, T> {
public:
    Cache_Input_LP() = default;
    explicit Cache_Input_LP(message_landing_point_t initial_lp) : Cache_Input<message_landing_point_t, T>(initial_lp){}
};

int main() {
	int test_set_enumeration = 0;

	const string i_base_dir = string(PROJECT_DIRECTORY) + string("/test/input_data/cache_input/");
	const string o_base_dir = string(PROJECT_DIRECTORY) + string("/test/simulation_results/cache_input/");

	do {
		// Input Files
		string input_dir = i_base_dir + to_string(test_set_enumeration);
		string input_file_new_input = input_dir + string("/new_input.txt");
		string input_file_get_input = input_dir + string("/get_input.txt");

		// Output locations
		string out_directory = o_base_dir + to_string(test_set_enumeration);
		string out_messages_file = out_directory + string("/output_messages.txt");
		string out_state_file = out_directory + string("/output_state.txt");
		string out_info_file = out_directory + string("/output_info.txt");

		if (!boost::filesystem::exists(input_file_new_input) ||
			!boost::filesystem::exists(input_file_get_input)
		) {
			printf("One of the input files do not exist\n");
			return 1;
		}

		// Create the output location
		boost::filesystem::create_directories(out_directory.c_str()); // Creates if it does not exist. Does nothing if it does.

		// Instantiate the atomic model to test
		std::shared_ptr<cadmium::dynamic::modeling::model> cache_input = cadmium::dynamic::translate::make_dynamic_atomic_model<Cache_Input_LP, TIME, message_landing_point_t>("cache_input", message_landing_point_t(0, 10, 45.0, -75.0, 100.0, 5.0));

		// Instantiate the input readers.
		// One for each input
		std::shared_ptr<cadmium::dynamic::modeling::model> ir_new_input =
			cadmium::dynamic::translate::make_dynamic_atomic_model<Input_Reader_Mavlink_Mission_Item, TIME, const char* >("ir_new_input", input_file_new_input.c_str());
		std::shared_ptr<cadmium::dynamic::modeling::model> ir_get_input =
			cadmium::dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_get_input", input_file_get_input.c_str());

		// The models to be included in this coupled model
		// (accepts atomic and coupled models)
	 	cadmium::dynamic::modeling::Models submodels_TestDriver = {
			cache_input,
			ir_new_input,
			ir_get_input
		};

	 	cadmium::dynamic::modeling::Ports iports_TestDriver = { };

	 	cadmium::dynamic::modeling::Ports oports_TestDriver = { };

	 	cadmium::dynamic::modeling::EICs eics_TestDriver = {	};

		// The output ports will be used to export in logging
	 	cadmium::dynamic::modeling::EOCs eocs_TestDriver = { };

		// This will connect our outputs from our input reader to the file
	 	cadmium::dynamic::modeling::ICs ics_TestDriver = {
			cadmium::dynamic::translate::make_IC<cadmium::basic_models::pdevs::iestream_input_defs<message_landing_point_t>::out, Cache_Input_defs<message_landing_point_t>::i_new_input>("ir_new_input", "cache_input"),
			cadmium::dynamic::translate::make_IC<cadmium::basic_models::pdevs::iestream_input_defs<bool>::out, Cache_Input_defs<message_landing_point_t>::i_get_input>("ir_get_input", "cache_input")
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
