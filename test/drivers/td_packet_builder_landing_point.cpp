// C++ headers
#include <string>
#include <boost/filesystem.hpp>

// Cadmium Simulator headers
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>

// Time class header
#include <NDTime.hpp>

// Required for testing
#include "../../src/SupervisorConfig.hpp" // Generated by cmake.
#include "../../src/input_readers.hpp" // Input Reader Definitions.

#include "../../src/io_models/Packet_Builder.hpp"

using TIME = NDTime;

using namespace cadmium;

struct o_packet : public out_port<std::vector<char>> {};

int main() {
	int test_set_enumeration = 0;

	const std::string i_base_dir = std::string(PROJECT_DIRECTORY) + std::string("/test/input_data/packet_builder_landing_point/");
	const std::string o_base_dir = std::string(PROJECT_DIRECTORY) + std::string("/test/simulation_results/packet_builder_landing_point/");

	do {
		// Input locations
		string input_dir						= i_base_dir + to_string(test_set_enumeration);
		string input_file_initial_state			= input_dir + std::string("/initial_state.txt");
		string input_file_data				= input_dir + std::string("/data.txt");

		// Output locations
		string out_directory					= o_base_dir + to_string(test_set_enumeration);
		string out_messages_file				= out_directory + std::string("/output_messages.txt");
		string out_state_file					= out_directory + std::string("/output_state.txt");

		if (!boost::filesystem::exists(input_file_initial_state) ||
			!boost::filesystem::exists(input_file_data)) {
			printf("One of the input files do not exist\n");
			return 1;
		}

		// Create the output location
		boost::filesystem::create_directories(out_directory.c_str());

		// Instantiate the atomic model to test
		shared_ptr<dynamic::modeling::model> packet_builder =
				dynamic::translate::make_dynamic_atomic_model<Packet_Builder_Landing_Point, TIME>("packet_builder");

		// Instantiate the input readers.
		// One for each input
		shared_ptr<dynamic::modeling::model> ir_data =
				dynamic::translate::make_dynamic_atomic_model<Input_Reader_Mavlink_Mission_Item, TIME, const char* >("ir_data", input_file_data.c_str());

		// The models to be included in this coupled model
		// (accepts atomic and coupled models)
		dynamic::modeling::Models submodels_TestDriver = {
				ir_data,
				packet_builder
		};

		dynamic::modeling::Ports iports_TestDriver = {	};

		dynamic::modeling::Ports oports_TestDriver = {
				typeid(o_packet)
		};

		dynamic::modeling::EICs eics_TestDriver = {	};

		// The output ports will be used to export in logging
		dynamic::modeling::EOCs eocs_TestDriver = {
				dynamic::translate::make_EOC<Packet_Builder_Landing_Point<TIME>::defs::o_packet,o_packet>("packet_builder")
		};

		// This will connect our outputs from our input reader to the file
		dynamic::modeling::ICs ics_TestDriver = {
				dynamic::translate::make_IC<iestream_input_defs<message_landing_point_t>::out, Packet_Builder_Landing_Point<TIME>::defs::i_data>("ir_data", "packet_builder")
		};

		shared_ptr<dynamic::modeling::coupled<TIME>> TEST_DRIVER = make_shared<dynamic::modeling::coupled<TIME>>(
				"TEST_DRIVER", submodels_TestDriver, iports_TestDriver, oports_TestDriver, eics_TestDriver, eocs_TestDriver, ics_TestDriver
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

		dynamic::engine::runner<NDTime, logger_supervisor> r(TEST_DRIVER, { 0 });

		r.run_until_passivate();
		test_set_enumeration++;
	} while (boost::filesystem::exists(i_base_dir + std::to_string(test_set_enumeration)));

	fflush(nullptr);
	string path_to_script = PROJECT_DIRECTORY + std::string("/test/scripts/simulation_cleanup.py");
	string path_to_simulation_results = PROJECT_DIRECTORY + std::string("/test/simulation_results");
	if (system("python3 --version") == 0) {
		string command = "python3 " + path_to_script + std::string(" ") + path_to_simulation_results;
		system(command.c_str());
	} else if (system("python --version") == 0) {
		string command = "python " + path_to_script + std::string(" ") + path_to_simulation_results;
		system(command.c_str());
	} else {
		cout << "\nPython is not installed!\n";
	}

	return 0;
}
