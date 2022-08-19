// C++ headers
#include <string>
#include <filesystem>

// Cadmium Simulator headers
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>

// Time class header
#include <NDTime.hpp>

// Required for testing
#include "SupervisorConfig.hpp" // Generated by cmake.
#include "input_readers.hpp" // Input Reader Definitions.

#include "atomic_models/Handle_Waypoint.hpp"

using TIME = NDTime;

struct o_fcc_waypoint_update : public out_port<message_fcc_command_t> {};
struct o_request_gps_time : public out_port<bool> {};

int main() {
	int test_set_enumeration = 0;

	const std::string i_base_dir = std::string(PROJECT_DIRECTORY) + std::string("/test/input_data/handle_waypoint/");
	const std::string o_base_dir = std::string(PROJECT_DIRECTORY) + std::string("/test/simulation_results/handle_waypoint/");

	do {
		// Input locations
		string input_dir						= i_base_dir + to_string(test_set_enumeration);
		string input_file_initial_state			= input_dir + std::string("/initial_state.txt");
		string input_file_pilot_takeover		= input_dir + std::string("/pilot_takeover.txt");
		string input_file_start_mission			= input_dir + std::string("/start_mission.txt");
		string input_file_waypoint				= input_dir + std::string("/waypoint.txt");

		// Output locations
		string out_directory					= o_base_dir + to_string(test_set_enumeration);
		string out_messages_file				= out_directory + std::string("/output_messages.txt");
		string out_state_file					= out_directory + std::string("/output_state.txt");

		if (!filesystem::exists(input_file_initial_state) ||
			!filesystem::exists(input_file_pilot_takeover) ||
			!filesystem::exists(input_file_start_mission) ||
			!filesystem::exists(input_file_waypoint)) {
			printf("One of the input files do not exist\n");
			return 1;
		}

		// Read the initial state of the model.
		string initial_state_string;
		fstream f;
		f.open(input_file_initial_state, ios::in);
		if (!f.is_open()) {
			printf("Failed to open initial_state file\n");
			return 1;
		}
		getline(f, initial_state_string);
		f.close();

		Handle_Waypoint<TIME>::States initial_state = Handle_Waypoint<TIME>::stringToEnum(initial_state_string);

		// Create the output location
		filesystem::create_directories(out_directory.c_str());

		// Instantiate the atomic model to test
		shared_ptr<dynamic::modeling::model> handle_waypoint = dynamic::translate::make_dynamic_atomic_model<Handle_Waypoint, TIME, Handle_Waypoint<TIME>::States>("handle_waypoint", move(initial_state));

		// Instantiate the input readers.
		// One for each input
		shared_ptr<dynamic::modeling::model> ir_pilot_takeover =
				dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_pilot_takeover", input_file_pilot_takeover.c_str());
		shared_ptr<dynamic::modeling::model> ir_start_mission =
				dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_start_mission", input_file_start_mission.c_str());
		shared_ptr<dynamic::modeling::model> ir_waypoint =
				dynamic::translate::make_dynamic_atomic_model<Input_Reader_Fcc_Command, TIME, const char* >("ir_waypoint", input_file_waypoint.c_str());

		// The models to be included in this coupled model
		// (accepts atomic and coupled models)
		dynamic::modeling::Models submodels_TestDriver = {
				ir_pilot_takeover,
				ir_start_mission,
				ir_waypoint,
				handle_waypoint
		};

		dynamic::modeling::Ports iports_TestDriver = {	};

		dynamic::modeling::Ports oports_TestDriver = {
				typeid(o_fcc_waypoint_update),
				typeid(o_request_gps_time)
		};

		dynamic::modeling::EICs eics_TestDriver = {	};

		// The output ports will be used to export in logging
		dynamic::modeling::EOCs eocs_TestDriver = {
				dynamic::translate::make_EOC<Handle_Waypoint_defs::o_fcc_waypoint_update,o_fcc_waypoint_update>("handle_waypoint")
		};

		// This will connect our outputs from our input reader to the file
		dynamic::modeling::ICs ics_TestDriver = {
				dynamic::translate::make_IC<iestream_input_defs<bool>::out,Handle_Waypoint_defs::i_pilot_takeover>("ir_pilot_takeover", "handle_waypoint"),
				dynamic::translate::make_IC<iestream_input_defs<int>::out,Handle_Waypoint_defs::i_start_mission>("ir_start_mission", "handle_waypoint"),
				dynamic::translate::make_IC<iestream_input_defs<message_fcc_command_t>::out,Handle_Waypoint_defs::i_waypoint>("ir_waypoint", "handle_waypoint")
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
	} while (filesystem::exists(i_base_dir + std::to_string(test_set_enumeration)));

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
