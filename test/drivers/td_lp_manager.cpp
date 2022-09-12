// C++ headers
#include <string>
#include <boost/filesystem.hpp>

// Cadmium Simulator headers
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>

// Time class header
#include <NDTime.hpp>

//Messages structures
#include "../../src/message_structures/message_hover_criteria_t.hpp"
#include "../../src/message_structures/message_landing_point_t.hpp"

// Required for testing
#include "../../src/SupervisorConfig.hpp" // Generated by cmake
#include "../../src/input_readers.hpp" // Input Reader Definitions

//Atomic model headers
#include "../../src/atomic_models/LP_Manager.hpp"

using namespace cadmium;
using namespace cadmium::basic_models::pdevs;

using TIME = NDTime;

// Used for oss_sink_state and oss_sink_messages


/**
* ==========================================================
* MAIN METHOD
* ==========================================================
*/
int main() {
	int test_set_enumeration = 0;

	const string i_base_dir = string(PROJECT_DIRECTORY) + string("/test/input_data/lp_manager/");
	const string o_base_dir = string(PROJECT_DIRECTORY) + string("/test/simulation_results/lp_manager/");

	do {
		// Input Files
		string input_dir = i_base_dir + to_string(test_set_enumeration);
		string input_file_initial_state = input_dir + string("/initial_state.txt");
		string input_file_aircraft_state = input_dir + string("/aircraft_state.txt");
		string input_file_fcc_command_land = input_dir + string("/fcc_command_land.txt");
		string input_file_lp_recv = input_dir + string("/lp_recv.txt");
		string input_file_plp_ach = input_dir + string("/plp_ach.txt");
		string input_file_pilot_takeover = input_dir + string("/pilot_takeover.txt");
		string input_file_control_yielded = input_dir + string("/control_yielded.txt");
		string input_file_start_mission = input_dir + string("/start_mission.txt");

		// Output locations
		string out_directory = o_base_dir + to_string(test_set_enumeration);
		string out_messages_file = out_directory + string("/output_messages.txt");
		string out_state_file = out_directory + string("/output_state.txt");

		if (!boost::filesystem::exists(input_file_initial_state) ||
			!boost::filesystem::exists(input_file_aircraft_state) ||
			!boost::filesystem::exists(input_file_fcc_command_land) ||
			!boost::filesystem::exists(input_file_lp_recv) ||
			!boost::filesystem::exists(input_file_plp_ach) ||
			!boost::filesystem::exists(input_file_pilot_takeover) ||
			!boost::filesystem::exists(input_file_control_yielded) ||
			!boost::filesystem::exists(input_file_start_mission)) {
			printf("One of the input files do not exist\n");
			return 1;
		}

        // Read the initial state of the model.
        fstream f;
        f.open(input_file_initial_state, ios::in);
        if (!f.is_open()) {
            printf("Failed to open initial_state file\n");
            return 1;
        }
        string initial_state_string;
        getline(f, initial_state_string);
        f.close();

        LP_Manager<TIME>::States initial_state = LP_Manager<TIME>::stringToEnum(initial_state_string);
		TIME lp_accept_time = seconds_to_time<TIME>(REPO_TIMER);
		TIME orbit_time = seconds_to_time<TIME>(REPO_TIMER);

		// Create the output location
		boost::filesystem::create_directories(out_directory.c_str()); // Creates if it does not exist. Does nothing if it does.

		// Instantiate the atomic model to test
		shared_ptr<cadmium::dynamic::modeling::model> lp_manager = cadmium::dynamic::translate::make_dynamic_atomic_model<LP_Manager, TIME, TIME, TIME, LP_Manager<TIME>::States>("lp_manager", std::move(lp_accept_time), std::move(orbit_time), std::move(initial_state));

		// Instantiate the input readers.
		// One for each input
		shared_ptr<cadmium::dynamic::modeling::model> ir_aircraft_state =
			cadmium::dynamic::translate::make_dynamic_atomic_model<Input_Reader_Aircraft_State, TIME, const char* >("ir_aircraft_state", input_file_aircraft_state.c_str());
		shared_ptr<cadmium::dynamic::modeling::model> ir_fcc_command_land =
			cadmium::dynamic::translate::make_dynamic_atomic_model<Input_Reader_Fcc_Command, TIME, const char* >("ir_fcc_command_land", input_file_fcc_command_land.c_str());
		shared_ptr<cadmium::dynamic::modeling::model> ir_lp_recv =
			cadmium::dynamic::translate::make_dynamic_atomic_model<Input_Reader_Mavlink_Mission_Item, TIME, const char* >("ir_lp_recv", input_file_lp_recv.c_str());
		shared_ptr<cadmium::dynamic::modeling::model> ir_plp_ach =
			cadmium::dynamic::translate::make_dynamic_atomic_model<Input_Reader_Mavlink_Mission_Item, TIME, const char* >("ir_plp_ach", input_file_plp_ach.c_str());
		shared_ptr<cadmium::dynamic::modeling::model> ir_pilot_takeover =
			cadmium::dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_pilot_takeover", input_file_pilot_takeover.c_str());
		shared_ptr<cadmium::dynamic::modeling::model> ir_control_yielded =
			cadmium::dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_control_yielded", input_file_control_yielded.c_str());
        shared_ptr<cadmium::dynamic::modeling::model> ir_start_mission =
                cadmium::dynamic::translate::make_dynamic_atomic_model<Input_Reader_Int, TIME, const char* >("ir_start_mission", input_file_start_mission.c_str());

		// The models to be included in this coupled model
		// (accepts atomic and coupled models)
	 	cadmium::dynamic::modeling::Models submodels_TestDriver = {
			ir_aircraft_state,
			ir_fcc_command_land,
			ir_lp_recv,
			ir_plp_ach,
			ir_pilot_takeover,
			ir_control_yielded,
            ir_start_mission,
			lp_manager
		};

	 	cadmium::dynamic::modeling::Ports iports_TestDriver = { };

	 	cadmium::dynamic::modeling::Ports oports_TestDriver = { };

	 	cadmium::dynamic::modeling::EICs eics_TestDriver = {	};

		// The output ports will be used to export in logging
	 	cadmium::dynamic::modeling::EOCs eocs_TestDriver = { };

		// This will connect our outputs from our input reader to the file
	 	cadmium::dynamic::modeling::ICs ics_TestDriver = {
			cadmium::dynamic::translate::make_IC<cadmium::basic_models::pdevs::iestream_input_defs<message_aircraft_state_t>::out,LP_Manager<TIME>::defs::i_aircraft_state>("ir_aircraft_state", "lp_manager"),
			cadmium::dynamic::translate::make_IC<cadmium::basic_models::pdevs::iestream_input_defs<message_fcc_command_t>::out,LP_Manager<TIME>::defs::i_fcc_command_land>("ir_fcc_command_land", "lp_manager"),
			cadmium::dynamic::translate::make_IC<cadmium::basic_models::pdevs::iestream_input_defs<message_landing_point_t>::out,LP_Manager<TIME>::defs::i_lp_recv>("ir_lp_recv", "lp_manager"),
			cadmium::dynamic::translate::make_IC<cadmium::basic_models::pdevs::iestream_input_defs<message_landing_point_t>::out,LP_Manager<TIME>::defs::i_plp_ach>("ir_plp_ach", "lp_manager"),
			cadmium::dynamic::translate::make_IC<cadmium::basic_models::pdevs::iestream_input_defs<bool>::out,LP_Manager<TIME>::defs::i_pilot_takeover>("ir_pilot_takeover", "lp_manager"),
			cadmium::dynamic::translate::make_IC<cadmium::basic_models::pdevs::iestream_input_defs<bool>::out,LP_Manager<TIME>::defs::i_control_yielded>("ir_control_yielded", "lp_manager"),
			cadmium::dynamic::translate::make_IC<cadmium::basic_models::pdevs::iestream_input_defs<int>::out,LP_Manager<TIME>::defs::i_start_mission>("ir_start_mission", "lp_manager")
		};

		shared_ptr<cadmium::dynamic::modeling::coupled<TIME>> test_driver = make_shared<cadmium::dynamic::modeling::coupled<TIME>>(
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

		using state = cadmium::logger::logger<cadmium::logger::logger_state, cadmium::dynamic::logger::formatter<TIME>, oss_sink_state>;
		using log_messages = cadmium::logger::logger<cadmium::logger::logger_messages, cadmium::dynamic::logger::formatter<TIME>, oss_sink_messages>;
		using global_time_mes = cadmium::logger::logger<cadmium::logger::logger_global_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_messages>;
		using global_time_sta = cadmium::logger::logger<cadmium::logger::logger_global_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_state>;

		using logger_supervisor = cadmium::logger::multilogger<state, log_messages, global_time_mes, global_time_sta>;

		cadmium::dynamic::engine::runner<NDTime, logger_supervisor> r(test_driver, { 0 });

		r.run_until_passivate();
		test_set_enumeration++;
	} while (boost::filesystem::exists(i_base_dir + std::to_string(test_set_enumeration)));

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
