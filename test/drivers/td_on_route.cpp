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

#include "../../src/coupled_models/On_Route.hpp"

using TIME = NDTime;

// Model output ports
struct o_fcc_waypoint_update : public cadmium::out_port<message_fcc_command_t> {};
struct o_request_gps_time : public cadmium::out_port<bool> {};

int main() {
	int test_set_enumeration = 0;

	const std::string i_base_dir = std::string(PROJECT_DIRECTORY) + std::string("/test/input_data/on_route/");
	const std::string o_base_dir = std::string(PROJECT_DIRECTORY) + std::string("/test/simulation_results/on_route/");

	do {
		// Input locations
		string input_dir						= i_base_dir + to_string(test_set_enumeration);
		string input_file_pilot_takeover		= input_dir + std::string("/pilot_takeover.txt");
		string input_file_start_mission			= input_dir + std::string("/start_mission.txt");
		string input_file_waypoint				= input_dir + std::string("/waypoint.txt");

		// Output locations
		string out_directory					= o_base_dir + to_string(test_set_enumeration);
		string out_messages_file				= out_directory + std::string("/output_messages.txt");
		string out_state_file					= out_directory + std::string("/output_state.txt");

		if (!boost::filesystem::exists(input_file_start_mission) ||
			!boost::filesystem::exists(input_file_waypoint)) {
			printf("One of the input files do not exist\n");
			return 1;
		}

		// Create the output location
		boost::filesystem::create_directories(out_directory.c_str());

		// Instantiate the coupled model to test
		On_Route onr = On_Route();
		shared_ptr<cadmium::dynamic::modeling::coupled<TIME>> on_route = make_shared<cadmium::dynamic::modeling::coupled<TIME>>("on_route", onr.submodels, onr.iports, onr.oports, onr.eics, onr.eocs, onr.ics);

		// Instantiate the input readers.
		shared_ptr<cadmium::dynamic::modeling::model> ir_pilot_takeover =
				cadmium::dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boolean, TIME, const char* >("ir_pilot_takeover", input_file_pilot_takeover.c_str());
		shared_ptr<cadmium::dynamic::modeling::model> ir_start_mission =
				cadmium::dynamic::translate::make_dynamic_atomic_model<Input_Reader_Int, TIME, const char* >("ir_start_mission", input_file_start_mission.c_str());
		shared_ptr<cadmium::dynamic::modeling::model> ir_waypoint =
				cadmium::dynamic::translate::make_dynamic_atomic_model<Input_Reader_Fcc_Command, TIME, const char* >("ir_waypoint", input_file_waypoint.c_str());

		// The models to be included in this coupled model
		// (accepts atomic and coupled models)
	 	cadmium::dynamic::modeling::Models submodels_TestDriver = {
				ir_pilot_takeover,
				ir_start_mission,
				ir_waypoint,
				on_route
		};

	 	cadmium::dynamic::modeling::Ports iports_TestDriver = { };

	 	cadmium::dynamic::modeling::Ports oports_TestDriver = {
				typeid(o_fcc_waypoint_update),
				typeid(o_request_gps_time)
		};

	 	cadmium::dynamic::modeling::EICs eics_TestDriver = {	};

		// The output ports will be used to export in logging
	 	cadmium::dynamic::modeling::EOCs eocs_TestDriver = {
				cadmium::dynamic::translate::make_EOC<On_Route::defs::o_fcc_waypoint_update,o_fcc_waypoint_update>("on_route"),
		};

		// This will connect our outputs from our input reader to the file
	 	cadmium::dynamic::modeling::ICs ics_TestDriver = {
				cadmium::dynamic::translate::make_IC<cadmium::basic_models::pdevs::iestream_input_defs<bool>::out,On_Route::defs::i_pilot_takeover>("ir_pilot_takeover", "on_route"),
				cadmium::dynamic::translate::make_IC<cadmium::basic_models::pdevs::iestream_input_defs<int>::out,On_Route::defs::i_start_mission>("ir_start_mission", "on_route"),
				cadmium::dynamic::translate::make_IC<cadmium::basic_models::pdevs::iestream_input_defs<message_fcc_command_t>::out,On_Route::defs::i_waypoint>("ir_waypoint", "on_route")
		};

		shared_ptr<cadmium::dynamic::modeling::coupled<TIME>> TEST_DRIVER = make_shared<cadmium::dynamic::modeling::coupled<TIME>>(
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

		using state = cadmium::logger::logger<cadmium::logger::logger_state, cadmium::dynamic::logger::formatter<TIME>, oss_sink_state>;
		using log_messages = cadmium::logger::logger<cadmium::logger::logger_messages, cadmium::dynamic::logger::formatter<TIME>, oss_sink_messages>;
		using global_time_mes = cadmium::logger::logger<cadmium::logger::logger_global_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_messages>;
		using global_time_sta = cadmium::logger::logger<cadmium::logger::logger_global_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_state>;

		using logger_supervisor = cadmium::logger::multilogger<state, log_messages, global_time_mes, global_time_sta>;

		cadmium::dynamic::engine::runner<NDTime, logger_supervisor> r(TEST_DRIVER, { 0 });

		r.run_until_passivate();
		test_set_enumeration++;
	} while (boost::filesystem::exists(i_base_dir + std::to_string(test_set_enumeration)));

	return 0;
}
