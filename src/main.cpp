//Cadmium Simulator headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/logger/common_loggers.hpp>

//Time class header
#include <NDTime.hpp>

//Messages structures
#include "message_structures/message_hover_criteria_t.hpp"
#include "message_structures/message_landing_point_t.hpp"

//Atomic model headers
#include <cadmium/basic_model/pdevs/iestream.hpp> //Atomic model for inputs

//Coupled model headers
#include "coupled_models/Landing.hpp"

// Project information headers this is created by cmake at generation time!!!!
#include "SupervisorConfig.hpp"

//C++ headers
#include <chrono>
#include <algorithm>
#include <string>
#include <iostream>
#include <filesystem>

using namespace std;
using namespace cadmium;
using namespace cadmium::basic_models::pdevs;

using TIME = NDTime;


/**
* Create an atomic model to read the input file.
*/
template<typename T>
class InputReader_message_mavlink_mission_item_t : public iestream_input<message_landing_point_t, T> {
public:
	InputReader_message_mavlink_mission_item_t() = default;
	InputReader_message_mavlink_mission_item_t(const char* file_path) : iestream_input<message_landing_point_t, T>(file_path) {}
};

int main(int argc, char* argv[]) {
	if (argc < 2) {
		printf("The program should be invoked as follows\n");
		printf("%s path/to/input/file\n", argv[0]);
		return 1;
	}

	if (!filesystem::exists(argv[1])) {
		printf("The input file does not exist\n");
		return 1;
	}

	/**
	* Instantiate the InputReader_Int Atomic model.
	*/
	string input = argv[1];
	const char* i_input = input.c_str();
	shared_ptr<dynamic::modeling::model> input_reader = dynamic::translate::make_dynamic_atomic_model<InputReader_message_mavlink_mission_item_t, TIME, const char* >("input_reader", move(i_input));

	/*
	 * ==========================================================
	 * SUPERVISOR COUPLED MODEL
	 * ==========================================================
	 */

	//Instantiate the Landing with the ports, models, and couplings previsouly defined.
	Landing landing = Landing();
	shared_ptr<dynamic::modeling::coupled<TIME>> supervisor = make_shared<dynamic::modeling::coupled<TIME>>("landing", landing.submodels, landing.iports, landing.oports, landing.eics, landing.eocs, landing.ics);


	/*
	 * ==========================================================
	 * TEST DRIVER COUPLED MODEL
	 * ==========================================================
	 */

	dynamic::modeling::Ports iports_TestDriver = {	};

	dynamic::modeling::Ports oports_TestDriver = {	};

	dynamic::modeling::Models submodels_TestDriver = {
		input_reader,
		supervisor
	};

	dynamic::modeling::EICs eics_TestDriver = {	};
	dynamic::modeling::EOCs eocs_TestDriver = {	};

	//Define the one coupling from the input reader to the landing point receipt port on the Landing.
	dynamic::modeling::ICs ics_TestDriver = {
		dynamic::translate::make_IC<iestream_input_defs<message_landing_point_t>::out, Landing_defs::i_LP_recv>("input_reader", "landing")
	};

	shared_ptr<dynamic::modeling::coupled<TIME>> test_driver;
	test_driver = make_shared<dynamic::modeling::coupled<TIME>>(
		"test_driver", submodels_TestDriver, iports_TestDriver, oports_TestDriver, eics_TestDriver, eocs_TestDriver, ics_TestDriver
	);

	/*************** Loggers *******************/
	string out_directory = string(PROJECT_DIRECTORY) + string("/simulation_results");
	string out_messages_file = string(PROJECT_DIRECTORY) + string("/simulation_results/output_messages.txt");
	string out_state_file = string(PROJECT_DIRECTORY) + string("/simulation_results/output_state.txt");

	filesystem::create_directory(out_directory.c_str()); // Creates if it does not exist. Does nothing if it does.

	static ofstream out_messages(out_messages_file.c_str());
	struct oss_sink_messages {
		static ostream& sink() {
			return out_messages;
		}
	};
	static ofstream out_state(out_state_file.c_str());
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

	dynamic::engine::runner<NDTime, logger_supervisor> r(test_driver, { 0 });
	r.run_until_passivate();
	return 0;
}