//C++ headers
#include <string>
#include <boost/filesystem.hpp>

//Cadmium Simulator headers
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>

//Time class header
#include <NDTime.hpp>

//Messages structures
#include "../../src/message_structures/message_boss_mission_update_t.hpp"

// Project information headers this is created by cmake at generation time!!!!
#include "../../src/SupervisorConfig.hpp"
#include "../../src/input_readers.hpp"

//Coupled model headers
#include "../../src/io_models/Packet_Builder.hpp"
#include "../../src/io_models/UDP_Output.hpp"

using namespace cadmium;

using hclock = std::chrono::high_resolution_clock;
using TIME = NDTime;

int main() {
	int test_set_enumeration = 0;

	const string i_base_dir = string(PROJECT_DIRECTORY) + string("/test/input_data/udp_output_boss/");
	const string o_base_dir = string(PROJECT_DIRECTORY) + string("/test/simulation_results/udp_output_boss/");

	do {
		// Input Files
		string input_dir = i_base_dir + to_string(test_set_enumeration);
		string input_file_in = input_dir + string("/message.txt");

		// Output locations
		string out_directory = o_base_dir + to_string(test_set_enumeration);
		string out_messages_file = out_directory + string("/output_messages.txt");
		string out_state_file = out_directory + string("/output_state.txt");
		string out_info_file = out_directory + string("/output_info.txt");

		if (!boost::filesystem::exists(input_file_in)) {
			printf("One of the input files do not exist\n");
			return 1;
		}

		// Create the output location
		boost::filesystem::create_directories(out_directory.c_str()); // Creates if it does not exist. Does nothing if it does.

		// Instantiate the atomic model to test
		std::shared_ptr<cadmium::dynamic::modeling::model> udp_output = cadmium::dynamic::translate::make_dynamic_atomic_model<UDP_Output, TIME, const char*, const unsigned short, bool>("udp_output", IPV4_BOSS, PORT_BOSS, false);

		// Instantiate the input readers.
		// One for each input
		std::shared_ptr<cadmium::dynamic::modeling::model> ir_message =
			cadmium::dynamic::translate::make_dynamic_atomic_model<Input_Reader_Boss_Mission_Update, TIME, const char* >("ir_message", input_file_in.c_str());
		std::shared_ptr<cadmium::dynamic::modeling::model> packet_builder =
			cadmium::dynamic::translate::make_dynamic_atomic_model<Packet_Builder_Boss, TIME>("packet_builder");

		// The models to be included in this coupled model
		// (accepts atomic and coupled models)
		cadmium::dynamic::modeling::Models submodels_TestDriver = {
			udp_output,
			ir_message,
			packet_builder
		};

		cadmium::dynamic::modeling::Ports iports_TestDriver = { };

		cadmium::dynamic::modeling::Ports oports_TestDriver = { };

		cadmium::dynamic::modeling::EICs eics_TestDriver = {	};

		// The output ports will be used to export in logging
		cadmium::dynamic::modeling::EOCs eocs_TestDriver = { };

		// This will connect our outputs from our input reader to the file
		cadmium::dynamic::modeling::ICs ics_TestDriver = {
			cadmium::dynamic::translate::make_IC<cadmium::basic_models::pdevs::iestream_input_defs<message_boss_mission_update_t>::out, Packet_Builder_Boss<TIME>::defs::i_data>("ir_message", "packet_builder"),
			cadmium::dynamic::translate::make_IC<Packet_Builder_Boss<TIME>::defs::o_packet, UDP_Output<TIME>::defs::i_message>("packet_builder", "udp_output")
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

	return 0;
}
