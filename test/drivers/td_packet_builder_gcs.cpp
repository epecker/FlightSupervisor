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

struct o_packet : public cadmium::out_port<std::vector<char>> {};

int main() {
	int test_set_enumeration = 0;

	const std::string i_base_dir = std::string(PROJECT_DIRECTORY) + std::string("/test/input_data/packet_builder_gcs/");
	const std::string o_base_dir = std::string(PROJECT_DIRECTORY) + std::string("/test/simulation_results/packet_builder_gcs/");

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
		shared_ptr<cadmium::dynamic::modeling::model> packet_builder =
				cadmium::dynamic::translate::make_dynamic_atomic_model<Packet_Builder_GCS, TIME>("packet_builder");

		// Instantiate the input readers.
		// One for each input
		shared_ptr<cadmium::dynamic::modeling::model> ir_data =
				cadmium::dynamic::translate::make_dynamic_atomic_model<Input_Reader_Update_GCS, TIME, const char* >("ir_data", input_file_data.c_str());

		// The models to be included in this coupled model
		// (accepts atomic and coupled models)
		cadmium::dynamic::modeling::Models submodels_TestDriver = {
				ir_data,
				packet_builder
		};

		cadmium::dynamic::modeling::Ports iports_TestDriver = {	};

		cadmium::dynamic::modeling::Ports oports_TestDriver = {
				typeid(o_packet)
		};

		cadmium::dynamic::modeling::EICs eics_TestDriver = {	};

		// The output ports will be used to export in logging
		cadmium::dynamic::modeling::EOCs eocs_TestDriver = {
				cadmium::dynamic::translate::make_EOC<Packet_Builder_GCS<TIME>::defs::o_packet,o_packet>("packet_builder")
		};

		// This will connect our outputs from our input reader to the file
		cadmium::dynamic::modeling::ICs ics_TestDriver = {
				cadmium::dynamic::translate::make_IC<cadmium::basic_models::pdevs::iestream_input_defs<message_update_gcs_t>::out, Packet_Builder_GCS<TIME>::defs::i_data>("ir_data", "packet_builder")
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
