//Cadmium Simulator headers
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/logger/common_loggers.hpp>

//Time class header
#include <NDTime.hpp>

//Messages structures
#include "../data_structures/message.hpp"

//Atomic model headers
#include <cadmium/basic_model/pdevs/iestream.hpp> //Atomic model for inputs
#include "../atomic_models/Hand.hpp"
#include "../atomic_models/Man.hpp"
#include "../atomic_models/Repo.hpp"

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
class InputReader_Int : public iestream_input<int, T> {
public:
	InputReader_Int() = default;
	InputReader_Int(const char* file_path) : iestream_input<int, T>(file_path) {}
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
	shared_ptr<dynamic::modeling::model> input_reader = dynamic::translate::make_dynamic_atomic_model<InputReader_Int, TIME, const char* >("input_reader", move(i_input));

	/**
	* Instantiate the Repo and Man Atomic models.
	*/
	shared_ptr<dynamic::modeling::model> hand = dynamic::translate::make_dynamic_atomic_model<Hand, TIME>("hand");
	shared_ptr<dynamic::modeling::model> repo = dynamic::translate::make_dynamic_atomic_model<Repo, TIME>("repo");
	shared_ptr<dynamic::modeling::model> man = dynamic::translate::make_dynamic_atomic_model<Man, TIME>("man");

	return 0;
}