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
#include "../atomic_models/man.hpp"
#include "../atomic_models/Repo.hpp"

//C++ headers
#include <chrono>
#include <algorithm>
#include <string>
#include <iostream>

using namespace std;
using namespace cadmium;
using namespace cadmium::basic_models::pdevs;

using TIME = NDTime;

int main(int argc, char* argv[])
{
	shared_ptr<dynamic::modeling::model> repo = dynamic::translate::make_dynamic_atomic_model<Repo, TIME>("repo");
	shared_ptr<dynamic::modeling::model> man = dynamic::translate::make_dynamic_atomic_model<LandingPointManager, TIME>("man");

	return 0;
}