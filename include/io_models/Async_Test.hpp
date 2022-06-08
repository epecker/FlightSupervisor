/**
* Ben Earle
* ARSLab - Carleton University
*
* Interrupt Input:
* Model to interface with a interrupt Input pin for Embedded Cadmium.
*/

#ifndef BOOST_SIMULATION_PDEVS_INTERRUPTINPUT_HPP
#define BOOST_SIMULATION_PDEVS_INTERRUPTINPUT_HPP

#include <limits>
#include <math.h>
#include <assert.h>
#include <memory>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <algorithm>
#include <limits>
#include <random>

#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <cadmium/modeling/dynamic_model.hpp>

#include "Callback_Tester.hpp"

//This class will interface with a interrupt input pin.
using namespace cadmium;
using namespace std;

//Port definition
struct Async_Test_defs {
	struct out : public out_port<int> { };
};

template<typename TIME>
class Async_Test {

private:
	cadmium::dynamic::modeling::AsyncEventSubject* _sub;

public:
	//Parameters to be overwriten when instantiating the atomic model
	Callback_Tester* tester;

	// default constructor
	Async_Test() {
	}

	Async_Test(cadmium::dynamic::modeling::AsyncEventSubject* sub, string name) {
		tester = new Callback_Tester(name);
		tester->monitor(sub);
		state.output = tester->read();
		state.last = state.output;
		state.prop = true;
	}

	// state definition
	struct state_type {
		int output;
		int last;
		bool prop;
	};
	state_type state;

	// ports definition
	using input_ports = std::tuple<>;
	using output_ports = std::tuple<typename Async_Test_defs::out>;

	// internal transition
	void internal_transition() {
		state.prop = false;
	}

	// external transition
	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		state.prop = true;
		state.last = state.output;
		state.output = tester->read();
	}

	// confluence transition
	void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), std::move(mbs));
	}

	// output function
	typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		if (state.last != state.output) {
			int out = state.output;
			get_messages<typename Async_Test_defs::out>(bags).push_back(out);
		}
		return bags;
	}

	// time_advance function
	TIME time_advance() const {
		if (state.prop) {
			return TIME("00:00:00:00");
		}

		return std::numeric_limits<TIME>::infinity();
	}

	friend std::ostringstream& operator<<(std::ostringstream& os, const typename Async_Test<TIME>::state_type& i) {
		os << "Input Value: " << i.output;
		return os;
	}
};

#endif // BOOST_SIMULATION_PDEVS_INTERRUPTINPUT_HPP