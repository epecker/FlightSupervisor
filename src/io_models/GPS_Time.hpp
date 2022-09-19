/**
 * 	\file		GPS_Time.hpp
 *	\brief		Definition of the GPS Time atomic model.
 *	\details	This header file defines the GPS Time atomic model for use in the Cadmium DEVS
				simulation software. The model sits idle and is used to add gps times to the log.
 *	\image		html io_models/gps_time.png
 *	\author		Tanner Trautrim
 *	\author		James Horner
 */

#ifndef GPS_TIME_H
#define GPS_TIME_H

// Utility functions
#include "../enum_string_conversion.hpp"

// Cadmium Simulator Headers
#include <cadmium/modeling/message_bag.hpp>
#include <sharedmemorymodel/SharedMemoryModel.h>

// System libraries
#include <iostream>
#include <limits>

/**
 *	\class		GPS_Time
 *	\brief		Definition of the GPS Time atomic model.
 *	\details	This class defines the GPS Time atomic model for use in the Cadmium DEVS
				simulation software. The model sits idle and is used to add gps times to the log.
 *	\image		html io_models/gps_time.png
 */
template<typename TIME>
class GPS_Time {
public:
	/**
	 *	\par	States
	 * 	Declaration of the states of the atomic model.
	 */
    DEFINE_ENUM_WITH_STRING_CONVERSIONS(States, (GPS_TIME));

	/**
	 * 	\anchor	GPS_Time_input_ports
	 *	\par	Input Ports
	 * 	Definition of the input ports for the model.
	 */
    using input_ports = std::tuple<>;

	/**
	 *	\anchor	GPS_Time_output_ports
	 * 	\par 	Output Ports
	 * 	Definition of the output ports for the model.
	 */
    using output_ports = std::tuple<>;

	/**
	 *	\anchor	GPS_Time_state_type
	 *	\par	State
	 * 	Definition of the states of the atomic model.
	 * 	\param 	current_state 	Current state of atomic model.
	 */
    struct state_type {
        States current_state;
    } state;

	/**
	 * \brief 	Default constructor for the model.
	 */
    GPS_Time() {
        state.current_state = States::GPS_TIME;
        model.connectSharedMem();
        if (!model.isConnected()) {
            throw(std::runtime_error("Could not connect to shared memory."));
        }
    }

	/**
	 * \brief 	Destructor for disconnecting from shared memory.
	 */
    ~GPS_Time() {
        model.disconnectSharedMem();
    }

	/// Internal transitions of the model
    void internal_transition() {}

	/// External transitions of the model
    void external_transition([[maybe_unused]] TIME e, [[maybe_unused]] typename cadmium::make_message_bags<input_ports>::type mbs) {}

	/// Function used to decide precedence between internal and external transitions when both are scheduled simultaneously.
    void confluence_transition([[maybe_unused]] TIME e, [[maybe_unused]] typename cadmium::make_message_bags<input_ports>::type mbs) {}

	/// Function for generating output from the model before internal transitions.
	[[nodiscard]] typename cadmium::make_message_bags<output_ports>::type output() const {
        return {};
    }

	/// Function to declare the time advance value for each state of the model.
    TIME time_advance() const {
        return std::numeric_limits<TIME>::infinity();
    }

	/**
	 *  \brief 		Operator for defining how the model state will be represented as a string.
	 * 	\warning 	Prepended "State: " is required for log parsing, do not remove.
	 */
    friend std::ostringstream& operator<<(std::ostringstream& os, const typename GPS_Time<TIME>::state_type& i) {
        os << (std::string("State: ") + std::to_string(model.sharedMemoryStruct->hg1700.time));
        return os;
    }

private:
	/// Manages the connection to shared memory
    inline static SharedMemoryModel model{};

};

#endif // GPS_TIME_H
