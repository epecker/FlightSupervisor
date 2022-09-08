/**
 *	\brief		File defining commonly used input models for the Supervisor.
 *	\details	This header file defines commonly used input models for use
				in the integration of the Supervisor. The file defines models
				for all the message structures as well as primitive data-types.
 *	\author		James Horner
 */

#ifndef INPUT_MODELS_HPP
#define INPUT_MODELS_HPP

// Basic models for receiving inputs.
#ifdef RT_LINUX
#include "io_models/UDP_Input.hpp"
#include "io_models/UDP_Input_Async.hpp"
#endif

#include "io_models/Cache_Input.hpp"

// Messages structures
#include "message_structures/message_aircraft_state_t.hpp"
#include "message_structures/message_command_ack_t.hpp"
#include "message_structures/message_fcc_command_t.hpp"
#include "message_structures/message_hover_criteria_t.hpp"
#include "message_structures/message_landing_point_t.hpp"

/**
* ==========================================================
* ASYNCHRONOUS UDP INPUT MODELS
* ==========================================================
*/
#ifdef RT_LINUX

// Aircraft State
template<typename T>
class UDP_Input_Async_Aircraft_State : public UDP_Input_Async<message_aircraft_state_t, T> {
public:
	UDP_Input_Async_Aircraft_State() = default;
	UDP_Input_Async_Aircraft_State(cadmium::dynamic::modeling::AsyncEventSubject* sub, bool ack_required, std::string port) : UDP_Input_Async<message_aircraft_state_t, T>(sub, ack_required, port) {};
};

// Boolean
template<typename T>
class UDP_Input_Async_Boolean : public UDP_Input_Async<bool, T> {
public:
	UDP_Input_Async_Boolean() = default;
	UDP_Input_Async_Boolean(cadmium::dynamic::modeling::AsyncEventSubject* sub, bool ack_required, std::string port) : UDP_Input_Async<bool, T>(sub, ack_required, port) {};
};

// Command ACK
template<typename T>
class UDP_Input_Async_Command_Ack : public UDP_Input_Async<message_command_ack_t, T> {
public:
	UDP_Input_Async_Command_Ack() = default;
	UDP_Input_Async_Command_Ack(cadmium::dynamic::modeling::AsyncEventSubject* sub, bool ack_required, std::string port) : UDP_Input_Async<message_command_ack_t, T>(sub, ack_required, port) {};
};

// Double
template<typename T>
class UDP_Input_Async_Double : public UDP_Input_Async<double, T> {
public:
	UDP_Input_Async_Double() = default;
	UDP_Input_Async_Double(cadmium::dynamic::modeling::AsyncEventSubject* sub, bool ack_required, std::string port) : UDP_Input_Async<double, T>(sub, ack_required, port) {};
};

// FCC Command
template<typename T>
class UDP_Input_Async_Fcc_Command : public UDP_Input_Async<message_fcc_command_t, T> {
public:
	UDP_Input_Async_Fcc_Command() = default;
	UDP_Input_Async_Fcc_Command(cadmium::dynamic::modeling::AsyncEventSubject* sub, bool ack_required, std::string port) : UDP_Input_Async<message_fcc_command_t, T>(sub, ack_required, port) {};
};

// Float
template<typename T>
class UDP_Input_Async_Float : public UDP_Input_Async<float, T> {
public:
	UDP_Input_Async_Float() = default;
	UDP_Input_Async_Float(cadmium::dynamic::modeling::AsyncEventSubject* sub, bool ack_required, std::string port) : UDP_Input_Async<float, T>(sub, ack_required, port) {};
};

// Hover Criteria Message
template<typename T>
class UDP_Input_Async_Hover_Criteria : public UDP_Input_Async<message_hover_criteria_t, T> {
public:
	UDP_Input_Async_Hover_Criteria() = default;
	UDP_Input_Async_Hover_Criteria(cadmium::dynamic::modeling::AsyncEventSubject* sub, bool ack_required, std::string port) : UDP_Input_Async<message_hover_criteria_t, T>(sub, ack_required, port) {};
};

// Int
template<typename T>
class UDP_Input_Async_Int : public UDP_Input_Async<int, T> {
public:
	UDP_Input_Async_Int() = default;
	UDP_Input_Async_Int(cadmium::dynamic::modeling::AsyncEventSubject* sub, bool ack_required, std::string port) : UDP_Input_Async<int, T>(sub, ack_required, port) {};
};

// Landing Point
template<typename T>
class UDP_Input_Async_Landing_Point : public UDP_Input_Async<message_landing_point_t, T> {
public:
	UDP_Input_Async_Landing_Point() = default;
	UDP_Input_Async_Landing_Point(cadmium::dynamic::modeling::AsyncEventSubject* sub, bool ack_required, std::string port) : UDP_Input_Async<message_landing_point_t, T>(sub, ack_required, port) {};
};

// String
template<typename T>
class UDP_Input_Async_String : public UDP_Input_Async<std::string, T> {
public:
	UDP_Input_Async_String() = default;
	UDP_Input_Async_String(cadmium::dynamic::modeling::AsyncEventSubject* sub, bool ack_required, std::string port) : UDP_Input_Async<std::string, T>(sub, ack_required, port) {};
};


/**
* ==========================================================
* POLLING UDP INPUT MODELS
* ==========================================================
*/

// Aircraft State
template<typename T>
class UDP_Input_Aircraft_State : public UDP_Input<message_aircraft_state_t, T> {
public:
	UDP_Input_Aircraft_State() = default;
	UDP_Input_Aircraft_State(T rate, bool ack_required, std::string port) : UDP_Input<message_aircraft_state_t, T>(rate, ack_required, port) {};
};

// Boolean
template<typename T>
class UDP_Input_Boolean : public UDP_Input<bool, T> {
public:
	UDP_Input_Boolean() = default;
	UDP_Input_Boolean(T rate, bool ack_required, std::string port) : UDP_Input<bool, T>(rate, ack_required, port) {};
};

// Command ACK
template<typename T>
class UDP_Input_Command_Ack : public UDP_Input<message_command_ack_t, T> {
public:
	UDP_Input_Command_Ack() = default;
	UDP_Input_Command_Ack(T rate, bool ack_required, std::string port) : UDP_Input<message_command_ack_t, T>(rate, ack_required, port) {};
};

// Double
template<typename T>
class UDP_Input_Double : public UDP_Input<double, T> {
public:
	UDP_Input_Double() = default;
	UDP_Input_Double(T rate, bool ack_required, std::string port) : UDP_Input<double, T>(rate, ack_required, port) {};
};

// FCC Command
template<typename T>
class UDP_Input_Fcc_Command : public UDP_Input<message_fcc_command_t, T> {
public:
	UDP_Input_Fcc_Command() = default;
	UDP_Input_Fcc_Command(T rate, bool ack_required, std::string port) : UDP_Input<message_fcc_command_t, T>(rate, ack_required, port) {};
};

// Float
template<typename T>
class UDP_Input_Float : public UDP_Input<float, T> {
public:
	UDP_Input_Float() = default;
	UDP_Input_Float(T rate, bool ack_required, std::string port) : UDP_Input<float, T>(rate, ack_required, port) {};
};

// Hover Criteria Message
template<typename T>
class UDP_Input_Hover_Criteria : public UDP_Input<message_hover_criteria_t, T> {
public:
	UDP_Input_Hover_Criteria() = default;
	UDP_Input_Hover_Criteria(T rate, bool ack_required, std::string port) : UDP_Input<message_hover_criteria_t, T>(rate, ack_required, port) {};
};

// Int
template<typename T>
class UDP_Input_Int : public UDP_Input<int, T> {
public:
	UDP_Input_Int() = default;
	UDP_Input_Int(T rate, bool ack_required, std::string port) : UDP_Input<int, T>(rate, ack_required, port) {};
};

// Landing Point
template<typename T>
class UDP_Input_Landing_Point : public UDP_Input<message_landing_point_t, T> {
public:
	UDP_Input_Landing_Point() = default;
	UDP_Input_Landing_Point(T rate, bool ack_required, std::string port) : UDP_Input<message_landing_point_t, T>(rate, ack_required, port) {};
};

// String
template<typename T>
class UDP_Input_String : public UDP_Input<std::string, T> {
public:
	UDP_Input_String() = default;
	UDP_Input_String(T rate, bool ack_required, std::string port) : UDP_Input<std::string, T>(rate, ack_required, port) {};
};

#endif

/**
* ==========================================================
* CACHE INPUT MODELS
* ==========================================================
*/

// Aircraft State
template<typename T>
class Cache_Input_Aircraft_State : public Cache_Input<message_aircraft_state_t, T> {
public:
	Cache_Input_Aircraft_State() = default;
	explicit Cache_Input_Aircraft_State(message_aircraft_state_t initial_cached_input) : Cache_Input<message_aircraft_state_t, T>(initial_cached_input) {};
};

// Boolean
template<typename T>
class Cache_Input_Boolean : public Cache_Input<bool, T> {
public:
	Cache_Input_Boolean() = default;
	explicit Cache_Input_Boolean(bool initial_cached_input) : Cache_Input<bool, T>(initial_cached_input) {};
};

// Command ACK
template<typename T>
class Cache_Input_Command_Ack : public Cache_Input<message_command_ack_t, T> {
public:
	Cache_Input_Command_Ack() = default;
	explicit Cache_Input_Command_Ack(message_command_ack_t initial_cached_input) : Cache_Input<message_command_ack_t, T>(initial_cached_input) {};
};

// Double
template<typename T>
class Cache_Input_Double : public Cache_Input<double, T> {
public:
	Cache_Input_Double() = default;
	explicit Cache_Input_Double(double initial_cached_input) : Cache_Input<double, T>(initial_cached_input) {};
};

// FCC Command
template<typename T>
class Cache_Input_Fcc_Command : public Cache_Input<message_fcc_command_t, T> {
public:
	Cache_Input_Fcc_Command() = default;
	explicit Cache_Input_Fcc_Command(message_fcc_command_t initial_cached_input) : Cache_Input<message_fcc_command_t, T>(initial_cached_input) {};
};

// Float
template<typename T>
class Cache_Input_Float : public Cache_Input<float, T> {
public:
	Cache_Input_Float() = default;
	explicit Cache_Input_Float(float initial_cached_input) : Cache_Input<float, T>(initial_cached_input) {};
};

// Hover Criteria Message
template<typename T>
class Cache_Input_Hover_Criteria : public Cache_Input<message_hover_criteria_t, T> {
public:
	Cache_Input_Hover_Criteria() = default;
	explicit Cache_Input_Hover_Criteria(message_hover_criteria_t initial_cached_input) : Cache_Input<message_hover_criteria_t, T>(initial_cached_input) {};
};

// Int
template<typename T>
class Cache_Input_Int : public Cache_Input<int, T> {
public:
	Cache_Input_Int() = default;
	explicit Cache_Input_Int(int initial_cached_input) : Cache_Input<int, T>(initial_cached_input) {};
};

// Landing Point
template<typename T>
class Cache_Input_Landing_Point : public Cache_Input<message_landing_point_t, T> {
public:
	Cache_Input_Landing_Point() = default;
	explicit Cache_Input_Landing_Point(message_landing_point_t initial_cached_input) : Cache_Input<message_landing_point_t, T>(initial_cached_input) {};
};

// String
template<typename T>
class Cache_Input_String : public Cache_Input<std::string, T> {
public:
	Cache_Input_String() = default;
	explicit Cache_Input_String(std::string initial_cached_input) : Cache_Input<std::string, T>(initial_cached_input) {};
};

#endif // INPUT_MODELS_HPP
