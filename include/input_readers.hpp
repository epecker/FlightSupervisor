/**
* ==========================================================
* INPUT READERS
* ==========================================================
*/

#ifndef INPUT_READERS_HPP
#define INPUT_READERS_HPP

// Basic model for reading from a file.
#include <cadmium/basic_model/pdevs/iestream.hpp>

//Messages structures
#include "message_structures/message_aircraft_state_t.hpp"
#include "message_structures/message_hover_criteria_t.hpp"
#include "message_structures/message_landing_point_t.hpp"
#include "message_structures/message_fcc_command_t.hpp"
#include "message_structures/message_start_supervisor_t.hpp"

using namespace cadmium::basic_models::pdevs;



// Aircraft State
template<typename T>
class Input_Reader_Aircraft_State : public iestream_input<message_aircraft_state_t, T> {
public:
	Input_Reader_Aircraft_State() = default;
	Input_Reader_Aircraft_State(const char* file_path) : iestream_input<message_aircraft_state_t, T>(file_path) {};
};

// Boolean
template<typename T>
class Input_Reader_Boolean : public iestream_input<bool, T> {
public:
	Input_Reader_Boolean() = default;
	Input_Reader_Boolean(const char* file_path) : iestream_input<bool, T>(file_path) {};
};

// Hover Criteria Message
template<typename T>
class Input_Reader_Fcc_Command : public iestream_input<message_fcc_command_t, T> {
public:
	Input_Reader_Fcc_Command() = default;
	Input_Reader_Fcc_Command(const char* file_path) : iestream_input<message_fcc_command_t, T>(file_path) {};
};

// Hover Criteria Message
template<typename T>
class Input_Reader_Hover_Criteria : public iestream_input<message_hover_criteria_t, T> {
public:
	Input_Reader_Hover_Criteria() = default;
	Input_Reader_Hover_Criteria(const char* file_path) : iestream_input<message_hover_criteria_t, T>(file_path) {};
};

// Mavlink Mission Item
template<typename T>
class Input_Reader_Mavlink_Mission_Item : public iestream_input<message_landing_point_t, T> {
public:
	Input_Reader_Mavlink_Mission_Item() = default;
	Input_Reader_Mavlink_Mission_Item(const char* file_path) : iestream_input<message_landing_point_t, T>(file_path) {};
};

// Start Supervisor Item
template<typename T>
class Input_Reader_Start_Supervisor : public iestream_input<message_start_supervisor_t, T> {
public:
	Input_Reader_Start_Supervisor() = default;
	Input_Reader_Start_Supervisor(const char* file_path) : iestream_input<message_start_supervisor_t, T>(file_path) {};
};

#endif // INPUT_READERS_HPP
