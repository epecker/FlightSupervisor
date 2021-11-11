#ifndef FCC_COMMAND_HPP
#define FCC_COMMAND_HPP

struct FccCommandMessage_t {
	double  temp;		            // Decimal degrees

	FccCommandMessage_t() : temp(0){}
};

#endif