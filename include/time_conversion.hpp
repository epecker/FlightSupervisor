#ifndef TIME_CONVERSION_H
#define TIME_CONVERSION_H

#include <string>

template<typename TIME>
TIME seconds_to_time(double time) {
	int hours = time / 3600;
	int mins = (time - hours * 3600) / 60;
	int secs = (time - hours * 3600 - mins * 60);
	int millis = (time - hours * 3600 - mins * 60 - secs) * 100;
	return TIME(to_string(hours) + ":" + to_string(mins) + ":" + to_string(secs) + ":" + to_string(millis));
}

#endif /* TIME_CONVERSION_H */
