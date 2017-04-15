/*
 * cpp_utils.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: liu
 */

#include "cpp_utils.h"

// A sprintf-like function for std::string
string format(const char *fmt, ...)
{
	if (!fmt) return string("");

	int   result = -1, length = 1024;
	vector<char> buffer;
	while (result == -1)
	{
		buffer.resize(length + 10);

		va_list args;  // This must be done WITHIN the loop
		va_start(args,fmt);
		result = ::vsnprintf(&buffer[0], length, fmt, args);
		va_end(args);

		// Truncated?
		if (result>=length) result=-1;
		length*=2;
	}
	string s(&buffer[0]);
	return s;
}





