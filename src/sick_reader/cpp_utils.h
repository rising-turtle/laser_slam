/*
 * cpp_utils.h
 *
 *  Created on: Nov 26, 2012
 *      Author: liu
 */

#ifndef CPP_UTILS_H_
#define CPP_UTILS_H_

#include <string>
#include <vector>
#include <iostream>
#include <time.h>
#include <math.h>
#include <cmath>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

#include "timestamp.h"
// Generic constants and defines:
// ---------------------------------------------------------
#ifndef M_PI
#	define M_PI 3.14159265358979323846264338327950288		// PI constant
#endif

#ifndef M_2PI
#	define M_2PI 6.283185307179586476925286766559	// The 2*PI constant
#endif

using namespace std;

/** \def THROW_EXCEPTION(msg)
 * \param msg This can be a char*, a std::string, or a literal string.
 * Defines a unified way of reporting exceptions
 * \sa MRPT_TRY_START, MRPT_TRY_END, THROW_EXCEPTION_CUSTOM_MSG1
 */
#define THROW_EXCEPTION(msg)	\
	{\
		std::ostringstream auxCompStr;\
		auxCompStr << "\n\n =============== SICK EXCEPTION =============\n";\
		auxCompStr << msg << std::endl; \
		throw std::logic_error( auxCompStr.str() );\
	}\

string format(const char *fmt, ...);


// a empty class for CPOSE3D
class CPose3D{ };


/** Inline function for the square of a number. */
template<class T>
inline T square(const T x)    { return x*x; }

/** Modifies the given angle to translate it into the [0,2pi[ range.
  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
  * \sa wrapToPi, wrapTo2Pi, unwrap2PiSequence
  */
template <class T>
inline void wrapTo2PiInPlace(T &a)
{
	bool was_neg = a<0;
	a = fmod(a, static_cast<T>(M_2PI) );
	if (was_neg) a+=static_cast<T>(M_2PI);
}

/** Modifies the given angle to translate it into the [0,2pi[ range.
  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
  * \sa wrapToPi, wrapTo2Pi, unwrap2PiSequence
  */
template <class T>
inline T wrapTo2Pi(T a)
{
	wrapTo2PiInPlace(a);
	return a;
}

/** Modifies the given angle to translate it into the ]-pi,pi] range.
  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
  * \sa wrapTo2Pi, wrapToPiInPlace, unwrap2PiSequence
  */
template <class T>
inline T wrapToPi(T a)
{
	return wrapTo2Pi( a + static_cast<T>(M_PI) )-static_cast<T>(M_PI);
}

/** Modifies the given angle to translate it into the ]-pi,pi] range.
  * \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
  * \sa wrapToPi,wrapTo2Pi, unwrap2PiSequence
  */
template <class T>
inline void wrapToPiInPlace(T &a)
{
	a = wrapToPi(a);
}


/** Normalize a vector, such as its norm is the unity.
  *  If the vector has a null norm, the output is a null vector.
  */
template<class VEC1,class VEC2>
void normalize(const VEC1 &v, VEC2 &out_v)
{
	typename VEC1::value_type total=0;
	const size_t N = v.size();
	for (size_t i=0;i<N;i++)
		total += square(v[i]);
	total = std::sqrt(total);
	if (total)
	{
		out_v = v * (1.0/total);
	}
	else out_v.assign(v.size(),0);
}

#endif /* CPP_UTILS_H_ */
