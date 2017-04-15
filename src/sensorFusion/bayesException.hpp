/*
 * bayesException.hpp
 *
 *  Created on: Nov 28, 2012
 *      Author: liu
 */

#ifndef BAYESEXCEPTION_HPP_
#define BAYESEXCEPTION_HPP_



// Common headers required for declerations
#include <exception>

/* Filter namespace */
namespace Bayesian_filter
{


class Filter_exception : virtual public std::exception
/*
 *	Base class for all exception produced by filter hierarchy
 */
{
public:
	const char *what() const throw()
	{	return error_description;
	}
protected:
	Filter_exception (const char* description)
	{	error_description = description;
	}
private:
	const char* error_description;
};

class Logic_exception : virtual public Filter_exception
/*
 * Logic Exception
 */
{
public:
	Logic_exception (const char* description) :
		Filter_exception (description)
	{}
};

class Numeric_exception : virtual public Filter_exception
/*
 * Numeric Exception
 */
{
public:
	Numeric_exception (const char* description) :
		Filter_exception (description)
	{}
};


}//namespace

#endif /* BAYESEXCEPTION_HPP_ */
