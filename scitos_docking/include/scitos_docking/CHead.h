#ifndef CHEAD_H
#define CHEAD_H

/**
@author Tom Krajnik
*/
#include <sys/time.h>
#include <stdlib.h>
#include <string>
#include "ros/ros.h"
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#define TIMEOUT_INTERVAL 40000

class CHead
{
	public:
		CHead();
		~CHead();
		bool get();
		void set(bool stuff);

	private:
		bool status;
		bool known;
};

#endif
