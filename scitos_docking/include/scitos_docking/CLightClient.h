#ifndef CLIGHTCLIENT_H
#define CLIGHTCLIENT_H

/**
@author Tom Krajnik
*/
#include <sys/time.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#define TIMEOUT_INTERVAL 40000

class CLightClient
{
	public:
		CLightClient();
		~CLightClient();
		bool get();
		void set(bool stuff);
	private:
		bool status;
		bool known;
};

#endif
