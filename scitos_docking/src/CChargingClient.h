#ifndef CCHARGINGCLIENT_H
#define CCHARGINGCLIENT_H

/**
@author Tom Krajnik
*/
#include <sys/time.h>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include "ros/ros.h"
#include <scitos_docking/ChargingAction.h>

#define TIMEOUT_INTERVAL 40000

class CChargingClient
{
	public:
		CChargingClient();
		~CChargingClient();
		void initiateCharging();
		void initiateUndocking();

	private:
		static void doneCb(const actionlib::SimpleClientGoalState& state,const scitos_docking::ChargingResultConstPtr& result);
		static void activeCb();
		static void feedbackCb(const scitos_docking::ChargingFeedbackConstPtr& feedback);
};

#endif
