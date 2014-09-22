#include "scitos_docking/CLightClient.h"

CLightClient::CLightClient()
{
	status = false;
	known = false;
}

CLightClient::~CLightClient()
{
}

void CLightClient::set(bool what)
{
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::BoolParameter param;
	dynamic_reconfigure::Config conf;
	param.name = "Port0_12V_Enabled";
	param.value = status = what;
	conf.bools.push_back(param);
	srv_req.config = conf;
	ros::service::call("/EBC/set_parameters", srv_req, srv_resp);
	known = true;
}

bool CLightClient::get()
{
	if (known==false) ROS_WARN("Status of the lights is uknown at the moment. Turn them on or off first.");
	return status;
}
