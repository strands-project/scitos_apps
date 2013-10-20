#ifndef EMERGENCYSTOP_H
#define EMERGENCYSTOP_H

#include <stdio.h>
#include <signal.h>
#include <string>

#include <QWidget>
#include <QTimer>
#include <QStringList>

#include <boost/thread.hpp>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <scitos_msgs/ResetMotorStop.h>
#include <scitos_msgs/EmergencyStop.h>
#include <rqt_gui_cpp/plugin.h>
#include <pluginlib/class_list_macros.h>
#endif


#include "ui_emergencystop.h"

#define EMERGENCY_STOP "/emergency_stop"
#define RESET_MOTORS "/reset_motorstop"
#define BUMPER "/bumper"


namespace rqt_emergency_stop {

class EmergencyStop : public rqt_gui_cpp::Plugin
{
    Q_OBJECT
    
public:
    EmergencyStop();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
    
protected slots:
    void on_stopButton_clicked();
    void changeColour(bool);
		
signals:
    void motorStatusChanged(bool);

private:
	void bumperCallback(const std_msgs::Bool::ConstPtr& msg);
	bool isMotorsOn();
	
    Ui::EmergencyStop ui;
	QWidget* widget;
	boost::thread m_Thread;
	boost::mutex service_mut, bumper_mut;
	bool motors_on;
	ros::ServiceClient reset_client, emergency_client;
    scitos_msgs::ResetMotorStop reset_srv;
    scitos_msgs::EmergencyStop emergency_srv;
	ros::Subscriber sub;
};
}

#endif // EMERGENCYSTOP_H
