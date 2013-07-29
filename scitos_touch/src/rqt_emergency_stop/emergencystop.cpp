#include "rqt_emergency_stop/emergencystop.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace rqt_emergency_stop {

EmergencyStop::EmergencyStop() : rqt_gui_cpp::Plugin()
  , widget(0) {
    setObjectName("EmergencyStop");
}

void EmergencyStop::initPlugin(qt_gui_cpp::PluginContext& context) {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget);
    // add widget to the user interface
    context.addWidget(widget);
    
    rc = new RosComm("EmergencyStop");
}

void EmergencyStop::shutdownPlugin()
{
  // TODO unregister all publishers here
}

void EmergencyStop::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void EmergencyStop::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

//Button callback
void EmergencyStop::on_stopButton_clicked() {	
		if(rc->isMotorsOn())	{ //Check state the motor is in and choose functionality according to it.
				if(rc->callService(EMERGENCY_STOP)) { //If service call successful, change colour and text of button	
						ui.stopButton->setStyleSheet("background-color: green");
						ui.stopButton->setText("GO");
				}
		} else {
				if(rc->callService(RESET_MOTORS)) { //If service call successful, change colour and text of button
						ui.stopButton->setStyleSheet("background-color: red");
						ui.stopButton->setText("STOP");
				}
		}
}

//Call stop function of RosThread. 
//Neccessary because ros will end up in a segfault if parent Qt component is killed and ros is not stopped.
void EmergencyStop::stopRos() {
		//rt->stop();
}

//Catch sigint and sigterm signal and call Qt quit function
void EmergencyStop::cleanupAtEndOfProgram(int catched_signal)
{
		if (catched_signal == SIGINT) {
				QCoreApplication::quit();
    }
		if (catched_signal == SIGTERM) {
				QCoreApplication::quit();
    }
}

}
PLUGINLIB_DECLARE_CLASS(rqt_emergency_stop, EmergencyStop, rqt_emergency_stop::EmergencyStop, rqt_gui_cpp::Plugin)
