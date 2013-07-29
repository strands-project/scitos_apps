#include "rqt_emergency_stop/emergencystop.h"

namespace rqt_emergency_stop {

EmergencyStop::EmergencyStop() : rqt_gui_cpp::Plugin()
  , widget(0) {
    setObjectName("EmergencyStop");
}

void EmergencyStop::initPlugin(qt_gui_cpp::PluginContext& context) {
    ROS_INFO("Init");
    
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget);
    // add widget to the user interface
    context.addWidget(widget);
    
    ros::NodeHandle n("EmergencyStop");		
    reset_client = n.serviceClient<scitos_msgs::ResetMotorStop>(RESET_MOTORS);
    emergency_client = n.serviceClient<scitos_msgs::EmergencyStop>(EMERGENCY_STOP);
	sub = n.subscribe(BUMPER, 1000, &EmergencyStop::bumperCallback, this);
    
    connect(ui.stopButton, SIGNAL(clicked()), this, SLOT(on_stopButton_clicked()));
    connect(this, SIGNAL(motorStatusChanged(bool)), this, SLOT(changeColour(bool)));
    
    start();
}

void EmergencyStop::start() {
    m_Thread = boost::thread(&EmergencyStop::spin, this);  
}

void EmergencyStop::spin() {
    ros::spin();
}

void EmergencyStop::shutdownPlugin()
{
    sub.shutdown();
    ros::shutdown();
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
    if(isMotorsOn()) {		
        if (!emergency_client.call(emergency_srv)) {
            ROS_ERROR("Failed to call service /emergency_stop");
        } else {
            ROS_INFO("Motors stopped");
        } 
    } else {
        if (!reset_client.call(reset_srv)) {
            ROS_ERROR("Failed to call service /reset_motorstop");
        } else {
	        ROS_INFO("Motors started");
        } 
    }
}

void EmergencyStop::bumperCallback(const std_msgs::Bool::ConstPtr& msg) {
    boost::lock_guard<boost::mutex> lock(bumper_mut);
    if(msg->data == motors_on) {
        motors_on = !msg->data;
        emit motorStatusChanged(motors_on);
    }
}

bool EmergencyStop::isMotorsOn() {
    boost::lock_guard<boost::mutex> lock(bumper_mut);
    return motors_on;
}

void EmergencyStop::changeColour(bool motors) {
    if(!motors) { //If service call successful, change colour and text of button	
        ui.stopButton->setStyleSheet("background-color: green");
        ui.stopButton->setText("GO");
    } else {
        ui.stopButton->setStyleSheet("background-color: red");
	    ui.stopButton->setText("STOP");
    }
}

}
PLUGINLIB_EXPORT_CLASS(rqt_emergency_stop::EmergencyStop, rqt_gui_cpp::Plugin)
