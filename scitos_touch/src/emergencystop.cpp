#include "emergencystop.h"
#include "ui_emergencystop.h"

EmergencyStop::EmergencyStop(QWidget *parent, RosThread *rt) :
    QWidget(parent),
    ui(new Ui::EmergencyStop),
		rt(rt) {

		//Set up UI    
		ui->setupUi(this);

		//Check motor state for inital button colour. Doen't work at the moment.
		if(rt->isMotorsOn())
				ui->stopButton->setStyleSheet("background-color: red");
		else
				ui->stopButton->setStyleSheet("background-color: green");

		//Catch system signals and call clanup function.
		//Neccessary because Qt does not do well with Ctrl+C
		signal(SIGINT, cleanupAtEndOfProgram);
}

EmergencyStop::~EmergencyStop() {
    delete ui;
}

//Button callback
void EmergencyStop::on_stopButton_clicked() {	
		if(rt->isMotorsOn())	{ //Check state the motor is in and choose functionality according to it.
				if(rt->callService(EMERGENCY_STOP)) { //If service call successful, change colour and text of button	
						ui->stopButton->setStyleSheet("background-color: green");
						ui->stopButton->setText("GO");
				}
		} else {
				if(rt->callService(RESET_MOTORS)) { //If service call successful, change colour and text of button
						ui->stopButton->setStyleSheet("background-color: red");
						ui->stopButton->setText("STOP");
				}
		}
}

//Call stop function of RosThread. 
//Neccessary because ros will end up in a segfault if parent Qt component is killed and ros is not stopped.
void EmergencyStop::stopRos() {
		rt->stop();
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
