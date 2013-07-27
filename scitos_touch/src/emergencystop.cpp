#include "emergencystop.h"
#include "ui_emergencystop.h"

EmergencyStop::EmergencyStop(QWidget *parent, RosThread *rt) :
    QWidget(parent),
    ui(new Ui::EmergencyStop),
		rt(rt) {
    ui->setupUi(this);
		motors_on = true;
		ui->stopButton->setStyleSheet("background-color: red");
		signal(SIGINT, cleanupAtEndOfProgram);
}

EmergencyStop::~EmergencyStop() {
    delete ui;
}

void EmergencyStop::on_stopButton_clicked() {	
		if(motors_on)	{
				motors_on = !rt->callService(EMERGENCY_STOP);
				if(!motors_on) {			
						ui->stopButton->setStyleSheet("background-color: green");
						ui->stopButton->setText("GO");
				}
		} else {
				motors_on = rt->callService(RESET_MOTORS);
				if(motors_on) {
						ui->stopButton->setStyleSheet("background-color: red");
						ui->stopButton->setText("STOP");
				}
		}
}

void EmergencyStop::stopRos() {
		rt->stop();
}

void EmergencyStop::cleanupAtEndOfProgram(int catched_signal)
{
		if (catched_signal == SIGINT) {
				QCoreApplication::quit();
    }
		if (catched_signal == SIGTERM) {
				QCoreApplication::quit();
    }
}
