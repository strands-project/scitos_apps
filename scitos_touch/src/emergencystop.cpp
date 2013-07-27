#include "emergencystop.h"
#include "ui_emergencystop.h"

EmergencyStop::EmergencyStop(QWidget *parent, RosThread *rt) :
    QWidget(parent),
    ui(new Ui::EmergencyStop),
		rt(rt) {
    ui->setupUi(this);
		if(rt->isMotorsOn())
				ui->stopButton->setStyleSheet("background-color: red");
		else
				ui->stopButton->setStyleSheet("background-color: green");
		signal(SIGINT, cleanupAtEndOfProgram);
}

EmergencyStop::~EmergencyStop() {
    delete ui;
}

void EmergencyStop::on_stopButton_clicked() {	
		if(rt->isMotorsOn())	{
				rt->callService(EMERGENCY_STOP);
				if(!rt->isMotorsOn()) {			
						ui->stopButton->setStyleSheet("background-color: green");
						ui->stopButton->setText("GO");
				}
		} else {
				rt->callService(RESET_MOTORS);
				if(rt->isMotorsOn()) {
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
