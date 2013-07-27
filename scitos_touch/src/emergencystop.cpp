#include "emergencystop.h"
#include "ui_emergencystop.h"

EmergencyStop::EmergencyStop(QWidget *parent, RosThread *rt) :
    QWidget(parent),
    ui(new Ui::EmergencyStop),
		rt(rt) {
    ui->setupUi(this);
		motors_on = true;
		ui->stopButton->setStyleSheet("background-color: red");
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
