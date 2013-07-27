#include "emergencystop.h"
#include "ui_emergencystop.h"

EmergencyStop::EmergencyStop(QWidget *parent, RosThread *rt) :
    QWidget(parent),
    ui(new Ui::EmergencyStop),
		rt(rt) {
    ui->setupUi(this);
		motors_on = true;
}

EmergencyStop::~EmergencyStop() {
    delete ui;
}

void EmergencyStop::on_stopButton_clicked() {	
		printf("Button\n");	
		if(motors_on)	
				motors_on = !rt->callService(EMERGENCY_STOP);
		else
				motors_on = !rt->callService(RESET_MOTORS);
}
