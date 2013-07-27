#include "emergencystop.h"
#include "ui_emergencystop.h"

EmergencyStop::EmergencyStop(QWidget *parent, RosThread *rt) :
    QWidget(parent),
    ui(new Ui::EmergencyStop),
		rt(rt) {
    ui->setupUi(this);
}

EmergencyStop::~EmergencyStop() {
    delete ui;
}

void EmergencyStop::on_stopButton_clicked() {	
		printf("Button\n");		
		rt->callService(EMERGENCY_STOP);
}
