#include "emergencystop.h"
#include "ui_emergencystop.h"

EmergencyStop::EmergencyStop(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::EmergencyStop)
{
    ui->setupUi(this);
}

EmergencyStop::~EmergencyStop()
{
    delete ui;
}

void EmergencyStop::on_stopButton_clicked()
{

}
