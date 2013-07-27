#ifndef EMERGENCYSTOP_H
#define EMERGENCYSTOP_H

#include <stdio.h>

#include <QWidget>
#include <QTimer>

#include "ros_thread.h"


namespace Ui {
class EmergencyStop;
}

class EmergencyStop : public QWidget
{
    Q_OBJECT
    
public:
    explicit EmergencyStop(QWidget *parent = 0, RosThread *rt = 0);
    ~EmergencyStop();
    
private slots:
    void on_stopButton_clicked();

private:
    Ui::EmergencyStop *ui;
		RosThread *rt;
		bool motors_on;
};

#endif // EMERGENCYSTOP_H
