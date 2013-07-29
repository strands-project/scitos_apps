#ifndef EMERGENCYSTOP_H
#define EMERGENCYSTOP_H

#include <stdio.h>
#include <signal.h>

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
		void stopRos();

private:
		static void cleanupAtEndOfProgram(int catched_signal);
    Ui::EmergencyStop *ui;
		RosThread *rt;
};

#endif // EMERGENCYSTOP_H
