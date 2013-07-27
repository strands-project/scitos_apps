#ifndef EMERGENCYSTOP_H
#define EMERGENCYSTOP_H

#include <QWidget>

#include <ros/ros.h>
#include <ros/console.h>
#include <scitos_msgs/ResetMotorStop.h>
#include <scitos_msgs/EmergencyStop.h>

namespace Ui {
class EmergencyStop;
}

class EmergencyStop : public QWidget
{
    Q_OBJECT
    
public:
    explicit EmergencyStop(QWidget *parent = 0);
    ~EmergencyStop();
    
private slots:
    void on_stopButton_clicked();

private:
    Ui::EmergencyStop *ui;
    ros::ServiceClient reset_client, emergency_client;
    scitos_msgs::ResetMotorStop reset_srv;
    scitos_msgs::EmergencyStop emergency_srv;
};

#endif // EMERGENCYSTOP_H
