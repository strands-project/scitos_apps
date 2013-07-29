#ifndef EMERGENCYSTOP_H
#define EMERGENCYSTOP_H

#include <stdio.h>
#include <signal.h>

#include <QWidget>
#include <QTimer>

#include <rqt_gui_cpp/plugin.h>

#include "ui_emergencystop.h"
#include "rqt_emergency_stop/ros_comm.h"


namespace rqt_emergency_stop {

class EmergencyStop : public rqt_gui_cpp::Plugin
{
    Q_OBJECT
    
public:
    EmergencyStop();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
    
protected slots:
        void on_stopButton_clicked();
		void stopRos();

private:
		static void cleanupAtEndOfProgram(int catched_signal);
        Ui::EmergencyStop ui;
		QWidget* widget;
		RosComm* rc;
};
}

#endif // EMERGENCYSTOP_H
