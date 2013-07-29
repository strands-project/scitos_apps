#include <QtGui/QApplication>
#include "emergencystop.h"

int main(int argc, char *argv[])
{
		//Init ros and start thread
		ros::init(argc, argv, "touch_stop");
		RosThread rt("touch_stop");
		rt.start();

		//Init Qt and show gui
		QApplication app(argc, argv);
    EmergencyStop gui(0, &rt);
    gui.show();

		//Bind the RosThread::stopRos() function to the aboutToQuit() function og Qt
		//About to quit is an internal Qt function which will be called before the Gui dies.
		QObject::connect(&app, SIGNAL(aboutToQuit()), &gui, SLOT(stopRos()));

		//Start Qt loop. Because of this we have to run ros::spin() in another thread.
		return app.exec();
}
