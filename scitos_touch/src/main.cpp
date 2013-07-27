#include <QtGui/QApplication>
#include "emergencystop.h"

int main(int argc, char *argv[])
{
		ros::init(argc, argv, "touch_stop");
		RosThread rt("touch_stop");
		rt.start();

		QApplication app(argc, argv);
    EmergencyStop gui(0, &rt);
    gui.show();

		return app.exec();
}
