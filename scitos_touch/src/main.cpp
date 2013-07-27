#include <QtGui/QApplication>
#include "emergencystop.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    EmergencyStop w;
    w.show();
    
    return a.exec();
}
