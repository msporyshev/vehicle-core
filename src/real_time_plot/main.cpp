#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ros::Time::init();
    MainWindow w;
    w.connect_to_vehicle_ipc(argc, argv);
    w.show();

    return a.exec();
}
