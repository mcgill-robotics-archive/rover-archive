//
// Created by david on 18/06/17.
//

#include <QApplication>
#include <view/MainView.h>
#include <controller/MainController.h>
#include <QGst/Init>

int main(int argc, char ** argv)
{

    QGst::init();

    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(":/rover3.jpg"));

    ros::init(argc, argv, "hci");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    MainController ctrl(nh, pnh);

    return app.exec();
}