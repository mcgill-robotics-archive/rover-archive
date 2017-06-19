//
// Created by david on 18/06/17.
//

#include <QApplication>
#include <view/AttitudeView.h>

int main(int argc, char ** argv)
{
    QApplication app(argc, argv);
    AttitudeView display;
    display.show();
    return app.exec();
}