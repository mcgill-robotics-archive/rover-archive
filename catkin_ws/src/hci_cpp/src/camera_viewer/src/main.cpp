//
// Created by david on 17/06/17.
//

#include <QtWidgets/QApplication>
#include <QGst/Init>

#include "rimstreamer/GstVideoFeed.h"
#include "rimstreamer/NyanVideoFeed.h"
#include "rimstreamer/AxisF34VideoFeed.h"

#include "MainWindowQt9.h"

int main(int argc, char **argv)
{
    QGst::init();

    QApplication app(argc, argv);

    rimstreamer::GstVideoFeedPtr catFeed(new rimstreamer::AxisF34VideoFeed("192.168.3.100", rimstreamer::F34_CAMERA_1, rimstreamer::F34_480x270));
    rimstreamer::GstVideoFeedPtr dogFeed(new rimstreamer::NyanVideoFeed(rimstreamer::DOG));

    rimstreamer::MainWindowQt9 win("Cat Video", catFeed, "Dog Video", dogFeed);

    win.show();
    return app.exec();
}
