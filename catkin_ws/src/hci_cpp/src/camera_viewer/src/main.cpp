//
// Created by david on 17/06/17.
//

#include <QtWidgets/QApplication>
#include <QGst/Init>

#include "rimstreamer/GstVideoFeed.h"
#include "rimstreamer/NyanVideoFeed.h"

#include "MainWindowQt9.h"

int main(int argc, char **argv)
{
    QGst::init();

    QApplication app(argc, argv);

    rimstreamer::GstVideoFeedPtr catFeed(new rimstreamer::NyanVideoFeed(rimstreamer::CAT));
    rimstreamer::GstVideoFeedPtr dogFeed(new rimstreamer::NyanVideoFeed(rimstreamer::DOG));

    rimstreamer::MainWindowQt9 win("Cat", catFeed, "Dog", dogFeed);

    win.show();
    return app.exec();
}
