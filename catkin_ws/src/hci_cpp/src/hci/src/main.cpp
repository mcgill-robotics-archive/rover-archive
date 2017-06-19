//
// Created by david on 18/06/17.
//

#include <QApplication>
#include <view/MainView.h>

int main(int argc, char ** argv)
{
    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(":/rover3.jpg"));

    MainView display;
    display.show();
    return app.exec();
}