//
// Created by david on 18/06/17.
//

#include <QApplication>
#include <view/MainView.h>
#include <controller/MainController.h>
#include <QGst/Init>

void myMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    QByteArray localMsg = msg.toLocal8Bit();
    switch (type) {
        case QtDebugMsg:
            //fprintf(stderr, "Debug: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
            break;
        case QtInfoMsg:
            fprintf(stderr, "Info: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
            break;
        case QtWarningMsg:
            //fprintf(stderr, "Warning: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
            break;
        case QtCriticalMsg:
            fprintf(stderr, "Critical: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
            break;
        case QtFatalMsg:
            fprintf(stderr, "Fatal: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
            abort();
    }
}

int main(int argc, char ** argv)
{
    qInstallMessageHandler(myMessageOutput);
    QGst::init();

    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(":/rover3.jpg"));

    ros::init(argc, argv, "hci");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    MainController ctrl(nh, pnh);

    return app.exec();
}