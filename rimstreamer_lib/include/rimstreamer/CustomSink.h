//
// Created by dd8641 on 2016-07-18.
//

#ifndef RIMSTREAMER_CUSTOMSINK_H
#define RIMSTREAMER_CUSTOMSINK_H

#include <QGst/Buffer>
#include <QGst/memory.h>

#include <QGst/Utils/ApplicationSink>
#include <QDebug>
#include <QtGui/QImage>
#include <QtCore/QMutex>

class CustomSink : public QObject, public QGst::Utils::ApplicationSink
{
	Q_OBJECT
public:
    CustomSink();
    virtual QGst::FlowReturn newSample();
    QImage getFrame();

signals:
	void receivedFrame();

private:
    QMutex mMutex;
    QImage mImage;
};


#endif //RIMSTREAMER_CUSTOMSINK_H
