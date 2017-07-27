//
// Created by dd8641 on 2016-07-19.
//

#ifndef RIMSTREAMER_NEWGRAPHICSDISPLAY_H
#define RIMSTREAMER_NEWGRAPHICSDISPLAY_H

#include <QLabel>
#include <QtCore/QPointer>
#include "CustomSink.h"

class NewGraphicsDisplay : public QLabel
{
public:
    NewGraphicsDisplay(CustomSink* sink, QWidget *parent = 0);
    NewGraphicsDisplay(QWidget *parent = 0);
    virtual void paintEvent(QPaintEvent* ev);
    QGst::ElementPtr videoSink();

public slots:
	void callUpdate();

private:
	void draw(QPainter &painter);
    QPointer<CustomSink> mSink;
};


#endif //RIMSTREAMER_NEWGRAPHICSDISPLAY_H
