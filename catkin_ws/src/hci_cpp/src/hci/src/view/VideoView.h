//
// Created by David Lavoie-Boutin on 30/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_VIDEOVIEW_H
#define HCI_CPP_VIDEOVIEW_H

#include <QtWidgets/QWidget>
#include <rimstreamer/VideoFeedWidget.h>

using rimstreamer::GstVideoFeedPtr;

class VideoView : public QWidget {
Q_OBJECT
public:
    VideoView(QWidget *parent = 0);

    virtual ~VideoView() {};

public slots:
    void changeTopFeed(const GstVideoFeedPtr& videoFeed);

private:
    rimstreamer::VideoFeedWidget* topScreen;
};


#endif //HCI_CPP_VIDEOVIEW_H
