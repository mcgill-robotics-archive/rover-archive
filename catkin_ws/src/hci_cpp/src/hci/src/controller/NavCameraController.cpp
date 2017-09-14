//
// Created by David Lavoie-Boutin on 05/07/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include "NavCameraController.h"
#include <rimstreamer/AxisF34VideoFeed.h>

NavCameraController::NavCameraController(NavCameraView* navCameraView) : navCamViewPtr(navCameraView)
{
//    topFeedPtr.reset(new rimstreamer::NyanVideoFeed(rimstreamer::CAT));
    topFeedPtr.reset(new rimstreamer::AxisF34VideoFeed("192.168.3.100", rimstreamer::F34_CAMERA_1, rimstreamer::F34_854x480, rimstreamer::Landscape));
//    leftFeedPtr.reset(new rimstreamer::NyanVideoFeed(rimstreamer::DOG));
    leftFeedPtr.reset(new rimstreamer::AxisF34VideoFeed("192.168.3.100", rimstreamer::F34_CAMERA_2, rimstreamer::F34_480x360, rimstreamer::Portrait));
//    rightFeedPrt.reset(new rimstreamer::NyanVideoFeed(rimstreamer::DOG));
    rightFeedPrt.reset(new rimstreamer::AxisF34VideoFeed("192.168.3.100", rimstreamer::F34_CAMERA_3, rimstreamer::F34_480x360, rimstreamer::Portrait));

    navCamViewPtr->setVideoFeed(topFeedPtr, NavCameraView::TOP);
    navCamViewPtr->setVideoFeed(leftFeedPtr, NavCameraView::LEFT);
    navCamViewPtr->setVideoFeed(rightFeedPrt, NavCameraView::RIGHT);

    topFeedPtr->play();
    leftFeedPtr->play();
    rightFeedPrt->play();

//    rightFeedPrt->takeSnapshot("./file.jpeg");
}
