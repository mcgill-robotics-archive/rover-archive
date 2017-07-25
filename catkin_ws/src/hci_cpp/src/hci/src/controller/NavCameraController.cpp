//
// Created by David Lavoie-Boutin on 05/07/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include "NavCameraController.h"
#include <rimstreamer/AxisF34VideoFeed.h>

NavCameraController::NavCameraController(NavCameraView* navCameraView) : navCamViewPtr(navCameraView)
{
//    topFeedPtr.reset(new rimstreamer::NyanVideoFeed(rimstreamer::CAT));
    topFeedPtr.reset(new rimstreamer::AxisF34VideoFeed("192.168.1.1", rimstreamer::F34_CAMERA_1, rimstreamer::F34_1024x768));
    leftFeedPtr.reset(new rimstreamer::NyanVideoFeed(rimstreamer::DOG));
//    leftFeedPtr.reset(new rimstreamer::AxisF34VideoFeed("192.168.1.1", rimstreamer::F34_CAMERA_1, rimstreamer::F34_640x480));
    rightFeedPrt.reset(new rimstreamer::NyanVideoFeed(rimstreamer::DOG));
//    rightFeedPrt.reset(new rimstreamer::AxisF34VideoFeed("192.168.1.1", rimstreamer::F34_CAMERA_1, rimstreamer::F34_640x480));

//    navCamViewPtr->setVideoFeed(topFeedPtr, NavCameraView::TOP);
    navCamViewPtr->setVideoFeed(leftFeedPtr, NavCameraView::LEFT);
    navCamViewPtr->setVideoFeed(rightFeedPrt, NavCameraView::RIGHT);

//    topFeedPtr->play();
    leftFeedPtr->play();
    rightFeedPrt->play();

//    rightFeedPrt->takeSnapshot("./file.jpeg");
}
