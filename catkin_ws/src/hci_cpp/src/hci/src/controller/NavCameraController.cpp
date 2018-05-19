//
// Created by David Lavoie-Boutin on 05/07/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include "NavCameraController.h"
#include <rimstreamer/AxisF34VideoFeed.h>

NavCameraController::NavCameraController(NavCameraView* navCameraView) : navCamViewPtr(navCameraView)
{
    //topFeedPtr.reset(new rimstreamer::NyanVideoFeed(rimstreamer::CAT));
    feed1Ptr.reset(new rimstreamer::AxisF34VideoFeed("192.168.3.100", rimstreamer::F34_CAMERA_1, rimstreamer::F34_854x480, rimstreamer::Landscape));
//    leftFeedPtr.reset(new rimstreamer::NyanVideoFeed(rimstreamer::DOG));
    feed2Ptr.reset(new rimstreamer::AxisF34VideoFeed("192.168.3.100", rimstreamer::F34_CAMERA_2, rimstreamer::F34_480x360, rimstreamer::Portrait));
//    rightFeedPrt.reset(new rimstreamer::NyanVideoFeed(rimstreamer::DOG));
    feed3Ptr.reset(new rimstreamer::AxisF34VideoFeed("192.168.3.100", rimstreamer::F34_CAMERA_3, rimstreamer::F34_480x360, rimstreamer::Portrait));

    navCamViewPtr->setVideoFeed(feed1Ptr, NavCameraView::TOP);
    navCamViewPtr->setVideoFeed(feed2Ptr, NavCameraView::BOTTOM_LEFT);
    navCamViewPtr->setVideoFeed(feed3Ptr, NavCameraView::BOTTOM_RIGHT);

    topInd = 1;
    bottomLeftInd = 2;
    bottomRightInd = 3;

    feed1Ptr->play();
    feed2Ptr->play();
    feed3Ptr->play();


    //connect camera feeds to the index of the SingleCameraView (through NavCameraView)
    //connect(navCameraView, &NavCameraView::changeFeed, this, &NavCameraController::changeFeed);

//    rightFeedPrt->takeSnapshot("./file.jpeg");
}

void NavCameraController::changeFeed(NavCameraView::ScreenID id, int index){
    stopFeedOf(id, index);
    if (index == 0){
        navCamViewPtr->releaseVideoFeed(id);
        navCamViewPtr->setVideoFeed(feed1Ptr, id);
        feed1Ptr->play();
    }
    else if (index == 1){
        navCamViewPtr->releaseVideoFeed(id);
        navCamViewPtr->setVideoFeed(feed2Ptr, id);
        feed2Ptr->play();
    }
    else if (index == 2){
        navCamViewPtr->releaseVideoFeed(id);
        navCamViewPtr->setVideoFeed(feed3Ptr, id);
        feed3Ptr->play();
    }
    //add any new camera feeds here
    else{
        navCamViewPtr->releaseVideoFeed(id);
        navCamViewPtr->setVideoFeed(feed1Ptr, id);
        feed1Ptr->play();
    }          
}

/*stopFeedOf
    Input: NavCameraView::ScreenID id, int ind
    ouptut: None
    Effect: Given an ScreenID id, it will stop the feed currently running at id and update the id index to the new index
*/

void NavCameraController::stopFeedOf(NavCameraView::ScreenID id, int ind){
    switch(id){
        case NavCameraView::TOP:
            if(topInd ==1)
                feed1Ptr->stop();
            else if (topInd ==2)
                feed2Ptr->stop();
            else if (topInd == 3)
                feed3Ptr->stop();
            else
                feed1Ptr->stop();
            topInd = ind;
        case NavCameraView::BOTTOM_LEFT:
            if(bottomLeftInd ==1)
                feed1Ptr->stop();
            else if (bottomLeftInd ==2)
                feed2Ptr->stop();
            else if (bottomLeftInd == 3)
                feed3Ptr->stop();
            else
                feed1Ptr->stop();
            bottomLeftInd=ind;
      case NavCameraView::BOTTOM_RIGHT:
            if(bottomRightInd ==1)
                feed1Ptr->stop();
            else if (bottomRightInd ==2)
                feed2Ptr->stop();
            else if (bottomRightInd == 3)
                feed3Ptr->stop();
            else
                feed1Ptr->stop();
            bottomRightInd=ind;
     //default:     ??
            
    }
}



