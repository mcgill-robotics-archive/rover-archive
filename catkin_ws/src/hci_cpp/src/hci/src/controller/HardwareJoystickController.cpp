//
// Created by David Lavoie-Boutin on 01/07/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include "HardwareJoystickController.h"
#include <ros/ros.h>
//the bleow line was added
#include <unistd.h>

HardwareJoystickController::HardwareJoystickController() :
        mJoystick("/dev/input/js0")
{
    mData.buttons.resize(12);
}

bool HardwareJoystickController::joystickFound() {
    return mJoystick.isFound();
}

void HardwareJoystickController::process() {
    ros::Rate r(30);

    while (mJoystick.isFound()) // change to endless loop
    {
        // poll joystick for new info
        JoystickEvent event;
	while (mJoystick.sample(&event)) {//added a second while loop
        	if (event.isButton())
        	{
            	mData.buttons[event.number] = event.value;
        	}

        	else if (event.isAxis())
        	{
            	if (event.number == 0)
                	mData.a1 = (float) event.value / (float) JoystickEvent::MAX_AXES_VALUE;
           	else if (event.number == 1)
                	mData.a2 = (float) event.value / (float) JoystickEvent::MAX_AXES_VALUE;
           	else if (event.number == 2)
                	mData.a3 = (float) event.value / (float) JoystickEvent::MAX_AXES_VALUE;
            	else if (event.number == 3)
                	mData.a4 = (float) event.value / (float) JoystickEvent::MAX_AXES_VALUE;
        	}

        	emit joystickDataUpdated(mData);
	}//added
        r.sleep();
    }

}
