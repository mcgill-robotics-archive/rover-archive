//
// Created by David Lavoie-Boutin on 23/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_ARMDATA_H
#define HCI_CPP_ARMDATA_H

enum ArmJoint
{
    BASE = 1,
    D1,
    D2,
    END_EFFECTOR
};

enum ArmMode
{
    CLOSED = 1,
    OPEN
};

enum ArmClosedLoopMode
{
    POSITION = 1,
    ORIENTATION
};
#endif //HCI_CPP_ARMDATA_H
