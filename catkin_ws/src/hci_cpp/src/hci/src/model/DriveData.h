//
// Created by David Lavoie-Boutin on 19/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_DRIVEDATA_H
#define HCI_CPP_DRIVEDATA_H

struct DriveStatusData
{
    bool flGood;
    bool frGood;
    bool mlGood;
    bool mrGood;
    bool blGood;
    bool brGood;
};

enum SteeringMode
{
    ACKERMANN = 0,
    POINT = 1,
    TRANSLATE = 2
};


#endif //HCI_CPP_DRIVEDATA_H
