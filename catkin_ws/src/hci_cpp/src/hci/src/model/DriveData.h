//
// Created by David Lavoie-Boutin on 19/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_DRIVEDATA_H
#define HCI_CPP_DRIVEDATA_H

/// \brief Container for the status information of the wheels
struct DriveStatusData
{
    bool flGood;
    bool frGood;
    bool mlGood;
    bool mrGood;
    bool blGood;
    bool brGood;
};

/// \brief Possible steering modes
enum SteeringMode
{
    ACKERMANN = 1,
    POINT = 2,
    TRANSLATE = 3
};


#endif //HCI_CPP_DRIVEDATA_H
