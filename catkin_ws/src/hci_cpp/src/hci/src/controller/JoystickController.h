//
// Created by David Lavoie-Boutin on 20/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_JOYSTICKCONTROLLER_H
#define HCI_CPP_JOYSTICKCONTROLLER_H

#include <QObject>
#include <view/JoystickView.h>
#include "JoystickInterface.h"
#include "JoystickAcquisition.h"

/**
 * !@brief Main controller class for joystick applications
 * Owner of the joystick acquisition class and controls the dispatch of the
 * joystick data. Class wishing to receive joystick data must register
 * themselves to this class in order to eventually be activated.
 */
class JoystickController : public QObject {
    Q_OBJECT
public:

    /**
     * !@brief Constructor. Initializes acquisition and connects the view
     * components to the appropriate slots
     *
     * @param joystickWidget Initialized JoystickWidget that will display the
     * button and emit the signal to change the active joystick handler
     */
    JoystickController(JoystickView *joystickWidget);

    virtual ~JoystickController() {};

    /**
     * !@brief Registers a new controller for joystick capability
     *
     * Add a class to the list of control mode on the linked widget and add
     * it to the map of available joystick classes.
     *
     * @param controller Pointer to the controller class that will be connected
     * to the acquisition signal when activated.
     * @param name String displayed on the button to activate that class.
     * Will also be the key in the map for that controller
     */
    void registerController(JoystickInterface *controller, QString name);

    /**
     * !@brief Change which class receives joystick inputs.
     *
     * Disconnect previously active class and connect joystick acquisition to
     * this class. The available classes are placed in a map, string keyed,
     * so the name passed in argument must be a string present in the map of
     * available controllers.
     *
     * @param name String used as a key to find the controller to activate.
     */
    void setActiveController(QString name);

signals:

    /**
     * !@brief Signal emitted when a change of active controller has occurred
     * @param name The string used as key to find the newly activated controller
     */
    void activeControllerChanged(QString name);

private:
    JoystickView* mJoystickView;
    QHash<QString, JoystickInterface*> mControllerHash;
    JoystickInterface* mActiveController;

    JoystickAcquisition* mJoystickAcquisition;
};


#endif //HCI_CPP_JOYSTICKCONTROLLER_H
