//
// Created by David Lavoie-Boutin on 25/07/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_CAMERAFILTERVIEW_H
#define HCI_CPP_CAMERAFILTERVIEW_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QSpinBox>


class CameraFilterView : public QWidget {
Q_OBJECT
public:
    CameraFilterView(QWidget *parent = 0);

    virtual ~CameraFilterView() {};
private:
    QSpinBox* h_low_lbl;
    QSpinBox* h_high_lbl;
    QSpinBox* s_low_lbl;
    QSpinBox* s_high_lbl;
    QSpinBox* v_low_lbl;
    QSpinBox* v_high_lbl;
};


#endif //HCI_CPP_CAMERAFILTERVIEW_H
