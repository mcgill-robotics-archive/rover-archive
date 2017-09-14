//
// Created by David Lavoie-Boutin on 18/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_DRIVEVIEW_H
#define HCI_CPP_DRIVEVIEW_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QLabel>
#include <model/DriveData.h>
#include <QtWidgets/QPushButton>


class DriveView : public QWidget {
    Q_OBJECT
public:
    DriveView(QWidget *parent = 0);

    virtual ~DriveView() {};

signals:
    void steeringModeChanged(const SteeringMode& mode);

public slots:
    void updateDriveStatus(const DriveStatusData& statusData);
    void updateSteeringMode(const SteeringMode& mode);

private:
    QLabel* pFLStatus;
    QLabel* pFRStatus;
    QLabel* pMLStatus;
    QLabel* pMRStatus;
    QLabel* pBLStatus;
    QLabel* pBRStatus;

    QPushButton* pEnableAckermann;
    QPushButton* pEnablePoint;
    QPushButton* pEnableTranslate;
    QLabel* pActiveSteeringMode;

private slots:
    void onModeButtonClick(const QAbstractButton *button);
};


#endif //HCI_CPP_DRIVEVIEW_H
