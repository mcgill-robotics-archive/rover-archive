//
// Created by david on 18/06/17.
//

#ifndef HCI_CPP_MAINVIEW_H
#define HCI_CPP_MAINVIEW_H


#include <QtWidgets/QWidget>

class MainView : public QWidget{

    Q_OBJECT

public:
    MainView(QWidget* parent=0);
    virtual ~MainView() {};

};


#endif //HCI_CPP_MAINVIEW_H
