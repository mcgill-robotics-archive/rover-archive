//
// Created by David Lavoie-Boutin on 19/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include "utilities.h"

void setBadBG(QLabel* widget)
{
    widget->setStyleSheet("background-color:#ff0000");
    widget->setText("Bad");
}

void setOkBG(QLabel* widget)
{
    widget->setStyleSheet("background-color:#33CC33");
    widget->setText("Ok");
}

void setBadBG(QLabel* widget, QString string)
{
    widget->setStyleSheet("background-color:#ff0000");
    widget->setText(string);
}

void setOkBG(QLabel* widget, QString string)
{
    widget->setStyleSheet("background-color:#33CC33");
    widget->setText(string);
}
