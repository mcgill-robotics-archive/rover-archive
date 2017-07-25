//
// Created by David Lavoie-Boutin on 25/07/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include "CameraFilterView.h"

CameraFilterView::CameraFilterView(QWidget *parent) : QWidget(parent) {
    QGridLayout* grid = new QGridLayout();
    grid->addWidget(new QLabel("Hue Low", this), 0, 0);
    grid->addWidget(new QLabel("Hue High", this), 1, 0);
    grid->addWidget(new QLabel("Saturation Low", this), 2, 0);
    grid->addWidget(new QLabel("Saturation High", this), 3, 0);
    grid->addWidget(new QLabel("Value Low", this), 0, 2);
    grid->addWidget(new QLabel("Value High", this), 1, 2);
    
    h_low_lbl = new QSpinBox(this);
    grid->addWidget(h_low_lbl, 0, 1);
    
    h_high_lbl = new QSpinBox(this);
    grid->addWidget(h_high_lbl, 1, 1);
    
    s_low_lbl = new QSpinBox(this);
    grid->addWidget(s_low_lbl, 2, 1);
    
    s_high_lbl = new QSpinBox(this);
    grid->addWidget(s_high_lbl, 3, 1);
    
    v_low_lbl = new QSpinBox(this);
    grid->addWidget(v_low_lbl, 0, 3);
    
    v_high_lbl = new QSpinBox(this);
    grid->addWidget(v_high_lbl, 1, 3);

    QLabel* title_lbl = new QLabel("Camera settings", this);
    QFont title_font;
    title_font.setPointSize(15);
    title_font.setBold(true);
    title_font.setWeight(75);
    title_lbl->setFont(title_font);

    QVBoxLayout* v = new QVBoxLayout();
    v->addWidget(title_lbl);
    v->addItem(grid);
    setLayout(v);
}
