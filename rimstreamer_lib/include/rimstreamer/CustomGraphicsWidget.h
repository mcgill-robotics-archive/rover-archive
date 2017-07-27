//
// Created by dd8641 on 2016-07-06.
//

#ifndef RIMSTREAMER_CUSTOMGRAPHICSWIDGET_H
#define RIMSTREAMER_CUSTOMGRAPHICSWIDGET_H

#include <QtWidgets/QGraphicsWidget>
#include <QtCore/QPointer>
//#include "CustomVideoSurface.h"


namespace rimstreamer
{
class CustomVideoSurface;

class CustomGraphicsWidget : public QGraphicsWidget
{
public:
    explicit CustomGraphicsWidget(QGraphicsItem *parent = 0, Qt::WindowFlags wFlags = 0);
    virtual ~CustomGraphicsWidget();

    /*! Reimplemented from QGraphicsWidget. Do not call directly. */
    virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = 0);

    CustomVideoSurface *surface() const;
    void setSurface(CustomVideoSurface *surface);

private:
    QPointer<CustomVideoSurface> m_surface;
};
}


#endif //RIMSTREAMER_CUSTOMGRAPHICSWIDGET_H
