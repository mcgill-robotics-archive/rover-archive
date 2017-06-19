//
// Created by david on 10/10/16.
//

#include "qFlightInstruments/QCompass.h"


QCompass::QCompass(QWidget *parent)
    : QWidget(parent)
{
    connect(this, SIGNAL(canvasReplot(void)), this, SLOT(canvasReplot_slot(void)));

    m_sizeMin = 200;
    m_sizeMax = 600;
    m_offset = 2;
    m_size = m_sizeMin - 2*m_offset;

    setMinimumSize(m_sizeMin, m_sizeMin);
    setMaximumSize(m_sizeMax, m_sizeMax);
    resize(m_sizeMin, m_sizeMin);

    setFocusPolicy(Qt::NoFocus);

    m_yaw  = 0.0;
    m_alt  = 0.0;
    m_h    = 0.0;
}

QCompass::~QCompass()
{

}

void QCompass::canvasReplot_slot(void)
{
    update();
}

void QCompass::resizeEvent(QResizeEvent *event)
{
    m_size = qMin(width(),height()) - 2*m_offset;
}

void QCompass::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    QBrush bgGround(QColor(48,172,220));

    QPen   whitePen(Qt::white);
    QPen   blackPen(Qt::black);
    QPen   redPen(Qt::red);
    QPen   bluePen(Qt::blue);
    QPen   greenPen(Qt::green);

    whitePen.setWidth(1);
    blackPen.setWidth(2);
    redPen.setWidth(2);
    bluePen.setWidth(2);
    greenPen.setWidth(2);

    painter.setRenderHint(QPainter::Antialiasing);

    painter.translate(width() / 2, height() / 2);


    // draw background
    {
        painter.setPen(blackPen);
        painter.setBrush(bgGround);

        painter.drawEllipse(-m_size/2, -m_size/2, m_size, m_size);
    }


    // draw yaw lines
    {
        int     nyawLines = 36;
        float   rotAng = 360.0 / nyawLines;
        int     yawLineLeng = m_size/25;
        double  fx1, fy1, fx2, fy2;
        int     fontSize = 8;
        QString s;
        painter.rotate(-m_yaw);
        blackPen.setWidth(1);
        painter.setPen(blackPen);

        for(int i=0; i<nyawLines; i++) {

            if( i == 0 ) {
                s = "N";
                painter.setPen(bluePen);

                painter.setFont(QFont("", fontSize*1.3));
            } else if ( i == 9 ) {
                s = "W";
                painter.setPen(blackPen);

                painter.setFont(QFont("", fontSize*1.3));
            } else if ( i == 18 ) {
                s = "S";
                painter.setPen(redPen);

                painter.setFont(QFont("", fontSize*1.3));
            } else if ( i == 27 ) {
                s = "E";
                painter.setPen(blackPen);

                painter.setFont(QFont("", fontSize*1.3));
            } else {
                s = QString("%1").arg(i*rotAng);
                painter.setPen(blackPen);

                painter.setFont(QFont("", fontSize));
            }

            fx1 = 0;
            fy1 = -m_size/2 + m_offset;
            fx2 = 0;

            if( i % 3 == 0 ) {
                fy2 = fy1 + yawLineLeng;
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2));

                fy2 = fy1 + yawLineLeng+4;
                painter.drawText(QRectF(-50, fy2, 100, fontSize+2),
                                 Qt::AlignCenter, s);
            } else {
                fy2 = fy1 + yawLineLeng/2;
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2));
            }

            painter.rotate(-rotAng);
        }
        painter.rotate(m_yaw);
    }

    // draw S/N arrow
    {
        int     arrowWidth = m_size/5;
        double  fx1, fy1, fx2, fy2, fx3, fy3;

        fx1 = 0;
        fy1 = -m_size/2 + m_offset + m_size/25 + 15;
        fx2 = -arrowWidth/2;
        fy2 = 0;
        fx3 = arrowWidth/2;
        fy3 = 0;

        painter.setPen(Qt::NoPen);

        painter.setBrush(QBrush(Qt::blue));
        QPointF pointsN[3] = {
            QPointF(fx1, fy1),
            QPointF(fx2, fy2),
            QPointF(fx3, fy3)
        };
        painter.drawPolygon(pointsN, 3);


        fx1 = 0;
        fy1 = m_size/2 - m_offset - m_size/25 - 15;
        fx2 = -arrowWidth/2;
        fy2 = 0;
        fx3 = arrowWidth/2;
        fy3 = 0;

        painter.setBrush(QBrush(Qt::red));
        QPointF pointsS[3] = {
            QPointF(fx1, fy1),
            QPointF(fx2, fy2),
            QPointF(fx3, fy3)
        };
        painter.drawPolygon(pointsS, 3);
    }


    // draw yaw marker
//    {
//        int     yawMarkerSize = m_size/12;
//        double  fx1, fy1, fx2, fy2, fx3, fy3;
//
//        painter.rotate(-m_yaw);
//        painter.setBrush(QBrush(QColor(0xFF, 0x00, 0x00, 0xE0)));
//
//        fx1 = 0;
//        fy1 = -m_size/2 + m_offset;
//        fx2 = fx1 - yawMarkerSize/2;
//        fy2 = fy1 + yawMarkerSize;
//        fx3 = fx1 + yawMarkerSize/2;
//        fy3 = fy1 + yawMarkerSize;
//
//        QPointF points[3] = {
//            QPointF(fx1, fy1),
//            QPointF(fx2, fy2),
//            QPointF(fx3, fy3)
//        };
//        painter.drawPolygon(points, 3);
//
//        painter.rotate(m_yaw);
//    }

    // draw altitude
    {
        int     altFontSize = 13;
        int     fx, fy, w, h;
        QString s;
        char    buf[200];

        w  = 130;
        h  = 2*(altFontSize + 8);
        fx = -w/2;
        fy = -h/2;

        blackPen.setWidth(2);
        painter.setPen(blackPen);
        painter.setBrush(QBrush(Qt::white));
        painter.setFont(QFont("", altFontSize));

        painter.drawRoundedRect(fx, fy, w, h, 6, 6);

        painter.setPen(bluePen);
        sprintf(buf, "ALT: %6.1f m", m_alt);
        s = buf;
        painter.drawText(QRectF(fx, fy+2, w, h/2), Qt::AlignCenter, s);

        sprintf(buf, "H: %6.1f m", m_h);
        s = buf;
        painter.drawText(QRectF(fx, fy+h/2, w, h/2), Qt::AlignCenter, s);
    }
}

void QCompass::setYaw(double val) {
    m_yaw  = val;
    if( m_yaw < 0   ) m_yaw = 360 + m_yaw;
    if( m_yaw > 360 ) m_yaw = m_yaw - 360;

    emit canvasReplot();
}

void QCompass::setH(double val) {
    m_h = val;

    emit canvasReplot();
}

void QCompass::setAlt(double val) {
    m_alt = val;

    emit canvasReplot();
}
