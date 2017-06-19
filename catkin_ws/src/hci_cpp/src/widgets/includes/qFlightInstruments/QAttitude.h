//
// Created by david on 10/10/16.
//

#ifndef QFLIGHTINSTRUMENTS_QATTITUDE_H
#define QFLIGHTINSTRUMENTS_QATTITUDE_H

#include <QWidget>

///
/// \brief The Attitude indicator class
///
class QADI : public QWidget
{
    Q_OBJECT

public:
    QADI(QWidget *parent = 0);
    ~QADI();

    ///
    /// \brief Set roll & pitch values (in degree)
    /// \param r - roll
    /// \param p - pitch
    ///
    void setData(double r, double p) {
        m_roll = r;
        m_pitch = p;
        if( m_roll < -180 ) m_roll = -180;
        if( m_roll > 180  ) m_roll =  180;
        if( m_pitch < -90 ) m_pitch = -90;
        if( m_pitch > 90  ) m_pitch =  90;

        emit canvasReplot();
    }

    ///
    /// \brief Set roll angle (in degree)
    /// \param val - roll
    ///
    void setRoll(double val);

    ///
    /// \brief Set pitch value (in degree)
    /// \param val
    ///
    void setPitch(double val);

    ///
    /// \brief Get roll angle (in degree)
    /// \return roll angle
    ///
    double getRoll() {return m_roll;}

    ///
    /// \brief Get pitch angle (in degree)
    /// \return pitch angle
    ///
    double getPitch(){return m_pitch;}


signals:
    void canvasReplot(void);

protected slots:
    void canvasReplot_slot(void);

protected:
    void paintEvent(QPaintEvent *event);
    void resizeEvent(QResizeEvent *event);

protected:
    int     m_sizeMin, m_sizeMax;           ///< widget's min/max size (in pixel)
    int     m_size, m_offset;               ///< current size & offset

    double  m_roll;                         ///< roll angle (in degree)
    double  m_pitch;                        ///< pitch angle (in degree)
};

#include <QtCore>
#include <QtGui>
#include <QTableWidget>

#endif //QFLIGHTINSTRUMENTS_QATTITUDE_H
