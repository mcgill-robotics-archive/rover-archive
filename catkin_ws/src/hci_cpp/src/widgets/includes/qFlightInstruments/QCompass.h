//
// Created by david on 10/10/16.
//

#ifndef QFLIGHTINSTRUMENTS_QCOMPASS_H
#define QFLIGHTINSTRUMENTS_QCOMPASS_H

#include <QWidget>

///
/// \brief The Compass & altitude display class
///
class QCompass : public QWidget
{
    Q_OBJECT

public:
    QCompass(QWidget *parent = 0);
    ~QCompass();

    ///
    /// \brief Set all data (yaw, alt, height)
    ///
    /// \param y - yaw ( in degree)
    /// \param a - altitude ( in m)
    /// \param h - height from ground (in m)
    ///
    void setData(double y, double a, double h) {
        m_yaw = y;
        m_alt = a;
        m_h   = h;

        if( m_yaw < 0   ) m_yaw = 360 + m_yaw;
        if( m_yaw > 360 ) m_yaw = m_yaw - 360;

        emit canvasReplot();
    }

    ///
    /// \brief Set yaw angle (in degree)
    /// \param val - yaw angle (in degree)
    ///
    void setYaw(double val);

    ///
    /// \brief Set altitude value
    /// \param val - altitude (in m)
    ///
    void setAlt(double val);

    ///
    /// \brief Set height from ground
    /// \param val - height (in m)
    ///
    void setH(double val);

    ///
    /// \brief Get yaw angle
    /// \return yaw angle (in degree)
    ///
    double getYaw() {return m_yaw;}

    ///
    /// \brief Get altitude value
    /// \return altitude (in m)
    ///
    double getAlt() {return m_alt;}

    ///
    /// \brief Get height from ground
    /// \return height from ground (in m)
    ///
    double getH()   {return m_h;}

signals:
    void canvasReplot(void);

protected slots:
    void canvasReplot_slot(void);

protected:
    void paintEvent(QPaintEvent *event);
    void resizeEvent(QResizeEvent *event);

protected:
    int     m_sizeMin, m_sizeMax;               ///< widget min/max size (in pixel)
    int     m_size, m_offset;                   ///< widget size and offset size

    double  m_yaw;                              ///< yaw angle (in degree)
    double  m_alt;                              ///< altitude (in m)
    double  m_h;                                ///< height from ground (in m)
};

#include <QtCore>
#include <QtGui>
#include <QTableWidget>

#endif //QFLIGHTINSTRUMENTS_QCOMPASS_H
