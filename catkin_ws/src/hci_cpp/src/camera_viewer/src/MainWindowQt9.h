/*
 * MainWindowQt9.h
 *
 *  Created on: 2016-02-11
 *      Author: cf0548
 */

#ifndef MAINWINDOWQT9_H_
#define MAINWINDOWQT9_H_

#include <QPushButton>
#include <QMainWindow>
#include <QStackedLayout>

#include "rimstreamer/GstVideoFeed.h"
#include "rimstreamer/VideoFeedWidget.h"

namespace rimstreamer
{

class MainWindowQt9: public QMainWindow
{
Q_OBJECT

public:
    MainWindowQt9(const QString& feed1Name, GstVideoFeedPtr feed1,
            const QString& feed2Name, GstVideoFeedPtr feed2);
    virtual ~MainWindowQt9();

protected:
    void closeEvent(QCloseEvent* event) Q_DECL_OVERRIDE;
    void showEvent(QShowEvent* event) Q_DECL_OVERRIDE;

private slots:
    void onDualViewButtonClicked(QAbstractButton* button);
    void onSingleViewButtonClicked(QAbstractButton* button);
    void onSelectViewButtonClicked(QAbstractButton* button);
    void onQuit();

private:
    GstVideoFeedPtr mVideoFeed1;
    GstVideoFeedPtr mVideoFeed2;

    VideoFeedWidget* mSingleVidWidget;
    QPushButton* mSingleButton1;
    QPushButton* mSingleButton2;

    VideoFeedWidget* mLeftVidWidget;
    VideoFeedWidget* mRightVidWidget;
    QPushButton* mDualButton1;
    QPushButton* mDualButton2;

    QPushButton* mSingleViewButton;
    QPushButton* mDualViewButton;

    QStackedLayout* mViewsLayout;
    int mSingleViewIndex;
    int mDualViewIndex;

    QWidget* createDualViewPanel(const QString& feed1Name,
            const QString& feed2Name);
    QWidget* createSingleViewPanel(const QString& feed1Name,
            const QString& feed2Name);
    void selectSingleView();
    void unselectSingleView();
    void selectDualView();
    void unselectDualView();

    bool canQuit();
};

} /* namespace rimstreamer */
#endif /* MAINWINDOWQT9_H_ */
