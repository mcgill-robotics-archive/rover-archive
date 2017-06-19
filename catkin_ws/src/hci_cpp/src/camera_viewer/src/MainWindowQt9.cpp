/*
 * MainWindowQt9.cpp
 *
 *  Created on: 2016-02-11
 *      Author: cf0548
 */

#include "MainWindowQt9.h"

#include <QApplication>
#include <QButtonGroup>
#include <QCloseEvent>
#include <qFlightInstruments/QAttitude.h>

namespace rimstreamer
{

MainWindowQt9::MainWindowQt9(const QString& feed1Name, GstVideoFeedPtr feed1,
        const QString& feed2Name, GstVideoFeedPtr feed2)
{
    setWindowTitle("Demo 9");
    QADI* adi = new QADI(this);

    mVideoFeed1 = feed1;
    mVideoFeed2 = feed2;

    QWidget* singlePanel = createSingleViewPanel(feed1Name, feed2Name);

    QWidget* dualPanel = createDualViewPanel(feed1Name, feed2Name);

    mSingleViewButton = new QPushButton("Single View");
    mSingleViewButton->setCheckable(true);
    mSingleViewButton->setChecked(true);
    mDualViewButton = new QPushButton("Dual View");
    mDualViewButton->setCheckable(true);

    QWidget* viewButtonPanel = new QWidget();
    QGridLayout* viewButtonLayout = new QGridLayout(viewButtonPanel);
    viewButtonLayout->addWidget(mSingleViewButton);
    viewButtonLayout->addWidget(mDualViewButton);
    viewButtonLayout->addWidget(adi);

    QButtonGroup* viewButtonGroup = new QButtonGroup(viewButtonPanel);
    viewButtonGroup->addButton(mSingleViewButton);
    viewButtonGroup->addButton(mDualViewButton);

    connect(viewButtonGroup, SIGNAL(buttonClicked(QAbstractButton*)), this,
            SLOT(onSelectViewButtonClicked(QAbstractButton*)));

    QWidget* viewsPanel = new QWidget();
    mViewsLayout = new QStackedLayout(viewsPanel);
    mSingleViewIndex = mViewsLayout->addWidget(singlePanel);
    mDualViewIndex = mViewsLayout->addWidget(dualPanel);
    mViewsLayout->setCurrentIndex(mSingleViewIndex);

    QWidget* mainPanel = new QWidget();
    QVBoxLayout* mainLayout = new QVBoxLayout(mainPanel);
    mainLayout->addWidget(viewsPanel);
    mainLayout->addWidget(viewButtonPanel);

    setCentralWidget(mainPanel);
}

bool MainWindowQt9::canQuit()
{
//    QMessageBox msgBox;
//    msgBox.setText("Do you really want to quit?");
//    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
//    msgBox.setDefaultButton(QMessageBox::No);
//
//    int answer = msgBox.exec();
//    return (answer == QMessageBox::Yes) ? true : false;

    return true;
}

void MainWindowQt9::closeEvent(QCloseEvent* event)
{
    if (canQuit())
        event->accept();
    else
        event->ignore();
}

QWidget* MainWindowQt9::createDualViewPanel(const QString& feed1Name,
        const QString& feed2Name)
{
    mLeftVidWidget = new VideoFeedWidget();
    mLeftVidWidget->setScale(0.5, 1.0);
    mLeftVidWidget->setFixedSize(270, 360);

    mRightVidWidget = new VideoFeedWidget();
    mRightVidWidget->setScale(0.5, 1.0);
    mRightVidWidget->setFixedSize(270, 360);

    mDualButton1 = new QPushButton(feed1Name);
    mDualButton1->setCheckable(true);

    mDualButton2 = new QPushButton(feed2Name);
    mDualButton2->setCheckable(true);

    QWidget* viewPanel = new QWidget();
    QHBoxLayout* viewLayout = new QHBoxLayout(viewPanel);
    viewLayout->addStretch();
    viewLayout->addWidget(mLeftVidWidget);
    viewLayout->addWidget(mRightVidWidget);
    viewLayout->addStretch();

    QWidget* buttonPanel = new QWidget();
    QGridLayout* buttonLayout = new QGridLayout(buttonPanel);
    buttonLayout->addWidget(mDualButton1, 0, 0);
    buttonLayout->addWidget(mDualButton2, 0, 1);

    QButtonGroup* buttonGroup = new QButtonGroup(buttonPanel);
    buttonGroup->addButton(mDualButton1);
    buttonGroup->addButton(mDualButton2);

    connect(buttonGroup, SIGNAL(buttonClicked(QAbstractButton*)), this,
            SLOT(onDualViewButtonClicked(QAbstractButton*)));

    QWidget* panel = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->addWidget(viewPanel);
    layout->addWidget(buttonPanel);

    return panel;
}

QWidget* MainWindowQt9::createSingleViewPanel(const QString& feed1Name,
        const QString& feed2Name)
{
    mSingleVidWidget = new VideoFeedWidget();
    mSingleVidWidget->setFixedSize(540, 360);

    mSingleButton1 = new QPushButton(feed1Name);
    mSingleButton1->setCheckable(true);

    mSingleButton2 = new QPushButton(feed2Name);
    mSingleButton2->setCheckable(true);

    QWidget* viewPanel = new QWidget();
    QHBoxLayout* viewLayout = new QHBoxLayout(viewPanel);
    viewLayout->addStretch();
    viewLayout->addWidget(mSingleVidWidget);
    viewLayout->addStretch();

    QWidget* buttonPanel = new QWidget();
    QGridLayout* buttonLayout = new QGridLayout(buttonPanel);
    buttonLayout->addWidget(mSingleButton1, 0, 0);
    buttonLayout->addWidget(mSingleButton2, 0, 1);

    QButtonGroup* buttonGroup = new QButtonGroup(buttonPanel);
    buttonGroup->addButton(mSingleButton1);
    buttonGroup->addButton(mSingleButton2);

    connect(buttonGroup, SIGNAL(buttonClicked(QAbstractButton*)), this,
            SLOT(onSingleViewButtonClicked(QAbstractButton*)));

    QWidget* panel = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->addWidget(viewPanel);
    layout->addWidget(buttonPanel);

    return panel;
}

void MainWindowQt9::onDualViewButtonClicked(QAbstractButton* button)
{
    qDebug() << "onDualViewButtonClicked:" << button->text()
            << "button pressed.";

    if (button == mDualButton1)
    {
        //If the button pressed is the one of the current feed, do nothing.
        VideoFeedPtr feed = mLeftVidWidget->getVideoFeed();
        if (feed && feed == mVideoFeed1)
            return;

        //If there are feeds running, stop them.
        if (feed)
        {
            mVideoFeed1->stop();
            mVideoFeed2->stop();
            mLeftVidWidget->releaseVideoFeed();
            mRightVidWidget->releaseVideoFeed();
        }

        //Play the feeds
        mLeftVidWidget->setVideoFeed(mVideoFeed1);
        mRightVidWidget->setVideoFeed(mVideoFeed2);
        mVideoFeed1->play();
        mVideoFeed2->play();
    }

    if (button == mDualButton2)
    {
        //If the button pressed is the one of the current feed, do nothing.
        VideoFeedPtr feed = mLeftVidWidget->getVideoFeed();
        if (feed && feed == mVideoFeed2)
            return;

        //If there are feeds running, stop them.
        if (feed)
        {
            mVideoFeed1->stop();
            mVideoFeed2->stop();
            mLeftVidWidget->releaseVideoFeed();
            mRightVidWidget->releaseVideoFeed();
        }

        //Play the feeds
        mLeftVidWidget->setVideoFeed(mVideoFeed2);
        mRightVidWidget->setVideoFeed(mVideoFeed1);
        mVideoFeed1->play();
        mVideoFeed2->play();
    }
}

void MainWindowQt9::onSingleViewButtonClicked(QAbstractButton* button)
{
    qDebug() << "onSingleViewButtonClicked:" << button->text()
            << "button pressed.";

    if (button == mSingleButton1)
    {
        //If the button pressed is the one of the current feed, do nothing.
        VideoFeedPtr feed = mSingleVidWidget->getVideoFeed();
        if (feed && feed == mVideoFeed1)
            return;

        //If there is a feed running, stop it.
        if (feed)
        {
            feed->stop();
            mSingleVidWidget->releaseVideoFeed();
        }

        //Play the feed
        mSingleVidWidget->setVideoFeed(mVideoFeed1);
        mVideoFeed1->play();
    }

    if (button == mSingleButton2)
    {
        //If the button pressed is the one of the current feed, do nothing.
        VideoFeedPtr feed = mSingleVidWidget->getVideoFeed();
        if (feed == mVideoFeed2)
            return;

        //If there is a feed running, stop it.
        if (feed)
        {
            feed->stop();
            mSingleVidWidget->releaseVideoFeed();
        }

        //Play the feed
        mSingleVidWidget->setVideoFeed(mVideoFeed2);
        mVideoFeed2->play();
    }
}

void MainWindowQt9::onSelectViewButtonClicked(QAbstractButton* button)
{
    qDebug() << button->text() << "button pressed.";

    if (button == mSingleViewButton)
    {
        if (mViewsLayout->currentIndex() == mSingleViewIndex)
        {
            qDebug() << "Already in single view";
            return;
        }

        unselectDualView();
        mViewsLayout->setCurrentIndex(mSingleViewIndex);
        selectSingleView();
    }
    else if (button == mDualViewButton)
    {
        if (mViewsLayout->currentIndex() == mDualViewIndex)
        {
            qDebug() << "Already in dual view";
            return;
        }

        unselectSingleView();
        mViewsLayout->setCurrentIndex(mDualViewIndex);
        selectDualView();
    }
    else
    {
        qCritical() << "Invalid button:" << button->text();
    }
}

void MainWindowQt9::onQuit()
{
    if (canQuit())
        QApplication::quit();
}

void MainWindowQt9::showEvent(QShowEvent* event)
{
    //Do nothing.
}

void MainWindowQt9::selectDualView()
{
    if (mDualButton1->isChecked())
    {
        mLeftVidWidget->setVideoFeed(mVideoFeed1);
        mRightVidWidget->setVideoFeed(mVideoFeed2);
        mVideoFeed1->play();
        mVideoFeed2->play();
        return;
    }

    if (mDualButton2->isChecked())
    {
        mLeftVidWidget->setVideoFeed(mVideoFeed2);
        mRightVidWidget->setVideoFeed(mVideoFeed1);
        mVideoFeed1->play();
        mVideoFeed2->play();
        return;
    }
}

void MainWindowQt9::selectSingleView()
{
    if (mSingleButton1->isChecked())
    {
        mSingleVidWidget->setVideoFeed(mVideoFeed1);
        mVideoFeed1->play();
        return;
    }

    if (mSingleButton2->isChecked())
    {
        mSingleVidWidget->setVideoFeed(mVideoFeed2);
        mVideoFeed2->play();
        return;
    }
}

void MainWindowQt9::unselectDualView()
{
    VideoFeedPtr feed = mLeftVidWidget->getVideoFeed();
    if (feed)
    {
        feed->stop();
        mLeftVidWidget->releaseVideoFeed();
    }

    feed = mRightVidWidget->getVideoFeed();
    if (feed)
    {
        feed->stop();
        mRightVidWidget->releaseVideoFeed();
    }
}

void MainWindowQt9::unselectSingleView()
{
    VideoFeedPtr feed = mSingleVidWidget->getVideoFeed();
    if (feed)
    {
        feed->stop();
        mSingleVidWidget->releaseVideoFeed();
    }
}

} /* namespace rimstreamer */
