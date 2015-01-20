/*
 *  pal_joystick.h
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 2015
 *      Author:  victor
 */

#ifndef PALJOYSTICK_H
#define PALJOYSTICK_H

#include <QLabel>
#include <QTouchEvent>
#include <QTimer>
namespace pal
{
class PalJoystick : public QLabel
{
    Q_OBJECT

public:
    explicit PalJoystick(QWidget *parent = 0, int updatePeriod = 100);
    ~PalJoystick();

protected:
    void mouseMoveEvent(QMouseEvent *);
    void mouseReleaseEvent(QMouseEvent *);
    void updateJoystick(const QPointF &point);
    virtual void resizeEvent(QResizeEvent *);
    virtual bool event(QEvent *e);
    bool touchEvent(QTouchEvent *e);
Q_SIGNALS:
    void posChanged(qreal X, qreal Y);

public Q_SLOTS:
    void reportStickPosition();
    void lock(bool locked);
    //return true;
private:
    // x and y are a percentile of how far from the center the x and y are located
    void updateStickPosition(qreal x,qreal y);

    qreal _lastX;
    qreal _lastY;
    QTimer _timer;
    QLabel *_stick;
    int _stickX;
    int _stickY;
};
}
#endif // PALJOYSTICK_H
