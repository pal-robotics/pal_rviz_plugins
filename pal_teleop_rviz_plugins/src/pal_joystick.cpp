#include <pal_teleop_rviz_plugins/pal_joystick.h>
#include <math.h>
#include <iostream>
namespace pal
{
PalJoystick::PalJoystick(QWidget *parent, int updatePeriod) :
    QLabel(parent),
    _timer()
{

    //mouseMoveEvents will only be triggered when a mouse button is pressed
    setMouseTracking(false);

    setAttribute(Qt::WA_AcceptTouchEvents,true);
    setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);

    _stick = new QLabel(this);
    //_stick->setFixedSize(width()/2, height()/2);

    setStyleSheet("border-image: url(:/joy/joyBG);"); // circle around joystick
    _stick->setStyleSheet("border-image: url(:/joy/joyCenter);");

    updateStickPosition(0,0);

    connect(&_timer,SIGNAL(timeout()),this,SLOT(reportStickPosition()));
    _timer.start(updatePeriod);
}

PalJoystick::~PalJoystick()
{
}

void PalJoystick::lock(bool locked) {
    updateStickPosition(0,0);
}

void PalJoystick::resizeEvent(QResizeEvent *)
{
    _stick->setFixedSize(width()/4, height()/4);
    updateStickPosition(_lastX, _lastY);

}

void PalJoystick::mouseMoveEvent(QMouseEvent *ev)
{
    //Mouse position can be out of the bounds of the widget, in this case we enforce it to be within the bounds

    updateJoystick(ev->posF());

}

void PalJoystick::updateJoystick(const QPointF &point)
{
    int maxWindowHeight = height()/2;
    int maxWindowWidth = width()/2;
    int mouseX = point.x() - maxWindowWidth;
    //Y coordinates are inverted, the top is -MAX_HEIGHT; and bottom is positive
    int mouseY = -point.y() + maxWindowHeight;

    /// Max allowed distance from the center of the widget
    /// in a circular joystick (the max radius)
    int maxDistance = std::min(maxWindowHeight, maxWindowWidth) - _stick->width()/2;
    if (sqrt(mouseX*mouseX + mouseY*mouseY) >= maxDistance)
    {

        if (mouseY == 0)
            mouseX = (mouseX > 0 ? maxDistance : -maxDistance);
        else if (mouseX == 0)
            mouseY = (mouseY > 0 ? maxDistance : -maxDistance);
        else
        {
            double k = fabs(double(mouseX)/double(mouseY));
            double newX, newY;
            double yDist = sqrt((maxDistance*maxDistance)/(1+k*k));
            if (mouseY > 0)
                mouseY = yDist;
            else
                mouseY = -yDist;

            if (mouseX > 0)
                mouseX = yDist * k;
            else
                mouseX = -yDist * k;

        }
    }

    updateStickPosition(mouseX, mouseY);
}

void PalJoystick::updateStickPosition(qreal x, qreal y)
{
    _lastX = x;
    _lastY = y;

    _stickX = x + width()/2 - _stick->width()/2;
    _stickY = -y + height()/2 - _stick->height()/2;

    _stick->move(_stickX, _stickY);
}

void PalJoystick::reportStickPosition()
{
    Q_EMIT posChanged(_lastX,_lastY);
}

void PalJoystick::mouseReleaseEvent(QMouseEvent *)
{
    updateStickPosition(0,0);
}


bool PalJoystick::event(QEvent *event)
{

    //qDebug() << "Got event: " << event->type();
    if ((event->type() == QEvent::TouchBegin) || (event->type() == QEvent::TouchUpdate) || (event->type() == QEvent::TouchEnd))
        return touchEvent(static_cast<QTouchEvent*>(event));
    return QLabel::event(event);
}

bool PalJoystick::touchEvent(QTouchEvent *event)
{
    //return true;
    event->accept();

    if (event->type() == QEvent::TouchEnd)
    {
        //qDebug() << "Got touch end in " << this << " occured in " << event->widget();

        QList<QTouchEvent::TouchPoint> pointList = event->touchPoints();
        if (pointList.size() > 1)
            qDebug("GETTING MORE THAN 1 POINT IN END TOUCH EVENT");
        updateStickPosition(0,0);
    }
    else
    {
        QList<QTouchEvent::TouchPoint>::iterator it;
        QList<QTouchEvent::TouchPoint> pointList = event->touchPoints();
        if (pointList.size() > 1)
            qDebug("GETTING MORE THAN 1 POINT IN TOUCH EVENT");
        for (it = pointList.begin(); it != pointList.end(); ++it)
        {
            QTouchEvent::TouchPoint p;

            //qDebug() << it->pos().x() << " WW " << it->pos().x();
            updateJoystick(it->pos());
        }
    }
    return true;
}

}
