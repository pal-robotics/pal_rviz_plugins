#include <QTimer>
#include <QHBoxLayout>
#include <pal_teleop_rviz_plugins/pal_joystick.h>
#include <pal_teleop_rviz_plugins/pal_teleop_panel.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include "ui_pal_teleop_panel.h"
namespace pal
{

class SquareLayout : public QHBoxLayout
{
public:
    SquareLayout(QWidget *parent)
        : QHBoxLayout(parent)
    {

    }

    virtual ~SquareLayout()
    {

    }

    virtual bool hasHeightForWidth() const
    {
        return true;
    }
    virtual int heightForWidth(int w) const
    {
        return w;
    }
};

PalTeleopPanel::PalTeleopPanel(QWidget *parent)
    : rviz::Panel(parent),
    ui(new Ui::PalTeleopPanel)
{
    ui->setupUi(this);

//    SquareLayout *layout = new SquareLayout(this);
//    setLayout(layout);
//    PalJoystick *joy = new PalJoystick;
//    layout->addWidget(joy);
//    joy->setSizePolicy(QSizePolicy::MinimumExpanding,
//                       QSizePolicy::MinimumExpanding);
//    QSizePolicy p(sizePolicy());
//    p.setHeightForWidth(true);
//    setSizePolicy(p);
    connect(ui->joystick, SIGNAL(posChanged(qreal,qreal)), this, SLOT(joystickPos(qreal,qreal)));
    _pub = _nh.advertise<geometry_msgs::Twist>("/twist_mux/cmd_vel", 1);

}

PalTeleopPanel::~PalTeleopPanel()
{

}

int PalTeleopPanel::heightForWidth(int w)
{
    return w;
}

qreal calculateExponentialSpeed(qreal x)
{
    //Old speed
    //See the plot of the speed here: http://www.wolframalpha.com/input/?i=e+^%28%28x^2%29*ln+2%29+-1%2C+from+x%3D-1+to+1
  //qreal speed = pow(M_E,((pow(x,2)*(log(2))))) -1;

  // Speed is x^2
  qreal speed = pow(x, 2);

  if (x >= 0)
    return speed;
  else
    return -speed;
  //e ^((x^2)*ln 2) -1

}

void PalTeleopPanel::joystickPos(qreal x, qreal y)
{
    static const qreal MAX_LINEAR_SPEED = 0.6;
    static const qreal MAX_ANGULAR_SPEED = 0.6;

    qreal linearX, angularZ;
    linearX = MAX_LINEAR_SPEED*calculateExponentialSpeed(y/100.0);
    angularZ = - MAX_ANGULAR_SPEED*x/100.0;

    geometry_msgs::Twist twist;
    twist.linear.x = linearX;
    twist.angular.z = angularZ;

    ui->angSpeed->setValue(angularZ);
    ui->linSpeed->setValue(linearX);
    _pub.publish(twist);

}

} // end namespace pal

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pal::PalTeleopPanel, rviz::Panel )
