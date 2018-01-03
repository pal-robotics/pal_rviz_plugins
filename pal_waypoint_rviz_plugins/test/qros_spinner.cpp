#include "qros_spinner.h"

QRosSpinner::QRosSpinner()
    : QObject()
{
    tim.setInterval(100);
    connect(&tim, SIGNAL(timeout()), this, SLOT(doSpin()));
    tim.start();
}

QRosSpinner::~QRosSpinner()
{

}

void QRosSpinner::doSpin()
{
    ros::spinOnce();
}
