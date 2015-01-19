
#include <pal_waypoint_rviz_plugins/waypoint_group_panel.h>
#include <qapplication.h>
// Include header files for application components.
// ...

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "waypoint_group_panel_test");
    ros::console::initialize();
    ROS_INFO_STREAM("Online");
    QApplication app(argc, argv);
//    QWidget window;
//    window.resize(320, 240);

    pal::WaypointGroupPanel panel;
    panel.show();
    return app.exec();
}
