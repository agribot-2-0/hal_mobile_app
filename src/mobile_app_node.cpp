#include <ros/ros.h>
#include <ros/console.h>
#include <hal_mobile_app/MobileAppHandler.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mobile_app_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    return 0;
}
