#include <string>
// Ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
// Custom messages
#include "trader/Task.h"
#include "trader/metrics.h"
#include "trader/announcement.h"
// Task class
#include "task.h"

using namespace std;

int idRobot = 0;
string ns;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traderRobot");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // For the n NodeHandle, we get param from the launch file
    n.param<int>("idRobot", idRobot, 0);

    while (ros::ok())
    {
        loop_rate.sleep();
    }

    return 0;
}
