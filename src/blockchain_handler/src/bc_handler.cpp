#include <string>
#include <iostream>
#include <vector>
// Ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
// Custom messages
#include "trader/Task.h"
#include "blockchain_handler/transactionBC.h"

using namespace std;

string ns;
int idRobot = 1000;
string robotPK;

bool transactionAvailable = false;
blockchain_handler::transactionBC transaction_msg;

bool sendTransaction_bc(blockchain_handler::transactionBC::Request  &req,
                        blockchain_handler::transactionBC::Response &res)
{
    res.status = false;
    if (!transactionAvailable)
    {
        res.status = true;
        transactionAvailable = true;
        transaction_msg.request = req;
    }
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "blockchainHandler");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // For the n NodeHandle, we get param from the launch file
    n.param<int>("idRobot", idRobot, 0);

    ns = ros::this_node::getNamespace();
    ns = ns.substr(1, ns.size() - 1); // Delete an extra / at the beginning

    // Get transaction from traderNode
    ros::ServiceServer transactionBC_srvS;
    transactionBC_srvS = n.advertiseService("transactionBC", sendTransaction_bc);

    // Send task to JS node / eth client
    ros::ServiceClient deployTransaction_srvC;
    deployTransaction_srvC = n.serviceClient
            <blockchain_handler::transactionBC>("/deployTransactionBC");

    n.param<int>("idRobot", idRobot, 0);
    n.param<string>("robotPK", robotPK, "xxx"); // TODO

    while(ros::ok())
    {
        if (transactionAvailable)
        {
            ROS_INFO("A transaction is available to be sent to the BC");
            if (deployTransaction_srvC.call(transaction_msg))
            {
                ROS_INFO("Contacted JS node");
                if (transaction_msg.response.status)
                {
                    ROS_INFO("Transaction will be processed by the node");
                    transactionAvailable = false;
                }
                else
                {
                    ROS_INFO("The node is busy, we will wait for 1 sec");
                    ros::Duration(1).sleep();
                }
            }
            else
                ROS_ERROR("Could not connect JS node");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
