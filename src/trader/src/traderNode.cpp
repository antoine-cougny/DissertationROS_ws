#include <string>
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
#include "trader/announcement.h"
#include "trader/bid.h"
// Custom service
#include "trader/metrics.h"
#include "trader/auctionWinner.h"
#include "trader/taskToBeTraded.h"
// Task class
#include "task.h"

using namespace std;

string ns;
int idRobot = 0;
bool isIdle = true,
     hasTasks = false;
// Global var used by callback fct on auction
bool is_task_available_for_trading = false,
     is_robot_available_for_trading = true,
     external_auction_available = false;
trader::Task receivedTaskToTrade;
trader::Task receivedTaskToSave;
trader::announcement announceTask;

// When we get a bid
int nbBid = 0;
vector<int> vecBid;
vector<int> vecIdRobotSrv;

int indexofSmallestElement(vector<int>& vec, int size);

void announcement_cb(const trader::announcement &msg)
{
    ROS_INFO("got msg");
    if (is_task_available_for_trading || !is_robot_available_for_trading)
    {
        // Ignore message because we triggered this auction
        // !!! Not safe if 2 auctions at the same time.
        // !!! TODO: verif on ID but will be implemented later by using the PK
        //           of the token on the blockchain to id it.
    }
    else
    {
        /* receivedTaskToSave = msg; */
    }
}

void taskToTrade_cb(const trader::Task &msg)
{
    // Do stuff
    receivedTaskToTrade = msg;
    is_task_available_for_trading = true;
}

bool taskToTrade_cb_srv(trader::taskToBeTraded::Request &req,
                        trader::taskToBeTraded::Response &res)
{
    receivedTaskToTrade = req.task;
    res.auctionReady.data = is_robot_available_for_trading;
    is_task_available_for_trading = is_robot_available_for_trading;

    return true;
}

void isIdle_cb(const std_msgs::Bool &msg)
{
    isIdle = msg.data;
}

void hasTasks_cb(const std_msgs::Bool &msg)
{
    hasTasks = msg.data;
}

void bidTrade_cb(const trader::bid &msg)
{
    nbBid++;
    if (msg.idTask.compare(announceTask.idTask) == 0)
    {
        // The received bid is about the task we announced
        vecBid.push_back(msg.bid);
        vecIdRobotSrv.push_back(msg.idRobot);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "traderRobot");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // For the n NodeHandle, we get param from the launch file
    n.param<int>("idRobot", idRobot, 0);

    ns = ros::this_node::getNamespace();

    // Announcement Messages
    ros::Publisher  announcement_pub;
    ros::Subscriber announcement_sub;
    announcement_pub = n.advertise<trader::announcement>("/announcementGlobal", 1);
    announcement_sub = n.subscribe("/announcementGlobal", 1, announcement_cb);

    // Getting robot state from decisionNode and taskExec
    ros::Subscriber isIdle_sub = n.subscribe("isIdle", 10, isIdle_cb);
    ros::Subscriber hasTasks_sub = n.subscribe("hasTasks", 10, hasTasks_cb);

    // Getting task from the decision node
    ros::Subscriber taskToTrade_sub = n.subscribe("taskToTrade", 2, taskToTrade_cb);
    ros::ServiceServer taskToTrade_srv = n.advertiseService("taskToTrade", taskToTrade_cb_srv);

    // Sending a new task to decisionNode
    ros::Publisher newTask_pub = n.advertise<trader::Task>("newTask", 2);

    // Variables declaration
    int sleepTrading = 10; // sec.
    
    while(ros::ok())
    {
        // TODO: divide code in functions :p
        if (is_task_available_for_trading)
        {
            // We trigger an auction 
            is_robot_available_for_trading = false;
            // Announcement message
            announceTask.idTask = to_string(ros::Time::now().sec); // TODO: use pk of blockchain
            announceTask.idRobot = idRobot;
            announceTask.task = receivedTaskToTrade;
            announceTask.topic = ns + "_" + announceTask.idTask;

            // Create topic on the fly and Publish annoucement message
            ros::Publisher customTopic_pub;
            customTopic_pub = n.advertise<trader::bid>(announceTask.topic, 10);
            ros::Subscriber customTopic_sub;
            customTopic_sub = n.subscribe(announceTask.topic, 10, bidTrade_cb);
            announcement_pub.publish(announceTask);

            // We wait 10 sec
            // TODO: find a better implementation
            ros::Duration(sleepTrading).sleep();

            // We will select the winner and a service message to him
            // The others will get a no
            // TODO: what is the smallest cost is bigger than our cost?
            int indexWinner = indexofSmallestElement(vecBid, nbBid);
            
            // We tell the winner that his victory
            trader::auctionWinner winnerSrv;
            winnerSrv.request.isWinner = true;
            ros::ServiceClient winnerSrvClient;
            winnerSrvClient = n.serviceClient<trader::auctionWinner>
                (announceTask.idTask + "_" + to_string(vecIdRobotSrv[indexWinner]));
            winnerSrvClient.call(winnerSrv);

            // We send a signal to the blockchainNode
            // TODO

            // We tell the others that they lost
            for (int i = 0; i < nbBid ; i++)
            {
                if (i != indexWinner)
                {
                    trader::auctionWinner looserSrv;
                    looserSrv.request.isWinner = false;
                    ros::ServiceClient looserSrvClient;
                    looserSrvClient = n.serviceClient<trader::auctionWinner>
                        (announceTask.idTask + "_" + to_string(vecIdRobotSrv[i]));
                    looserSrvClient.call(looserSrv);
                }
            }



        }
        if (external_auction_available)
        {
            // We participate in an external auction
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


int indexofSmallestElement(vector<int>& vec, int size)
{
    int index = 0;

    for(int i = 1; i < size; i++)
    {
        if(vec[i] < vec[index])
            index = i;              
    }

    return index;
}
