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
string robotPK;
bool isIdle = true,
     hasTasks = false;
// Global var used by callback fct on auction
bool is_task_available_for_trading = false,
     is_robot_available_for_trading = true,
     external_auction_available = false;
trader::Task receivedTaskToTrade;
trader::announcement receivedTaskToSave;
trader::announcement announceTask;

// When we get a bid
vector<int> vecBid;
vector<int> vecIdRobotSrv;

// When we won an auction
ros::Publisher newTask_pub;

int indexofSmallestElement(vector<int>& vec);

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
        receivedTaskToSave = msg;
        external_auction_available = true;
    }
}

/* void taskToTrade_cb(const trader::Task &msg) */
/* { */
/*     // Do stuff */
/*     receivedTaskToTrade = msg; */
/*     is_task_available_for_trading = true; */
/* } */

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
    if (msg.idTask.compare(announceTask.idTask) == 0)
    {
        // The received bid is about the task we announced
        vecBid.push_back(msg.bid);
        vecIdRobotSrv.push_back(msg.idRobot);
    }
}


bool biddingStatus_cb(trader::auctionWinner::Request  &req,
                      trader::auctionWinner::Response &res)
{
    // Used to be told if we won or not
    if (req.isWinner)
    {
        // We send the task to the decisionNode and send back the pk
        res.pk = robotPK;
        newTask_pub.publish(receivedTaskToSave.task);
    }
    else
        res.pk = "";


    return true;
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
    /* ros::Subscriber taskToTrade_sub = n.subscribe("taskToTrade", 2, taskToTrade_cb); */
    ros::ServiceServer taskToTrade_srv = n.advertiseService("taskToTrade", taskToTrade_cb_srv);

    // Sending a new task to decisionNode
    // Use of global variable in order to use the publisher in a callback fct
    newTask_pub = n.advertise<trader::Task>("newTask", 2);

    // Metrics Service client, to get the metrics value
    ros::ServiceClient metrics_srvC;
    metrics_srvC = n.serviceClient<trader::metrics>("metricsNode");

    // Variables declaration
    int sleepTrading = 10; // sec.
    int acceptanceThreshold; // Used to know if we do the task or we trigger an auction
    n.param<int>("acceptanceThreshold", acceptanceThreshold, 2);
    n.param<string>("robotPK", robotPK, "xxx"); // TODO
    
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
            // TODO: switch to service?
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
            int indexWinner = indexofSmallestElement(vecBid);
            
            // We tell the winner his victory
            trader::auctionWinner winnerSrv;
            winnerSrv.request.isWinner = true;
            ros::ServiceClient winnerSrvClient;
            winnerSrvClient = n.serviceClient<trader::auctionWinner>
                (announceTask.idTask + "_" + to_string(vecIdRobotSrv[indexWinner]));
            winnerSrvClient.call(winnerSrv);

            // We send a signal to the blockchainNode
            // TODO

            // We tell the others that they lost
            for (int i = 0; i < vecIdRobotSrv.size() ; i++)
            {
                if (i != indexWinner)
                {
                    trader::auctionWinner looserSrv;
                    looserSrv.request.isWinner = false;
                    ros::ServiceClient looserSrvClient = n.serviceClient
                        <trader::auctionWinner>
                        (announceTask.idTask + "_" + to_string(vecIdRobotSrv[i]));
                    looserSrvClient.call(looserSrv);
                }
            }
 
            // Auction has ended, clear variables
            is_robot_available_for_trading = true;
            vecIdRobotSrv.clear();
            vecBid.clear();
        }

        // We participate in an external auction
        if (external_auction_available)
        {
            // Check the metrics of the selected task
            trader::metrics metric_srv;
            metric_srv.request.task = receivedTaskToSave.task; 
            if (metrics_srvC.call(metric_srv))
            {
                ROS_INFO("Cost of the task: %.3f", metric_srv.response.cost);
                if (metric_srv.response.cost < acceptanceThreshold)
                {
                    // We decide to bid on the task
                    ros::Publisher customTopic_bid_pub;
                    customTopic_bid_pub = n.advertise<trader::bid>(receivedTaskToSave.topic, 10);

                    // Create bid message
                    trader::bid bid_msg;
                    bid_msg.idTask = receivedTaskToSave.idTask;
                    bid_msg.idRobot = idRobot;
                    bid_msg.bid = metric_srv.response.cost;
                    // Create service server before sending bid th handle the result of the auction
                    ros::ServiceServer biddingSrvServer = n.advertiseService
                        (receivedTaskToSave.idTask + "_" + to_string(idRobot), biddingStatus_cb);

                    // Sending bid
                    customTopic_bid_pub.publish(bid_msg);

                    // Wait for answer - THIS IS DONE WITH THE CALLBACK FCT
                    // If we won, send the task to the decisionNode
                }
                else
                {
                    ROS_INFO("We do not bid on this task");
                }
            }
            else
            {
                ROS_ERROR("Failed to call service metricsNode");
            }
            // Clear Variables
            external_auction_available = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


int indexofSmallestElement(vector<int>& vec)
{
    int index = 0;

    for(int i = 1; i < vec.size(); i++)
    {
        if(vec[i] < vec[index])
            index = i;              
    }

    return index;
}
