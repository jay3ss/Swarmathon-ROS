#ifndef TOPICS_H
#define TOPICS_H

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>

#include <ros/ros.h>

typedef nav_msgs::Odometry ConstPtr;
typedef geometry_msgs::Twist ConstPtr;

static class SubscribeAndPublish
{
public:
	// NodeHandler, Pub's, and Sub's
	ros::NodeHandle messageHandler_;

	// Pubs
	ros::Publisher targetsFoundPublish_;
	ros::Publisher velocityPublish_;
	ros::Publisher positionPublisher_;

	// Subs
	ros::Subscriber joySubscriber_;
    ros::Subscriber positionSubscriber_;
    ros::Subscriber obstacleSubscriber_;
    ros::Subscriber modeSubscriber_;
    ros::Subscriber targetsFoundSubscriber_;

    // Variables
    int targetsFound_;
    geometry_msgs::Pose2D currentLocation_;
    string publishedName_;
    std_msgs::String clusterData_;


    // Constructor
	SubscribeAndPublish();
	void initialize();

	// Publisher Functions
	void setVelocity(double linearVel, double angularVel);
	void getPosition(const nav_msgs::Odometry::ConstPtr& message);

	// Callback Handlers
	void positionHandler(const nav_msgs::Odometry::ConstPtr& message);
	void joyCmdHandler(const geometry_msgs::Twist::ConstPtr& message);
	void targetHandler(const std_msgs::Int16::ConstPtr& tagInfo);
	void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
	void modeHandler(const std_msgs::UInt8::ConstPtr& message);
	void targetsFoundHandler(const std_msgs::Int16::ConstPtr& message);

private:
	continue;

};

#endif