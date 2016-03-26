#include <ros/ros.h>
#include "Topics.h"

SubscribeAndPublish::SubscribeAndPublish(std::string& hostname, ros::NodeHandle& nh)
{

	targetsFound_ = 0;

	targetsFoundPublish_ = nh.advertise<std_msgs::Int16>(("/targetsFound"), 1, true);
	velocityPublish_ = nh.advertise<geometry_msgs::Twist>((hostname + "/velocity"), 10);
	positionPublisher_ = nh.advertise<nav_msgs::Odometry>((hostname + "/position"), 10);
	clusterPublisher_ = nh.advertise<std_msgs::String>(("/clusterSize"), 100);


	//clusterSubscriber_ = nh.subscribe(("/clusterSize"), 100, clusterHandler);
	joySubscriber_ = nh.subscribe((hostname + "/joystick"), 100, joyCmdHandler);
	positionSubscriber_ = nh.subscribe((hostname + "/odom/ekf"), 1000, positionHandler);
	targetSubscriber = nh.subscribe((hostname + "/targets"), 100, targetHandler);
	obstacleSubscriber_ = nh.subscribe((hostname + "/obstacle"), 100, obstacleHandler);
	modeSubscriber_ = nh.subscribe((hostname + "/mode"), 100, modeHandler);
	targetsFoundSubscriber_ = nh.subscribe(("/targetsFound"), 100, targetsFoundHandler);


}


// void SubscribeAndPublish::clusterHandler(const std_msgs::String::ConstPtr& message)
// {
// 	// TODO: Fill with cluster handler callback
// }


void SubscribeAndPublish::positionHandler(const nav_msgs::Odometry::ConstPtr& message) 
{
	// Get (x, y) location from pose
	// This position should be saved in some sort of cache on the grid map.
	currentLocation.x = message;
	currentLocation.y = message;
}

void SubscribeAndPublish::obstacleHandler(const std_msgs::UInt8::ConstPtr& message) 
{
	// TODO: Fill in with obstacle avoidance calls to Rover.
}

void SubscribeAndPublish::modeHandler(const std_msgs::UInt8::ConstPtr& message) 
{
	currentMode = message->data;
}

void SubscribeAndPublish::targetsFoundHandler(const std_msgs::Int16::ConstPtr& message)
{
	++targetsFound_;
}

// Publisher functions start here.

void SubscribeAndPublish::setVelocity(double linearVel, double angularVel)
{
	// Taken from Original Swarm Code
	// These calculations get sent to the
	// arduino which sends to the motor.
	velocity.linear.x = linearVel * 1.5;
	velocity.angular.z = angularVel * 8;
	velocityPublish_.publish(velocity);
}

void SubscribeAndPublish::getPosition(const nav_msgs::Odometry::ConstPtr& message)
{
	message.linear.x = currentLocation.x;
	message.linear.y = currentLocation.y;
	positionPublisher_.publish(message)
}

void SubscribeAndPublish::targetHandler(const std_msgs::Int16::ConstPtr& message) 
{
	/*
	Get this from target.cpp

	check if tagID matches database
	if not:
		getLastLocation.x;
		getLastLocation.y;
		(tagCounter at (x,y))++;
		tagID = tagID[-1];
		tagState = state;
	else:
		continue

	*/ 
	targetDetected = *message;
	targetFoundPublish_.publish(targetDetected);

}

// void SubscribeAndPublish::clusterData(std_msgs::String::ConstPtr& message)
// {
// 	// TODO: Fill with desired data for Cluster calculation
// }