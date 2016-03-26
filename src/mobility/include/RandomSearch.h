#ifndef RandomSearch_H
#define RandomSearch_H

#include "MobilityHelper.h"

// To handle shutdown signals so the node quits properly in response to "rosnode kill"
#include <ros/ros.h>
class RandomSearch : public MobilityHelper
{
public:
	RandomSearch(Rover* rover);
	void searchss(const ros::TimerEvent&);
};
#endif