#include <iostream>
#include <math.h>
#include <geometry_msgs/Pose2D.h>

#ifndef Uniform_H_
#define Uniform_H_

class Uniform
{

public:
  enum Orientation {NORTH, WEST, SOUTH, EAST, NORTHWEST, NORTHEAST, SOUTHWEST, SOUTHEAST};
  Uniform();
  ~Uniform();
  void setOrientation(Orientation orientation);
  geometry_msgs::Pose2D getNextPositionThreeRovers();// Return the next position
  geometry_msgs::Pose2D getNextPositionSixRovers();// Return the next position
  geometry_msgs::Pose2D getCurrentPosition();// Return the position
  Orientation getOrientation();

private:
  Orientation orientation_;

  geometry_msgs::Pose2D position_; //set equal to (0,0)

  int tiles_traveled_;
  int tiles_to_travel_;
  bool n_switch_; // when true n+2; for 3 rovers
  bool start_;

  void move();
  void rotate();

};
#endif
