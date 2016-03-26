//  Uniform

#include "Uniform.h"
#include <geometry_msgs/Pose2D.h>
#include <math.h>

//Class
Uniform::Uniform()
{
  position_.x = 0;
  position_.y = 0;
  position_.theta = 0;

  tiles_traveled_ = 0;
  tiles_to_travel_ = 1;

  n_switch_ = true; // when true n+2; for 3 rovers
  start_ = true;

  orientation_ = Uniform::NORTH;
  position_.theta = M_PI_2;
}

Uniform::~Uniform(){}

void Uniform::setOrientation(Orientation orientation)
{
    orientation_ = orientation;
    if(orientation_ == NORTH)
    {
        position_.theta = M_PI_2;
    }
    else if(orientation_ == WEST)
    {
        orientation_ = SOUTH;
      position_.theta = (M_PI);
    }
    else if(orientation_ == SOUTH)
    {
        position_.theta = M_PI_2  * 3.0;
    }
    else if(orientation_ == EAST)
    {
        orientation_ = NORTH;
       position_.theta = 0.0;
    }

    else if(orientation_ == NORTHEAST)
    {
        position_.theta = M_PI_2 / 2.0;
    }
    else if(orientation_ == NORTHWEST)
    {
        position_.theta = M_PI_2 *  3.0 / 2.0;
    }
    else if(orientation_ == SOUTHWEST)
    {
        position_.theta = M_PI_2 *  5.0 / 2.0;
    }
    else if(orientation_ == SOUTHEAST)
    {
        position_.theta = M_PI_2 *  7.0 / 2.0;
    }
}


// 3 Rovers
geometry_msgs::Pose2D Uniform::getNextPositionThreeRovers()
{
    if(start_){
    // rover A
        if (orientation_ == NORTHEAST)
        {
            move();

      orientation_ = WEST;

            tiles_traveled_ = 0;
            tiles_to_travel_ = 2;

      start_ = false;
            n_switch_ = false;

          return position_;
        }

    // rover B
        else if (orientation_ == EAST)
        {
            move();
            ++tiles_traveled_;
      if (tiles_traveled_ == 2)
      {
        rotate();
          start_ = false;
        tiles_traveled_ = 0;
                tiles_to_travel_ = 2;
      }
          return position_;
        }

    // rover C
        // if (orientation_ == SOUTH)
        else
        {
            move();
      rotate();

      tiles_traveled_ = 0;
      tiles_to_travel_ = 3;

      start_ = false;
            n_switch_ = false;
          return position_;
        }

    }

  // END of start
  move();
  ++tiles_traveled_;

  if(tiles_traveled_ == tiles_to_travel_)
  {
    tiles_traveled_ = 0;
      if(n_switch_)
    {
      tiles_to_travel_ += 2;
    }
    else
    {
      ++tiles_to_travel_;
    }
    n_switch_ = !n_switch_;
    rotate();
  }
  return position_;
}

// 6 Rovers
geometry_msgs::Pose2D Uniform::getNextPositionSixRovers()
{
    if(start_){
    // rover A
        if (orientation_ == WEST)
        {
          move();
      rotate();

            tiles_traveled_ = 0;
            tiles_to_travel_ = 3;

      start_ = false;

          return position_;
        }

    // rover B
        else if (orientation_ == NORTH)
        {
            move();
      rotate();

      tiles_traveled_ = 0;
      tiles_to_travel_ = 2;

      start_ = false;

          return position_;
        }

    // rover C
        else if (orientation_ == NORTHEAST)
        {
            move();
      rotate();

      tiles_traveled_ = 0;
      tiles_to_travel_ = 1;

      start_ = false;
          return position_;
        }

    // rover D
        else if (orientation_ == EAST)
        {
            move();

      ++tiles_traveled_;
      if (tiles_traveled_ == 2)
      {
        rotate();
          start_ = false;
        tiles_traveled_ = 0;
        tiles_to_travel_ = 3;
      }

          return position_;
        }

    // rover E
    else if (orientation_ == SOUTHEAST)
        {
            move();
      rotate();

      tiles_traveled_ = 0;
      tiles_to_travel_ = 2;

      start_ = false;
          return position_;
        }

    // rover F
    else if (orientation_ == SOUTH)
        {
            move();

      ++tiles_traveled_;
      if (tiles_traveled_ == 2)
      {
        rotate();
          start_ = false;
        tiles_traveled_ = 0;
                tiles_to_travel_ = 4;
      }

      return position_;
        }

    }

  // END of start
  move();
  ++tiles_traveled_;

  if(tiles_traveled_ == tiles_to_travel_)
  {
    rotate();

    tiles_traveled_ = 0;
    tiles_to_travel_ += 3;
  }
  return position_;
}

geometry_msgs::Pose2D Uniform::getCurrentPosition()
{
  return position_;
}


void Uniform::rotate()
{
  if(orientation_ == NORTH)
  {
    orientation_ = WEST;
    position_.theta = M_PI;
  }
  else if(orientation_ == WEST)
  {
      orientation_ = SOUTH;
    position_.theta = (M_PI_2 * 3);
  }
  else if(orientation_ == SOUTH)
  {
      orientation_ = EAST;
        position_.theta = 0;
  }
  else if(orientation_ == EAST)
  {
      orientation_ = NORTH;
     position_.theta = M_PI_2;
  }

    // Only for 6 rovers
  else if(orientation_ == NORTHEAST)
  {
      orientation_ = NORTH;
      position_.theta = M_PI_2;
  }
  else if(orientation_ == NORTHWEST)
  {
      orientation_ = WEST;
    position_.theta = M_PI;
  }

  else if(orientation_ == SOUTHWEST)
  {
      orientation_ = SOUTH;
      position_.theta = M_PI_2  * 3;
  }
  else if(orientation_ == SOUTHEAST)
  {
      orientation_ = EAST;
        position_.theta = 0.0;
  }
}

void Uniform::move()
{
  if(orientation_ == NORTH)
  {
    position_.y += .1;
  }
  else if(orientation_ == WEST)
  {
    position_.x -= .1;
  }
  else if(orientation_ == SOUTH)
  {
    position_.y -= .1;
  }
  else if(orientation_ == EAST)
  {
    position_.x += .1;
  }

    else if(orientation_ == NORTHEAST)
  {
    position_.x += .1;
    position_.y += .1;
  }
  else if(orientation_ == NORTHWEST)
  {
    position_.x -= .1;
    position_.y += .1;
  }
  else if(orientation_ == SOUTHWEST)
  {
    position_.x -= .1;
    position_.y -= .1;
  }
  else if(orientation_ == SOUTHEAST)
  {
    position_.x += .1;
    position_.y -= .1;
  }
}
