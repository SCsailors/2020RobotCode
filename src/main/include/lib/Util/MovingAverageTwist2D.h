/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <vector>

#include <lib/Geometry/Twist2D.h>
namespace Utility{

class MovingAverageTwist2D {
  std::vector<std::shared_ptr<Twist2D>> twists;
  int maxSize;
 public:
  MovingAverageTwist2D(int maxSize);

  void add(std::shared_ptr<Twist2D> twist);

  std::shared_ptr<Twist2D> getAverage();

  int getSize();

  bool isUnderMaxSize();

  void clear();
};
}