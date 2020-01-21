/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "lib/Geometry/Pose2D.h"
#include "lib/Util/Util.h"

#include <memory>
#include <vector>
using namespace std; 

class InterpolatingTreeMap {
 int MaximumSize;
 shared_ptr<Util> util= make_shared<Util>();
 public:
  
  
  InterpolatingTreeMap(int max_size);
  class InterpolatingStates{
    public:
    InterpolatingStates(double timestamp, shared_ptr<Pose2D> state);
    double timestamp_;
    shared_ptr<Pose2D> state_;
  };
  void put(double timestamp, shared_ptr<Pose2D> state);
  shared_ptr<Pose2D> getInterpolated(double timestamp);
  
  vector<shared_ptr<InterpolatingTreeMap::InterpolatingStates>> pastStates;

};
