/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Action.h"
#include "lib/Geometry/Translation2D.h"
#include "Robot.h"

#include <memory>
using namespace std;



class WaitUntilInsideRegion : public Action {
  shared_ptr<Translation2D> mBottomLeft;
  shared_ptr<Translation2D> mTopRight;
 public:
  WaitUntilInsideRegion(shared_ptr<Translation2D> bottomLeft, shared_ptr<Translation2D> topRight, bool isOnLeft);
  bool isFinished();
  void start(){}
  void update(){}
  void done(){}
};
  