/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

class Action {
 public:
  Action();
  //return whether the action is finished
  virtual bool isFinished(){return true;}

  //called by runAction iteratively until isFinished() returns true
  virtual void update(){};

  //clean up
  virtual void done(){};

  //set up
  virtual void start(){};
};
