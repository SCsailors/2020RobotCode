/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

namespace Subsystems{
class Subsystem {
 public:
  Subsystem();
  virtual void writeToLog(){}
  //optional: caching of periodic writes to avoid hammering HAL/CAN
  virtual void readPeriodicInputs(){}

   virtual void writePeriodicOutputs(){}

   virtual bool checkSystem(){return true;}

   virtual void outputTelemetry(){}

   virtual void OnStart(double timestamp){}

   virtual void OnLoop(double timestamp){}
  
   virtual void OnStop(double timestamp){}

   virtual void zeroSensors(){}


};
}