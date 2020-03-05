/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Joysticks.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include <memory>
#include <vector>

#include <lib/Geometry/Rotation2D.h>
#include <lib/Util/Util.h>

namespace ControlBoard {

class TurretCardinal {
  
 public:
  std::shared_ptr<Rotation2D> rotation;
  std::shared_ptr<Rotation2D> inputDirection;
  TurretCardinal(int degrees)
  {
    double deg = (double) degrees;
    rotation = rotation->fromDegrees(deg);
    inputDirection = inputDirection->fromDegrees(deg);
  }
  
  TurretCardinal(double degrees)
  {
    rotation = rotation->fromDegrees(degrees);
    inputDirection = inputDirection->fromDegrees(degrees);
  }

  TurretCardinal(double degrees, double inputDirectionDegrees)
  {
    rotation = rotation->fromDegrees(degrees);
    inputDirection = inputDirection->fromDegrees(inputDirectionDegrees);
  }

  TurretCardinal()
  {
    rotation = rotation->fromDegrees(0.0);
    inputDirection = NULL;
  }
};

class ControlBoardBase {
 public: 
  ControlBoardBase(){reset();}
  virtual double getThrottle(){return 0.0;};
  virtual double getTurn(){return 0.0;};
  virtual bool getQuickTurn(){return false;};
  virtual bool getWantsHighGear(){return false;};
  virtual bool getShoot(){return false;}
  virtual bool getWheel(){return false;}
  virtual bool getWantsRotation(){return false;}

  virtual bool getClimber(){return false;}
  virtual bool getIntake(){return false;}
  virtual bool getCancel(){return false;}

  virtual double getTurretJog(){return 0.0;}

  virtual bool isTurretJogging(){return 0.0;}
  
  virtual std::shared_ptr<Rotation2D> getTurretCardinal()
  {
    return Rotation2D::fromDegrees(0.0);
  }

  virtual bool getAutoAim(){return false;}
  virtual double getBallShootCount(bool preshoot){return 0.0;}
  virtual void reset(){};

  virtual double getHood(){return 0.0;};
  virtual double getShooter(){return 0.0;};
  virtual double getBallPath(){return 0.0;}

  virtual bool getDriveShifterManual(){return true;}
  virtual bool getDriveStraight(){return false;}
  
  virtual bool getClimbRun(){return false;}
  virtual bool getCloseShoot(){return false;}
  virtual bool getLineShoot(){return false;}

  Util util{}; 
  double kDeadband = .4;
  
};

}