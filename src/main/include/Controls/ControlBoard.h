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
  
  virtual TurretCardinal getTurretCardinal()
  {
    return TurretCardinal{0.0};
  }

  virtual bool getAutoAim(){return false;}
  virtual double getBallShootCount(bool preshoot){return 0.0;}
  virtual void reset(){};
  virtual double getHood(){return 0.0;};
  virtual double getShooter(){return 0.0;};
  virtual bool getDriveShifterManual(){return true;}
  virtual double getBallPath(){return 0.0;}
  virtual bool getBallPathToggle(){return false;}
  virtual bool getClimbRun(){return false;}

  Util util{}; 
  double turretDeadband = .4;
  enum TurretCardinalEnum 
  {
    BACK = 180,
    FRONT = 0,
    LEFT = 90,
    RIGHT = 270,
    FRONT_LEFT = 45,
    FRONT_RIGHT = 315,
    BACK_LEFT = 135,
    BACK_RIGHT = 225,
    NONE = 0
  };
  //                                                  back                front                 left                  right                 Front left            Front right            Back left              Back Right
  std::vector<TurretCardinal> TurretCardinalVecotr{TurretCardinal{180.0}, TurretCardinal{0.0}, TurretCardinal{90.0}, TurretCardinal{270.0}, TurretCardinal{45.0}, TurretCardinal{315.0}, TurretCardinal{135.0}, TurretCardinal{225.0}};
  
  TurretCardinal findClosest(double xAxis, double yAxis)
  {
    std::shared_ptr<Rotation2D> rot = std::make_shared<Rotation2D>(yAxis, -xAxis, true);
    return findClosest(rot);
  }

  TurretCardinal findClosest(std::shared_ptr<Rotation2D> stickDirection)
  {
    TurretCardinal closest;
    double closestDistance = INFINITY;
    for (auto checkDirection : TurretCardinalVecotr)
    {
      double distance = std::fabs(stickDirection->inverse()->rotateBy(checkDirection.inputDirection)->getDegrees());
      if (distance < closestDistance)
      {
        closestDistance = distance;
        closest = checkDirection;
      }
    }

    return closest;
  }

  bool isDiagonal(TurretCardinalEnum cardinal)
  {
    return cardinal == FRONT_LEFT || cardinal == FRONT_RIGHT || cardinal == BACK_LEFT || cardinal == BACK_RIGHT;
  }

  TurretCardinal enumToTurretCardinal(TurretCardinalEnum TC_enum)
  {
    return TurretCardinal{TC_enum};
  }

  TurretCardinalEnum TurretCardinalToEnum(TurretCardinal TC)
  {
    if (TC.inputDirection == NULL)
    {
      return TurretCardinalEnum::NONE;
    }
    
    if (util.epsilonEquals(TC.inputDirection->getDegrees(), 0.0))
    {
      return TurretCardinalEnum::FRONT;
    } else if (util.epsilonEquals(TC.inputDirection->getDegrees(), 180.0))
    {
      return TurretCardinalEnum::BACK;
    } else if (util.epsilonEquals(TC.inputDirection->getDegrees(), 90.0))
    {
      return TurretCardinalEnum::LEFT;
    } else if (util.epsilonEquals(TC.inputDirection->getDegrees(), 270.0))
    {
      return TurretCardinalEnum::RIGHT;
    } else if (util.epsilonEquals(TC.inputDirection->getDegrees(), 45.0))
    {
      return TurretCardinalEnum::FRONT_LEFT;
    } else if (util.epsilonEquals(TC.inputDirection->getDegrees(), 315.0))
    {
      return TurretCardinalEnum::FRONT_RIGHT;
    } else if (util.epsilonEquals(TC.inputDirection->getDegrees(), 135.0))
    {
      return TurretCardinalEnum::BACK_LEFT;
    } else if (util.epsilonEquals(TC.inputDirection->getDegrees(), 225.0))
    {
      return TurretCardinalEnum::BACK_RIGHT;
    }
  
    return TurretCardinalEnum::NONE;
  }

  
  
};

}