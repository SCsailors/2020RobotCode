/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <Subsystems/ServoMotorSubsystem.h>

#include <frc/DigitalInput.h>
#include <frc/Timer.h>
#include <lib/Util/LatchedBoolean.h>

#include <Constants.h>

#include <memory>

namespace Subsystems {

class BallPathTop : public Subsystems::TalonSRXSubsystem {
  frc::DigitalInput mFirstBreak{Constants::kDIO_FirstBreak};
  frc::DigitalInput mFirstMake{Constants::kDIO_FirstMake};
  frc::DigitalInput mLastBreak{Constants::kDIO_LastBreak};
  frc::DigitalInput mLastMake{Constants::kDIO_LastMake};

  bool mFirstBreakState = false;
  bool mFirstMakeState = false;
  bool mLastBreakState = false;
  bool mLastMakeState = false;
  bool mPhotoEyeState = false;

  bool mPrevFirstBreakState = false;
  //bool mPrevFirstMakeState = false;
  bool mPrevLastBreakState = false;
  //bool mPrevLastMakeState = false;

  double mFirstBreakStartTimestamp = 0.0;
  double mLastBreakStartTimestamp = 0.0;

  int mBallCount = 0;
  bool mHasBalls = false;
  bool mHasFiveBalls = false;
  
  static std::shared_ptr<Subsystems::BallPathTop> mInstance;
 public:
  BallPathTop(std::shared_ptr<Subsystems::TalonConstants> constants);
  static std::shared_ptr<Subsystems::BallPathTop> getInstance();

  void updateFirst();
  void updateLast();
  void readPeriodicInputs() override;

  void outputTelemetry() override;
  
  int getBallCount();
  bool hasBalls();
  bool hasFiveBalls();

  void SetBallCount(int count);

  bool getPhotoEyeState(){return mPhotoEyeState;}
  bool getLastBreakState(){return mLastBreakState;}
  bool getFirstBreakState(){return mFirstBreakState;}
};
}