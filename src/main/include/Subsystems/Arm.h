/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Subsystems/Subsystem.h"
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Solenoid.h>
//#define FRC_ROBORIO2 true
#ifdef FRC_ROBORIO2

#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>
#include <rev/CANSparkMaxLowLevel.h>
#endif
using namespace Subsystems;
namespace Subsystems{
class Arm : public Subsystems::Subsystem {
  bool clawState=false;
  bool extended=false;
  bool release=false;
  bool prevHatchPanel=false;
 public:
  Arm();
  void toggleClaw(bool toggleClaw);
  void togglePickup(bool togglePickup);
  void setArmPosition(double armThrottle);
  bool hasBall(); //true if have
  bool hasHatchPanel();
  frc::DigitalInput ballsensor{2};
  frc::DigitalInput hatchpanelsensor{3};
  frc::Solenoid releaser{2};
  frc::Solenoid extender{3}; 
  #ifdef FRC_ROBORIO2
  //ctre::phoenix::motorcontrol::can::WPI_VictorSPX leftmotor{11};
  //ctre::phoenix::motorcontrol::can::WPI_VictorSPX rightmotor{12};

  rev::CANSparkMax armMotor1{13,rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax armMotor2{20,rev::CANSparkMaxLowLevel::MotorType::kBrushless};

#endif

};


}