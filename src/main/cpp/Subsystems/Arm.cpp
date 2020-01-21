/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/Arm.h"


using namespace Subsystems;
namespace Subsystems{

Arm::Arm() {
    #ifdef FRC_ROBORIO2
    //leftmotor.SetInverted(false);
    //rightmotor.SetInverted(true);
    
    armMotor1.SetInverted(true);
    armMotor2.Follow(armMotor1,true);
    armMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    armMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    
    #endif
}
}



bool Arm::hasBall(){
    return !(ballsensor.Get());
}

bool Arm::hasHatchPanel(){
    return !(hatchpanelsensor.Get());
}

void Arm::toggleClaw(bool toggleClaw){
    if (toggleClaw!=clawState){
        if(toggleClaw){
            extender.Set(true);
            extended=true;

        }else{
            releaser.Set(true);
            release=true;

            extender.Set(false);
            extended=false;

        }
        clawState=toggleClaw;
       

    }
    if (!hasHatchPanel() && hasHatchPanel()!=prevHatchPanel){
            releaser.Set(false);
            release=false;
            prevHatchPanel=hasHatchPanel();
    }

}

void Arm::togglePickup(bool togglePickup){
    bool run=togglePickup;
    if (run && !hasBall()){
        #ifdef FRC_ROBORIO2
        //leftmotor.Set(-1.0);
        //rightmotor.Set(-1.0);
        #endif
    } else if (!run && hasBall()){
        #ifdef FRC_ROBORIO2
        //leftmotor.Set(1.0);
        //rightmotor.Set(1.0);
        #endif
    } else {
        #ifdef FRC_ROBORIO2
        //leftmotor.Set(.0);
        //rightmotor.Set(.0);
        #endif
        }

}

void Arm::setArmPosition(double armThrottle){
    #ifdef FRC_ROBORIO2
    armMotor1.Set(armThrottle);
    #endif
}