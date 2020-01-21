/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Controls/ControlBoard.h"

ControlBoard::ControlBoard() {}

double ControlBoard::getThrottle(){
    return joysticks->GetGamepad().GetY();
}

double ControlBoard::getTurn(){
    return joysticks->GetGamepad().GetX();
}

bool ControlBoard::overrideTrajectory(){
    bool STOP= joysticks->rt.Get();
    if (STOP !=stop){
        stop=STOP;
        return STOP;
        
    } else{
        return false;
    }
}

bool ControlBoard::getQuickTurn(){
    return joysticks->lt.Get();
}

bool ControlBoard::setHighGear(){
    high= joysticks->a.Get();
    if(high && high!=prev_high){
            prev_HighGear= !(prev_HighGear);
                        
        }
        prev_high=high;
    return prev_HighGear;
    
    
}

double ControlBoard::getArmThrottle(){
    return joysticks->GetGamepad().GetTwist();
}

bool ControlBoard::toggleClaw(){
    claw=joysticks->b.Get();
    if (claw&& claw!=prevClaw){
        prev_Claw=!(prev_Claw);
    }
    prevClaw=claw;
    return prev_Claw;
}

bool ControlBoard::toggleBallPickup(){
    pickup=joysticks->x.Get();
    if(pickup && pickup!=prevPickup){
        prev_Pickup=!(prev_Pickup);
    }
    prevPickup=pickup;
    return prev_Pickup;
}

void ControlBoard::testHighGearToggle(){
    if(highGear!=setHighGear()){
        highGear=setHighGear();
        if(highGear){
            
        }else if(!highGear){
        }
    }
}