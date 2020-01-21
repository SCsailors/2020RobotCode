/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Splines/WayPoints.h"
#include "frc/smartdashboard/SmartDashboard.h"


WayPoints::WayPoints() : Subsystem("ExampleSubsystem") {}

void WayPoints::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void WayPoints::InitializeWaypoints(){
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
void WayPoints::GetWayPoints(){
    
    
    add=frc::SmartDashboard::GetBoolean("Add Waypoints", false);
    currenttheta=frc::SmartDashboard::GetNumber("Waypoints Theta", 0.0);
    currentnumberx=frc::SmartDashboard::GetNumber("Waypoints X", 0.0);
    currentnumbery=frc::SmartDashboard::GetNumber("Waypoints Y", 0.0);
    if (add){
        if (currentnumbery!=prevnumbery || currentnumberx!=prevnumberx || currenttheta!=prevtheta){
            waypointsX.push_back(currentnumberx);
            waypointsY.push_back(currentnumbery);
            waypointsT.push_back(currenttheta);
            prevnumberx=currentnumberx;
            prevnumbery=currentnumbery;
            prevtheta=currenttheta;
            add=false;
    
        } 
    }
    frc::SmartDashboard::PutBoolean("Add Waypoints", add);
    

}


void WayPoints::PrintWayPoints(){
}