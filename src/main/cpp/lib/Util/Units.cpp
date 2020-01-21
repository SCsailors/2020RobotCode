/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Util/Units.h"

Units::Units() {}

double Units::inches_to_meters(double inches){
    return inches*.0254;
}

double Units::meters_to_inches(double meters){
    return meters/.0254;
}

double Units::toDegrees(double radians){
    return radians*180/pi;
}

double Units::toRadians(double degrees){
    return degrees*pi/180;
}