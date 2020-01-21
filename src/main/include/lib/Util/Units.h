/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

class Units {
  double pi=3.14159265359;
 public:
  
  Units();
  double inches_to_meters(double inches);
  double meters_to_inches(double meters);
  double toRadians(double degrees);
  double toDegrees( double radians);
};
