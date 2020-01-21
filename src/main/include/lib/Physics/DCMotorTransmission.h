/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <math.h>
using namespace std;

class DCMotorTransmission {
 public:
  DCMotorTransmission();
  DCMotorTransmission(double speed_per_volt, double torque_per_volt, double friction_voltage);
  double speed_per_volt=0.0;
  double torque_per_volt=0.0;
  double friction_voltage=0.0;
  double effective_voltage=0.0;
  double kEpsilon= 1E-5;

  double Speed_per_Volt();
  double Torque_per_Volt();
  double Friction_Voltage();
  double free_speed_at_voltage(double voltage);
  double getTorqueForVoltage(double output_speed, double voltage);
  double getVoltageForTorque(double output_speed, double torque);//CHECK

};
