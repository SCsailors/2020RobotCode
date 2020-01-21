/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "lib/Physics/DCMotorTransmission.h"
#include <cmath>
#include <memory>
using namespace std;

class DifferentialDrive {
//ALL units SI
 public:
 double kEpsilon=1E-5;
 //Equivalent mass when accelerating linearly, in kg
 //measure by doing drivetrain acceleration characterization in a straight line.
 double mass_=1.0;

 //moment of inertia when accelerating purely angularly, in kg*m^2
 //measure by doing drivetrain acceleration characterization while turning in place
 double moi_=1.0;

 //Drag torque (proportional to angular velocity) that resists turning, N*m/rad/s
 double angular_drag_=1.0;

 //roll robot a known distance and count encoder ticks
 double wheel_radius_=1.0;

 //"Effective kinematic wheelbase radius." Might be larger than theoretical to compensate for skid steer.
 //measure by turning the robot in place several times and figuring out what the equivalent wheelbase radius is.
 double effective_wheelbase_radius_=1.0;

 //Transmissions for both sides
 shared_ptr<DCMotorTransmission> left_transmission_;
 shared_ptr<DCMotorTransmission> right_transmission_;

  DifferentialDrive(double mass, double moi, double angular_drag, double wheel_radius, double effective_wheelbase_radius, shared_ptr<DCMotorTransmission> left_transmission, shared_ptr<DCMotorTransmission> right_transmission);
  double Mass();
  double Moi();
  double Wheel_radius();
  double Angular_drag();
  double Effective_wheelbase_radius();
  shared_ptr<DCMotorTransmission> Left_transmission();
  shared_ptr<DCMotorTransmission> Right_transmission();
  
  class ChassisState{
    public:
    double linear=0.0;
    double angular=0.0;
    ChassisState();
    ChassisState(double linear, double angular);
    
  };

  class MinMax{
    public:
    MinMax();
    double min=0.0;
    double max=0.0;
    
    shared_ptr<MinMax> getMinMaxAcceleration(shared_ptr<ChassisState> chassis_velocity, double curvature, double max_abs_voltage);
  };


  class WheelState{
  public:
   double left=0.0;
   double right=0.0;
   WheelState(double left, double right);
   WheelState();
   double get(bool get_left);
   void set(bool set_left, double val);
  };

  class DriveDynamics{
    public:
    DriveDynamics();
    double curvature=0.0; //m^-1
    double dcurvature=0.0; //m^-1/m
    shared_ptr<ChassisState> chassis_velocity= make_shared<ChassisState>(); // m/s
    shared_ptr<ChassisState> chassis_acceleration= make_shared<ChassisState>(); // m/s^2
    shared_ptr<WheelState> wheel_velocity= make_shared<WheelState>(); // rad/s
    shared_ptr<WheelState> wheel_acceleration= make_shared<WheelState>(); // rad/s^2
    shared_ptr<WheelState> voltage= make_shared<WheelState>(); // V
    shared_ptr<WheelState> wheel_torque= make_shared<WheelState>(); // N*m
  };

  shared_ptr<ChassisState> solveForwardKinematics(shared_ptr<WheelState> wheel_motion);
  shared_ptr<WheelState> solveInverseKinematics(shared_ptr<ChassisState> chassis_motion);
  
  shared_ptr<DriveDynamics> solveForwardDynamics(shared_ptr<ChassisState> chassis_velocity, shared_ptr<WheelState> voltage);
  shared_ptr<DriveDynamics> solveForwardDynamics(shared_ptr<WheelState> wheel_velocity, shared_ptr<WheelState> voltage);
  void solveForwardDynamics(shared_ptr<DriveDynamics> dynamics);

  shared_ptr<DriveDynamics> solveInverseDynamics(shared_ptr<ChassisState> chassis_velocity, shared_ptr<ChassisState> chassis_acceleration);
  shared_ptr<DriveDynamics> solveInverseDynamics(shared_ptr<WheelState> wheel_velocity, shared_ptr<WheelState> wheel_acceleration);
  void solveInverseDynamics(shared_ptr<DriveDynamics> dynamics);

  double getMaxAbsVelocity(double curvature, double max_abs_voltage);

  
};
