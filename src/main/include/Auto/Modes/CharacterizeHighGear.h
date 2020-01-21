/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Auto/AutoModeBase.h"
#include "Auto/Actions/CollectAccelerationData.h"
#include "Auto/Actions/CollectCurvatureData.h"
#include "Auto/Actions/CollectVelocityData.h"
#include "Auto/Actions/WaitAction.h"
#include "lib/Physics/DriveCharacterization.h"

#include "frc/smartdashboard/SmartDashboard.h"


#include <vector>
#include <memory>
#include <iostream>
using namespace std;


class CharacterizeHighGear: public AutoModeBase {

  vector<shared_ptr<DriveCharacterization::VelocityDataPoint>> velocityData;
  vector<shared_ptr<DriveCharacterization::AccelerationDataPoint>> accelerationData;
 public:
  CharacterizeHighGear();
  void routine() override;
  string getID() override;
};
