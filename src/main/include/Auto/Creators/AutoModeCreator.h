/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Auto/AutoModeBase.h"
#include <memory>
using namespace std;

class AutoModeCreator {
 public:
  AutoModeCreator();
  virtual shared_ptr<AutoModeBase> getStateDependentAutoMode(bool Left){
    shared_ptr<AutoModeBase> base= make_shared<AutoModeBase>();
    cout<<"In AutoModeCreator Base. Change creator."<<endl;
    return base;
  };// change input to field state if the game requires it.
};
