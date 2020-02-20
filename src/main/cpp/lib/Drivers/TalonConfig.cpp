/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Drivers/TalonConfig.h"
using namespace Drivers;

TalonConfig::TalonConfig() 
{
    std::cout<<"TalonConfig: Started Constructor"<< std::endl;
}

TalonConfigBase::TalonConfigBase()
{
    std::cout<<"TalonConfigBase: Started Constructor"<< std::endl;
    kSlaveConfig = std::make_shared<TalonConfig>();
    kDefaultConfig = std::make_shared<TalonConfig>();
    
    std::cout<<"TalonConfigBase: constructed slave and default"<< std::endl;
    //Increase as much as possible, but too high and leds behave strangely
    kSlaveConfig->CONTROL_FRAME_PERIOD_MS = 100;
    kSlaveConfig->MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
    kSlaveConfig->GENERAL_STATUS_FRAME_RATE_MS = 1000;
    kSlaveConfig->FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
    kSlaveConfig->QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
    kSlaveConfig->ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
    kSlaveConfig->PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
    std::cout<<"TalonConfigBase: configed slave"<< std::endl;
}
