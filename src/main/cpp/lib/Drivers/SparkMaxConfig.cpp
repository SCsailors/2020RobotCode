/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Drivers/SparkMaxConfig.h"
using namespace Drivers;
SparkMaxConfig::SparkMaxConfig()
{
    std::cout<<"Started SparkMaxConfig"<<std::endl;
}


SparkMaxConfigBase::SparkMaxConfigBase()
{
    std::cout<<"Started SparkMaxConfigBase"<<std::endl;
    kDefaultConfig = std::make_shared<SparkMaxConfig>();
    kSlaveConfig = std::make_shared<SparkMaxConfig>();

    std::cout<<"SparkMaxConfigBase: constructed slave and default"<<std::endl;
    kSlaveConfig->STATUS_FRAME_0_RATE_MS = 1000.0;
    kSlaveConfig->STATUS_FRAME_1_RATE_MS = 1000.0;
    kSlaveConfig->STATUS_FRAME_2_RATE_MS = 1000.0;
    std::cout<<"SparkMaxConfigBase: slave frame rates"<<std::endl;
}