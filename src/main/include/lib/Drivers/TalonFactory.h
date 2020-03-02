/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>

#include <lib/Drivers/TalonConfig.h>

#include <frc/DriverStation.h>
#include <sstream>

using namespace ctre::phoenix::motorcontrol;
namespace Drivers{

class TalonFactory {
  const static int kTimeoutMs{100};
 public:
  TalonFactory();
  
  //Talon SRX creators
  static std::shared_ptr<TalonSRX> createDefaultTalonSRX(int id);
  template <typename T>
  static std::shared_ptr<TalonSRX> createSlaveTalonSRX(int id, T leader)
  {
    Drivers::TalonConfigBase config{};
    std::shared_ptr<TalonSRX> talon = createTalonSRX(id, config.kSlaveConfig);
    
    talon->Follow(*leader);
    return talon;
  }
  
  static std::shared_ptr<TalonSRX> createTalonSRX(int id, std::shared_ptr<TalonConfig> config);

  //Falcon 500/ TalonFX creators
  static std::shared_ptr<TalonFX> createDefaultTalonFX(int id);
  
  static std::shared_ptr<TalonFX> createSlaveTalonFX(int id, std::shared_ptr<TalonFX> leader)
  {
    Drivers::TalonConfigBase config{};
    std::shared_ptr<TalonFX> talon = createTalonFX(id, config.kSlaveConfig);
    
    talon->Follow(*leader);
    return talon;
  }

  static std::shared_ptr<TalonFX> createSlaveTalonFX(int id, std::shared_ptr<TalonSRX> leader)
  {
    Drivers::TalonConfigBase config{};
    std::shared_ptr<TalonFX> talon = createTalonFX(id, config.kSlaveConfig);
    talon->Follow(*leader);
    return talon;
  }

  static std::shared_ptr<TalonFX> createTalonFX(int id, std::shared_ptr<TalonConfig> config);

  template <typename T>
  static void configTalon(int id, std::shared_ptr<TalonConfig> config, T talon);

  static bool handleCANError(int id, ctre::phoenix::ErrorCode error, const std::string &method_name);
};
}