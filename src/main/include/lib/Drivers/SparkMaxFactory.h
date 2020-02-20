/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <lib/Drivers/SparkMaxConfig.h>

#include <rev/CANSparkMax.h>
#include <frc/DriverStation.h>
#include <sstream>

using namespace rev;

namespace Drivers{

class SparkMaxFactory {
	
 public:
  SparkMaxFactory(){}
  static bool handleCANError(int id, CANError error, std::string message)
  {
    
    
    if (error == CANError::kOk)
	{
		return true;
	}else
    {
      std::string error_name;

	switch(error)
	{
		case rev::CANError::kOk:
			break;
		case rev::CANError::kError:
			error_name = "kError";
			break;
		case rev::CANError::kTimeout:
			error_name = "kTimeout";
			break;
		case rev::CANError::kNotImplmented:
			error_name = "kNotImplmented";
			break;
		case rev::CANError::kHALError:
			error_name = "kHALError";
			break;
		case rev::CANError::kCantFindFirmware:
			error_name = "kCantFindFirmware";
			break;
		case rev::CANError::kFirmwareTooOld:
			error_name = "kFirmwareTooOld";
			break;
		case rev::CANError::kFirmwareTooNew:
			error_name = "kFirmwareTooNew";
			break;
		case rev::CANError::kParamInvalidID:
			error_name = "kParamInvalidID";
			break;
		case rev::CANError::kParamMismatchType:
			error_name = "kParamMismatchType";
			break;
		case rev::CANError::kParamAccessMode:
			error_name = "kParamAccessMode";
			break;
		case rev::CANError::kParamInvalid:
			error_name = "kParamInvalid";
			break;
		case rev::CANError::kParamNotImplementedDeprecated:
			error_name = "kParamNotImplementedDeprecated";
			break;
		case rev::CANError::kFollowConfigMismatch:
			error_name = "kFollowConfigMismatch";
			break;
		case rev::CANError::kInvalid:
			error_name = "kInvalid";
			break;
		case rev::CANError::kSetpointOutOfRange:
			error_name = "kSetpointOutOfRange";
			break;
		default:
			{
				std::stringstream s;
				s << "Unknown Spark Max error " << static_cast<int>(error);
				error_name = s.str();
				break;
			}
      
      
    	}
		frc::DriverStation::ReportError("Could not configure spark id: " + std::to_string(id) + " error: " + error_name + " message: "+ message);
		return false;
	}

  }

  static std::shared_ptr<CANSparkMax> createFollowerSparkMax(int id, std::shared_ptr<CANSparkMax> leader, CANSparkMax::MotorType type)
  {
	SparkMaxConfigBase config{};
    std::shared_ptr<CANSparkMax> canSparkMax = createSparkMax(id, config.kSlaveConfig, type);
    handleCANError(id, canSparkMax->Follow(*leader.get()), "Setting Follower");
    return canSparkMax;
  }

  static std::shared_ptr<CANSparkMax> createDefaultSparkMax(int id, CANSparkMax::MotorType type)
  {
	  SparkMaxConfigBase config{};
	  return createSparkMax(id, config.kDefaultConfig, type);
  }
  
  
  static std::shared_ptr<CANSparkMax> createSparkMax(int id, std::shared_ptr<SparkMaxConfig> config, CANSparkMax::MotorType type)
  {
    std::shared_ptr<CANSparkMax> canSparkMax = std::make_shared<CANSparkMax>(id, type);
	canSparkMax->Set(0.0);

    handleCANError(id, canSparkMax->SetPeriodicFramePeriod(CANSparkMaxLowLevel::PeriodicFrame::kStatus0, config->STATUS_FRAME_0_RATE_MS), "set status0 rate");
    handleCANError(id, canSparkMax->SetPeriodicFramePeriod(CANSparkMaxLowLevel::PeriodicFrame::kStatus1, config->STATUS_FRAME_0_RATE_MS), "set status1 rate");
    handleCANError(id, canSparkMax->SetPeriodicFramePeriod(CANSparkMaxLowLevel::PeriodicFrame::kStatus2, config->STATUS_FRAME_0_RATE_MS), "set status2 rate");
  
    //add more default config values as necessary 
	return canSparkMax; 
  }
};



}