/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Drivers/TalonFactory.h"
using namespace Drivers;
TalonFactory::TalonFactory() {}

std::shared_ptr<TalonSRX> TalonFactory::createDefaultTalonSRX(int id)
{
	//std::cout<<"Entered TalonFactory CreateDefaultTalonSRX"<< std::endl;
    Drivers::TalonConfigBase config{};
	//std::cout<<"created Config CreateDefaultTalonSRX"<< std::endl;
    return createTalonSRX(id, config.kDefaultConfig);
}

std::shared_ptr<TalonSRX> TalonFactory::createTalonSRX(int id, std::shared_ptr<TalonConfig> config)
{
    std::shared_ptr<TalonSRX> talon = std::make_shared<TalonSRX>(id);
    //std::cout<<"Made Talon SRX in TalonFactory CreateTalonSRX"<< std::endl;
	//configTalon(id, config, talon);
	//std::cout<<"Configed Talon TalonFactory CreateTalonSRX"<< std::endl;
    return talon;
}

template <typename T>
void TalonFactory::configTalon(int id, std::shared_ptr<TalonConfig> config, T talon)
{

	//std::cout<<"TalonFactory Starting Config Talon"<< std::endl;
    talon->Set(ControlMode::PercentOutput, 0.0);
	//std::cout<<"TalonFactory Starting Config Talon"<< std::endl;
    talon->ChangeMotionControlFramePeriod(config->MOTION_CONTROL_FRAME_PERIOD_MS);
    talon->ClearMotionProfileHasUnderrun(kTimeoutMs);
    talon->ClearMotionProfileTrajectories();
	//std::cout<<"TalonFactory Config Talon: Motion Profile"<< std::endl;
    talon->ClearStickyFaults(kTimeoutMs);
	//std::cout<<"TalonFactory Config Talon: clear stickyfaults"<< std::endl;
    talon->ConfigSetParameter(ctre::phoenix::ParamEnum::eClearPositionOnLimitF, 0.0, 0.0, 0.0, kTimeoutMs);
    talon->ConfigSetParameter(ctre::phoenix::ParamEnum::eClearPositionOnLimitR, 0.0, 0.0, 0.0, kTimeoutMs);
	//std::cout<<"TalonFactory Config Talon: Clear Limit F and R"<< std::endl;
    talon->ConfigNominalOutputForward(0.0, kTimeoutMs);
    talon->ConfigNominalOutputReverse(0.0, kTimeoutMs);
    talon->ConfigNeutralDeadband(config->NEUTRAL_DEADBAND, kTimeoutMs);
	//std::cout<<"TalonFactory Config Talon: Config Output and Deadband"<< std::endl;
    talon->ConfigPeakOutputForward(1.0, kTimeoutMs);
    talon->ConfigPeakOutputReverse(-1.0, kTimeoutMs);
	//std::cout<<"TalonFactory Config Talon: peak output"<< std::endl;
    talon->ConfigVelocityMeasurementPeriod(config->VELOCITY_MEASUREMENT_PERIOD, kTimeoutMs);
    talon->ConfigVelocityMeasurementWindow(config->VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, kTimeoutMs);
	
    talon->ConfigVoltageMeasurementFilter(32.0, kTimeoutMs);
	//std::cout<<"TalonFactory Config Talon: Velocity Measurement"<< std::endl;
    talon->SetStatusFramePeriod(Status_1_General, config->GENERAL_STATUS_FRAME_RATE_MS, kTimeoutMs);
    talon->SetStatusFramePeriod(Status_2_Feedback0, config->FEEDBACK_STATUS_FRAME_RATE_MS, kTimeoutMs);
    talon->SetStatusFramePeriod(Status_3_Quadrature, config->QUAD_ENCODER_STATUS_FRAME_RATE_MS, kTimeoutMs);
    talon->SetStatusFramePeriod(Status_4_AinTempVbat, config->ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS, kTimeoutMs);
    talon->SetStatusFramePeriod(Status_8_PulseWidth, config->PULSE_WIDTH_STATUS_FRAME_RATE_MS, kTimeoutMs);
    talon->SetControlFramePeriod(ControlFrame::Control_3_General, config->CONTROL_FRAME_PERIOD_MS);
	//std::cout<<"TalonFactory Config Talon: Status Frames"<< std::endl;
}

std::shared_ptr<TalonFX> TalonFactory::createDefaultTalonFX(int id)
{
    Drivers::TalonConfigBase config{};
    return createTalonFX(id, config.kDefaultConfig);
}


std::shared_ptr<TalonFX> TalonFactory::createTalonFX(int id, std::shared_ptr<TalonConfig> config)
{
    std::shared_ptr<TalonFX> talon = std::make_shared<TalonFX>(id);
    //configTalon(id, config, talon);
    return talon;
}

bool TalonFactory::handleCANError(int id, ctre::phoenix::ErrorCode error_code, const std::string &method_name)
{
    std::string error_name;
	switch (error_code)
	{
		case ctre::phoenix::OK :
			return true; // Yay us!

		case ctre::phoenix::CAN_MSG_STALE :
			error_name = "CAN_MSG_STALE/CAN_TX_FULL/TxFailed";
			break;
		case ctre::phoenix::InvalidParamValue :
			error_name = "InvalidParamValue/CAN_INVALID_PARAM";
			break;

		case ctre::phoenix::RxTimeout :
			error_name = "RxTimeout/CAN_MSG_NOT_FOUND";
			break;
		case ctre::phoenix::TxTimeout :
			error_name = "TxTimeout/CAN_NO_MORE_TX_JOBS";
			break;
		case ctre::phoenix::UnexpectedArbId :
			error_name = "UnexpectedArbId/CAN_NO_SESSIONS_AVAIL";
			break;
		case ctre::phoenix::BufferFull :
			error_name = "BufferFull";
			break;
		case ctre::phoenix::CAN_OVERFLOW:
			error_name = "CAN_OVERFLOW";
			break;
		case ctre::phoenix::SensorNotPresent :
			error_name = "SensorNotPresent";
			break;
		case ctre::phoenix::FirmwareTooOld :
			error_name = "FirmwareTooOld";
			break;
		case ctre::phoenix::CouldNotChangePeriod :
			error_name = "CouldNotChangePeriod";
			break;
		case ctre::phoenix::BufferFailure :
			error_name = "BufferFailure";
			break;

		case ctre::phoenix::GENERAL_ERROR :
			error_name = "GENERAL_ERROR";
			break;

		case ctre::phoenix::SIG_NOT_UPDATED :
			error_name = "SIG_NOT_UPDATED";
			break;
		case ctre::phoenix::NotAllPIDValuesUpdated :
			error_name = "NotAllPIDValuesUpdated";
			break;

		case ctre::phoenix::GEN_PORT_ERROR :
			error_name = "GEN_PORT_ERROR";
			break;
		case ctre::phoenix::PORT_MODULE_TYPE_MISMATCH :
			error_name = "PORT_MODULE_TYPE_MISMATCH";
			break;

		case ctre::phoenix::GEN_MODULE_ERROR :
			error_name = "GEN_MODULE_ERROR";
			break;
		case ctre::phoenix::MODULE_NOT_INIT_SET_ERROR :
			error_name = "MODULE_NOT_INIT_SET_ERROR";
			break;
		case ctre::phoenix::MODULE_NOT_INIT_GET_ERROR :
			error_name = "MODULE_NOT_INIT_GET_ERROR";
			break;

		case ctre::phoenix::WheelRadiusTooSmall :
			error_name = "WheelRadiusTooSmall";
			break;
		case ctre::phoenix::TicksPerRevZero :
			error_name = "TicksPerRevZero";
			break;
		case ctre::phoenix::DistanceBetweenWheelsTooSmall :
			error_name = "DistanceBetweenWheelsTooSmall";
			break;
		case ctre::phoenix::GainsAreNotSet :
			error_name = "GainsAreNotSet";
			break;
		case ctre::phoenix::WrongRemoteLimitSwitchSource :
			error_name = "WrongRemoteLimitSwitchSource";
			break;

		case ctre::phoenix::IncompatibleMode :
			error_name = "IncompatibleMode";
			break;
		case ctre::phoenix::InvalidHandle :
			error_name = "InvalidHandle";
			break;

		case ctre::phoenix::FeatureRequiresHigherFirm:
			error_name = "FeatureRequiresHigherFirm";
			break;
		case ctre::phoenix::TalonFeatureRequiresHigherFirm:
			error_name = "TalonFeatureRequiresHigherFirm";
			break;
		case ctre::phoenix::ConfigFactoryDefaultRequiresHigherFirm:
			error_name = "ConfigFactoryDefaultRequiresHigherFirm";
			break;
		case ctre::phoenix::LibraryCouldNotBeLoaded :
			error_name = "LibraryCouldNotBeLoaded";
			break;
		case ctre::phoenix::MissingRoutineInLibrary :
			error_name = "MissingRoutineInLibrary";
			break;
		case ctre::phoenix::ResourceNotAvailable :
			error_name = "ResourceNotAvailable";
			break;

		case ctre::phoenix::PulseWidthSensorNotPresent :
			error_name = "PulseWidthSensorNotPresent";
			break;
		case ctre::phoenix::GeneralWarning :
			error_name = "GeneralWarning";
			break;
		case ctre::phoenix::FeatureNotSupported :
			error_name = "FeatureNotSupported";
			break;
		case ctre::phoenix::NotImplemented :
			error_name = "NotImplemented";
			break;
		case ctre::phoenix::FirmVersionCouldNotBeRetrieved :
			error_name = "FirmVersionCouldNotBeRetrieved";
			break;
		case ctre::phoenix::FeaturesNotAvailableYet :
			error_name = "FeaturesNotAvailableYet";
			break;
		case ctre::phoenix::ControlModeNotValid :
			error_name = "ControlModeNotValid";
			break;

		case ctre::phoenix::ControlModeNotSupportedYet :
			error_name = "ConrolModeNotSupportedYet";
			break;
		case ctre::phoenix::CascadedPIDNotSupporteYet:
			error_name = "CascadedPIDNotSupporteYet/AuxiliaryPIDNotSupportedYet";
			break;
		case ctre::phoenix::RemoteSensorsNotSupportedYet:
			error_name = "RemoteSensorsNotSupportedYet";
			break;
		case ctre::phoenix::MotProfFirmThreshold:
			error_name = "MotProfFirmThreshold";
			break;
		case ctre::phoenix::MotProfFirmThreshold2:
			error_name = "MotProfFirmThreshold2";
			break;

		default:
			{
				std::stringstream s;
				s << "Unknown Talon error " << error_code;
				error_name = s.str();
				break;
			}

	}
    frc::DriverStation::ReportError("Could not configure spark id: " + std::to_string(id) + " error: " + error_name + " message: "+ method_name);
    return false;
}