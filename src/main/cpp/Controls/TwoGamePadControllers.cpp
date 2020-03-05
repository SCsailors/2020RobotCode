/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Controls/TwoGamePadControllers.h"
using namespace ControlBoard;
std::shared_ptr<TwoGamePadControllers> TwoGamePadControllers::mInstance;

TwoGamePadControllers::TwoGamePadControllers() 
{
    reset();
}

std::shared_ptr<TwoGamePadControllers> TwoGamePadControllers::getInstance()
{
    if (!mInstance)
    {
        mInstance = std::make_shared<TwoGamePadControllers>();
    }
    return mInstance;
}

double TwoGamePadControllers::getThrottle()
{
    return mDriveController.getJoystick(XBoxController::Side::LEFT, XBoxController::Axis::y);
}

double TwoGamePadControllers::getTurn()
{
    return mDriveController.getJoystick(XBoxController::Side::LEFT, XBoxController::Axis::x);
}

bool TwoGamePadControllers::getQuickTurn()
{
    LT_Multi_Drive.update(mDriveController.getTrigger(XBoxController::Side::LEFT));
    return LT_Multi_Drive.isHeld();
}

bool TwoGamePadControllers::getWantsHighGear()
{
    RT_Multi_Drive.update(mDriveController.getTrigger(XBoxController::Side::RIGHT));
    return RT_Multi.holdStarted();
}

bool TwoGamePadControllers::getShoot()
{
    return A.update(mButtonController.getButton(XBoxController::Button::A));
}

bool TwoGamePadControllers::getWheel()
{
    return B.update(mButtonController.getButton(XBoxController::Button::B));
}

bool TwoGamePadControllers::getWantsRotation()
{
    return Start.update(mButtonController.getButton(XBoxController::Button::START));
}

bool TwoGamePadControllers::getClimber()
{
    return X.update(mButtonController.getButton(XBoxController::Button::X));
}

bool TwoGamePadControllers::getIntake()
{
    return Y.update(mButtonController.getButton(XBoxController::Button::Y));
}

bool TwoGamePadControllers::getCancel()
{
    //RB_Multi.update(mController.getButton(XBoxController::Button::RB));
    return RB_Multi.holdStarted();
}

double TwoGamePadControllers::getTurretJog()
{
    double jog = mButtonController.getJoystick(XBoxController::Side::RIGHT, XBoxController::Axis::x);
    if (util.epsilonEquals(jog, 0.0, kDeadband))
    {
        return 0.0;
    }
    double pre_jog = std::fabs(jog);
    //rescale
    double adj_jog = std::copysign((pre_jog-kDeadband)/ (1.0-kDeadband), jog);
    return Constants::kTurretJogMultiplier * (std::pow(adj_jog, Constants::kTurretJogPower));
}

std::shared_ptr<Rotation2D> TwoGamePadControllers::getTurretCardinal()
{
    int dPad = mButtonController.getDPad();
    
    return Rotation2D::fromDegrees(0.0);
    
}

void TwoGamePadControllers::reset()
{
    mDPadValid.reset(frc::Timer::GetFPGATimestamp(), mDPadDelay);
}


bool TwoGamePadControllers::getAutoAim()
{
    return Back.update(mButtonController.getButton(XBoxController::Button::BACK));
}

//put this first
double TwoGamePadControllers::getBallShootCount(bool preshoot)
{
    LT_Multi.update(mButtonController.getTrigger(XBoxController::Side::LEFT));
    RT_Multi.update(mButtonController.getTrigger(XBoxController::Side::RIGHT));
    LB_Multi.update(mButtonController.getButton(XBoxController::Button::LB));
    RB_Multi.update(mButtonController.getButton(XBoxController::Button::RB));
    if (preshoot){
        if (LT_Multi.wasTapped())
        {
            shootCount = 1.0;
        } else if (LB_Multi.wasTapped())
        {
            shootCount = 2.0;
        } else if (RT_Multi.wasTapped())
        {
            shootCount = 3.0;
        } else if (RB_Multi.wasTapped())
        {
            shootCount = 4.0;
        }
    } else
    {
        return 5.0;
    }
    
    //default bounds will be checked to make sure not shooting more balls than we have.
    return shootCount;
}
