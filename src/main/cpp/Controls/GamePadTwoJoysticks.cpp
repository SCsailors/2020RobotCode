/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Controls/GamePadTwoJoysticks.h"

using namespace ControlBoard;
std::shared_ptr<GamePadTwoJoysticks> GamePadTwoJoysticks::mInstance;

GamePadTwoJoysticks::GamePadTwoJoysticks() 
{
    reset();
}


std::shared_ptr<GamePadTwoJoysticks> GamePadTwoJoysticks::getInstance()
{
    if (!mInstance)
    {
        mInstance = std::make_shared<GamePadTwoJoysticks>();
    }
    return mInstance;
}

double GamePadTwoJoysticks::getThrottle()
{
    return -mLeftJoystick.GetY();
}

double GamePadTwoJoysticks::getTurn()
{
    return -mRightJoystick.GetY();
}

bool GamePadTwoJoysticks::getQuickTurn()
{
    return false;
}

bool GamePadTwoJoysticks::getWantsHighGear()
{
    bool wantsHigh = HighGear.update(mLeftJoystick.GetRawButton(1));
    if (wantsHigh && !wantsHighGear)
    { //want manual shift
        wantsHighGear = true;
    
    } else if (wantsHigh && wantsHighGear)
    {
        wantsHighGear = false;
    }
    return wantsHighGear;
}

bool GamePadTwoJoysticks::getDriveStraight()
{
    return mRightJoystick.GetRawButton(1);
}

bool GamePadTwoJoysticks::getShoot()
{
    return A.update(mController.getButton(XBoxController::Button::A));
}

double GamePadTwoJoysticks::getBallShootCount(bool preshoot)
{
    LT_Multi.update(mController.getTrigger(XBoxController::Side::LEFT));
    RT_Multi.update(mController.getTrigger(XBoxController::Side::RIGHT));
    LB_Multi.update(mController.getButton(XBoxController::Button::LB));
    RB_Multi.update(mController.getButton(XBoxController::Button::RB));

    if (preshoot){
        if (LT_Multi.wasTapped())
        {
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

bool GamePadTwoJoysticks::getWheel()
{
    return B.update(mController.getButton(XBoxController::Button::B));
}

bool GamePadTwoJoysticks::getWantsRotation()
{
    return false;//Start.update(mController.getButton(XBoxController::Button::START));
}

bool GamePadTwoJoysticks::getClimber()
{
    frc::SmartDashboard::PutBoolean("Start State (climb)",mController.getButton(8));
    Start.update(mController.getButton(XBoxController::Button::START));
    return Start.wasTapped();
}

bool GamePadTwoJoysticks::getClimbRun()
{
    return Start.isHeld();
}

bool GamePadTwoJoysticks::getIntake()
{
    return Y.update(mController.getButton(XBoxController::Button::Y));
}

bool GamePadTwoJoysticks::getCancel()
{
    //RB_Multi.update(mController.getButton(XBoxController::Button::RB));
    return RB_Multi.holdStarted();
}


bool GamePadTwoJoysticks::isTurretJogging()
{
    return !(util.epsilonEquals(mController.getJoystick(XBoxController::Side::RIGHT, XBoxController::Axis::x), 0.0, kDeadband));
}

std::shared_ptr<Rotation2D> GamePadTwoJoysticks::getTurretCardinal()
{
    int dPad = mController.getDPad();
    frc::SmartDashboard::PutNumber("DPad", dPad);
    if (dPad == -1)
    {
        dPadValid = false;
        prev_dpad = dPad;
        return Rotation2D::fromDegrees(0.0);
    } 
    
    bool dPadUpdate = mDPadValid.update(dPad == prev_dpad, mDPadDelay);
    if (mDPadUpdate.update(dPadUpdate))
    {
        dPadValid = true;
        mTurretCardinalOutput = util.convertTurretAngle((double) dPad);
    } else
    {
        dPadValid = false;
    }
    
    prev_dpad = dPad;
    return Rotation2D::fromDegrees(mTurretCardinalOutput);
    
}

bool GamePadTwoJoysticks::getValidTurretCardinal()
{
    return dPadValid;
}

bool GamePadTwoJoysticks::getAutoAim()
{
    if (Back.update(mController.getButton(ControlBoard::XBoxController::BACK)))
    {
        AutoAim = !AutoAim;
    }
    return AutoAim;
}

bool GamePadTwoJoysticks::getFieldRelative()
{
    if (LB_Multi.holdStarted())
    {
        FieldRelative = !FieldRelative;
    }
    return FieldRelative;
}

void GamePadTwoJoysticks::reset() {}

double GamePadTwoJoysticks::getHood()
{
    double hood = mController.getJoystick(XBoxController::Side::RIGHT, XBoxController::Axis::y);
    if (util.epsilonEquals(hood, 0.0, kDeadband))
    {
        return 0.0;
    }
    double pre_hood = std::fabs(hood);
    //rescale
    double adj_hood = std::copysign((pre_hood-kDeadband)/ (1.0-kDeadband), hood);
    return adj_hood;
}

double GamePadTwoJoysticks::getShooter()
{
    double shooter = mController.getJoystick(XBoxController::Side::LEFT, XBoxController::Axis::x);
    if (util.epsilonEquals(shooter, 0.0, kDeadband))
    {
        return 0.0;
    }
    double pre_shooter = std::fabs(shooter);
    //rescale
    double adj_shooter = std::copysign((pre_shooter-kDeadband)/ (1.0-kDeadband), shooter);
    return adj_shooter;
}

double GamePadTwoJoysticks::getBallPath()
{
    double path = mController.getJoystick(XBoxController::Side::LEFT, XBoxController::Axis::y);
    if (util.epsilonEquals(path, 0.0, kDeadband))
    {
        return 0.0;
    }
    
    return path;
}

double GamePadTwoJoysticks::getTurretJog()
{
    double jog = mController.getJoystick(XBoxController::Side::RIGHT, XBoxController::Axis::x);
    if (util.epsilonEquals(jog, 0.0, kDeadband))
    {
        return 0.0;
    }
    double pre_jog = std::fabs(jog);
    //rescale
    double adj_jog = std::copysign((pre_jog-kDeadband)/ (1.0-kDeadband), jog);
    return Constants::kTurretJogMultiplier * (std::pow(adj_jog, Constants::kTurretJogPower));
}

bool GamePadTwoJoysticks::getDriveShifterManual()
{
    bool manual = ManualShiftToggle.update(mLeftJoystick.GetRawButton(3));
    if (manual && !wantsManual)
    { //want manual shift
        wantsManual = true;
    
    } else if (manual && wantsManual)
    { //want auto shift
        wantsManual = false;
    }
    return wantsManual;
}

bool GamePadTwoJoysticks::getCloseShoot()
{
    return LT_Multi.holdStarted();
}

bool GamePadTwoJoysticks::getLineShoot()
{
    return RT_Multi.holdStarted();
}