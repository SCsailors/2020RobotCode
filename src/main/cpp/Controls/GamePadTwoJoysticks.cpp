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
    return -mRightJoystick.GetY();
}

double GamePadTwoJoysticks::getTurn()
{
    return -mLeftJoystick.GetY();
}

bool GamePadTwoJoysticks::getQuickTurn()
{
    /*
    if (LT_Multi.holdStarted())
    {
        i++;
    }
    frc::SmartDashboard::PutNumber("CheckPoint/ ControlBoard/ quickTurn counter", i);
    //LT_Multi.update(mController.getTrigger(XBoxController::Side::LEFT));
    return LT_Multi.isHeld();
    */
    return false;
}

bool GamePadTwoJoysticks::getWantsHighGear()
{
    bool wantsHigh = HighGear.update(mLeftJoystick.GetRawButton(1));
    if (wantsHigh && !wantsHighGear)
    { //want manual shift
        wantsHighGear = true;
    
    } else if (wantsHigh && wantsHighGear)
    { //want auto shift
        wantsHighGear = false;
    }
    return wantsHighGear;
}

bool GamePadTwoJoysticks::getShoot()
{
    bool a = A.update(mController.getButton(XBoxController::Button::A));
    if (a)
    {
        k++;
    }
    frc::SmartDashboard::PutNumber("CheckPoint/ ControlBoard/ getShoot counter", k);
    return a;
}

bool GamePadTwoJoysticks::getWheel()
{
    return B.update(mController.getButton(XBoxController::Button::B));
}

bool GamePadTwoJoysticks::getWantsRotation()
{
    return Start.update(mController.getButton(XBoxController::Button::START));
}

bool GamePadTwoJoysticks::getClimber()
{
    X.update(mController.getButton(XBoxController::Button::X));
    return X.wasTapped();
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

double GamePadTwoJoysticks::getTurretJog()
{
    double jog = mController.getJoystick(XBoxController::Side::RIGHT, XBoxController::Axis::x);
    if (util.epsilonEquals(jog, 0.0, turretDeadband))
    {
        return 0.0;
    }
    double pre_jog = std::fabs(jog);
    //rescale
    double adj_jog = std::copysign((pre_jog-turretDeadband)/ (1.0-turretDeadband), jog);
    return Constants::kTurretJogMultiplier * (std::pow(adj_jog, Constants::kTurretJogPower));
}

bool GamePadTwoJoysticks::isTurretJogging()
{
    return !(util.epsilonEquals(mController.getJoystick(XBoxController::Side::RIGHT, XBoxController::Axis::x), 0.0, turretDeadband));
}

TurretCardinal GamePadTwoJoysticks::getTurretCardinal()
{
    int dPad = mController.getDPad();
    frc::SmartDashboard::PutNumber("DPad", dPad);
    TurretCardinalEnum newCardinal = dPad == -1 ? TurretCardinalEnum::NONE : TurretCardinalToEnum(findClosest(Rotation2D::fromDegrees((double) dPad)));
    frc::SmartDashboard::PutNumber("NewCardinal degrees", newCardinal);
    if (newCardinal != TurretCardinalEnum::NONE && isDiagonal(newCardinal))
    {
        //Latch previous direction on diagonal presses because D-Pad is bad at diagonals
        newCardinal = mLastCardinal;
    }
    bool valid = mDPadValid.update(frc::Timer::GetFPGATimestamp(), newCardinal != TurretCardinalEnum::NONE && (mLastCardinal == TurretCardinalEnum::NONE || newCardinal == mLastCardinal));
    if (valid)
    {
        if (mLastCardinal == TurretCardinalEnum::NONE)
        {
            mLastCardinal = newCardinal;
        }
        return mLastCardinal;
    } else
    {
        mLastCardinal = newCardinal;
    }
    return TurretCardinalEnum::NONE;
    
}

void GamePadTwoJoysticks::reset()
{
    mLastCardinal = TurretCardinalEnum::NONE;
    mDPadValid.reset(frc::Timer::GetFPGATimestamp(), mDPadDelay);
}

/*
bool GamePadTwoJoysticks::getAutoAim()
{
    return Back.update(mController.getButton(XBoxController::Button::BACK));
}
*/
//put this first
double GamePadTwoJoysticks::getBallShootCount(bool preshoot)
{
    LT_Multi.update(mController.getTrigger(XBoxController::Side::LEFT));
    RT_Multi.update(mController.getTrigger(XBoxController::Side::RIGHT));
    LB_Multi.update(mController.getButton(XBoxController::Button::LB));
    RB_Multi.update(mController.getButton(XBoxController::Button::RB));

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

double GamePadTwoJoysticks::getHood()
{
    double hood = mController.getJoystick(XBoxController::Side::RIGHT, XBoxController::Axis::y);
    if (util.epsilonEquals(hood, 0.0, turretDeadband))
    {
        return 0.0;
    }
    double pre_hood = std::fabs(hood);
    //rescale
    double adj_hood = std::copysign((pre_hood-turretDeadband)/ (1.0-turretDeadband), hood);
    return adj_hood;
}

double GamePadTwoJoysticks::getShooter()
{
    double shooter = mController.getJoystick(XBoxController::Side::LEFT, XBoxController::Axis::y);
    if (util.epsilonEquals(shooter, 0.0, turretDeadband))
    {
        return 0.0;
    }
    double pre_shooter = std::fabs(shooter);
    //rescale
    double adj_shooter = std::copysign((pre_shooter-turretDeadband)/ (1.0-turretDeadband), shooter);
    return adj_shooter;
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

double GamePadTwoJoysticks::getBallPath()
{
    return -mController.getJoystick(ControlBoard::XBoxController::Side::LEFT, ControlBoard::XBoxController::Axis::y);
}

bool GamePadTwoJoysticks::getBallPathToggle()
{
    return LB_Multi.holdStarted();
}

bool GamePadTwoJoysticks::getClimbRun()
{
    return X.isHeld();
}

bool GamePadTwoJoysticks::getCloseShoot()
{
    return LT_Multi.holdStarted();
}

bool GamePadTwoJoysticks::getLineShoot()
{
    return RT_Multi.holdStarted();
}