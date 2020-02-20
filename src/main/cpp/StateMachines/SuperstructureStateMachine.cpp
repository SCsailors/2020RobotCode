/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "StateMachines/SuperstructureStateMachine.h"
#include "Subsystems/Superstructure.h"
#include "Constants.h"
#include "States/TimedLEDState.h"

using namespace StateMachines;
SuperstructureStateMachine::SuperstructureStateMachine() 
{
    mLED = Subsystems::LED::getInstance();
}

SuperstructureState SuperstructureStateMachine::updateShooting(double timestamp, WantedAction wantedAction, SuperstructureState currentState, double range)
{
    mDesiredShooterState.reset(); //sets it to zero, so previous states don't interfere
    SystemState newState = mSystemShooterState;
    double timeInState = timestamp - mCurrentStateStartTimeShooter;

    switch (mSystemShooterState) {
        case IDLE:
            newState = handleIdleStateTransitions(wantedAction);
            break;
        case INTAKING_BALL:
            newState = handleIntakingBallStateTransitions(currentState, timestamp, timeInState, wantedAction);
            break;
        case PRE_EXHAUSTING_BALL:
            newState = handlePreExhaustingBallStateTransitions(currentState, timestamp, timeInState, wantedAction);
            break;
        case EXHAUSTING_BALL:
            newState = handleExhaustingBallStateTransitions(currentState, timestamp, timeInState, wantedAction);
            break;
        case HAVE_BALLS:
            newState = handleHaveBallStateTransitions(currentState, timestamp, timeInState, wantedAction);
            break;
        default:
            std::cout << "Unexpected Superstructure system state: " << mSystemShooterState << std::endl;
            newState = mSystemShooterState;
    }

    if (newState != mSystemShooterState)
    {
        std::cout << timestamp << ": Superstructure Changed State: " << mSystemShooterState << " -> " << newState <<std::endl;
        mSystemShooterState = newState;
        mCurrentStateStartTimeShooter = frc::Timer::GetFPGATimestamp();
        timeInState = 0.0;
    }

    switch (mSystemShooterState) {
        
    case IDLE:
        getIdleDesiredState(currentState, timeInState, mDesiredShooterState, true);
        break;
    case INTAKING_BALL:
        getIntakingBallDesiredState(currentState, mDesiredShooterState);
        break;
    case PRE_EXHAUSTING_BALL:
        getPreExhaustingBallDesiredState(currentState, timeInState, mDesiredShooterState);
        break;   
    case EXHAUSTING_BALL:
        getExhaustingBallDesiredState(currentState, mDesiredShooterState);
        break; 
    case HAVE_BALLS:
        getHaveBallDesiredState(currentState, timeInState, mDesiredShooterState);
        break;
    default:
        std::cout << "Unexpected Superstructure system state: " << mSystemShooterState << std::endl;
        break;
    }
    return mDesiredShooterState;
    
}

SuperstructureState SuperstructureStateMachine::updateIntaking(double timestamp, WantedAction wantedAction, SuperstructureState currentState)
{
    
    mDesiredIntakeState.reset(); //sets it to zero, so previous states don't interfere

    SystemState newState = mSystemIntakeState;
    double timeInState = timestamp - mCurrentStateStartTimeIntake;

    switch (mSystemIntakeState) {
        case IDLE:
            newState = handleIdleStateTransitions(wantedAction);
            break;
        case INTAKING_BALL:
            newState = handleIntakingBallStateTransitions(currentState, timestamp, timeInState, wantedAction);
            break;
        case PRE_EXHAUSTING_BALL:
            newState = handlePreExhaustingBallStateTransitions(currentState, timestamp, timeInState, wantedAction);
            break;
        case EXHAUSTING_BALL:
            newState = handleExhaustingBallStateTransitions(currentState, timestamp, timeInState, wantedAction);
            break;
        case HAVE_BALLS:
            newState = handleHaveBallStateTransitions(currentState, timestamp, timeInState, wantedAction);
            break;
        default:
            std::cout << "Unexpected Superstructure system state: " << mSystemIntakeState << std::endl;
            newState = mSystemIntakeState;
    }

    if (newState != mSystemIntakeState)
    {
        std::cout << timestamp << ": Changed State: " << mSystemIntakeState << " -> " << newState <<std::endl;
        mSystemIntakeState = newState;
        mCurrentStateStartTimeIntake = frc::Timer::GetFPGATimestamp();
        timeInState = 0.0;
    }

    switch (mSystemIntakeState) {
        
    case IDLE:
        getIdleDesiredState(currentState, timeInState, mDesiredIntakeState, false);
        break;
    case INTAKING_BALL:
        getIntakingBallDesiredState(currentState, mDesiredIntakeState);
        break;
    case PRE_EXHAUSTING_BALL:
        getPreExhaustingBallDesiredState(currentState, timeInState, mDesiredIntakeState);
        break;   
    case EXHAUSTING_BALL:
        getExhaustingBallDesiredState(currentState, mDesiredIntakeState);
        break; 
    case HAVE_BALLS:
        getHaveBallDesiredState(currentState, timeInState, mDesiredIntakeState);
        break;
    default:
        std::cout << "Unexpected Superstructure system state: " << mSystemIntakeState << std::endl;
        break;
    }
    return mDesiredIntakeState;
    
}

SuperstructureGoal SuperstructureStateMachine::mergedShootingIntaking(double timestamp, WantedAction wantedActionShooter, WantedAction wantedActionIntake, SuperstructureState currentState, double range)
{
    mRangeInches = range;
    changedIntake = false;
    changedWheelie = false;
    
    SuperstructureState tmpShooter = updateShooting(timestamp, wantedActionShooter, currentState, mRangeInches);
    SuperstructureState tmpIntaking = updateIntaking(timestamp, wantedActionIntake, currentState);

    mDesiredGoal.state.ballPathBottom = resolveState(tmpShooter.ballPathBottom, tmpIntaking.ballPathBottom, currentState.ballPathBottom);
    mDesiredGoal.state.ballPathTop = resolveState(tmpShooter.ballPathTop, tmpIntaking.ballPathTop, currentState.ballPathTop);
    mDesiredGoal.state.centeringIntake = resolveState(tmpShooter.centeringIntake, tmpIntaking.centeringIntake, currentState.centeringIntake);
    mDesiredGoal.state.hood = resolveState(tmpShooter.hood, tmpIntaking.hood, currentState.hood);
    mDesiredGoal.state.shooter = resolveState(tmpShooter.shooter, tmpIntaking.shooter, currentState.shooter);
    mDesiredGoal.state.turret = resolveState(tmpShooter.turret, tmpIntaking.turret, currentState.turret); //should be currentState.turret
    mDesiredGoal.state.numBalls = resolveState(tmpShooter.numBalls, tmpIntaking.numBalls, currentState.numBalls); // should be currentState.numBalls
    
    if (changedIntake)
    {
        mDesiredGoal.state.extendIntake = resolveState(tmpIntaking.extendIntake, tmpShooter.extendIntake);
    } else 
    {
        mDesiredGoal.state.extendIntake = currentState.extendIntake;
    }

    if (changedWheelie)
    {
        mDesiredGoal.state.extendWheelieBar = resolveState(tmpIntaking.extendWheelieBar, tmpShooter.extendWheelieBar);
    } else
    {
        mDesiredGoal.state.extendWheelieBar = currentState.extendWheelieBar;
    }
    frc::SmartDashboard::PutBoolean("SuperstructureStateMachine/ shooter priority: ", shooterPriority);
    frc::SmartDashboard::PutBoolean("SuperstructureStateMachine/ intake priority: ", intakePriority);
    return mDesiredGoal;
}

double SuperstructureStateMachine::resolveState(double shooterState, double intakeState, double currentState)
{
    if (shooterPriority)
    {
        return shooterState;
    }
    if (intakePriority)
    {
        return intakeState;
    }

    if (std::isnan(shooterState) && std::isnan(intakeState))
    {
        return currentState;
    } else if (std::isnan(shooterState))
    {
        return intakeState;
    } else if (std::isnan(intakeState))
    {
        return shooterState;
    } else if (util.epsilonEquals(shooterState, intakeState, .4) && util.epsilonEquals(shooterState, currentState, .4))
    {//states are the same
        return currentState;
    }
    else
    {
        //std::cout << "Not sure which one to return! Shooter State: " << shooterState <<" Intake State: " <<intakeState <<" CurrentState: " << currentState <<std::endl;  
        return currentState;
    }
    
}

bool SuperstructureStateMachine::resolveState(bool s1, bool s2)
{
    if (s1 || s2)
    {
        return true;
    } else
    {
        return false;
    }
    
}


SuperstructureStateMachine::SystemState SuperstructureStateMachine::defaultTransition(WantedAction wantedAction)
{
    switch (wantedAction) {
        case WantedAction::WANTED_IDLE:
            return SystemState::IDLE;
        case WantedAction::WANTED_PRE_EXHAUST_BALL:
            return SystemState::PRE_EXHAUSTING_BALL;
        case WantedAction::WANTED_EXHAUST_BALL:
            return SystemState::EXHAUSTING_BALL;
        case WantedAction::WANTED_INTAKE_BALL:
            return SystemState::INTAKING_BALL;
        case WantedAction::WANTED_HAVE_BALLS:
            return SystemState::HAVE_BALLS;
    }
    return SystemState::IDLE;
}

//IDLE
SuperstructureStateMachine::SystemState SuperstructureStateMachine::handleIdleStateTransitions(WantedAction wantedAction)
{
    return defaultTransition(wantedAction);
}

void SuperstructureStateMachine::getIdleDesiredState(SuperstructureState currentState, double timeInState, SuperstructureState &desiredState, bool shooter)
{ //No Balls and no demands
    if (shooter)
    { //shooter
    
    preExhausting = false;
    mLEDPriority = false;

    desiredState.ballPathBottom = NAN;
    desiredState.ballPathTop = NAN;
    desiredState.centeringIntake = NAN;
    desiredState.hood = 0.0; //set to home
    desiredState.shooter = 0.0; //zero velocity
    desiredState.turret = currentState.turret; //pass it through
    desiredState.numBalls = currentState.numBalls; //pass it through
    desiredState.extendWheelieBar = false; //shouldn't be extended, but when merged, could be true
    
    changedWheelie = true;
    shooterPriority = false;
    
    if (doneShooting && timeInState < 1.0)
    { //set to shooting complete if it transitions from exhausting to idle
        mLED->setShootLEDState(mLEDStates.kShootingComplete);
    } else
    {//set everything to off, but don't change the wanted action, that will be handled by climber or wheel if it needs to be changed
        mLED->setShootLEDState(mLEDStates.kIdle);
        doneShooting = false;
    }
    } else
    { //intake
    desiredState.ballPathBottom = 0.0;
    desiredState.ballPathTop = 0.0;
    desiredState.centeringIntake = 0.0;
    desiredState.hood = NAN;
    desiredState.shooter = NAN;
    desiredState.turret = currentState.turret;
    desiredState.numBalls = currentState.numBalls;
    desiredState.extendIntake = false; //shouldn't be extended, but when merged, could be true
    
    changedIntake = true;

    intakePriority = false;

    //set everything to off, but don't change the wanted action, that will be handled by climber or wheel if it needs to be changed
    mLED->setIntakeLEDState(mLEDStates.kIdle);
    mLED->setBallLEDState(mLEDStates.kIdle);
    
    }
}

//INTAKING_BALL
SuperstructureStateMachine::SystemState SuperstructureStateMachine::handleIntakingBallStateTransitions(SuperstructureState currentState, double timestamp, double timeInState, WantedAction wantedAction)
{   
    //check against time limits and ball count
    if (timeInState < Constants::kMinIntaking)
    {
        return SystemState::INTAKING_BALL;
    }
    //Close intake if 5 balls
    if (currentState.hasFiveBalls())
    {
        std::cout <<"Closing intake: 5 balls!"<<std::endl;
        return SystemState::HAVE_BALLS;
    }
    
    //Close intake if timed out and have no balls
    if (!currentState.hasBalls() && timeInState > Constants::kMaxIntaking)
    {
        std::cout <<"Closing intake: timed out and no balls!"<<std::endl;
        return SystemState::IDLE;
    }

    //Close Intake if timed out and have balls
    if (currentState.hasBalls() && timeInState > Constants::kMaxIntaking)
    {
        std::cout <<"Closing intake: timed out and have balls!"<<std::endl;
        return SystemState::HAVE_BALLS;
    }

    return defaultTransition(wantedAction);
}

void SuperstructureStateMachine::getIntakingBallDesiredState(SuperstructureState currentState, SuperstructureState &desiredState)
{
    if (preExhausting)
    { //shouldn't even use these values because of shooter priority, but just in case
        desiredState.ballPathBottom = 0.0;
        desiredState.ballPathTop = 0.0;
        desiredState.centeringIntake = 0.0;
        desiredState.hood = NAN;
        desiredState.shooter = NAN;
        desiredState.turret = currentState.turret;
        desiredState.numBalls = currentState.numBalls;
        desiredState.extendIntake = true; //change to false? It's not running while preExhausting    
        
        changedIntake = true;
    } else 
    {
        desiredState.ballPathBottom = Constants::kBallPathBottomDefaultSpeed;
        desiredState.ballPathTop = Constants::kBallPathTopDefaultSpeed;
        desiredState.centeringIntake = Constants::kCenteringIntakeDefaultSpeed;
        desiredState.hood = NAN;
        desiredState.shooter = NAN;
        desiredState.turret = currentState.turret;
        desiredState.numBalls = currentState.numBalls;
        desiredState.extendIntake = true;
    
        changedIntake = true;
    }
    if (!(mLEDPriority || mLEDMaxPriority))
    {
        mLED->setWantedAction(Subsystems::LED::WantedAction::DISPLAY_INTAKING);
    }
    
    if (util.epsilonEquals(currentState.numBalls, 0.0, .4))
    {
        mLED->setIntakeLEDState(mLEDStates.kIntakeOut);
        frc::SmartDashboard::PutNumber("Intake setLEDState 5 balls: One", mLEDStates.kIntakeOut->mStaticState.PWM);
        frc::SmartDashboard::PutNumber("Intake setLEDState 5 balls: One", mLEDStates.kIntakeOut->mStaticState.PWM);
    } else if (util.epsilonEquals(currentState.numBalls, 1.0, .4))
    {
        mLED->setIntakeLEDState(mLEDStates.kIntakingOne);
    } else if (util.epsilonEquals(currentState.numBalls, 2.0, .4))
    {
        mLED->setIntakeLEDState(mLEDStates.kIntakingTwo);
    } else if (util.epsilonEquals(currentState.numBalls, 3.0, .4))
    {
        mLED->setIntakeLEDState(mLEDStates.kIntakingThree);
    } else if (util.epsilonEquals(currentState.numBalls, 4.0, .4))
    {
        mLED->setIntakeLEDState(mLEDStates.kIntakingFour);        
    } else if (util.epsilonEquals(currentState.numBalls, 5.0, .4))
    {
        mLED->setIntakeLEDState(mLEDStates.kIntakingFive);
    } else 
    {
        std::cout << "Unexpected number of balls (Intaking): " << currentState.numBalls << " : Setting LED to intaking" <<std::endl;
        mLED->setIntakeLEDState(mLEDStates.kIntakeOut);
    }
}

//PRE_EXHAUSTING
SuperstructureStateMachine::SystemState SuperstructureStateMachine::handlePreExhaustingBallStateTransitions(SuperstructureState currentState, double timestamp, double timeInState, WantedAction wantedAction)
{
    //check if we have balls
    if (!currentState.hasBalls())
    {
        std::cout <<"Can't exhaust balls if ball count is zero!"<<std::endl;
        return SystemState::IDLE;
    }

    if (timeInState < Constants::kMinPreExhausting)
    {
        return SystemState::PRE_EXHAUSTING_BALL;
    }

    return defaultTransition(wantedAction);
}

void SuperstructureStateMachine::getPreExhaustingBallDesiredState(SuperstructureState currentState, double timeInState, SuperstructureState &desiredState)
{
    mLEDPriority = true;
    bool preExhausting = true;
    double ShooterSpeed = 0.0;
    switch (mRange){
        case CLOSE:
            ShooterSpeed = Constants::kShooterCloseDefaultSpeed;
            break;
        case MID:
            ShooterSpeed = Constants::kShooterMidDefaultSpeed;
            break;
        case FAR:
            ShooterSpeed = Constants::kShooterFarDefaultSpeed;
            break;
    }

    if (timeInState < Constants::kBallPathBackTime)
    {
        desiredState.ballPathBottom = Constants::kBallPathBackSpeed;
        desiredState.ballPathTop = Constants::kBallPathBackSpeed;
        shooterPriority = true;

        desiredState.centeringIntake = currentState.centeringIntake;
        desiredState.hood = Constants::kHoodDefaultAngle; //will be calculated here
        desiredState.shooter = 0.0;
        desiredState.turret = currentState.turret; //handled by vision
    } else 
    {
        desiredState.ballPathBottom = NAN;
        desiredState.ballPathTop = NAN;
        shooterPriority = false;

        desiredState.centeringIntake = NAN;
        desiredState.hood = Constants::kHoodDefaultAngle; //will be calculated here
        desiredState.shooter = ShooterSpeed;
        desiredState.turret = currentState.turret; //handled by vision
        desiredState.numBalls = currentState.numBalls;
    }

    if (!mLEDMaxPriority)
    {
        mLED->setWantedAction(Subsystems::LED::WantedAction::DISPLAY_SHOOTING);
    }
    
    if (mDesiredGoal.isAtDesiredState(currentState))
    {
        mLED->setShootLEDState(mLEDStates.kShooterReady);
    } else
    {
        mLED->setShootLEDState(mLEDStates.kPreShooting);
    }
    
}

//EXHAUSTING
SuperstructureStateMachine::SystemState SuperstructureStateMachine::handleExhaustingBallStateTransitions(SuperstructureState currentState, double timestamp, double timeInState, WantedAction wantedAction)
{   
    // balls is at the specified number
    if (util.epsilonEquals(currentState.numBalls, ballgoal, .4))
    {
        doneShooting = true;
        std::cout <<"Stopping shooting: reached ball count goal! At: "<<ballgoal<<std::endl;
        return SystemState::IDLE;
    }
    //Out of Balls 
    if (!currentState.hasBalls())
    {
        doneShooting = true;
        std::cout <<"Stopping shooting: out of balls!"<<std::endl;
        return SystemState::IDLE;
    }

    return defaultTransition(wantedAction);
}

void SuperstructureStateMachine::getExhaustingBallDesiredState(SuperstructureState currentState, SuperstructureState &desiredState)
{
    bool preExhausting = false;
    //add look up tables for speed and angle based on range.
    double ShooterSpeed = 0.0;
    switch (mRange){
        case CLOSE:
            ShooterSpeed = Constants::kShooterCloseDefaultSpeed;
            break;
        case MID:
            ShooterSpeed = Constants::kShooterMidDefaultSpeed;
            break;
        case FAR:
            ShooterSpeed = Constants::kShooterFarDefaultSpeed;
            break;
    }
    desiredState.ballPathBottom = Constants::kBallPathShootSpeed;
    desiredState.ballPathTop = Constants::kBallPathShootSpeed;
    desiredState.centeringIntake = currentState.centeringIntake;
    desiredState.hood = Constants::kHoodDefaultAngle; //should just be set in Superstructure.h
    desiredState.shooter = ShooterSpeed;
    desiredState.turret = currentState.turret; //handled else where
    desiredState.numBalls = currentState.numBalls; // handled else where
    desiredState.extendWheelieBar = true;
    shooterPriority = true;

    changedWheelie = true;
    if (!mLEDMaxPriority)
    {
        mLED->setWantedAction(Subsystems::LED::WantedAction::DISPLAY_SHOOTING);
    }
    mLED->setShootLEDState(mLEDStates.kShooting);
    //Goes to idle and idle sets it to flashing once the number of balls is reached.
}

//HAVE_BALL
SuperstructureStateMachine::SystemState SuperstructureStateMachine::handleHaveBallStateTransitions(SuperstructureState currentState, double timestamp, double timeInState, WantedAction wantedAction)
{
    if (util.epsilonEquals(currentState.numBalls, 0.0, .4))
    { //out of balls from shooting
        return SystemState::IDLE;
    }
    return defaultTransition(wantedAction);
}

void SuperstructureStateMachine::getHaveBallDesiredState(SuperstructureState currentState, double timeInState, SuperstructureState &desiredState)
{      
    desiredState.ballPathBottom = 0.0;
    desiredState.ballPathTop = 0.0;
    desiredState.centeringIntake = 0.0;
    desiredState.hood = NAN;
    desiredState.shooter = NAN;
    desiredState.turret = currentState.turret;
    desiredState.numBalls = currentState.numBalls;
    desiredState.extendIntake = false;

    changedIntake = true;
    if (!(mLEDPriority || mLEDMaxPriority))
    {
        mLED->setWantedAction(Subsystems::LED::WantedAction::DISPLAY_BALLS);
    }
    
    if (timeInState < 1.0)
    {
        if (util.epsilonEquals(currentState.numBalls, 0.0, .4))
        {
            mLED->setBallLEDState(mLEDStates.kStowingZero);
        } else if (util.epsilonEquals(currentState.numBalls, 1.0, .4))
        {
            mLED->setBallLEDState(mLEDStates.kStowingOne);
        } else if (util.epsilonEquals(currentState.numBalls, 2.0, .4))
        {
            mLED->setBallLEDState(mLEDStates.kStowingTwo);
        } else if (util.epsilonEquals(currentState.numBalls, 3.0, .4))
        {
            mLED->setBallLEDState(mLEDStates.kStowingThree);
        } else if (util.epsilonEquals(currentState.numBalls, 4.0, .4))
        {
            mLED->setBallLEDState(mLEDStates.kStowingFour);        
        } else if (util.epsilonEquals(currentState.numBalls, 5.0, .4))
        {
            mLED->setBallLEDState(mLEDStates.kStowingFive);
        } else 
        {
            std::cout << "Unexpected number of balls (Have Balls: stowing): " << currentState.numBalls << " : Setting LED to Stowing" <<std::endl;
            mLED->setBallLEDState(mLEDStates.kStowing);
        }   
    } else
    {
        if (util.epsilonEquals(currentState.numBalls, 0.0, .4))
        {
            mLED->setBallLEDState(mLEDStates.kIdle);
        } else if (util.epsilonEquals(currentState.numBalls, 1.0, .4))
        {
            mLED->setBallLEDState(mLEDStates.kOneBall);
        } else if (util.epsilonEquals(currentState.numBalls, 2.0, .4))
        {
            mLED->setBallLEDState(mLEDStates.kTwoBalls);
        } else if (util.epsilonEquals(currentState.numBalls, 3.0, .4))
        {
            mLED->setBallLEDState(mLEDStates.kThreeBalls);
        } else if (util.epsilonEquals(currentState.numBalls, 4.0, .4))
        {
            mLED->setBallLEDState(mLEDStates.kFourBalls);        
        } else if (util.epsilonEquals(currentState.numBalls, 5.0, .4))
        {
            mLED->setBallLEDState(mLEDStates.kFiveBalls);
        } else 
        {
            std::cout << "Unexpected number of balls (Have Balls): " << currentState.numBalls << " : Setting LED to off" <<std::endl;
            mLED->setBallLEDState(mLEDStates.kIdle);
        }   
    }  
}
