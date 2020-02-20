/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include "lib/Util/CSVWriter.h"

Util Robot::util;
Units Robot::units;
DriveAssist Robot::driveAssist;

Kinematics Robot::kinematics;
AutoModeSelector Robot::autoModeSelector;

shared_ptr<Subsystems::Drive> Robot::drive(new Subsystems::Drive());
shared_ptr<Subsystems::RobotStateEstimator> Robot::robotStateEstimator(new Subsystems::RobotStateEstimator());

TrajectoryGenerator Robot::trajectoryGenerator;





void Robot::RobotInit() {
  
  mBallPathBottom = Subsystems::BallPathBottom::getInstance();
  
  mBallPathTop = Subsystems::BallPathTop::getInstance();
  mCenteringIntake = Subsystems::CenteringIntake::getInstance();
  mHood = Subsystems::Hood::getInstance();
  mInfrastructure = Subsystems::Infrastructure::getInstance();
  mLED = Subsystems::LED::getInstance();
  mLimelightManager = Subsystems::LimelightManager::getInstance();
  mShooter = Subsystems::Shooter::getInstance();
  mSuperstructure = Subsystems::Superstructure::getInstance();
  mTurret = Subsystems::Turret::getInstance();
  
  mRobotState = FRC_7054::RobotState::getInstance();
  
  //Controls
  if (Constants::kJoystickOne)
  {
    mControlBoard = std::make_shared<ControlBoard::SingleGamePadController>();
    prev_controller_one = true;
  } else
  {
    mControlBoard = std::make_shared<ControlBoard::TwoGamePadControllers>();
    prev_controller_one = false;
  }

  mTurret->zeroSensors();
  mHood->zeroSensors();
  
  //auto camera0=frc::CameraServer::GetInstance()->StartAutomaticCapture(0);

  //has to be in Initialization because in RobotState constructor, it would cause a crash (RobotState constructor before drive constructor).
  //robot Starts forward
  drive->setHeading(make_shared<Rotation2D>());
  mRobotState->reset();

  mLimelightManager->setAllLEDS(Subsystems::Limelight::LedMode::OFF);
 
  //shared_ptr<CharacterizeHighGear> characterizeHighGear=make_shared<CharacterizeHighGear>();
  //mAutoModeExecutor= make_shared<AutoModeExecutor>(characterizeHighGear);
  
  //shared_ptr<PIDTuningMode> tuningMode = make_shared<PIDTuningMode>();
  //mAutoModeExecutor= make_shared<AutoModeExecutor>(tuningMode);
  shared_ptr<AutoModeBase> baseMode = make_shared<AutoModeBase>();
  mAutoModeExecutor= make_shared<AutoModeExecutor>(baseMode);
  
  autoModeSelector.updateModeCreator();
  
  //add subsystem loops to vector
  
  subsystems.push_back(drive);
  subsystems.push_back(robotStateEstimator);
  subsystems.push_back(mBallPathBottom);
  subsystems.push_back(mBallPathTop);
  subsystems.push_back(mCenteringIntake);
  subsystems.push_back(mHood);
  subsystems.push_back(mShooter);
  subsystems.push_back(mTurret);
  subsystems.push_back(mSuperstructure);
  
  subsystems.push_back(mInfrastructure);

  subsystems.push_back(mLimelightManager);
  subsystems.push_back(mLED);
  
  //m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  //m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  //generate Trajectories for auto
  frc::SmartDashboard::PutNumber("Subsystems/Ball Path Top/ get Ball Count: ", 0.0);
  trajectoryGenerator.generateTrajectories();
  std::cout<<"generated Trajectories"<<std::endl;
  //create and start subsystem loops
  mSubsystemLoops=make_shared<Looper>(subsystems);

  mSubsystemLoops->resetSensors();
  std::cout<<"reset sensors"<<std::endl;
}


void Robot::RobotPeriodic() {
  #ifndef CompetitionBot
  int count = (int) frc::SmartDashboard::GetNumber("Subsystems/Ball Path Top/ get Ball Count: ", 0.0);
  mBallPathTop->SetBallCount(count);
  #endif
  mRobotState->outputToSmartDashboard();
  //std::cout<<"RobotState: output to SmartDashboard"<<std::endl;
  bool controller_one = frc::SmartDashboard::GetBoolean("One Controller?", true);
  if (controller_one && controller_one != prev_controller_one)
  {
    mControlBoard = std::make_shared<ControlBoard::SingleGamePadController>();
  } else if (!controller_one && controller_one != prev_controller_one)
  {
    mControlBoard = std::make_shared<ControlBoard::TwoGamePadControllers>();
  }
  prev_controller_one = controller_one;
  //std::cout<<"ControlBoard: One or Two Controllers"<<std::endl;
}

void Robot::DisabledInit(){
  mSubsystemLoops->stopEnabledLoops();
  mSubsystemLoops->startDisabledLoops();
  mAutoModeExecutor->stop();
  autoModeSelector.reset();
  autoModeSelector.updateModeCreator();
}

void Robot::DisabledPeriodic(){
  autoModeSelector.updateModeCreator();

  shared_ptr<AutoModeBase> autoMode = autoModeSelector.getAutoMode(true); //add autofieldstate
  if (autoMode->getID()!= mAutoModeExecutor->getAutoMode()->getID()){
    cout<<"Setting AutoMode to: "+ autoMode->getID()<<endl;
    mAutoModeExecutor->setAutoMode(autoMode);
  }
}

void Robot::AutonomousInit() {
  mSubsystemLoops->resetSensors();
  mSubsystemLoops->stopDisabledLoops();
  mSubsystemLoops->startEnabledLoops();
 /*
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
  //execute auto mode
  */

  mAutoModeExecutor->start();
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  mSubsystemLoops->stopDisabledLoops();
  mSubsystemLoops->startEnabledLoops();
  mAutoModeExecutor->stop();
  
}

void Robot::TeleopPeriodic() {
  manualControl();
}

void Robot::manualControl()
{
  double throttle = mControlBoard->getThrottle();
  double turn = mControlBoard->getTurn();
  double turretJog = mControlBoard->getTurretJog();
  double cardinalDegrees = mControlBoard->getTurretCardinal().inputDirection->getDegrees();
  double ballShootCount = mControlBoard->getBallShootCount(preshoot);

  frc::SmartDashboard::PutNumber("CheckPoint/ ControlBoard/ getThrottle()", throttle);
  frc::SmartDashboard::PutNumber("CheckPoint/ ControlBoard/ getTurn()", turn);
  frc::SmartDashboard::PutNumber("CheckPoint/ ControlBoard/ getTurretJog()", turretJog);
  frc::SmartDashboard::PutNumber("CheckPoint/ ControlBoard/ getTurretCardinal()", cardinalDegrees);
  frc::SmartDashboard::PutNumber("CheckPoint/ ControlBoard/ getBallShoot()", ballShootCount);

  bool quickTurn = mControlBoard->getQuickTurn();
  bool wantsLowGear = mControlBoard->getWantsLowGear();
  bool shoot = mControlBoard->getShoot();
  bool wheel = mControlBoard->getWheel();
  bool wantsRotation = mControlBoard->getWantsRotation();
  bool climber = mControlBoard->getClimber();
  bool intake = mControlBoard->getIntake();
  bool cancel = mControlBoard->getCancel();
  bool isTurretJogging = mControlBoard->isTurretJogging();
  bool autoAim = mControlBoard->getAutoAim();

  frc::SmartDashboard::PutBoolean("CheckPoint/ ControlBoard/ getQuickTurn()", quickTurn);
  frc::SmartDashboard::PutBoolean("CheckPoint/ ControlBoard/ getWantsLowGear()", wantsLowGear);
  frc::SmartDashboard::PutBoolean("CheckPoint/ ControlBoard/ getShoot()", shoot);
  frc::SmartDashboard::PutBoolean("CheckPoint/ ControlBoard/ getWheel()", wheel);
  frc::SmartDashboard::PutBoolean("CheckPoint/ ControlBoard/ getWantsRotation()", wantsRotation);
  frc::SmartDashboard::PutBoolean("CheckPoint/ ControlBoard/ getClimber()", climber);
  frc::SmartDashboard::PutBoolean("CheckPoint/ ControlBoard/ getIntake()", intake);
  frc::SmartDashboard::PutBoolean("CheckPoint/ ControlBoard/ getCancel()", cancel);
  frc::SmartDashboard::PutBoolean("CheckPoint/ ControlBoard/ isTurretJogging()", isTurretJogging);
  frc::SmartDashboard::PutBoolean("CheckPoint/ ControlBoard/ getAutoAim()", autoAim);

  
  double timestamp = frc::Timer::GetFPGATimestamp();
  //bool clear = mControlBoard->getCancel(); - set above
  
  //Drive
  drive->setHighGear(!wantsLowGear);
  std::shared_ptr<DriveSignal> signal = driveAssist.Drive(throttle, turn, quickTurn, drive->isHighGear());
  drive->setOpenLoop(signal);

  //Shoot
  //Only can be changed during preshoot otherwise balls_to_shoot should equal 5.0;
  double balls_to_shoot = mControlBoard->getBallShootCount(preshoot);
  
  
  if ( balls_to_shoot != prev_ball)
  {
    std::cout <<"Balls: setting number of balls to shoot: "<<balls_to_shoot <<std::endl;
    mSuperstructure->setGoalNumBalls(balls_to_shoot);
    i++;
    frc::SmartDashboard::PutNumber("CheckPoint/ Balls to Shoot counter", i);
    frc::SmartDashboard::PutNumber("CheckPoint/ Balls to Shoot number", balls_to_shoot);
  }

  prev_ball = balls_to_shoot;
  //if idle not shooting or preshooting
  if (mSuperstructure->getShooterState() == StateMachines::SuperstructureStateMachine::SystemState::IDLE && (shooting || preshoot))
  {
    preshoot = false;
    shooting = false;
    j++;
    frc::SmartDashboard::PutNumber("CheckPoint/ Shooter State Idle", j);
  }

  
  frc::SmartDashboard::PutBoolean("CheckPoint/ preshoot ", preshoot);
  frc::SmartDashboard::PutBoolean("CheckPoint/ shooting", shooting);
  
  //set shooting
  frc::SmartDashboard::PutBoolean("CheckPoint/ Superstructure at Desired State", mSuperstructure->isAtDesiredState());
  if (shoot && preshoot && !shooting && mSuperstructure->isAtDesiredState()) 
  {
    std::cout <<"Starting shooting: "<<timestamp <<std::endl;
    mSuperstructure->setWantedActionShooter(StateMachines::SuperstructureStateMachine::WANTED_EXHAUST_BALL);
    preshoot = false;
    shooting = true;
    shoot_start_time = timestamp;
    l++;
    frc::SmartDashboard::PutNumber("CheckPoint/ Shooting counter", l);
    frc::SmartDashboard::PutNumber("CheckPoint/ Shoot Timestamp", shoot_start_time);
  }

  //set preshooting
  if (shoot && !shooting)
  {
    std::cout <<"Starting Preshoot: "<<timestamp <<std::endl;
    mSuperstructure->setWantedActionShooter(StateMachines::SuperstructureStateMachine::WANTED_PRE_EXHAUST_BALL);
    preshoot = true;
    pre_shoot_start_time = timestamp;
    k++;
    frc::SmartDashboard::PutNumber("CheckPoint/ PreShooting counter", k);
    frc::SmartDashboard::PutNumber("CheckPoint/ PreShooting Timestamp", pre_shoot_start_time);

    
  }

  //cancel shooting action
  if (cancel && (shooting || preshoot))
  {
    std::cout <<"Canceling shooting: "<<timestamp <<std::endl;
    mSuperstructure->setWantedActionShooter(StateMachines::SuperstructureStateMachine::WANTED_IDLE);
    shooting = false;
    preshoot = false;
    m++;
    frc::SmartDashboard::PutNumber("CheckPoint/ Cancel counter", m);
  }

  //Turret

  //Jog turret-auto aiming disabled, need to add: getAutoAim, field relative, robot relative,
  //currently only jogging
  if (isTurretJogging)
  {
    std::cout <<"Updating jog: "<<mControlBoard->getTurretJog() <<std::endl;
    mSuperstructure->jogTurret(mControlBoard->getTurretJog());
    jogging = true;

    n++;
    frc::SmartDashboard::PutNumber("CheckPoint/ Turret Jog counter", n);
    frc::SmartDashboard::PutNumber("CheckPoint/ Turret Jog number", mControlBoard->getTurretJog());
  } else
  {
    //mSuperstructure->setWantAutoAim(Rotation2D::fromDegrees(180.0));
    jogging = false;
  }

  //wheel - later

  //Climb - later

  //Intake
  if (intake)
  {
      if (!intake_extended)
    { //extend intake
      std::cout <<"Extending Intake: "<<timestamp <<std::endl;
      mSuperstructure->setWantedActionIntake(StateMachines::SuperstructureStateMachine::WANTED_INTAKE_BALL);

      o++;
      frc::SmartDashboard::PutNumber("CheckPoint/ intake extend counter", o);
     
    } else
    {
      if (util.epsilonEquals(mBallPathTop->getBallCount(), 0.0))
      { //idle
        std::cout <<"Closing Intake-IDLE: "<<timestamp <<std::endl;
        mSuperstructure->setWantedActionIntake(StateMachines::SuperstructureStateMachine::WANTED_IDLE);
      } else
      {  //have balls
        std::cout <<"Closing Intake-HAVE_BALLS: "<<timestamp <<std::endl;
        mSuperstructure->setWantedActionIntake(StateMachines::SuperstructureStateMachine::WANTED_HAVE_BALLS);
      }
      //intake_extended = mSuperstructure->()
      p++;
      frc::SmartDashboard::PutNumber("CheckPoint/ intake extend counter", p);
    }
  }
  intake_extended = mSuperstructure->isIntakeExtended();
  frc::SmartDashboard::PutBoolean("CheckPoint/ intake extend number", intake_extended);
  
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
