/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "AutoModeSelector.h"
#include "Auto/Creators/PIDTuningCreator.h"
#include "Auto/Creators/DriveCharacterizationCreator.h"
#include "Auto/Creators/CompetitionCreators/DoNothingCreator.h"
#include "Auto/Creators/MotionProfileTestingCreators/StraightCreator.h"
#include "Auto/Creators/MotionProfileTestingCreators/SwerveCreator.h"
#include "Auto/Creators/MotionProfileTestingCreators/SwerveAngledCreator.h"

AutoModeSelector::AutoModeSelector() {
    mModeChooser.SetDefaultOption("Competition", AutoModeSelector::COMPETITION);
    mModeChooser.AddOption("PID Tuning", AutoModeSelector::PID_TUNING);
    mModeChooser.AddOption("Drive Characterization", AutoModeSelector::DRIVE_CHARACTERIZATION);
    mModeChooser.AddOption("Motion Profile Testing", AutoModeSelector::MOTION_PROFILE_TESTING);
    frc::SmartDashboard::PutData("Auto Mode", &mModeChooser);

    mCompetitionModeChooser.SetDefaultOption("Do Nothing", AutoModeSelector::DO_NOTHING);
    frc::SmartDashboard::PutData("Competition Mode", &mCompetitionModeChooser);

    mTestingModeChooser.SetDefaultOption("Straight Test", AutoModeSelector::STRAIGHT_TEST);
    mTestingModeChooser.AddOption("Swerve Test", AutoModeSelector::SWERVE_TEST);
    mTestingModeChooser.AddOption("Swerve Angled Test", AutoModeSelector::SWERVE_ANGLED_TEST);
    frc::SmartDashboard::PutData("Testing Mode", &mTestingModeChooser);

    mStartPositionChooser.SetDefaultOption("Right", AutoModeSelector::RIGHT);
    mStartPositionChooser.AddOption("Center", AutoModeSelector::CENTER);
    mStartPositionChooser.AddOption("Left", AutoModeSelector::LEFT);
    frc::SmartDashboard::PutData("Starting Position", &mStartPositionChooser);
}

void AutoModeSelector::updateModeCreator(){
    StartingPosition position = mStartPositionChooser.GetSelected();
    Mode mode = mModeChooser.GetSelected();
    CompetitionMode competition = mCompetitionModeChooser.GetSelected();
    TestingMode testing = mTestingModeChooser.GetSelected();

    if (mCachedMode != mode || mCachedTestingMode != testing || mStartingPosition != position || mCachedCompetitionMode != competition){
        
        mCreator = getCreatorForParams(mode, competition, testing, position);
        cout<<"Auto selection changed, updating creator, Mode: "+mMode+ ", Starting Position: "+mPosition<<endl;
    }
    mStartingPosition=position;
    mCachedMode=mode;
    mCachedCompetitionMode=competition;
    mCachedTestingMode=testing;
}

void AutoModeSelector::reset(){
    mCreator=make_shared<AutoModeCreator>();
    mCachedMode=Mode::NONE1;
    mCachedCompetitionMode=CompetitionMode::NONE2;
    mCachedTestingMode=TestingMode::NONE3;
}

shared_ptr<AutoModeBase> AutoModeSelector::getAutoMode(bool left){
    return mCreator->getStateDependentAutoMode(left);
}

shared_ptr<AutoModeCreator> AutoModeSelector::getCreatorForParams(Mode mode, CompetitionMode competition, TestingMode testing, StartingPosition position){
    bool startOnLeft= position==StartingPosition::LEFT;
    switch (position){
        case LEFT:
         mPosition="Left";
         break;
        case CENTER:
         mPosition="Center";
         break;
        case RIGHT:
         mPosition="Right";
         break;
    }
    
    switch (mode){
        case COMPETITION:
        
        switch (competition){
            case DO_NOTHING:
             mMode="Competition->DoNothing";
             return make_shared<DoNothingCreator>();
            break;
            default:
             cout<<"Invalid Competition Mode"<<endl;
             return make_shared<DoNothingCreator>();
             break;
        }
        break;
        case MOTION_PROFILE_TESTING:
         
        switch (testing){
            case STRAIGHT_TEST:
             mMode="Motion Profile Testing->StraightTest";
             return make_shared<StraightCreator>();
             break;
            case SWERVE_TEST:
             mMode="Motion Profile Testing->SwerveAngledTest";
             return make_shared<SwerveCreator>(startOnLeft);
             break;
            case SWERVE_ANGLED_TEST:
             mMode="Motion Profile Testing->SwerveAngledTest";
             return make_shared<SwerveAngledCreator>(startOnLeft);
             break;
            default:
             cout<<"Invalid Motion Profile Testing Mode"<<endl;
             return make_shared<DoNothingCreator>();
             break;
        }
        break;
        case PID_TUNING:
         mMode="PID Tuning";
         return make_shared<PIDTuningCreator>();
         break;
        case DRIVE_CHARACTERIZATION:
         mMode="Drive Characterization";
         return make_shared<DriveCharacterizationCreator>();
         break;
        default:
         cout<<"Invalid Mode"<<endl;
         return make_shared<DoNothingCreator>();
         break;

    }
}
