/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Modes/CompetitionModes/SimpleMode.h"

SimpleMode::SimpleMode() 
{
    mShoot = std::make_shared<Shoot>();
    mWait = std::make_shared<WaitAction>(8.0);
    mCancelShoot = std::make_shared<CancelShoot>();
    mOpenLoopDrive = std::make_shared<OpenLoopDrive>(-.75, -.75, 2.0, false);

    std::vector<std::shared_ptr<Action>> parallel{mShoot, mWait};
    mParallel = std::make_shared<ParallelAction>(parallel);

    std::vector<std::shared_ptr<Action>> series{mParallel, mCancelShoot, mOpenLoopDrive};
    mSeries = std::make_shared<SeriesAction>(series);
}

void SimpleMode::routine()
{
    runAction(mSeries);
}

std::string SimpleMode::getID()
{
    return "SimpleMode";
}