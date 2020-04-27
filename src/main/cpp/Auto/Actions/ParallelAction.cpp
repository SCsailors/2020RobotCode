/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Actions/ParallelAction.h"

ParallelAction::ParallelAction(vector<shared_ptr<Action>> actions) {
    mActions=actions;
}

bool ParallelAction::isFinished(){
    for(shared_ptr<Action> action: mActions){
       if( !action->isFinished()){
           return false;
       }
    }
    return true;
}

void ParallelAction::update(){
    for(shared_ptr<Action> action: mActions){
        action->update();
    }
}

void ParallelAction::done(){
    for(shared_ptr<Action> action: mActions){
        action->done();
    }
}

void ParallelAction::start(){
    for(shared_ptr<Action> action: mActions){
        action->start();
    }
}