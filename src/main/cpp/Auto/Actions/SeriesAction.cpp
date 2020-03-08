/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Actions/SeriesAction.h"

SeriesAction::SeriesAction(vector<shared_ptr<Action>> actions) {
    mRemainingActions=actions;
    mCurrAction=NULL;
}

bool SeriesAction::isFinished(){
    return mRemainingActions.empty() && mCurrAction==NULL;
}

void SeriesAction::start(){
    mCurrAction=mRemainingActions.at(0);
    mCurrAction->start();
}

void SeriesAction::update(){
    if(mCurrAction==NULL){
        if(mRemainingActions.empty()){
            return;
        }
        mRemainingActions.erase(mRemainingActions.begin());
        if (!mRemainingActions.empty()){
            mCurrAction = mRemainingActions.at(0);
            mCurrAction->start();
        } else{
            return;
        }
    }

    mCurrAction->update();
    if(mCurrAction->isFinished()){
            mCurrAction->done();
            mCurrAction=NULL;
    }
}

void SeriesAction::done(){}