/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Actions/WaitUntilInsideRegion.h"

WaitUntilInsideRegion::WaitUntilInsideRegion(shared_ptr<Translation2D> bottomLeft, shared_ptr<Translation2D> topRight, bool isOnLeft) {
    if (isOnLeft){
        mBottomLeft=make_shared<Translation2D>(bottomLeft->x(), -bottomLeft->y());
        mTopRight=make_shared<Translation2D>(topRight->x(), -topRight->y());
    }else{
    mBottomLeft=bottomLeft;
    mTopRight=topRight;
    }
    
}

bool WaitUntilInsideRegion::isFinished(){
    shared_ptr<Translation2D> position= Robot::robotState.getLatestFieldToVehicle()->getTranslation();
    return position->x()>mBottomLeft->x() && position->y()>mBottomLeft->y()
        && position->x()<mTopRight->x() && position->y()< mTopRight->y();
}