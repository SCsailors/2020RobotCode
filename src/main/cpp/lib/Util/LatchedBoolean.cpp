/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Util/LatchedBoolean.h"
using namespace Utility;
bool LatchedBoolean::update(bool newState) 
{
    bool state = false;
    if (invert)
    { // falling edge
        if (!(newState) && newState != mLast)
        {
            state = true;
        }
    } else 
    { //rising edge
        if (newState && newState != mLast)
        {
            state = true;
        }
    }

    mLast = newState;
    return state;
}
