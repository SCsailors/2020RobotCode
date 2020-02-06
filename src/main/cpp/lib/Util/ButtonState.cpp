/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Util/ButtonState.h"
using namespace Utility;

bool ButtonState::isPressed()
{
    bool state = Get();
    bool pressed = false;
    if (state && state!=prev_state_pressed)
    {
        pressed = true;
    }

    prev_state_pressed = state;
    return pressed;
}

bool ButtonState::isHeld()
{
    bool state = Get();
    bool held = false;
    if (!state)
    {
        ++debounce_iterations;
        if (debounce_iterations < max_iterations)
        { //error in the reading and flickered from true to false
            held = true;
        } else
        { //not held
            held = false;
        }
        
    } else //held
    { 
        debounce_iterations = 0;
        held = true;
    }
    
    return held;
}

bool ButtonState::isReleased()
{
    bool state = Get();
    bool released = false;
    if (!state && state != prev_state_released)
    {
        released = true;
    }
    prev_state_released = state;
    return released;
}
