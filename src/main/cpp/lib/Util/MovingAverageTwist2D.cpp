/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Util/MovingAverageTwist2D.h"
using namespace Utility;
MovingAverageTwist2D::MovingAverageTwist2D(int maxSize): maxSize(maxSize) {}


void MovingAverageTwist2D::add(std::shared_ptr<Twist2D> twist)
{
    twists.push_back(twist);
    if (twists.size() > maxSize)
    {
        twists.erase(twists.begin());
    }
}

std::shared_ptr<Twist2D> MovingAverageTwist2D::getAverage()
{
    double x = 0.0, y = 0.0, t = 0.0;

    for (auto twist : twists)
    {
        x += twist->dx;
        y += twist->dy;
        t += twist->dtheta;
    }
    double size = twists.size();
    return std::make_shared<Twist2D>(x/size, y/size, t/size);
}

int MovingAverageTwist2D::getSize()
{
    return twists.size();
}

bool MovingAverageTwist2D::isUnderMaxSize()
{
    return getSize() < maxSize;
}

void MovingAverageTwist2D::clear()
{
    twists.clear();
}