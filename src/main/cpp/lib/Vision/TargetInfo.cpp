/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Vision/TargetInfo.h"
#include "Constants.h"
using namespace VisionTargeting;

TargetInfo::TargetInfo( cv::Mat trans, cv::Mat rot)
{
    x = trans.at<double>(0)/25.4; //convert mm to inches;
    y = trans.at<double>(1)/25.4;
    z = trans.at<double>(2)/25.4;

    xTheta = rot.at<double>(0) * Constants::kRadsToDegrees; //convert radians to degrees
    yTheta = rot.at<double>(1) * Constants::kRadsToDegrees;
    zTheta = rot.at<double>(2) * Constants::kRadsToDegrees;

    std::cout<<"X: " << x << " Y: " << y << " Z: " << z << " xTheta: " << xTheta << " yTheta: " << yTheta << " zTheta: " << zTheta << std::endl;
    
}

TargetCorner::TargetCorner( double y, double z): y(y), z(z)
{
    radius = std::hypot(y, z);
    theta = std::atan2(z, y) * Constants::kRadsToDegrees;
}
