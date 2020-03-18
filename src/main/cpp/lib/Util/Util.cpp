/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Util/Util.h"
#include <frc/smartdashboard/SmartDashboard.h>

Util::Util() 
{
    frc::SmartDashboard::PutBoolean("Util Constructed", true);
}

bool Util::epsilonEquals(double a, double b){
    return (a-kEpsilon<=b)&&(a+kEpsilon>=b);
}

bool Util::epsilonEquals(double a, double b, double epsilon){
    return (a-epsilon<=b)&&(a+epsilon>=b);
}

double Util::limit(double v, double min, double max){
    return fmin(max, fmax(min, v));
}

double Util::limit(double v, double lim){
    return limit(v, -lim, lim);
}

double Util::interpolate(double a, double b, double x){
    x=limit(x,0.0, 1.0);
    return a+(b-a)*x;
}

double Util::convertTurretAngle(double angle)
{
    double mod_angle = std::fmod(angle, 360.0);
    if (mod_angle > 180.0)
    {
        mod_angle -= 360.0;
    } else if (mod_angle < -180.0)
    {
        mod_angle += 360.0;
    }
    return mod_angle;
}

string Util::toString(double value){
    stringstream stream;
    stream<<fixed<<setprecision(4)<<value;
    return stream.str();
}