/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Util/Util.h"

Util::Util() {}

bool Util::epsilonEquals(double a, double b){
    return (a-kEpsilon<=0)&&(a+kEpsilon>=b);
}

bool Util::epsilonEquals(double a, double b, double epsilon){
    return (a-epsilon<=0)&&(a+epsilon>=b);
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

string Util::toString(double value){
    stringstream stream;
    stream<<fixed<<setprecision(4)<<value;
    return stream.str();
}