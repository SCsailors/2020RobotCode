/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Util/DriveAssist.h"

DriveAssist::DriveAssist() {}

shared_ptr<DriveSignal> DriveAssist::Drive(double throttle, double wheel, bool isQuickTurn, bool isHighGear){
    wheel = handleDeadband(wheel, kWheelDeadband);
    throttle= handleDeadband(throttle, kThrottleDeadband);
    

    double negInertia=wheel-mOldWheel;
    mOldWheel=wheel;

    double wheelNonLinearity;
    if(isHighGear){
        wheelNonLinearity=kHighWheelNonLinearity;
        double denominator=sin(pi/2.0*wheelNonLinearity);
        //apply a scaled sin function to give a better feel
        wheel=sin(pi/2.0*wheelNonLinearity*wheel)/denominator;
        wheel=sin(pi/2.0*wheelNonLinearity*wheel)/denominator;
    } else{
        wheelNonLinearity=kLowWheelNonLinearity;
        double denominator=sin(pi/2.0*wheelNonLinearity);
        //apply a scaled sin function to give a better feel
        wheel=sin(pi/2.0*wheelNonLinearity*wheel)/denominator;
        wheel=sin(pi/2.0*wheelNonLinearity*wheel)/denominator;
        wheel=sin(pi/2.0*wheelNonLinearity*wheel)/denominator;
    }

    double leftOut, rightOut, overPower, sensitivity, angularPower, linearPower, negInertiaScalar;
    if(isHighGear){
        negInertiaScalar=kHighNegInertiaScalar;
        sensitivity=kHighSensitivity;
    } else{
        if(wheel*negInertia>0.0){
            //moving away from 0.0 (more turn).
            negInertiaScalar=kLowNegInertiaTurnScalar;
        }else{
            //Otherwise moving toward 0.0 (less turn).
            if(fabs(wheel)>kLowNegInertiaThreshold){
                negInertiaScalar=kLowNegInertiaFarScalar;
            } else{
                negInertiaScalar=kLowNegInertiaCloseScalar;
            }

        }
        sensitivity=kLowSensitivity;
    }

    double negInertiaPower= negInertia*negInertiaScalar;
    mNegInertiaAccumulator += negInertiaPower;

    wheel+= mNegInertiaAccumulator;
    if(mNegInertiaAccumulator>1.0){
        mNegInertiaAccumulator-=1.0;
    } else if (mNegInertiaAccumulator<-1.0){
        mNegInertiaAccumulator+=1.0;
    } else{
        mNegInertiaAccumulator=0.0;
    }
    linearPower=throttle;

    //Quickturn
    if(isQuickTurn){
        if(fabs(linearPower)<kQuickStopDeadband){
            double alpha= kQuickStopWeight;
            mQuickStopAccumulator= (1-alpha)*mQuickStopAccumulator+alpha*util->limit(wheel,-1.0, 1.0)*kQuickStopScalar;
        }
        overPower= 1.0;
        angularPower=wheel;
    } else{
        overPower=0.0;
        angularPower= fabs(throttle)*wheel*sensitivity-mQuickStopAccumulator;
        if(mQuickStopAccumulator>1.0){
        mQuickStopAccumulator-=1.0;
        } else if (mQuickStopAccumulator<-1.0){
        mQuickStopAccumulator+=1.0;
        } else{
        mQuickStopAccumulator=0.0;
        }
    }
    rightOut=leftOut=linearPower;
    leftOut+=angularPower;
    rightOut-=angularPower;

    if (leftOut>1.0){
        rightOut-=overPower*(leftOut-1.0);
        leftOut=1.0;
    }else if (rightOut>1.0){
        leftOut-=overPower*(rightOut-1.0);
        rightOut=1.0;
    }else if (leftOut<-1.0){
        rightOut+=overPower*(-1.0-leftOut);
        leftOut=-1.0;
    } else if (rightOut<-1.0){
        leftOut+=overPower*(-1.0-rightOut);
        rightOut=-1.0;
    }

    
    shared_ptr<DriveSignal> Signal = make_shared<DriveSignal>(leftOut, rightOut);
    return Signal;
}

double DriveAssist::handleDeadband(double val, double deadband){
    return (fabs(val)>fabs(deadband)) ? val : 0.0;
}