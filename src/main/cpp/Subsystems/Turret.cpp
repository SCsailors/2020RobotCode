#include "Subsystems/Turret.h"
#include <cmath>

namespace Subsystems{ 
Turret::Turret(){

}
 
void Turret::OnStart(double timestamp){
    turretencoder.SetPositionConversionFactor(degreesperrevolution);
    turretencoder.SetVelocityConversionFactor(degreesperrevolution);
    turretencoder.SetPosition(0.); //change
    pid.SetFeedbackDevice(turretencoder);
    pid.SetP(p); //change
    pid.SetI(i); //change
    pid.SetD(d); //change
    pid.SetSmartMotionMaxVelocity(maxdegpersec/60.);  //check units
}

void Turret::OnLoop(double timestamp){
    if (counterclockwiselimit.Get()){
        turretencoder.SetPosition(counterclockwisemax);
    }
    else if (clockwiselimit.Get()){
        turretencoder.SetPosition(clockwisemax);
    };

}

void Turret::OnStop(double timestamp){

}

void Turret::setAngle(double angle){
    angle=angleInRange(angle);
    //no. if between maxs: go to max nearest to prev setpoint? or turrettracking should anticipate where to put setpoint if outside of turret range
    if (angle<counterclockwisemax  && angle>=((counterclockwisemax+clockwisemax)/2.)){
        pid.SetReference(counterclockwisemax,rev::ControlType::kPosition);
    }
    else if(angle>clockwisemax && angle<((counterclockwisemax+clockwisemax)/2.)){
        pid.SetReference(clockwisemax,rev::ControlType::kPosition);
    }
    else{
        pid.SetReference(angle,rev::ControlType::kPosition);
    };
}


void Turret::setAngleRelative(double angleadd){
    setAngle(turretencoder.GetPosition()+angleadd);
}

double Turret::angleInRange(double angle){
    return fmod(fmod(angle,360.)+360.,360.);
}

}