#include "Subsystems/Turret.h"
#include <cmath>

namespace Subsystems{ 
Turret::Turret(){

}
 
void Turret::OnStart(double timestamp){
    turretencoder.SetPositionConversionFactor(degreesperrotation);
    offset=0.;
    pid.SetP(.5); //change
    pid.SetI(0.); //change
    pid.SetD(0.); //change
}

void Turret::OnLoop(double timestamp){
    if (counterclockwiselimit.Get()){
        offset=counterclockwisemax-turretencoder.GetPosition();
    }
    else if (clockwiselimit.Get()){
        offset=counterclockwisemax-turretencoder.GetPosition();
    };

}

void Turret::OnStop(double timestamp){

}

void Turret::setAngle(double angle){
    angle=angleInRange(angle+offset);
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