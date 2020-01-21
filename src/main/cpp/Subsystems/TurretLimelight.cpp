#include "Subsystems/TurretLimelight.h"
#include <cmath>

namespace Subsystems{

TurretLimelight::TurretLimelight(){
    table=inst.GetTable("limelight");
    table->GetEntry("pipeline").SetDouble(0);
    table->GetEntry("ledMode").SetDouble(3);
}

bool TurretLimelight::getTV(){
    table=inst.GetTable("limelight");
    return ((table->GetEntry("tv").GetDouble(0.))==1.);
}

double TurretLimelight::getTurretTX(){ //not adjusted for not centered on turret (need to know distance to do that)
    return getTX()+offsetx;
}

double TurretLimelight::getTurretTY(){
    return getTY()+offsety;
}

double TurretLimelight::getTY(){
    table=inst.GetTable("limelight");
    return (table->GetEntry("ty").GetDouble(0));
}

double TurretLimelight::getTX(){
    table=inst.GetTable("limelight");
    return (table->GetEntry("tx").GetDouble(0));
}

double TurretLimelight::getDistanceFromTY(){
    return (targetheight-height)/tan(getTurretTY()*M_PI/180.);
}




}