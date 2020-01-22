#pragma once

//do getturrets/realize that it must be centered
//return latency "tl"

#include "Subsystem.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

namespace Subsystems{

class TurretLimelight: public Subsystem{
  public:
    TurretLimelight();
    bool getTV();
    double getTurretTX();
    double getTurretTY();
    double getTY();
    double getTX();
    double getOffsetX();
    double getOffsetY();
    double getHeight();
    double getPosOffsetR(); //distance from center of turret
    double getDistanceFromTY(); //horizontal distance from limelight (change to turret center?)
    double getLatency();

  private:
    nt::NetworkTableInstance inst=nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table;

    double offsetx=0.; //anglular offset from perpendicular to radius of turret
    double offsety=0.; //angular offset from perpendicular to ground
    double height=30.; //change (inches)
    double posoffsetr=0.; //distance from center off turret
    double posoffsettheda=0.; //angle from center compared to angle of straight forward (center=centerofturret)
    const double targetheight=8.*12.+2.25; //inches
};
}
