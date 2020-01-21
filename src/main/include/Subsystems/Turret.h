//clockwise positive

//change for absolute encoder

#pragma once 
#include "Subsystems/Subsystem.h"
#include "frc/DigitalInput.h"
#include "rev/CANPIDController.h"
#include "rev/CANSparkMax.h"
#include "rev/CANEncoder.h"

namespace Subsystems{

class Turret : public Subsystems::Subsystem{
  public:
    Turret();
    void OnStart(double timestamp);
    void OnLoop(double timestamp);
    void OnStop(double timestamp);
    void setAngle(double angle);
    double getAngle();
    void setAngleRelative(double angleadd);
    double angleInRange(double angle);  //returns 0<=angle,360

  private:
    rev::CANSparkMax turretmotor{2,rev::CANSparkMaxLowLevel::MotorType::kBrushless}; //change
    rev::CANEncoder turretencoder=turretmotor.GetEncoder();  //probably change
    rev::CANPIDController pid{turretmotor};
    frc::DigitalInput counterclockwiselimit{0};
    double counterclockwisemax=185.;  //put 0 straight forward on robot (do not put 0 between maxs)
    frc::DigitalInput clockwiselimit{1};
    double clockwisemax=175.;
    double offset;
    double degreesperrotation=1.;  //change


};
}
#include "Robot.h"