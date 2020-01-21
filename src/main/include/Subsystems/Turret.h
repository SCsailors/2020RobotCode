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
    double angleInRange(double angle);

  private:
    rev::CANSparkMax turretmotor{2,rev::CANSparkMaxLowLevel::MotorType::kBrushless}; //change
    rev::CANEncoder turretencoder=turretmotor.GetEncoder();  //probably change
    rev::CANPIDController pid{turretmotor};
    frc::DigitalInput counterclockwiselimit{0};
    double counterclockwisemax;  //put 0 outside range between maxs if not 360
    frc::DigitalInput clockwiselimit{1};
    double clockwisemax;
    double offset;


};
}
#include "Robot.h"