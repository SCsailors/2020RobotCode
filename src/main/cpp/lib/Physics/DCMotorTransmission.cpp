/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Physics/DCMotorTransmission.h"

DCMotorTransmission::DCMotorTransmission() {}

DCMotorTransmission::DCMotorTransmission(double speed_per_volt, double torque_per_volt, double friction_voltage){
    this->speed_per_volt=speed_per_volt;
    this->torque_per_volt=torque_per_volt;
    this->friction_voltage=friction_voltage;
}

double DCMotorTransmission::Speed_per_Volt(){
    return speed_per_volt;
}

double DCMotorTransmission::Torque_per_Volt(){
    return torque_per_volt;
}

double DCMotorTransmission::Friction_Voltage(){
    return friction_voltage;
}

double DCMotorTransmission::free_speed_at_voltage(double voltage){
    if(voltage>1E-5){
        return fmax(0.0, voltage-Friction_Voltage())* Speed_per_Volt();
    }else if (voltage<-1E-5){
        return fmin(0.0, voltage+Friction_Voltage())* Speed_per_Volt();
    }else{
        return 0.0;
    }
    
}

double DCMotorTransmission::getTorqueForVoltage(double output_speed, double voltage){
    effective_voltage=voltage;
    if(output_speed>kEpsilon){
        //forward motion, rolling friction
        effective_voltage-=Friction_Voltage();
    } else if(output_speed<-kEpsilon){
        //reverse motion, rolling friction
        effective_voltage+=Friction_Voltage();
    } else if(voltage> kEpsilon){
        //system is static forward torque
        effective_voltage= fmax(0.0, voltage-Friction_Voltage());
    } else if(voltage<-kEpsilon){
        //system static, reverse torque
        effective_voltage=fmin(0.0, voltage+ Friction_Voltage());
    } else{
        //system static
        return 0.0;
    }
    return Torque_per_Volt()*(-output_speed/Speed_per_Volt()+effective_voltage);
}
//CHECK
double DCMotorTransmission::getVoltageForTorque(double output_speed, double torque){
    double friction_voltage;
    if(output_speed>kEpsilon){
        //forward motion, rolling friction
        friction_voltage=Friction_Voltage();
    } else if(output_speed<-kEpsilon){
        //reverse motion, rolling friction
        friction_voltage=-Friction_Voltage();
    } else if(torque> kEpsilon){
        //system is static forward torque
        friction_voltage= Friction_Voltage();
    } else if(torque<-kEpsilon){
        //system static, reverse torque
        friction_voltage= -Friction_Voltage();
    } else{
        //system static
        return 0.0;
    }
    return torque/Torque_per_Volt()+ output_speed/Speed_per_Volt()+Friction_Voltage();
}