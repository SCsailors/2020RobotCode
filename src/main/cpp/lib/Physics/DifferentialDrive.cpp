/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Physics/DifferentialDrive.h"

DifferentialDrive::DifferentialDrive(double mass, double moi, double angular_drag, double wheel_radius, double effective_wheelbase_radius, shared_ptr<DCMotorTransmission> left_transmission, shared_ptr<DCMotorTransmission> right_transmission){
    mass_=mass;
    moi_=moi;
    angular_drag_=angular_drag;
    wheel_radius_=wheel_radius;
    effective_wheelbase_radius_=effective_wheelbase_radius;
    left_transmission_=left_transmission;
    right_transmission_=right_transmission;
}

double DifferentialDrive::Mass(){
    return mass_;
}

double DifferentialDrive::Moi(){
    return moi_;
}

double DifferentialDrive::Wheel_radius(){
    return wheel_radius_;
}

double DifferentialDrive::Effective_wheelbase_radius(){
    return effective_wheelbase_radius_;
}

double DifferentialDrive::Angular_drag(){
    return angular_drag_;
}

shared_ptr<DCMotorTransmission> DifferentialDrive::Left_transmission(){
    return left_transmission_;
}

shared_ptr<DCMotorTransmission> DifferentialDrive::Right_transmission(){
    return right_transmission_;
}

DifferentialDrive::ChassisState::ChassisState(){
    linear=0.0;
    angular=0.0;
}

DifferentialDrive::ChassisState::ChassisState(double linear, double angular){
    this->linear= linear;
    this->angular= angular;
}

//MinMax class
DifferentialDrive::MinMax::MinMax(){}


// TODO later
shared_ptr<DifferentialDrive::MinMax> DifferentialDrive::MinMax::getMinMaxAcceleration(shared_ptr<ChassisState> chassis_velocity, double curvature, double max_abs_voltage){
    shared_ptr<DifferentialDrive::MinMax> result= make_shared<DifferentialDrive::MinMax>();
    //shared_ptr<DifferentialDrive::WheelState> wheel_velocities= solveInverseKinematics(chassis_velocity);
    result->min=-1E100;        
    result->max=1E100;

    //(Tl+Tr)/r_w=m*a
    //(Tr-Tl)/r_w*r_wb-drag*w=i*(a*k+v^2*dk)
    //solve for a and (Tl|Tr)

    //double linear_term=isinf(curvature)? 0.0: Mass()*Effective_wheelbase_radius();
    //double angular_term=isinf(curvature)? Moi(): Moi()*curvature;

    //double drag_torque= chassis_velocity->angular* Angular_drag();

    bool a[2]={false, true};
    double b[2]={1.0, -1.0};
    for(int i=0; i<2; i++){
        for(int j=0; j<2; j++){
            bool left=a[i];
            //shared_ptr<DCMotorTransmission> fixed_transmission = left? left_transmission_: right_transmission_;
            //shared_ptr<DCMotorTransmission> variable_transmission= left? left_transmission_: right_transmission_;

         //   double variable_voltage= variable_transmission->getVoltageForTorque()
        }

    }

    return result;
}



DifferentialDrive::WheelState::WheelState(){}

DifferentialDrive::WheelState::WheelState(double left, double right){
    this->left=left;
    this->right=right;
}

double DifferentialDrive::WheelState::get(bool get_left){
    return get_left? left: right;
}

void DifferentialDrive::WheelState::set(bool set_left, double val){
    if(set_left){
        left=val;
    }else {
        right=val;
    }
}

DifferentialDrive::DriveDynamics::DriveDynamics(){}

shared_ptr<DifferentialDrive::ChassisState> DifferentialDrive::solveForwardKinematics(shared_ptr<WheelState> wheel_motion){
    shared_ptr<ChassisState> chassis_motion = make_shared<ChassisState>();
    chassis_motion->linear= Wheel_radius()*(wheel_motion->right+wheel_motion->left)/2.0;
    chassis_motion->angular= Wheel_radius()*(wheel_motion->right-wheel_motion->left)/(2.0*Effective_wheelbase_radius());
    return chassis_motion;
}

shared_ptr<DifferentialDrive::WheelState> DifferentialDrive::solveInverseKinematics(shared_ptr<ChassisState> chassis_motion){
    shared_ptr<WheelState> wheel_motion = make_shared<WheelState>();
    wheel_motion->left= (chassis_motion->linear-Effective_wheelbase_radius()*chassis_motion->angular)/Wheel_radius();
    wheel_motion->right= (chassis_motion->linear+Effective_wheelbase_radius()*chassis_motion->angular)/Wheel_radius();
    return wheel_motion;
}

shared_ptr<DifferentialDrive::DriveDynamics> DifferentialDrive::solveForwardDynamics(shared_ptr<DifferentialDrive::ChassisState> chassis_velocity, shared_ptr<DifferentialDrive::WheelState> voltage){
    shared_ptr<DifferentialDrive::DriveDynamics> dynamics = make_shared<DifferentialDrive::DriveDynamics>();
    dynamics->wheel_velocity=solveInverseKinematics(chassis_velocity);
    dynamics->chassis_velocity= chassis_velocity;
    dynamics->curvature= dynamics->chassis_velocity->angular/dynamics->chassis_velocity->linear;
    if (isnan(dynamics->curvature)){ dynamics->curvature=0.0;}
    dynamics->voltage=voltage;
    solveForwardDynamics(dynamics);
    return dynamics;
}

shared_ptr<DifferentialDrive::DriveDynamics> DifferentialDrive::solveForwardDynamics(shared_ptr<DifferentialDrive::WheelState> wheel_velocity, shared_ptr<DifferentialDrive::WheelState> voltage){
    shared_ptr<DifferentialDrive::DriveDynamics> dynamics = make_shared<DifferentialDrive::DriveDynamics>();
    dynamics->wheel_velocity= wheel_velocity;
    dynamics->chassis_velocity= solveForwardKinematics(wheel_velocity);
    dynamics->curvature= dynamics->chassis_velocity->angular/dynamics->chassis_velocity->linear;
    if (isnan(dynamics->curvature)){ dynamics->curvature=0.0;}
    dynamics->voltage=voltage;
    solveForwardDynamics(dynamics);
    return dynamics;
}

void DifferentialDrive::solveForwardDynamics(shared_ptr<DifferentialDrive::DriveDynamics> dynamics){
    bool left_stationary= dynamics->wheel_velocity->left<1E-5 && dynamics->wheel_velocity->left>-(1E-5) && fabs(dynamics->voltage->left)<left_transmission_->Friction_Voltage();
    bool right_stationary= dynamics->wheel_velocity->right<1E-5 && dynamics->wheel_velocity->right>-(1E-5) && fabs(dynamics->voltage->right)<right_transmission_->Friction_Voltage();
    if(left_stationary&& right_stationary){
        //neither side breaks static friction
        dynamics->wheel_torque->left=dynamics->wheel_torque->right=0.0;
        dynamics->chassis_acceleration->linear= dynamics->chassis_acceleration->angular=0.0;
        dynamics->wheel_acceleration->left= dynamics->wheel_acceleration->right=0.0;
        dynamics->dcurvature=0.0;
        return;
    }
    //solve for torques
    dynamics->wheel_torque->left=left_transmission_->getTorqueForVoltage(dynamics->wheel_velocity->left, dynamics->voltage->left);
    dynamics->wheel_torque->right=right_transmission_->getTorqueForVoltage(dynamics->wheel_velocity->right, dynamics->voltage->right);
    // TODO forces and torques about center of mass
    dynamics->chassis_acceleration->linear=(dynamics->wheel_torque->right+dynamics->wheel_torque->left)/(Wheel_radius()*Mass());
    //Tr-Tl)/r_w*r_wb-drag*w=i*(angular_acceleration)
    dynamics->chassis_acceleration->angular= Effective_wheelbase_radius()*(dynamics->wheel_torque->right-dynamics->wheel_torque->left)/(Wheel_radius()*Moi())-dynamics->chassis_acceleration->angular*Angular_drag()/Moi();

    //solve for change in curvature from angular acceleration
    dynamics->dcurvature= (dynamics->chassis_acceleration->angular-dynamics->chassis_acceleration->linear*dynamics->curvature)/(dynamics->chassis_velocity->linear*dynamics->chassis_velocity->linear);
    if(isnan(dynamics->dcurvature)){dynamics->dcurvature=0.0;}

    //Resolve chassis accelerations to each wheel
    dynamics->wheel_acceleration->left= dynamics->chassis_acceleration->linear-dynamics->chassis_acceleration->angular*Effective_wheelbase_radius();
    dynamics->wheel_acceleration->right= dynamics->chassis_acceleration->linear+dynamics->chassis_acceleration->angular*Effective_wheelbase_radius();
}

shared_ptr<DifferentialDrive::DriveDynamics> DifferentialDrive::solveInverseDynamics(shared_ptr<DifferentialDrive::ChassisState> chassis_velocity, shared_ptr<DifferentialDrive::DifferentialDrive::ChassisState> chassis_acceleration){
    shared_ptr<DifferentialDrive::DriveDynamics> dynamics = make_shared<DifferentialDrive::DriveDynamics>();
    dynamics->chassis_velocity=chassis_velocity;
    dynamics->curvature=dynamics->chassis_velocity->angular/dynamics->chassis_velocity->linear;
    if (isnan(dynamics->curvature)){dynamics->curvature=0.0;} // TODO check if this should be 0.0 or 1E100
    dynamics->chassis_acceleration=chassis_acceleration;
    dynamics->dcurvature=(dynamics->chassis_acceleration->angular-dynamics->chassis_acceleration->linear*dynamics->curvature)/ (dynamics->chassis_velocity->linear*dynamics->chassis_velocity->linear);
    if (isnan(dynamics->dcurvature)){dynamics->dcurvature=0.0;}
    dynamics->wheel_velocity= solveInverseKinematics(chassis_velocity);
    dynamics->wheel_acceleration= solveInverseKinematics(chassis_acceleration);
    solveInverseDynamics(dynamics);
    return dynamics;
}
shared_ptr<DifferentialDrive::DriveDynamics> DifferentialDrive::solveInverseDynamics(shared_ptr<DifferentialDrive::WheelState> wheel_velocity, shared_ptr<DifferentialDrive::WheelState> wheel_acceleration){
    shared_ptr<DifferentialDrive::DriveDynamics> dynamics = make_shared<DifferentialDrive::DriveDynamics>();
    dynamics->chassis_velocity=solveForwardKinematics(wheel_velocity);
    dynamics->curvature=dynamics->chassis_velocity->angular/dynamics->chassis_velocity->linear;
    if (isnan(dynamics->curvature)){dynamics->curvature=0.0;}
    dynamics->chassis_acceleration=solveForwardKinematics(wheel_acceleration);
    dynamics->dcurvature=(dynamics->chassis_acceleration->angular-dynamics->chassis_acceleration->linear*dynamics->curvature)/ (dynamics->chassis_velocity->linear*dynamics->chassis_velocity->linear);
    if (isnan(dynamics->dcurvature)){dynamics->dcurvature=0.0;}
    dynamics->wheel_velocity= wheel_velocity;
    dynamics->wheel_acceleration= wheel_acceleration;
    solveInverseDynamics(dynamics);
    return dynamics;
}
void DifferentialDrive::solveInverseDynamics(shared_ptr<DifferentialDrive::DriveDynamics> dynamics){
// Determine the necessary torques on the left and right wheels to produce the desired wheel accelerations.
        dynamics->wheel_torque->left = Wheel_radius() / 2.0 * (dynamics->chassis_acceleration->linear * Mass() -
                dynamics->chassis_acceleration->angular * Moi() / Effective_wheelbase_radius() -
                dynamics->chassis_velocity->angular * Angular_drag() / Effective_wheelbase_radius());
        dynamics->wheel_torque->right = Wheel_radius() / 2.0 * (dynamics->chassis_acceleration->linear * Mass() +
                dynamics->chassis_acceleration->angular * Moi() / Effective_wheelbase_radius() +
                dynamics->chassis_velocity->angular * Angular_drag() / Effective_wheelbase_radius());

        // Solve for input voltages.
        dynamics->voltage->left = left_transmission_->getVoltageForTorque(dynamics->wheel_velocity->left, dynamics
                ->wheel_torque->left);
        dynamics->voltage->right = right_transmission_->getVoltageForTorque(dynamics->wheel_velocity->right, dynamics
                ->wheel_torque->right);
}


double DifferentialDrive::getMaxAbsVelocity(double curvature, double max_abs_voltage){
// Alternative implementation:
        // (Tr - Tl) * r_wb / r_w = I * v^2 * dk
        // (Tr + Tl) / r_w = 0
        // T = Tr = -Tl
        // 2T * r_wb / r_w = I*v^2*dk
        // T = 2*I*v^2*dk*r_w/r_wb
        // T = kt*(-vR/kv + V) = -kt*(-vL/vmax + V)
        // Vr = v * (1 + k*r_wb)
        // 0 = 2*I*dk*r_w/r_wb * v^2 + kt * ((1 + k*r_wb) * v / kv) - kt * V
        // solve using quadratic formula?
        // -b +/- sqrt(b^2 - 4*a*c) / (2a)

        // k = w / v
        // v = r_w*(wr + wl) / 2
        // w = r_w*(wr - wl) / (2 * r_wb)
        // Plug in max_abs_voltage for each wheel.
        double left_speed_at_max_voltage= left_transmission_->free_speed_at_voltage(max_abs_voltage);
        double right_speed_at_max_voltage= right_transmission_->free_speed_at_voltage(max_abs_voltage);

        if (curvature>-1E-5 && curvature<1E-5){
            return Wheel_radius()*fmin(left_speed_at_max_voltage, right_speed_at_max_voltage);
        }
        if (curvature>=1E99 || curvature<=-1E99){
            //turning in place
            double wheel_speed=fmin(left_speed_at_max_voltage, right_speed_at_max_voltage);
            double signum= (curvature>0)? 1.0 :((curvature<0)?-1:0);
            return signum* Wheel_radius()*wheel_speed/Effective_wheelbase_radius();
        }

        double right_speed_if_left_max= left_speed_at_max_voltage*(Effective_wheelbase_radius()*curvature+1)/(1.0-Effective_wheelbase_radius()*curvature);

        if( fabs(right_speed_if_left_max <= right_speed_at_max_voltage+kEpsilon)){
            //Left max is active constraint
            return Wheel_radius()*(left_speed_at_max_voltage+right_speed_at_max_voltage)/2.0;
        }

        double left_speed_if_right_max= right_speed_at_max_voltage*(Effective_wheelbase_radius()*curvature+1)/(1.0-Effective_wheelbase_radius()*curvature);
        return Wheel_radius()*(right_speed_at_max_voltage+left_speed_if_right_max)/2.0;
}