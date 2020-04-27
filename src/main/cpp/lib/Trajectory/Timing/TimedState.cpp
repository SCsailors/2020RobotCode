/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Trajectory/Timing/TimedState.h"

TimedState::TimedState(shared_ptr<Pose2DWithCurvature> state, double t, double velocity, double acceleration) {
    state_=state;
    t_=t;
    velocity_=velocity;
    acceleration_=acceleration;
}

TimedState::TimedState(){
    state_=make_shared<Pose2DWithCurvature>();
    t_=0.0;
    velocity_=0.0;
    acceleration_=0.0;
}

shared_ptr<Pose2DWithCurvature> TimedState::state(){
    return state_;
}

void TimedState::set_t(double t){
    t_=t;
}

void TimedState::set_velocity(double velocity){
    velocity_=velocity;
}

void TimedState::set_acceleration(double acceleration){
    acceleration_=acceleration;
}

double TimedState::t(){
    return t_;
}

double TimedState::velocity(){
    return velocity_;
}

double TimedState::acceleration(){
    return acceleration_;
}

shared_ptr<TimedState> TimedState::interpolate(shared_ptr<TimedState> other, double x){
    double new_t=util->interpolate(t(), other->t(),x);
    double delta_t=new_t-t();
    if(delta_t<0.0){
        shared_ptr<TimedState> timed_state= make_shared<TimedState>(state(), t(), velocity(), acceleration());
        return other->interpolate(timed_state, 1.0-x);
    }
    bool reversing= velocity()<0.0 || (util->epsilonEquals(0.0, velocity())) && acceleration() < 0.0;
    double new_v= velocity()+acceleration()*delta_t;
    double new_s= (reversing? -1.0: 1.0)* (velocity()*delta_t+.5*acceleration()*delta_t*delta_t);
    shared_ptr<TimedState> interState= make_shared<TimedState>(state()->interpolate(other->state(), new_s/state()->getDistance(other->state())), new_t, new_v, acceleration() );
    return interState;
}