/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Trajectory/Timing/TimingUtil.h"
#include <iostream>

TimingUtil::TimingUtil() {}
//TODO: reengineer for generics, so that 2D and 1D states can be used

vector<shared_ptr<TimedState>> TimingUtil::timeParameterizeTrajectory(
    bool reverse,
    vector<shared_ptr<Pose2DWithCurvature>> states,
    shared_ptr<TimingConstraint> constraints,
    double start_velocity,
    double end_velocity,
    double max_velocity,
    double max_abs_acceleration){
    //Forward pass. pairs of consecutive states (pose2dWithCurvature's). reachable velocity and acceleration for each point
    shared_ptr<ConstrainedState> predecessor=make_shared<ConstrainedState>();
    predecessor->state=states.front();
    predecessor->distance=0.0;
    predecessor->max_velocity= start_velocity;
    predecessor->min_acceleration= -max_abs_acceleration;
    predecessor->max_acceleration= max_abs_acceleration;
    
    //for (auto point: states){
    //    std::cout<<point->getTranslation()->x()<<","<<point->getTranslation()->y()<<","<<point->getRotation()->getDegrees()<<","<<std::endl;
    //} 
    
    for (auto state:states){
        //create next new state
        shared_ptr<ConstrainedState> constraint_state=make_shared<ConstrainedState>();
        
        constraint_state->state=state;
        double ds= constraint_state->state->getDistance(predecessor->state);
        constraint_state->distance= ds+predecessor->distance;
        
        
        //iterate to find maximum end velocity and common accelerations
        while(true){
            //enforce global max velocity and max reachable velocity by global acceleration limit
            constraint_state->max_velocity=fmin(max_velocity, sqrt(predecessor->max_velocity*predecessor->max_velocity+2.0*predecessor->max_acceleration*ds));
            
            //enforce global max absolute acceleration.
            constraint_state->min_acceleration=-max_abs_acceleration;
            constraint_state->max_acceleration=max_abs_acceleration;

            // TODO fix TimingConstraint
            //std::cout<<constraint_state->state->getTranslation()->x()<<","<<constraint_state->state->getTranslation()->y()<<","<<constraint_state->state->getRotation()->getDegrees()<<","<<constraint_state->max_velocity<<","<<constraint_state->max_acceleration<<","<<constraint_state->distance<<","<<std::endl;
            //fully constrained state, but needs dynamic constraints
            //Enforce all velocity constraints
            constraint_state->max_velocity=fmin(constraint_state->max_velocity, constraints->getMaxVelocity(constraint_state->state));
        
            //enforce all acceleration constraints
            shared_ptr<TimingConstraint::MinMaxAcceleration> min_max_accel= constraints->getMinMaxAcceleration(constraint_state->state, (reverse? -1.0:1.0)*constraint_state->max_velocity);

            //constraint_state->min_acceleration=fmax(constraint_state->min_acceleration, reverse? -min_max_accel->max_acceleration(): min_max_accel->min_acceleration());
            //constraint_state->min_acceleration=fmin(constraint_state->max_acceleration, reverse? -min_max_accel->min_acceleration(): min_max_accel->max_acceleration());

            if(ds<kEpsilon){
                break;
            }
        
            // TODO max acceleration for previous state
            double actual_acceleration= (constraint_state->max_velocity*constraint_state->max_velocity-predecessor->max_velocity*predecessor->max_velocity)/(2.0*ds);
            if (constraint_state->max_acceleration<actual_acceleration-kEpsilon){
                predecessor->max_acceleration=constraint_state->max_acceleration;
            } else{
                if(actual_acceleration>predecessor->min_acceleration+kEpsilon){
                    predecessor->max_acceleration=actual_acceleration;
                }

                //if actual acceleration is less than predecessor min accel, we will repair during backward pass
                break;
            }
            
        }
        
        constraint_states.push_back(predecessor);
        predecessor=constraint_state;
    }
    
    shared_ptr<ConstrainedState> successor=make_shared<ConstrainedState>();
    successor->state=states.at(states.size()-1);
    successor->distance=constraint_states.at(states.size()-1)->distance;
    successor->max_velocity=end_velocity;
    successor->min_acceleration=-max_abs_acceleration;
    successor->max_acceleration=max_abs_acceleration;
    for (int i=constraint_states.size()-2;i>=0;i--){ // TODO check
        shared_ptr<ConstrainedState> constraint_state=constraint_states.at(i);
        double ds=constraint_state->distance-successor->distance; //will be negative
        //cout<<successor->distance<<endl;
    
        //cout<<constraint_state->distance<<endl;
        //cout<<ds<<endl;
        while (true){
            //enforce max reachable velocity limit
            //vf=sqrt(vi^2+2*a*d)
            double new_max_velocity =sqrt(successor->max_velocity*successor->max_velocity+2*successor->min_acceleration*ds);
            if( new_max_velocity>=constraint_state->max_velocity){
                break; //no new limits
            }
            constraint_state->max_velocity=new_max_velocity;
            
                //enforce all acceleration constraints
            //shared_ptr<TimingConstraint::MinMaxAcceleration> min_max_accel= constraints->getMinMaxAcceleration(constraint_state->state, (reverse? -1.0:1.0)*constraint_state->max_velocity);

            //constraint_state->min_acceleration=fmax(constraint_state->min_acceleration, reverse? -min_max_accel->max_acceleration(): min_max_accel->min_acceleration());
            //constraint_state->min_acceleration=fmin(constraint_state->max_acceleration, reverse? -min_max_accel->min_acceleration(): min_max_accel->max_acceleration());

            if(ds<kEpsilon){
                break;
            }
            //reduce min acceleration and try again

            double actual_acceleration=(constraint_state->max_velocity*constraint_state->max_velocity-successor->max_velocity*successor->max_velocity)/(2.0*ds);
            if(constraint_state->min_acceleration>actual_acceleration+kEpsilon){
                successor->min_acceleration=constraint_state->min_acceleration;
            } else{
                successor->min_acceleration= actual_acceleration;
                break;
            }
        }
        //std::cout<<constraint_state->state->getTranslation()->x()<<","<<constraint_state->state->getTranslation()->y()<<","<<constraint_state->state->getRotation()->getDegrees()<<","<<constraint_state->max_velocity<<","<<constraint_state->max_acceleration<<","<<std::endl;
        //update list of states
        constraint_states.at(i+1)=successor;
        successor=constraint_state;
    }

    double t=0.0;
    double s=0.0;
    double v=0.0;
    for(shared_ptr<ConstrainedState> state: constraint_states){\
        i+=1;
        shared_ptr<ConstrainedState> constrained_state=state;
        double ds=constrained_state->distance-s;
        double accel=(constrained_state->max_velocity*constrained_state->max_velocity-v*v)/(2.0*ds);
        double dt=0.0;
        if(i>1){
            timed_states.at(i-1)->set_acceleration(reverse? -accel: accel);
            if(fabs(accel)>kEpsilon){
                dt=(constrained_state->max_velocity-v)/accel;
            } else if(fabs(v)>kEpsilon){
                dt=ds/v;
            }
        }
        t+=dt;
        v=constrained_state->max_velocity;
        s=constrained_state->distance;
        shared_ptr<TimedState> timed_state= make_shared<TimedState>(constrained_state->state, t, reverse? -v: v, reverse? -accel: accel);
        timed_states.push_back(timed_state);
    }
    timed_states.erase(timed_states.begin());
    return timed_states;
}
