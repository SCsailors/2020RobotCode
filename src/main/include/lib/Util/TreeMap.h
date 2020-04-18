/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <vector>
#include <memory>

#include <frc/Timer.h>
template<class T, class U>
class TreeMap {
  int mMaxSize;
  double maxGoalTrackAge = 2.5;
 public:
  TreeMap(int max_size): mMaxSize(max_size){}
  
  
  class Objects {
    public:
      Objects(T key, U object): mKey(key), mObject(object){}
      T mKey;
      U mObject;
  };

  
  std::vector<TreeMap<T,U>::Objects> pastObjects;

  void put(T key, U object)
  {
    Objects objects{key, object};
    
    if (mMaxSize>0 && mMaxSize<=pastObjects.size()){
        pastObjects.erase(pastObjects.begin());
    }
    
    pastObjects.push_back(objects);
  }

  bool isEmpty(){return pastObjects.size()==0;}

  void pruneByTime()
  {
    
    double delete_before = frc::Timer::GetFPGATimestamp()-maxGoalTrackAge;
    auto it = pastObjects.begin();
    int i = 0;
    while (true) 
    {
      if(pastObjects.at(i).mKey < delete_before)
      {
        pastObjects.erase(it);
      } else
      {
        break;
      }
      
    }
  }
  U getLatestObject(){return pastObjects.at(pastObjects.size()-1).mObject;}

  T getLatestKey(){return pastObjects.at(pastObjects.size()-1).mKey;}
/*
  U get(T time)
  {
    //check boundaries
    T time_ = pastObjects.at(0).mTime;
    if(time <= time_){return pastObjects.at(0).mObject;}
    time_ = pastObjects.at(pastObjects.size()-1).mTime;
    if (time >= time_){return pastObjects.at(pastObjects.size()-1).mTime;}
    else
    {
      for int i = 0; i<pastObjects.size()-1; i++)
      {

      }
    }



  }
  */
};
