/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <cmath>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

namespace VisionTargeting{
class TargetInfo {
  //xyz from Camera to target from solvePnP
  double x;
  double y;
  double z;
  double xTheta;
  double yTheta;
  double zTheta;
  //maybe add rotation?
 public:
  TargetInfo(cv::Mat trans, cv::Mat rot);
  TargetInfo(): x(0.0), y(0.0), z(0.0), xTheta(0.0), yTheta(0.0), zTheta(0.0) {}
  TargetInfo(std::vector<double> camTran);
  double getX(){return x;}
  double getY(){return y;}
  double getZ(){return z;}
  
  
};

class TargetCorner {
  //yz from camera image
  double x = 1.0;
  double y;
  double z;
  double radius; //pixels
  double theta; // -180 to 180
 public:
  TargetCorner(double y, double z);
  TargetCorner(): y(0.0), z(0.0), radius(0.0), theta(0.0){}
  double getX(){return x;}
  double getY(){return y;}
  double getZ(){return z;}
  double getRadius(){return radius;}
  double getTheta(){return theta;}
  


};
}