/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <cmath>
#include <opencv2/opencv.hpp>

#include <Constants.h>

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
 public:
  //Limelight mConstants->kName
  Constants::TargetNames mCamera;
  TargetInfo(cv::Mat trans, cv::Mat rot, Constants::TargetNames camera);
  TargetInfo(Constants::TargetNames camera);
  TargetInfo(std::vector<double> camTran, Constants::TargetNames camera);
  double getX(){return x;}
  double getY(){return y;}
  double getZ(){return z;}
  double getXTheta(){return xTheta;}
  double getYTheta(){return yTheta;}
  double getZTheta(){return zTheta;}
  
  
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