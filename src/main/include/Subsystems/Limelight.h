/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <memory>
#include <vector>
#include <iostream>
#include <cmath>

#include <opencv2/opencv.hpp>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/SmartDashboard/SmartDashboard.h>

#include <Subsystems/Subsystem.h>

#include <lib/Geometry/Pose2D.h>
#include <lib/Vision/TargetInfo.h>

namespace Subsystems{

class LimelightConstants {
 public:
  std::string kName;
  std::string kTableName;
  double kHeight;
  std::shared_ptr<Pose2D> kTurretToLens;
  std::shared_ptr<Rotation2D> kHorizontalPlaneToLens;
  LimelightConstants(){};
  LimelightConstants(std::string kName,
    std::string kTableName,
    double kHeight,
    std::shared_ptr<Pose2D> kTurretToLens,
    std::shared_ptr<Rotation2D> kHorizontalPlaneToLens
  ) :
    kName(kName), 
    kTableName(kTableName), 
    kHeight(kHeight), 
    kTurretToLens(kTurretToLens), 
    kHorizontalPlaneToLens(kHorizontalPlaneToLens){} 

};

class Limelight : public Subsystems::Subsystem{
  class PeriodicIO{
    public:
      PeriodicIO(){}
    //INPUTS
    
      double latency = 0.0;
      int givenLedMode = 0;
      int givenPipeline = 0;
      double xOffset = 0.0;
      double yOffset = 0.0;
      double xOffsetRaw = 0.0;
      double yOffsetRaw = 0.0;
      double area = 0.0; 
      double horPixels = 0.0;
      double vertPixels = 0.0;

    //Outputs
      int ledMode = 1; // 0 -use pipeline mode, 1 - off, 2 - blink, 3 - on
      int camMode =0; // 0 - vision processing, 1 - driver camera
      int pipeline = 0; // 0-9
      int stream = 2; // sets stream layout if another webcam is attached
      int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz

  };

  std::shared_ptr<NetworkTable> mNetworkTable;
  std::shared_ptr<LimelightConstants> mConstants;
  PeriodicIO mPeriodicIO{};
  bool mOutputsHaveChanged = true;
  std::vector<double> mZeroArray{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> mCornX; //should be 4 long
  std::vector<double> mCornY;
  const int mNumCorners = 4; 

  //field models of hex and rectangular vision tape (mm);
  const std::vector<cv::Point3d> mHexModelPoints{cv::Point3d(498.475, 215.8, 0.0), cv::Point3d(-498.475, 215.8, 0.0), cv::Point3d(-249.175, -215.8, 0.0), cv::Point3d(249.175, -215.8, 0.0)}; //outside hex corners in mm from the center.
  const std::vector<cv::Point3d> mRectModelPoints{cv::Point3d(88.9, 139.7, 0.0), cv::Point3d(-88.9, 139.7, 0.0), cv::Point3d(-88.9, -139.7, 0.0), cv::Point3d(88.9, -139.7, 0.0)}; //outside rectangular corners in mm from the center.

  double focal_length = 2.9272781257541; //mm 
  cv::Point2d center = cv::Point2d(160.0, 120.0);
  //cv::Mat mCameraMatrix = (cv::Mat_<double>(3,3) << focal_length, 0.0, center.x, 0.0, focal_length, center.y, 0.0, 0.0, 1.0); //go to https://readthedocs.org/projects/limelight/downloads/pdf/latest/
  cv::Mat mCameraMatrix = (cv::Mat_<double>(3,3) << 772.53876202/3.0, 0.0, 479.132337442/3.0, 0.0, 769.052151477/3.0, 359.143001808/3., 0.0, 0.0, 1.0);
  cv::Mat mDistCoeffs = (cv::Mat_<double>(5,1) << 0.29684613693070039, -1.4380252254747885, -0.0022098421479494509, -.003389456353390716, 2.5344430354806740);

  
  std::vector<VisionTargeting::TargetCorner> mTargetCorners;
  bool mSeesTarget = false;

  int i = 0;
 public:
  
  enum LedMode { PIPELINE, OFF, BLINK, ON};

  Limelight(std::shared_ptr<LimelightConstants> constants);
  Limelight(){}

  std::shared_ptr<Pose2D> getTurretToLens(){return mConstants->kTurretToLens;}

  double getLensHeight(){return mConstants->kHeight;}

  std::shared_ptr<Rotation2D> getHorizontalPlaneToLens(){return mConstants->kHorizontalPlaneToLens;}

  std::string getName(){return mConstants->kName; }

  void readPeriodicInputs() override;

  void writePeriodicOutputs() override;

  void outputTelemetry() override;
  
  void setLed(LedMode mode);

  void setPipeline(int mode);

  void triggerOutputs();

  int getPipeline(){return mPeriodicIO.pipeline;}

  bool seesTarget(){return mSeesTarget;}

  std::vector<VisionTargeting::TargetCorner> getCorners();

  double getLatency();


  void XYToTargetCorner(std::vector<double> Xs, std::vector<double> Ys); //converts mCornX and mCornY to TargetInfo for easier sorting

  void TargetCornerToCVPoint2d(std::vector<VisionTargeting::TargetCorner> corners, std::vector<cv::Point2d> &imagePoints);

  void filterTargetCorners(std::vector<VisionTargeting::TargetCorner> torgetCorners); //filters down to 4 outside corners

  VisionTargeting::TargetInfo getCameraXYZ();
  std::shared_ptr<Rotation2D> getAngleToTarget();
};



}