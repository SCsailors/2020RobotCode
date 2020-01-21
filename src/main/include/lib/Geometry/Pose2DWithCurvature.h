/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <memory>
#include "lib/Geometry/Pose2D.h"
#include "lib/Util/Util.h"
using namespace std;
 
class Pose2DWithCurvature {
  shared_ptr<Util> util=make_shared<Util>();
 public:
 double curvature_;
 double dcurvature_ds_;
  shared_ptr<Pose2D> pose_;
  Pose2DWithCurvature();
  Pose2DWithCurvature(shared_ptr<Pose2D> pose, double curvature, double dcurvature_ds);

  shared_ptr<Pose2D> getPose();
  shared_ptr<Rotation2D> getRotation();
  shared_ptr<Translation2D> getTranslation();

  shared_ptr<Pose2DWithCurvature> transformBy(shared_ptr<Pose2D> transform);
  double getCurvature();
  double getDCurvatureDs();

  double getDistance(shared_ptr<Pose2DWithCurvature> other);

  shared_ptr<Pose2DWithCurvature> interpolate(shared_ptr<Pose2DWithCurvature> other, double x);
  shared_ptr<Pose2DWithCurvature> mirror();
};
