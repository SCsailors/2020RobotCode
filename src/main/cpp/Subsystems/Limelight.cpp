/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/Limelight.h"

#include "Constants.h"
using namespace Subsystems;
Limelight::Limelight(std::shared_ptr<LimelightConstants> constants) 
{
    mConstants = constants;
    mNetworkTable = nt::NetworkTableInstance::GetDefault().GetTable(mConstants->kTableName);


}

void Limelight::readPeriodicInputs()
{
    frc::SmartDashboard::PutNumber(mConstants->kName + "/reading inputs", i++);
    //#ifdef CompetitionBot
    mPeriodicIO.givenLedMode = (int) mNetworkTable->GetEntry("ledmode").GetDouble(0.0);
    mPeriodicIO.givenPipeline = (int) mNetworkTable->GetEntry("pipeline").GetDouble(0.0);
    mPeriodicIO.latency = mNetworkTable->GetEntry("tl").GetDouble(0.0)/ 1000.0 + mPeriodicIO.givenPipeline == 1? Constants::kImageCaptureFastLatency: Constants::kImageCaptureSlowLatency;
    mPeriodicIO.xOffset = mNetworkTable->GetEntry("tx").GetDouble(0.0);
    mPeriodicIO.yOffset = mNetworkTable->GetEntry("ty").GetDouble(0.0);
    mPeriodicIO.xOffsetRaw = mNetworkTable->GetEntry("tx0").GetDouble(0.0);
    mPeriodicIO.yOffsetRaw = mNetworkTable->GetEntry("ty0").GetDouble(0.0);    
    mPeriodicIO.area = mNetworkTable->GetEntry("ta").GetDouble(0.0);
    mPeriodicIO.horPixels = mNetworkTable->GetEntry("thor").GetDouble(0.0);
    mPeriodicIO.vertPixels = mNetworkTable->GetEntry("tvert").GetDouble(0.0);
    mPeriodicIO.skewRaw = mNetworkTable->GetEntry("ts").GetDouble(0.0);
    mPeriodicIO.skew = (mPeriodicIO.skewRaw + mPeriodicIO.skewRaw < -45.0 ? 90.0 : 0.0); //adjust everything to be around 0: add scale constant to adjust it to degrees
    
    mSeesTarget = mNetworkTable->GetEntry("tv").GetDouble(0.0) == 1.0;
    //#endif
    if (mPeriodicIO.area >= 1.15) // tune: first guess
    {
        setPipeline(0);
    } else
    {
        setPipeline(1);
    }
}

void Limelight::writePeriodicOutputs()
{
    #ifdef CompetitionBot
    if(mPeriodicIO.givenLedMode != mPeriodicIO.ledMode || mPeriodicIO.givenPipeline != mPeriodicIO.pipeline)
    {
        //std::cout << "Table changed from expected. Retrigger!"<< std::endl;
        mOutputsHaveChanged = true;
    }
    if (mOutputsHaveChanged)
    {
        mNetworkTable->GetEntry("ledMode").SetDouble(mPeriodicIO.ledMode);
        mNetworkTable->GetEntry("camMode").SetDouble(mPeriodicIO.camMode);
        mNetworkTable->GetEntry("pipeline").SetDouble(mPeriodicIO.pipeline);
        mNetworkTable->GetEntry("stream").SetDouble(mPeriodicIO.stream);
        mNetworkTable->GetEntry("snapshot").SetDouble(mPeriodicIO.snapshot);
    
        mOutputsHaveChanged = false;
    }
    #endif
}

void Limelight::outputTelemetry()
{
    frc::SmartDashboard::PutNumber(mConstants->kName +"/Pipeline Latency (ms)", mPeriodicIO.latency);
    frc::SmartDashboard::PutBoolean(mConstants->kName + "/Has Target", mSeesTarget);
}

void Limelight::setLed(LedMode mode)
{ 
    frc::SmartDashboard::PutNumber(mConstants->kName + "/ mode", mode);
    if (mode !=mPeriodicIO.ledMode)
    {
        mPeriodicIO.ledMode = mode;
        mOutputsHaveChanged = true;
    }
}

void Limelight::setPipeline(int mode)
{
    if (mode != mPeriodicIO.pipeline)
    {

        mPeriodicIO.pipeline = mode;
        mOutputsHaveChanged = true;
        std::cout<<"changing pipeline"<<std::endl;
    }
}

void Limelight::triggerOutputs()
{
    mOutputsHaveChanged = true;
}

double Limelight::getLatency()
{
    return mPeriodicIO.latency;
}

std::vector<VisionTargeting::TargetCorner> Limelight::getCorners()
{   
    mTargetCorners.clear();
    mCornX = mNetworkTable->GetEntry("tcornx").GetDoubleArray(mZeroArray);
    mCornY = mNetworkTable->GetEntry("tcorny").GetDoubleArray(mZeroArray);

    XYToTargetCorner(mCornX, mCornY);
    if (mTargetCorners.size() == 4)
    {
        return mTargetCorners;
    } else if (mTargetCorners.size() > 4)
    { //filter to 4 outside corners
        filterTargetCorners(mTargetCorners);
    }
    
    return mTargetCorners;
}

void Limelight::XYToTargetCorner(std::vector<double> Xs, std::vector<double> Ys)
{
    for (int i = 0; i <Xs.size()-1 ; i++)
    {
        //for each corner, create a targetinfo class to store coordinates
        mTargetCorners.push_back(VisionTargeting::TargetCorner{Xs.at(i), Ys.at(i)});
    }
}

void Limelight::filterTargetCorners(std::vector<VisionTargeting::TargetCorner> targetCorners)
{
    VisionTargeting::TargetCorner tr{};
    VisionTargeting::TargetCorner br{};
    VisionTargeting::TargetCorner bl{};
    VisionTargeting::TargetCorner tl{};

    //divide into quadrants and keep farthest ones in each quadrant
    for (auto corner : targetCorners)
    {
        if (corner.getTheta() >= 90.0 ) //tl
        {
            if (corner.getRadius() > tl.getRadius())
            {
                tl = corner;
            }
        } else if (corner.getTheta() <= -90.0 ) //bl
        {
            if (corner.getRadius() > bl.getRadius())
            {
                bl = corner;
            }
        } else if (corner.getTheta() >= -90.0 && corner.getTheta() <= 0.0 ) //br
        {
            if (corner.getRadius() > br.getRadius())
            {
                br = corner;
            }
        } else if (corner.getTheta() <= 90.0 && corner.getTheta() > 0.0 ) //tr
        {
            if (corner.getRadius() > tr.getRadius())
            {
                tr = corner;
            }
        } else 
        {
            std::cout <<"corner failed to be put in a quadrant! "<< "Theta: " << corner.getTheta() <<std::endl;
        }
    }
    mTargetCorners.clear();
    mTargetCorners.push_back(tr);
    mTargetCorners.push_back(tl);
    mTargetCorners.push_back(bl);
    mTargetCorners.push_back(br);

}

VisionTargeting::TargetInfo Limelight::getCameraXYZ()
{
    //std::cout << "Starting GetCameraXYZ()" << std::endl;
    if (mPeriodicIO.horPixels == 0.0 && mPeriodicIO.vertPixels == 0.0)
    {
        
        frc::SmartDashboard::PutBoolean("Subsystems/" + mConstants->kName + "/No Target: getCameraXYZ:", true); 
        //std::cout << "Returning empty Target" << std::endl;
        return VisionTargeting::TargetInfo{};
    } 
    cv::Mat mRotationVector; //axis angle form
    cv::Mat mTranslationVector;
    if (mPeriodicIO.givenPipeline == 0)
    {
        std::vector<double> camTran = mNetworkTable->GetEntry("camtran").GetDoubleArray(mZeroArray);
        if (util.epsilonEquals(camTran.at(3), 0.0, 5.0))
        { //have a target with an x distance of 0 inches: not valid target
            setPipeline(1);
            return VisionTargeting::TargetInfo{};
        }
        
        return VisionTargeting::TargetInfo{camTran};

    } else if (mPeriodicIO.givenPipeline == 1)
    {
        frc::SmartDashboard::PutBoolean("Subsystems/" + mConstants->kName + "/No Target: getCameraXYZ:", false);
    
    std::vector<VisionTargeting::TargetCorner> corners = getCorners();
    if (corners.size() < 4)
    {
        std::cout << "Don't have 4 points: "<< mConstants->kName << std::endl;
        return VisionTargeting::TargetInfo{};
    }
    
    std::vector<cv::Point2d> imagePoints;
    
    TargetCornerToCVPoint2d(corners, imagePoints);
    //std::cout << "Convert corners to CVPoint2d" << std::endl;
    
        if (mPeriodicIO.horPixels > mPeriodicIO.vertPixels) 
        {
            //hex goal
            //std::cout<<"Using Hex Model: getCameraXYZ()"<< mConstants->kName<<std::endl;
            cv::solvePnP(mHexModelPoints, imagePoints, mCameraMatrix, mDistCoeffs, mRotationVector, mTranslationVector, false, cv::SOLVEPNP_P3P);
            //std::cout << "Used Hex Model" << std::endl; 
        } else 
        {
            //rectangular goal
            //std::cout<<"Using Rect Model: getCameraXYZ()"<< mConstants->kName<<std::endl;
            cv::solvePnP(mRectModelPoints, imagePoints, mCameraMatrix, mDistCoeffs, mRotationVector, mTranslationVector, false, cv::SOLVEPNP_P3P);
            //std::cout << "Used Rectangular Model" << std::endl;
        }
        
    
    
    return VisionTargeting::TargetInfo{mTranslationVector, mRotationVector};
    }
    
}

void Limelight::TargetCornerToCVPoint2d(std::vector<VisionTargeting::TargetCorner> corners, std::vector<cv::Point2d> &imagePoints)
{
    for (auto corner : corners)
    {
        imagePoints.push_back( cv::Point2d( corner.getY(), corner.getZ() ) );
    }
}

std::shared_ptr<Rotation2D> Limelight::getAngleToTarget()
{
    
    frc::SmartDashboard::PutNumber("Limelight Angle to Target", mPeriodicIO.xOffset);
    return Rotation2D::fromDegrees(mPeriodicIO.xOffset);
}
