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
    mTargetName = mConstants->kDefaultTargetName;

}

void Limelight::readPeriodicInputs()
{
    //frc::SmartDashboard::PutNumber(mConstants->kName + "/reading inputs", i++);
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
    /*
    std::vector<double> cornxy = mNetworkTable->GetEntry("tcornxy").GetDoubleArray(mZeroArray);

    frc::SmartDashboard::PutNumber("Vision Targeting/X0", cornxy.at(0));
    frc::SmartDashboard::PutNumber("Vision Targeting/Y0", cornxy.at(1));
    frc::SmartDashboard::PutNumber("Vision Targeting/X1", cornxy.at(2));
    frc::SmartDashboard::PutNumber("Vision Targeting/Y1", cornxy.at(3));
    frc::SmartDashboard::PutNumber("Vision Targeting/X2", cornxy.at(4));
    frc::SmartDashboard::PutNumber("Vision Targeting/Y2", cornxy.at(5));
    frc::SmartDashboard::PutNumber("Vision Targeting/X3", cornxy.at(6));
    frc::SmartDashboard::PutNumber("Vision Targeting/Y3", cornxy.at(7));
    */
    std::vector<double> camtran = mNetworkTable->GetEntry("camtran").GetDoubleArray(mZeroArray);

    frc::SmartDashboard::PutNumber("Vision Targeting/X", camtran.at(0));
    frc::SmartDashboard::PutNumber("Vision Targeting/Y", camtran.at(1));
    frc::SmartDashboard::PutNumber("Vision Targeting/Z", camtran.at(2));
    frc::SmartDashboard::PutNumber("Vision Targeting/X Theta", camtran.at(3));
    frc::SmartDashboard::PutNumber("Vision Targeting/Y Theta", camtran.at(4));
    frc::SmartDashboard::PutNumber("Vision Targeting/Z Theta", camtran.at(5));

    //#endif
    //if (mPipelineSwitch.update(mPeriodicIO.area >= 1.15, .3)) // tune: first guess
    //{
    //    PnPpipeline = !PnPpipeline;
    //}

    if (PnPpipeline) 
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
        std::cout<<"changing pipeline to: "<< mPeriodicIO.pipeline <<std::endl;
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

void Limelight::getCorners(std::vector<cv::Point2d> &imagePoints)
{
    getCorners(getLimelightCornerXYs(), imagePoints);
}

void Limelight::getCorners(std::vector<double> cornerXY, std::vector<cv::Point2d> &imagePoints)
{   
    double timestamp = frc::Timer::GetFPGATimestamp();
    std::vector<VisionTargeting::TargetCorner> filteredTargetCorners;
    
    if (cornerXY.size() == 8)
    {
        XYToTargetCorner(cornerXY, filteredTargetCorners);
    } else if (cornerXY.size() > 8)
    { //filter to 4 outside corners
        std::vector<VisionTargeting::TargetCorner> targetCorners;
        XYToTargetCorner(cornerXY, targetCorners);
        
        filterTargetCorners(targetCorners, filteredTargetCorners);
    } else 
    { //fill to zero if nothing worked
        std::cout << "Don't have 4 points: "<< mConstants->kName << std::endl;
        XYToTargetCorner(mZeroArray, filteredTargetCorners);
    }

    TargetCornerToCVPoint2d(filteredTargetCorners, imagePoints);
    
}

std::vector<double> Limelight::getLimelightCornerXYs()
{
    return mNetworkTable->GetEntry("tcornxy").GetDoubleArray(mZeroArray);
}

bool Limelight::getLimelightCornersValid(std::vector<cv::Point2d> imagePoints)
{
    for ( auto point : imagePoints)
    {
        if (util.epsilonEquals(point.x, 0.0) && util.epsilonEquals(point.y, 0.0))
        {
            return false;
        }
    }
    return true;

}

void Limelight::XYToTargetCorner(std::vector<double> XYs, std::vector<VisionTargeting::TargetCorner> &targetCorners)
{
    double centerX = 0.0;
    double centerY = 0.0;
    if (XYs.size() > 4)
    {
        for ( int k = 0; k < 4; k+=2)
        {
            centerX += XYs.at(k);
            centerY += XYs.at(k+1);
        }
        centerX /= 2.0;
        centerY /= 2.0;
    }
    frc::SmartDashboard::PutNumber("CenterX", centerX);
    frc::SmartDashboard::PutNumber("CenterY", centerY);
    
    targetCorners.clear();
    for (int i = 0; i < XYs.size()-1 ; i+=2)
    {
        //for each corner, create a targetinfo class to store coordinates
        targetCorners.push_back(VisionTargeting::TargetCorner{XYs.at(i), XYs.at(i+1)});
    }
}

void Limelight::filterTargetCorners(std::vector<VisionTargeting::TargetCorner> targetCorners, std::vector<VisionTargeting::TargetCorner> &filteredTargetCorners)
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
    filteredTargetCorners.clear();
    filteredTargetCorners.push_back(tl);
    filteredTargetCorners.push_back(tr);
    filteredTargetCorners.push_back(br);
    filteredTargetCorners.push_back(bl);
    

}

VisionTargeting::TargetInfo Limelight::getCameraXYZ()
{
    return getCameraXYZ(getLimelightCornerXYs());
}

VisionTargeting::TargetInfo Limelight::getCameraXYZ(std::vector<double> cornerXYs)
{
    //differentiate between the two targets and set mTargetname to the right one
    if (mPeriodicIO.horPixels > mPeriodicIO.vertPixels) 
    {   
        mTargetName = Constants::PowerPort;
    } else
    {   
        mTargetName = Constants::LoadingBay;
    }
    double timestamp = frc::Timer::GetFPGATimestamp();
    //std::cout << "Starting GetCameraXYZ()" << std::endl;
    
    //No Target
    if (mPeriodicIO.horPixels == 0.0 && mPeriodicIO.vertPixels == 0.0)
    {
        
        frc::SmartDashboard::PutBoolean("Subsystems/" + mConstants->kName + "/No Target: getCameraXYZ:", true); 
        //std::cout << "Returning empty Target" << std::endl;
        return VisionTargeting::TargetInfo{mConstants->kDefaultTargetName};
    } 
    cv::Mat mRotationVector; //axis angle form
    cv::Mat mTranslationVector;
    
    //Use Limelight PnP
    if (mPeriodicIO.givenPipeline == 0)
    {
        std::vector<double> camTran = mNetworkTable->GetEntry("camtran").GetDoubleArray(mZeroArray);
        if (util.epsilonEquals(camTran.at(3), 0.0, 5.0))
        { //have a target with an x distance of 0 inches: not valid target
            //setPipeline(1);
            return VisionTargeting::TargetInfo{mConstants->kDefaultTargetName};
        }
        
        return VisionTargeting::TargetInfo{camTran, mConstants->kDefaultTargetName};

    //Use Rio PnP
    } else if (mPeriodicIO.givenPipeline == 1)
    {
        frc::SmartDashboard::PutBoolean("Subsystems/" + mConstants->kName + "/No Target: getCameraXYZ:", false);

        std::vector<cv::Point2d> imagePoints;
        
        getCorners(cornerXYs, imagePoints);

        //check no points are equal to (0.0, 0.0): crashes solvePnP();
        if (!getLimelightCornersValid(imagePoints))
        {
            return VisionTargeting::TargetInfo{mConstants->kDefaultTargetName};
        }

        //Decide which Target
        if (mPeriodicIO.horPixels > mPeriodicIO.vertPixels) 
        {
            //hex goal
            //std::cout<<"Using Hex Model: getCameraXYZ()"<< mConstants->kName<<std::endl;
            cv::solvePnPRansac(mHexModelPoints, imagePoints, mCameraMatrix, mDistCoeffs, mRotationVector, mTranslationVector, false, 100, 2.0, cv::SOLVEPNP_P3P); //, 
            //std::cout << "Used Hex Model" << std::endl;

            
        } else 
        {
            //rectangular goal
            //std::cout<<"Using Rect Model: getCameraXYZ()"<< mConstants->kName<<std::endl;
            cv::solvePnP(mRectModelPoints, imagePoints, mCameraMatrix, mDistCoeffs, mRotationVector, mTranslationVector, false);
            //std::cout << "Used Rectangular Model" << std::endl;
        } 

    cv::Mat R;
    cv::Rodrigues(mRotationVector, R);
    R = R.t();
    mTranslationVector = -R * mTranslationVector;
    
    return VisionTargeting::TargetInfo{mTranslationVector, mRotationVector, mConstants->kDefaultTargetName};
    }
    //should never reach here
    return VisionTargeting::TargetInfo{mConstants->kDefaultTargetName};
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
