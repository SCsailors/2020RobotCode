/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/Modes/CharacterizeHighGear.h"

CharacterizeHighGear::CharacterizeHighGear() {
    
    
}

void CharacterizeHighGear::routine(){
    shared_ptr<CollectVelocityData> mVelData=make_shared<CollectVelocityData>(velocityData, false, false, false);
    shared_ptr<CollectAccelerationData> mAccelData=make_shared<CollectAccelerationData>(accelerationData, false, false, false);
    
    runAction(mVelData);
    runAction(make_shared<WaitAction>(3.0));
    runAction(mAccelData);
    
    if(mVelData->mVelocityData.size()==0 || mAccelData->mAccelerationData.size()==0){
      cout<<"no data collected"<< endl;
      cout<<"Vel: "<<mVelData->mVelocityData.size()<<endl;
      cout<<"Vel: "<<mAccelData->mAccelerationData.size()<<endl;
      frc::SmartDashboard::PutString("No acceleration or velocity data", "true");
    }else{
        cout<<"Characterizing Drive"<<endl;
    shared_ptr<DriveCharacterization> characterize = make_shared<DriveCharacterization>();
    cout<<"Starting Characterization"<<endl;
    shared_ptr<DriveCharacterization::CharacterizationConstants> constants = characterize->characterizeDrive(mVelData->mVelocityData, mAccelData->mAccelerationData);

    frc::SmartDashboard::PutNumber("ks: ", constants->ks);
    frc::SmartDashboard::PutNumber("kv: ", constants->kv);
    frc::SmartDashboard::PutNumber("ka: ", constants->ka);

    cout<<"ks: "<<constants->ks<<endl;
    cout<<"kv: "<<constants->kv<<endl;
    cout<<"ka: "<<constants->ka<<endl;
    }
    
  }

string CharacterizeHighGear::getID(){
  return "DriveCharacterization";
}