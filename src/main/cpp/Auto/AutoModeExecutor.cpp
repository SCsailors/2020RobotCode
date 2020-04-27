/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Auto/AutoModeExecutor.h"
#include "frc/smartdashboard/SmartDashboard.h"

AutoModeExecutor::AutoModeExecutor(shared_ptr<AutoModeBase> new_auto_mode) {
    m_auto_mode=new_auto_mode;
}

void AutoModeExecutor::start(){
    m_auto_mode->run();
}

void AutoModeExecutor::stop(){
    if (m_auto_mode->isActive()){
        cout<<"Ending Auto Mode early"<<endl;
        m_auto_mode->setActive(false);
        cout<<"stopped automode"<<endl;
    }
    
    //RunAuto.Stop();
}

shared_ptr<AutoModeBase> AutoModeExecutor::getAutoMode(){
    return m_auto_mode;
}

void AutoModeExecutor::setAutoMode(shared_ptr<AutoModeBase> autoMode){
    m_auto_mode=autoMode;
}