/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/Util/CSVWriter.h"

CSVWriter::CSVWriter(){}


CSVWriter::CSVWriter(string filename, string fields) {
    filename_=filePath+filename+Timestamp()+".csv";
    open(fields);
}


void CSVWriter::open(string typeClass){
     //open an existing or creates a new file.
     frc::SmartDashboard::PutString("filename: ", filename_);
    mOutput.open(filename_, std::ios::out | ios::app);
    if(mOutput){
        mOutput<<typeClass<<","<< endl; //in each logging file there needs to be getFields(), toCSV(), toString() for SmartDashBoard
        open_=true;
    }else{
        frc::SmartDashboard::PutBoolean("couldn't open log file", true);
    }
    //mOutput.close();
}

std::string CSVWriter::Timestamp(){
    std::ostringstream stream;
    time_t rawtime;
    tm * timeinfo;
    time(&rawtime);
    timeinfo= localtime(&rawtime);
    
    stream <<timeinfo->tm_mday<<"-"<<timeinfo->tm_mon<<"-"<<(timeinfo->tm_year)+1900<<"-"
    <<timeinfo->tm_hour<<":"<<timeinfo->tm_min<<":"<<timeinfo->tm_sec;
    return stream.str();
}


void CSVWriter::add(string value){
    if (mOutput){
        ++i;
        if(i%update_rate!=0){
            mOutput<<value<<","<<"\n";
        } else{
            mOutput<<value<<","<<endl;
        }
    }else{
        cout<<"couldn't write to file"<<endl;
    }
}

void CSVWriter::close(){
    if(mOutput){
        mOutput.flush();
        mOutput.close();
        open_=false;
    }
}

bool CSVWriter::open(){
    return open_;
}

string CSVWriter::filename(){
    return filename_;
}