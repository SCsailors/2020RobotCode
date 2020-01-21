/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <vector>
#include <memory>
#include "frc/smartdashboard/SmartDashboard.h"
using namespace std;




class CSVWriter {
  bool open_=false;
  fstream mOutput;
  string filename_;
  string filePath="/home/lvuser/";//"home/lvuser/";
  //const string filePath="/media/sda1/data_captures/";
  vector<string> mfields;
  string line_;
  int i=0;
  int update_rate=4;
 public:
  CSVWriter();
  
  CSVWriter(string filename, string fields);
  string Timestamp();
  void open(string typeClass);
  void add(string value);
  void close();
  bool open();
  string filename();
};
