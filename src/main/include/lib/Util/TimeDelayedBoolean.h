/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Timer.h>

namespace Utility{

class TimeDelayedBoolean {
  frc::Timer t{};
  bool m_old = false;
 public:
  TimeDelayedBoolean(){}
  bool update(bool value, double timeout)
  {
    if (!m_old && value)
    {
      t.Reset();
      t.Start();
    }
    m_old = value;
    return value && t.Get() >= timeout;
  }
};
}