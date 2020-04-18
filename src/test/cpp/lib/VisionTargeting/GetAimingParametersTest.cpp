#include "gtest/gtest.h"

#include <cmath>
#include <memory>

class GetAimingParametersTest : public ::testing::Test {
  protected:
    double kEpsilon = 1E-4;
    virtual void SetUp(){}
    virtual void TearDown(){}
};