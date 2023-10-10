#pragma once

#include "AMPCore.h"

namespace amp{
class CSpaceTools : public ConfigurationSpace2D{
    using ConfigurationSpace2D::ConfigurationSpace2D; 
    virtual bool inCollision(double x0, double x1) const;
};




}