#pragma once

#include "AMPCore.h"

class BugTools{
    public:
       static bool checkCollision(const amp::Problem2D& problem, const Eigen::Vector2d currentPosition);
       static Eigen::Vector2d TurnRight(const Eigen::Vector2d currentHeading);
       static Eigen::Vector2d TurnLeft(const Eigen::Vector2d currentHeading);
};

