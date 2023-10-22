#pragma once

#include "AMPCore.h"

namespace amp{
class CollisionChecker{
    public:
        bool checkCollision(const amp::Environment2D& problem, const Eigen::Vector2d currentPosition);
};
}