#pragma once

#include "AMPCore.h"
#include "hw/HW5.h"

namespace amp{

class gradientDescent : public PotentialFunction2D, public GDAlgorithm{
    public:
        virtual double operator()(const Eigen::Vector2d& q) const;

        virtual Path2D plan(const Problem2D& problem) override;

        Eigen::Vector2d takeStep(const Problem2D& problem, const Eigen::Vector2d& pos, const double& dStar, const double& QStar, const double& eps);

        Eigen::Vector2d getGradient(const Problem2D& problem, const Eigen::Vector2d& pos, const double& dStar, const double& QStar, const std::vector<Eigen::Vector2d>& minObsPoints);

        std::vector<Eigen::Vector2d> rangeSensor(const Problem2D& problem, const Eigen::Vector2d& pos);
};
}