#include "MyManipWFAlgo.h"

amp::MyManipWFAlgo::MyManipWFAlgo() : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) {}

amp::Path2D amp::MyManipWFAlgo::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace){
    amp::Path2D path;
    Eigen::Vector2d r(0, 0);
    path.waypoints.push_back(r);
    return path;
}