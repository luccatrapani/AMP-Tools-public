#include "pointAgentCSpace.h"

amp::pointAgentCSpace::pointAgentCSpace(const Problem2D& problem, const Eigen::VectorXd& lower_bounds, const Eigen::VectorXd& upper_bounds) : 
    amp::ConfigurationSpace::ConfigurationSpace(lower_bounds, upper_bounds), m_problem(problem){}



bool amp::pointAgentCSpace::inCollision(const Eigen::VectorXd& cspace_state) const {
    amp::CollisionChecker collisionChecker;

    Eigen::Vector2d state2D = {cspace_state[0], cspace_state[1]};
    
    bool collision = collisionChecker.checkCollision(m_problem, state2D);

    return collision;
}