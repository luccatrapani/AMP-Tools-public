#include "gradientDescent.h"

namespace amp{

double gradientDescent::operator()(const Eigen::Vector2d& q) const {
    double out = 3;
    return out;
}

Path2D gradientDescent::plan(const Problem2D& problem){
    double dStar = 5;
    double QStar = 2;
    double eps = .25;
    
    // Initialize path, start first waypoint
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    // Boolean goal reached value
    bool goalReached = false;

    // Initialize current position, get myself off the integers;
    Eigen::Vector2d addDouble(1.0E-16, 1.0E-16);
    Eigen::Vector2d posOld = problem.q_init+addDouble;

    // Start while loop, goal checking

    while (goalReached==false){
        // Take a step
        Eigen::Vector2d posNext = gradientDescent::takeStep(problem, posOld, dStar, QStar, eps);

        
        path.waypoints.push_back(posNext);
        Eigen::Vector2d vec2Goal = posNext - problem.q_goal;
        if (vec2Goal.norm() < .001) {goalReached = true; path.waypoints.push_back(problem.q_goal);};

        posOld = posNext;
 
    }
    

    return path;
}

Eigen::Vector2d gradientDescent::takeStep(const Problem2D& problem, const Eigen::Vector2d& pos, const double& dStar, const double& QStar, const double& eps){
    Eigen::Vector2d posNext;
    std::vector<Eigen::Vector2d> minObsPoints = gradientDescent::rangeSensor(problem, pos);
    int numObstacles = minObsPoints.size();
    std::vector<double> minObsDists;

    double closestObsDist = 1.0E+16;
    for(int k=0; k<numObstacles; k++){
        Eigen::Vector2d vec2Obs = minObsPoints[k] - pos;
        double obsDist = vec2Obs.norm();
        minObsDists.push_back(obsDist);

        if(obsDist < closestObsDist) {closestObsDist = obsDist;};
    }

    Eigen::Vector2d vec2Goal = problem.q_goal - pos;
    double dist2Goal = vec2Goal.norm();

    Eigen::Vector2d gradient = gradientDescent::getGradient(problem, pos, dStar, QStar, minObsPoints);

    if (gradient.norm() > eps){
        if (dist2Goal>closestObsDist){
            double step = closestObsDist/2;
            posNext = pos - step*(gradient/gradient.norm());
        } else {
            double step = dist2Goal/2;
            posNext = pos - step*(gradient/gradient.norm());
        }

    } else{
        posNext = problem.q_goal;
    }

    return posNext;
}

Eigen::Vector2d gradientDescent::getGradient(const Problem2D& problem, const Eigen::Vector2d& pos, const double& dStar, const double& QStar, const std::vector<Eigen::Vector2d>& minObsPoints){
    Eigen::Vector2d grad;
    Eigen::Vector2d gradAtt;
    Eigen::Vector2d gradRep(0, 0);

    double zeta = 10;  double eta = 1;
    int numObstacles = problem.obstacles.size(); //Get number of obstacles

    //Find distance to goal
    Eigen::Vector2d vec2Goal = pos - problem.q_goal;
    double dist2Goal = vec2Goal.norm();
    if(dist2Goal<=dStar){
        gradAtt = zeta*(pos - problem.q_goal);
    } else {
        gradAtt = (dStar*zeta*(pos - problem.q_goal))/dist2Goal;
    }

    // Now loop through each obstacle
    for (int k=0; k<numObstacles; k++){
        Eigen::Vector2d c = minObsPoints[k];
        Eigen::Vector2d vec2Obs = pos - c;
        double dist2Obs = vec2Obs.norm();
        Eigen::Vector2d gradRep_k;
        if(dist2Obs<=QStar){
            gradRep = gradRep + eta*((1/dist2Obs)-(1/QStar))*-(((pos-c)/dist2Obs)/(dist2Obs*dist2Obs));
        }
    }
    // Add gradients together
    grad = gradAtt+gradRep;


    return grad;
}

std::vector<Eigen::Vector2d> gradientDescent::rangeSensor(const Problem2D& problem, const Eigen::Vector2d& pos){
    int numObstacles = problem.obstacles.size(); //Get number of obstacles
    double thetaStep = .1;
    std::vector<Eigen::Vector2d> minObsPoints;

    //Loop through each obstacle (polygon)
    for (int k=0; k<numObstacles; k++){
        //For each polygon, get CCW ordered list of vertices
        std::vector<Eigen::Vector2d> vertices = problem.obstacles[k].verticesCCW();
        //Find number of vertices
        int numVertices = vertices.size();

        double minDistance = 1.0E+16;

        Eigen::Vector2d minPoint;

        double theta = 0;

        while(theta < 2*std::numbers::pi){
            // Define d vector from point
            Eigen::Vector2d dUnitVec(std::cos(theta), std::sin(theta));
            // Loop through the vertices and perform range checking 
            //j vertex is always previous vertex, i vertex is always current vertex
            for(int i=0, j = numVertices-1; i<numVertices; j = i++){
                Eigen::Vector2d a = vertices[i]; Eigen::Vector2d b = vertices[j];

                Eigen::Vector2d v1 = pos-a; Eigen::Vector2d v2 = b-a; Eigen::Vector2d v3(-dUnitVec[1], dUnitVec[0]);

                double v2_cross_v1 = v2[0]*v1[1] - v2[1]*v1[0];
                double v2_dot_v3 = v2.dot(v3);
                if (v2_dot_v3 == 0) {v2_dot_v3 = -1;};

                double t1 = v2_cross_v1/v2_dot_v3;  double t2 = (v1.dot(v3))/(v2_dot_v3);

                if(t1>=0 && (t2>=0 && t2<=1)){
                    Eigen::Vector2d intersectionPoint = pos + dUnitVec*t1;
                    Eigen::Vector2d vec2IntersectionPoint = pos-intersectionPoint;
                    double dist2Intersection = vec2IntersectionPoint.norm();

                    if(dist2Intersection < minDistance){
                        minPoint = intersectionPoint;
                        minDistance = dist2Intersection;
                    }
                }

            }

            theta = theta + thetaStep;
        }
        minObsPoints.push_back(minPoint);



    }
    return minObsPoints;
}

}