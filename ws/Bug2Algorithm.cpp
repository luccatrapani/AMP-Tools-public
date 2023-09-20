#include "Bug2Algorithm.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug2Algorithm::plan(const amp::Problem2D& problem) {
    // Initialize path, start first waypoint
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    
    // Use a stepsize of .1 for now
    double stepSize = .01;

    // Initialize right checking matrix
    Eigen::Matrix2d R;
    R << 0, 1, -1, 0;
    
    // Boolean goal reached value
    bool goalReached = false;

    // Initialize current position, get myself off the integers;
    Eigen::Vector2d addDouble(1.0E-16, 1.0E-16);
    Eigen::Vector2d posOld = problem.q_init+addDouble;

    // Initialize m-line values
    double x_1 = posOld[0]; double y_1 = posOld[1]; double x_2 = problem.q_goal[0]; double y_2 = problem.q_goal[1];

    // While loop to get us started
    while (goalReached == false){
        // Move forward toward goal
        Eigen::Vector2d Vec2Goal(problem.q_goal[0]-posOld[0], problem.q_goal[1]-posOld[1]);
        Eigen::Vector2d unitVec2Goal = Vec2Goal/Vec2Goal.norm();
        Eigen::Vector2d pos = stepSize*unitVec2Goal + posOld;

        // Check for collision
        bool collision = BugTools::checkCollision(problem, pos);
        if (collision==true){
            // Record hit
            Eigen::Vector2d posHit = posOld;
            double hitDist2Goal = (posHit - problem.q_goal).norm();
            // Step back
            pos = posOld;
            Eigen::Vector2d heading = unitVec2Goal;

            // Begin wall following
            bool followWall = true;
            Eigen::Vector2d closestPos;
            double dist2Goal = 1.0E16;

            // Initialize for while loop
            bool canLeave = false;
            bool turnLeft;
            bool checkRight;
            bool turnRight;
            double x_pos;
            double y_pos;
            Eigen::Vector2d newHeading;
            Eigen::Vector2d posCurrent;
            Eigen::Vector2d closestPointFinal;
            Eigen::Vector2d Vec2Hit;

            while (followWall == true){
                // Try taking a step forward
                Eigen::Vector2d posForward = stepSize*heading + pos;
                turnLeft = BugTools::checkCollision(problem, posForward);
                checkRight = !turnLeft;

                while (turnLeft == true){
                    Eigen::Vector2d newHeading = BugTools::TurnLeft(heading);
                    posForward = stepSize*newHeading + pos;
                    turnLeft = BugTools::checkCollision(problem, posForward);
                    heading = newHeading;
                }

                posCurrent = posForward;
                path.waypoints.push_back(posForward);
                

                if (checkRight == true){
                    Eigen::Vector2d posForwardTest;
                    Eigen::Vector2d posRightTest;
                    Eigen::Vector2d posRight = stepSize*R*heading + posCurrent;
                    turnRight = !BugTools::checkCollision(problem, posRight);

                    if (turnRight == true){
                        while (turnRight == true){
                            Eigen::Vector2d newHeading = BugTools::TurnRight(heading);
                            posForwardTest = stepSize*newHeading + posCurrent;
                            posRightTest = stepSize*R*newHeading + posForwardTest;
                            turnRight = !BugTools::checkCollision(problem, posRightTest);
                            heading = newHeading;
                        }
                        posCurrent = posForwardTest;
                        path.waypoints.push_back(posCurrent);
                        canLeave = true;
                    }
                }

                // Now have my current position
                pos = posCurrent;
                // Check distance to goal
                Vec2Goal = pos - problem.q_goal;
                Vec2Hit = pos - posHit;

                // Check m-line hit
                x_pos = pos[0]; y_pos = pos[1];
                if (std::abs(((y_pos-y_1) - ((y_2-y_1)/(x_2-x_1)) * (x_pos-x_1))) <= stepSize
                && Vec2Goal.norm()<hitDist2Goal && Vec2Hit.norm() > stepSize){
                    followWall = false;
                }

            }

            
        } else{
            // Add this position to waypoints, keep on truckin
            path.waypoints.push_back(pos);
        }

        // Check for arrival
        Eigen::Vector2d vec2Goal = pos - problem.q_goal;

        if (vec2Goal.norm() <= stepSize){
            goalReached = true;
            path.waypoints.push_back(problem.q_goal);
        }

        posOld = pos;
        
    }


    return path;
}