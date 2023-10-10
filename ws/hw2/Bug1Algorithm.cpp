#include "Bug1Algorithm.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug1Algorithm::plan(const amp::Problem2D& problem) {
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
            // Step back
            pos = posOld;
            Eigen::Vector2d heading = unitVec2Goal;

            // Begin wall following
            bool followWall = true;
            Eigen::Vector2d closestPos;
            double dist2Goal = 1.0E16;

            // Initialize for while loop
            bool canReturn = false;
            bool canLeave = false;
            bool turnLeft;
            bool checkRight;
            bool turnRight;
            Eigen::Vector2d newHeading;
            Eigen::Vector2d posCurrent;
            Eigen::Vector2d closestPointFinal;

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
                        canReturn = true;
                    }
                }

                // Now have my current position
                pos = posCurrent;

                // Check distance to goal
                Vec2Goal = pos - problem.q_goal;

                if (Vec2Goal.norm()<dist2Goal){
                    dist2Goal = Vec2Goal.norm();
                    closestPos = pos;
                }

                // Check distance to hit point
                Eigen::Vector2d vec2Hit = pos - posHit;
                if (canReturn==true && vec2Hit.norm()<=stepSize){
                    canLeave = true;
                    closestPointFinal = closestPos;
                }

                // Check distance to leave point
                Eigen::Vector2d vec2Leave = pos - closestPointFinal;
                if (canLeave == true && vec2Leave.norm()<=stepSize){
                    // Then wall following is done
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

