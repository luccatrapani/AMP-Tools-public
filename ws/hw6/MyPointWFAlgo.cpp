#include "MyPointWFAlgo.h"

std::unique_ptr<amp::GridCSpace2D> amp::MyPointWFAlgo::constructDiscretizedWorkspace(const amp::Environment2D& environment){
    //Get my collision checker
    amp::CollisionChecker collisionCheckerObj;

    //Define discretizations and cell values
    int x0Elements = 100;  int x1Elements = 100;

    double x0StepSize =  (environment.x_max - environment.x_max)/x0Elements;  
    double x1StepSize =  (environment.y_max - environment.y_max)/x1Elements; 

    double x0Min = environment.x_min; double x1Min = environment.y_min;
    double x0Max = environment.x_max-x0StepSize; double x1Max = environment.y_max-x1StepSize;

    //Instantiate my cSpace object 
    std::unique_ptr<amp::GridCSpace2D> cSpace = std::make_unique<MyGridCSpace>(x0Elements, x1Elements, x0Min, x0Max, x1Min, x1Max);

    // Loop through discrete configurations and put resulting boolean in collision into dense array
    double x0Val = x0Min;  

    bool collision = false;
    for(int i = 0; i<x0Elements; i++){
        double x1Val = x1Min;
        for(int j = 0; j<x1Elements; j++){
            Eigen::Vector2d pos(x0Val, x1Val);
            collision = collisionCheckerObj.checkCollision(environment, pos);
            (*cSpace)(i, j) = collision;

            x1Val = x1Val+x1StepSize;
        }
        x0Val = x0Val + x0StepSize;
    }

    return cSpace;

}

amp::Path2D amp::MyPointWFAlgo::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace){
    // Motion plan using wavefront algorithm, grow a wave from the goal state to initial 
    //state while avoiding obstacles
    amp::Path2D path;

    // Get some stuff I need from cSpace
    std::pair cSpaceSize = grid_cspace.size();
    int x0Elements = cSpaceSize.first; int x1Elements = cSpaceSize.second;
    int iMax = x0Elements-1; int jMax = x1Elements-1; int iMin = 0; int jMin = 0;

    std::pair x0MinMax = grid_cspace.x0Bounds();  std::pair x1MinMax = grid_cspace.x1Bounds();
    double x0Min = x0MinMax.first; double x0Max = x0MinMax.second;
    double x1Min = x1MinMax.first; double x1Max = x1MinMax.second;

    double x0Step = (x0Max - x0Min)/x0Elements; double x1Step = (x1Max - x1Min)/x1Elements;

    std::pair initInd = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
    std::pair goalInd = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);
    int iInit = initInd.first; int jInit = initInd.second;
    int iGoal = goalInd.first; int jGoal = goalInd.second;

    //Start path
    path.waypoints.push_back(q_goal);

    bool goalReached = false;
    
    int i = iGoal; int j = jGoal;
    Eigen::Vector2d pos = q_goal;

    while(!goalReached){
        // Clear my directions
        bool goUp = false; bool goDown = false; bool goRight = false; bool goLeft = false;

        //Find direction to start from current point
        Eigen::Vector2d vec2Start = pos - q_init;
        Eigen::Vector2d unitVec2Start = vec2Start/vec2Start.norm();

        if (std::abs(unitVec2Start[0]) > std::abs(unitVec2Start[1]) && unitVec2Start[0] > 0){
            goRight = true;
        } else if (std::abs(unitVec2Start[0]) > std::abs(unitVec2Start[1]) && unitVec2Start[0] < 0) {
            goLeft = true;
        } else if (std::abs(unitVec2Start[0]) < std::abs(unitVec2Start[1]) && unitVec2Start[1] > 0) {
            goUp = true;
        } else if (std::abs(unitVec2Start[0]) < std::abs(unitVec2Start[1]) && unitVec2Start[1] < 0){
            goDown = true;
        } else {
            std::cout << "Direction unclear" << std::endl;
            return path;
        }

        bool directionFound = false;
        int count = 0;
        Eigen::Vector2d step;

        while (count < 4 && !directionFound) {
            // If can go direction I want, try other grid spaces in CCW order
            if (goRight){
                if (((i+1)<=iMax) && ((i+1)>=iMin)) {
                    if (!grid_cspace(i+1, j)){
                        step << x0Step, 0;
                        directionFound = true;
                    } else {
                        goRight = false;
                        goUp = true; 
                        count++;
                    }
                } else {
                    goRight = false;
                    goUp = true; 
                    count++;
                }
            } else if (goUp){
                if (((j+1)<=jMax) && ((j+1)>=jMin)) {
                    if (!grid_cspace(i, j+1)){
                        step << 0, x1Step;
                        directionFound = true;
                    } else {
                        goUp = false;
                        goLeft = true; 
                        count++;
                    }
                } else {
                    goUp = false;
                    goLeft = true; 
                    count++;
                }
            } else if (goLeft){
                if (((i-1)<=iMax) && ((i-1)>=iMin)) {
                    if (!grid_cspace(i-1, j)){
                        step << -x0Step, 0;
                        directionFound = true;
                    } else {
                        goLeft = false;
                        goDown = true; 
                        count++;
                    }
                } else {
                    goLeft = false;
                    goDown = true; 
                    count++;
                }
            } else if (goDown){
                if (((j-1)<=jMax) && ((j-1)>=jMin)) {
                    if (!grid_cspace(i, j-1)){
                        step << 0, -x1Step;
                        directionFound = true;
                    } else {
                        goDown = false;
                        goRight = true; 
                        count++;
                    }
                } else {
                    goDown = false;
                    goRight = true; 
                    count++;
                }
            }
        }

        // Check to make sure algorithm did not fail
        if (!directionFound) {
            std::cout << "No path found!!!" << std::endl;
            return path;
        }

        // Take a step
        pos = pos + step;

        // Check indices of pos
        std::pair currentInd = grid_cspace.getCellFromPoint(pos[0], pos[1]);
        i = currentInd.first; j = currentInd.second;

        // Add my step to the path
        path.waypoints.push_back(pos);

        // Check for start
        if (i==iInit && j==jInit) {
            //start reached
            path.waypoints.push_back(q_init);
            goalReached == true;
        }


    }

    // Flip path vector because I start at q_goal
    std::reverse(path.waypoints.begin(), path.waypoints.end());

    return path;

}
