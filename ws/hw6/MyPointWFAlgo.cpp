#include "MyPointWFAlgo.h"

std::unique_ptr<amp::GridCSpace2D> amp::MyPointWFAlgo::constructDiscretizedWorkspace(const amp::Environment2D& environment){
    //Get my collision checker
    amp::CollisionChecker collisionCheckerObj;

    //Define discretizations and cell values
    int x0Elements = 100;  int x1Elements = 100;

    double x0StepSize =  (environment.x_max - environment.x_min)/x0Elements;  
    double x1StepSize =  (environment.y_max - environment.y_min)/x1Elements; 

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

    // Grow wavefront
    amp::DenseArray2D wavefront(x0Elements, x1Elements, 0);
    // Put values in to check boundaries easily
    std::vector<double> iStep{ 1, -1, 0, 0};
    std::vector<double> jStep{ 0, 0, 1, -1};
    bool waveDone = false;
    bool breakLoops = false;
    int cellsFilled = 1;
    wavefront(iGoal, jGoal) = 2;

    int force = 0;
    
    while (!waveDone) {
        //Start at bottom left, scan each row
        for (int i = 0; i<x0Elements; i++){
            for (int j = 0; j<x1Elements; j++){
                // Check each boundary
                int minBoundVal = 1000000;
                bool addCell = false;
                if (wavefront(i, j)==0){
                    for (int k = 0; k<4; k++){
                        // If boundary is wall, ignore
                        int iBound = i+iStep[k]; int jBound = j+jStep[k];
                        if ((iBound>x0Elements-1) || (jBound>x1Elements-1) || iBound<0 || jBound<0){
                            //Outside of wall
                            //std::cout << "Out of wall" << std::endl;
                            continue;
                        } else if (grid_cspace(iBound, jBound)) {
                            //Boundary in obstacle
                            //std::cout << "Boundary" << std::endl;
                            if (wavefront(iBound, jBound)!=1) {
                                // This is just for stop condition
                                wavefront(iBound, jBound) = 1;
                                cellsFilled++;
                            }
                            continue;
                        } else if (wavefront(iBound, jBound)>1) {
                            // Boundary is ok, put its value into the minimum cell value if necessary
                            //std::cout << "Found goal cell" << std::endl;
                            if (wavefront(iBound, jBound) < minBoundVal) {
                                minBoundVal = wavefront(iBound, jBound);
                            }
                            addCell = true;
                        } else {
                            // No nodes around have been checked, just keep going
                            //std::cout << iBound << "   " << jBound << "  :  " << wavefront(iBound, jBound) << std::endl;
                            continue;
                        }
                    }

                } else {
                    continue;
                }

                if (addCell) {
                    wavefront(i, j) = minBoundVal + 1;
                    //std::cout << "Filled a cell" << std::endl;
                    if (i==iInit && j==jInit){
                        // Init has been reached
                        std::cout << "Path found" << std::endl;
                        waveDone = true;
                        breakLoops = true;
                        cellsFilled++;
                    }
                }

                if (breakLoops) {break;};

            }
            if (breakLoops) {break;};
        }

        /*
        if (cellsFilled > (x0Elements*x1Elements)) {
            std::cout << "No path found" << std::endl;
            return path;
        }
        */

        force++;
        if (force>10000) {
            return path;
        }

    }

    int iNow = iInit; int jNow = jInit;

    force = 0;

    // Start path
    path.waypoints.push_back(q_init);

    bool goalReached = false;
    Eigen::Vector2d pos = q_init;

    while (!goalReached){
        int valNow = wavefront(iNow, jNow);
        for (int k = 0; k<4; k++){
            int iBound = iNow+iStep[k];
            int jBound = jNow+jStep[k];

            if (wavefront(iBound, jBound) == valNow-1) {
                //step in that direction
                Eigen::Vector2d step(iStep[k]*x0Step, jStep[k]*x1Step);
                pos = pos + step;

                path.waypoints.push_back(pos);
                if (iBound==iGoal && jBound==jGoal){
                    path.waypoints.push_back(q_goal);
                    goalReached = true;
                    break;
                }
                iNow = iBound;
                jNow = jBound;
                break;
            }

        }

        if (force>5000) { return path;};
        force++;
    }

    return path;

}
