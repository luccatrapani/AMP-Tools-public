#include "MyManipWFAlgo.h"

amp::MyManipWFAlgo::MyManipWFAlgo() : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) {}

amp::Path2D amp::MyManipWFAlgo::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace){
    // Motion plan using wavefront algorithm, grow a wave from the goal state to initial 
    //state while avoiding obstacles

    //In this algorithm I need to be able to wrap
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

    std::cout << iGoal << "  " << jGoal << std::endl;

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

                        // Wrap the angles here
                        if (iBound>x0Elements-1){iBound = 0;};
                        if (jBound>x1Elements-1){jBound = 0;};
                        if (iBound<0){iBound = x0Elements - 1;};
                        if (jBound<0){jBound = x1Elements - 1;};
                        if (grid_cspace(iBound, jBound)) {
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
        if (force>1000) {
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

            if (iBound>x0Elements-1){iBound = 0;};
            if (jBound>x1Elements-1){jBound = 0;};
            if (iBound<0){iBound = x0Elements - 1;};
            if (jBound<0){jBound = x1Elements - 1;};

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

    amp::Visualizer::makeFigure(grid_cspace, path);
    return path;
}