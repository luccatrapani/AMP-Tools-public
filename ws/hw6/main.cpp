#include "AMPCore.h"
#include "hw/HW6.h"
#include "hw/HW2.h"


//From HW6 folder
#include "MyPointWFAlgo.h"
#include "MyManipWFAlgo.h"
#include "linkTools.h"
#include "MyAStarAlgo.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Exercise 1
    /*
    MyPointWFAlgo pointWFalgo;

    Problem2D problem1_1 = HW2::getWorkspace1();
    Path2D path1_1 = pointWFalgo.plan(problem1_1);
    Visualizer::makeFigure(problem1_1, path1_1);
    std::cout << "Excerise 1.1 length: " << path1_1.length() << std::endl;

    Problem2D problem1_2 = HW2::getWorkspace2();
    Path2D path1_2 = pointWFalgo.plan(problem1_2);
    Visualizer::makeFigure(problem1_2, path1_2);
    std::cout << "Excerise 1.2 length: " << path1_2.length() << std::endl;
    */

    // Exercise 2
    /*
    MyManipWFAlgo manipWFalgo;
    std::vector<double> linkLengths{1, 1};
    linkTools manipulator2(linkLengths);

    Problem2D problem2_1 = HW6::getHW4Problem1();
    ManipulatorTrajectory2Link path2_1 = manipWFalgo.plan(manipulator2, problem2_1);
    Visualizer::makeFigure(problem2_1, manipulator2, path2_1);

    Problem2D problem2_2 = HW6::getHW4Problem2();
    ManipulatorTrajectory2Link path2_2 = manipWFalgo.plan(manipulator2, problem2_2);
    Visualizer::makeFigure(problem2_2, manipulator2, path2_2);

    Problem2D problem2_3 = HW6::getHW4Problem3();
    ManipulatorTrajectory2Link path2_3 = manipWFalgo.plan(manipulator2, problem2_3);
    Visualizer::makeFigure(problem2_3, manipulator2, path2_3);

    HW6::checkLinkManipulatorPlan(path2_3, manipulator2, problem2_3);
    */

    // Exercise 3
    MyAStarAlgo aStarAlgo;
    ShortestPathProblem problem3 = HW6::getEx3SPP();
    LookupSearchHeuristic heuristic3 = HW6::getEx3Heuristic();
    //bool h = problem3.(*graph).isReversible();
    MyAStarAlgo::GraphSearchResult res = aStarAlgo.search(problem3, heuristic3);
    HW6::checkGraphSearchResult(res, problem3, heuristic3);




    //Visualizer::showFigures();
    MyPointWFAlgo pointWFalgo;
    MyManipWFAlgo manipWFalgo;
    HW6::grade(pointWFalgo, manipWFalgo, aStarAlgo, "lucca.trapani@colorado.edu", argc, argv);
    return 0;

}