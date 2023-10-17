#include "AMPCore.h"

#include "hw/HW5.h"
#include "hw/HW2.h"

#include "gradientDescent.h"

using namespace amp;

int main(int argc, char** argv) {
    Problem2D problem = HW5::getWorkspace1();
    //Problem2D problem  = HW2::getWorkspace2();

    // Algorithm
    gradientDescent algo;
    Path2D path = algo.plan(problem);
    std::cout << "The path length for a. is " << path.length() << std::endl;
    

    Visualizer::makeFigure(problem, path);

    //Visualizer::showFigures();

    // Grade
    HW5::grade(algo, "lucca.trapani@colorado.edu", argc, argv);




    return 0;
}