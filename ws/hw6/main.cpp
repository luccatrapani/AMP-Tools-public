#include "AMPCore.h"
#include "hw/HW6.h"
#include "hw/HW2.h"


//From HW6 folder
#include "MyPointWFAlgo.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Exercise 1
    MyPointWFAlgo pointWFalgo;
    Problem2D problem1 = HW2::getWorkspace2();
    Path2D path1 = pointWFalgo.plan(problem1);
    Visualizer::makeFigure(problem1, path1);

    Visualizer::showFigures();

    //amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("nonhuman.biologic@myspace.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    return 0;

}