#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw/HW5.h"
//#include "MyPRM2D.h"
#include "MyGoalBiasRRT2D.h"

using namespace amp;

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    //Problem 1

    /*
    Problem2D problem1 =  HW5::getWorkspace1();
    Eigen::Vector2d lBounds = {-1, -3}; Eigen::Vector2d uBounds = {11, 3};
    MyPRM2D prm2DAlgo(lBounds, uBounds);
    Path2D path1 = prm2DAlgo.plan(problem1);
    Visualizer::makeFigure(problem1, path1);
    */
    //Problem 2
    Problem2D problem1 =  HW5::getWorkspace1();
    int n = 5000; double r = .5; double p = .05; double e = .25;
    Eigen::Vector2d lBounds = {-1, -3}; Eigen::Vector2d uBounds = {11, 3};
    MyGoalBiasRRT2D algo(lBounds, uBounds, n, r, p, e);
    Path2D path2 = algo.plan(problem1);
    Visualizer::makeFigure(problem1, path2);

    bool woah = HW7::generateAndCheck(algo);


    Visualizer::showFigures();


    return 0;
};
