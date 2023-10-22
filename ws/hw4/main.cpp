// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "CollisionChecker.h"
#include "makeCSpaceObstacles.h"
#include "linkTools.h"
#include "MyCSpaceCtor.h"


using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    /*
    // Exercise 1
    Obstacle2D obstacle_robot_E1 = HW4::getEx1TriangleObstacle();
    makeCSpaceObstacles::CSpaceObstacle cSpace_E1_1 = makeCSpaceObstacles::makeObstacles(obstacle_robot_E1, obstacle_robot_E1, 1);
    std::cout << "Exercise 1.a: C-Space Obstacle Vertices:" << std::endl;
    cSpace_E1_1.obstacles[0].print();

    makeCSpaceObstacles::CSpaceObstacle cSpace_E1_2 = makeCSpaceObstacles::makeObstacles(obstacle_robot_E1, obstacle_robot_E1, 12);
    //Visualizer::makeFigure(cSpace_E1_2.obstacles, cSpace_E1_2.heights);

    // Exercise 2
    Eigen::Vector3d configuration_E2_1(std::numbers::pi/6, std::numbers::pi/3, 7*std::numbers::pi/4);
    std::vector<double> linkLengths_E2_1 = {.5, 1, .5};
    const linkTools test_link_E2_1(linkLengths_E2_1);
    Eigen::Vector2d loc_E2_1 = test_link_E2_1.getJointLocation(configuration_E2_1, 2);
    bool E2_1_Check = HW4::checkFK(loc_E2_1, 2, test_link_E2_1, configuration_E2_1);
    Visualizer::makeFigure(test_link_E2_1, configuration_E2_1);

    
    std::vector<double> linkLengths_E2_2 = {1, .5, 1};
    Eigen::Vector2d loc_E2_2(2, 0);
    const linkTools test_link_E2_2(linkLengths_E2_2);
    ManipulatorState configuration_E2_2 = test_link_E2_2.getConfigurationFromIK(loc_E2_2);
    Visualizer::makeFigure(test_link_E2_2, configuration_E2_2);
    std::cout <<  configuration_E2_2[0] << configuration_E2_2[1] << configuration_E2_2[2] << std::endl;

    std::vector<double> linkLengths_E3 = {1, 1};
    linkTools manipulator3(linkLengths_E3);
    Eigen::Vector2d testEnd(1.19, .55);
    ManipulatorState2Link testMan = manipulator3.getConfigurationFromIK2Link(testEnd);
    Visualizer::makeFigure(manipulator3, testMan);
    */
    
    // Exercise 3_1
    MyCSpaceCtor ctorAlgo;
    Environment2D env3_1 = HW4::getEx3Workspace1();
    env3_1.print();
    std::vector<double> linkLengths_E3 = {1, 1};
    linkTools manipulator3(linkLengths_E3);
    std::unique_ptr<amp::GridCSpace2D> cSpace_E3_1 = ctorAlgo.construct(manipulator3, env3_1);
    HW4::checkCSpace(*cSpace_E3_1, manipulator3, env3_1, 10000);
    //Visualizer::makeFigure(*cSpace_E3_1);
    /*
    // Exercise 3_2
    Environment2D env3_2 = HW4::getEx3Workspace2();
    std::unique_ptr<amp::GridCSpace2D> cSpace_E3_2 = ctorAlgo.construct(manipulator3, env3_2);
    HW4::checkCSpace(*cSpace_E3_2, manipulator3, env3_2, 10000);
    Visualizer::makeFigure(*cSpace_E3_2);

    // Exercise 3_3
    Environment2D env3_3 = HW4::getEx3Workspace3();
    std::unique_ptr<amp::GridCSpace2D> cSpace_E3_3 = ctorAlgo.construct(manipulator3, env3_3);
    HW4::checkCSpace(*cSpace_E3_3, manipulator3, env3_3, 10000);
    Visualizer::makeFigure(*cSpace_E3_3);

    Eigen::Vector2d testCon(2.646, 1.881);
    Visualizer::makeFigure(env3_3, manipulator3, testCon);
    */



    //Visualizer::showFigures();

    // Grade method
    //amp::HW4::grade<linkTools>(ctorAlgo, "lucca.trapani@colorado.edu", argc, argv);
    return 0;
}