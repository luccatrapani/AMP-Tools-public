#include "MyCSpaceCtor.h"

std::unique_ptr<amp::GridCSpace2D> amp::MyCSpaceCtor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env){
    /* I have a manipulator and an environment.  I need to construct a cspace from this manipulator and environement
    to do this, I need to discretize the cspace, then "move" the manipulator around to each configuration.
    Then I need to "walk" along the length of the manipulator in discrete steps and check for collisions (using forward kinematics)
    Then add boolean values for these discrete configurations to the dense array
    */
    //Get my collision checker
    amp::CollisionChecker collisionCheckerObj;

    //Define discretizations and cell values
    int x0Elements = 100;  int x1Elements = 100;
    double x0Min = 0; double x1Min = 0;

    double x0StepSize = (2*std::numbers::pi)/x0Elements;  double x1StepSize = (2*std::numbers::pi)/x1Elements;

    double x0Max = 2*std::numbers::pi - x0StepSize;  double x1Max = 2*std::numbers::pi - x1StepSize;

    //Instantiate my cSpace object 
    std::unique_ptr<amp::GridCSpace2D> cSpace = std::make_unique<MyGridCSpace>(x0Elements, x1Elements, x0Min, x0Max, x1Min, x1Max);


    // Now deal with manipulator:
    // "Walk" along manipulator in 1/5 steps
    double manipulatorDis = 5;

    // Get some values that I need from the manipulator
    std::vector<double> linkLengths = manipulator.getLinkLengths();
    Eigen::Vector2d baseLocation = manipulator.getBaseLocation();

    // Loop through discrete configurations and put resulting boolean in collision into dense array
    double theta1 = x0Min;

    for(int i=0; i<x0Elements; i++){
        //Check first manipulator link
        double theta2 = x0Min;
        Eigen::Vector2d state(theta1, theta2);
        Eigen::Vector2d joint1Loc = manipulator.getJointLocation(state, 1);
        //Walk along link
        bool firstCollision = false;

        for(int k = 1; k <= manipulatorDis; k++){
            Eigen::Vector2d realLoc = (1-(k*1/manipulatorDis))*baseLocation + (k*1/manipulatorDis)*joint1Loc;
            firstCollision = collisionCheckerObj.checkCollision(env, realLoc);

            if (firstCollision==true){break;};
        }

        bool secondCollision = false;
        for(int j=0; j<x1Elements; j++){
            if (firstCollision==true){
                (*cSpace)(i, j) = true;
            }else{
                Eigen::Vector2d state(theta1, theta2);
                Eigen::Vector2d joint2Loc = manipulator.getJointLocation(state, 2);

                for(int k = 1; k <= manipulatorDis; k++){
                    Eigen::Vector2d realLoc = (1 - (k*1/manipulatorDis))*joint1Loc + (k*1/manipulatorDis)*joint2Loc;
                    secondCollision = collisionCheckerObj.checkCollision(env, realLoc);

                    if (secondCollision == true){break;};
                }

                (*cSpace)(i, j) = secondCollision;
            }
            theta2 = theta2 + x1StepSize;

        }
        theta1 = theta1 + x0StepSize;


    }

    return cSpace;

}