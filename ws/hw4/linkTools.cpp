#include "linkTools.h"

namespace amp{

Eigen::Vector2d linkTools::getJointLocation(const ManipulatorState& state, uint32_t joint_index) const{
    ///  Get the location of the nth joint using the current link attributes using Forward Kinematics
    /// Joint angle state (radians). Must have size() == nLinks()
    ///  Joint index in order of base to end effector 
    /// (joint_index = 0 should return the base location, joint_index = nLinks() should return the end effector location)
    /// Joint coordinate
    std::vector<double> linkLengths =  getLinkLengths();
    Eigen::Vector2d baseLocation = getBaseLocation();

    double xCurrent = baseLocation[0];
    double yCurrent = baseLocation[1];

    int jointCurrent  = 0;

    Eigen::Vector2d jointLocationCurrent(xCurrent, yCurrent);

    double thetaSum = 0;
    while (jointCurrent < joint_index)
    {
        double linkLengthCurrent = linkLengths[jointCurrent];

        thetaSum = thetaSum + state[jointCurrent];

        double x1 = xCurrent + linkLengthCurrent*std::cos(thetaSum);
        double y1 = yCurrent + linkLengthCurrent*std::sin(thetaSum);


        jointLocationCurrent[0] = x1; jointLocationCurrent[1] = y1;

        xCurrent = x1; yCurrent = y1; jointCurrent++;


    }

    return jointLocationCurrent;
    

}

ManipulatorState linkTools::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location)const{
    // This is not complete

    std::vector<double> linkLengths =  getLinkLengths();
    Eigen::Vector2d baseLocation =  getBaseLocation();
    int numLinks = linkLengths.size();

    double a1 = linkLengths[0];
    double a2 = linkLengths[1];

    double x = end_effector_location[0]; double y = end_effector_location[1];

    double ct2 = (1/(2*a1*a2))*((x*x+y*y)-(a1*a1-a2*a2));
    double st2 = std::sqrt(1-ct2*ct2);

    std::cout << ct2 << std::endl;
    std::cout << st2 << std::endl;
    
    double ct1 = (1/(x*x+y*y))*(x*(a1+a2*ct2)+y*a2*std::sqrt(1-ct2*ct2));
    double st1 = (1/(x*x+y*y))*(y*(a1+a2*ct2)-x*a2*std::sqrt(1-ct2*ct2));

    std::cout << ct1 << std::endl;
    std::cout << st1 << std::endl;

    double theta1 = std::atan2(st1, ct1); double theta2 = std::atan2(st2, ct2);

    std::vector<double> configuration = {theta1, theta2};
    if (numLinks==3){
        double phi = std::atan2(y, x);

        double theta3 = phi - theta1 - theta2;

        configuration.push_back(theta3);
    }

    return configuration;


}
}