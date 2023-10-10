#pragma once

#include "AMPCore.h"

namespace amp{
class linkTools : public LinkManipulator2D{
    public:  
        using LinkManipulator2D::LinkManipulator2D;
        Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const;

        ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const;


};


}