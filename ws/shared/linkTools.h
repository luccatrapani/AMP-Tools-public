#pragma once

#include "AMPCore.h"

namespace amp{
class linkTools : public LinkManipulator2D{
    public:  
        using LinkManipulator2D::LinkManipulator2D;
        virtual Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const override;

        virtual ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;

};


}