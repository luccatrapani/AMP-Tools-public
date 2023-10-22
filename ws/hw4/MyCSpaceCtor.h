#pragma once

/*
#include "AMPCore.h"
#include "CollisionChecker.h"
#include "hw/HW4.h"
#include "linkTools.h"
#include "MyGridCSpace.h"
*/

#include "hw/HW4.h"
#include "MyGridCSpace.h"
#include "CollisionChecker.h"

namespace amp{
class MyCSpaceCtor : public GridCSpace2DConstructor {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override;
};
}