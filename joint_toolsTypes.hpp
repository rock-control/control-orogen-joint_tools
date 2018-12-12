#ifndef joint_tools_TYPES_HPP
#define joint_tools_TYPES_HPP

#include <base/Time.hpp>
#include <vector>

namespace joint_tools {
    struct PWMPositionerJointError
    {
        float error;
        float duty;
    };

    struct PWMPositionerError
    {
        base::Time time;
        std::vector<PWMPositionerJointError> joints;
    };
}

#endif

