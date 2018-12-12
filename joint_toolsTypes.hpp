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

    struct PWMPositionerJointSettings
    {
        /** Proportional gain to be used on positive movements
         * (when the current position is less than the target
         */
        float Kpositive;
        /** Effort to be used on positive movements
         * (when the current position is less than the target
         */
        float Epositive;
        /** Proportional gain to be used on negative movements
         * (when the current position is more than the target
         */
        float Knegative;
        /** Effort to be used on negative movements
         * (when the current position is more than the target
         */
        float Enegative;
    };
}

#endif

