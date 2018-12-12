#ifndef joint_tools_TYPES_HPP
#define joint_tools_TYPES_HPP

#include <base/Time.hpp>
#include <joint_tools/PWMError.hpp>
#include <vector>

namespace joint_tools {
    struct PWMPositionerError
    {
        base::Time time;
        std::vector<PWMError> joints;
    };
}

#endif

