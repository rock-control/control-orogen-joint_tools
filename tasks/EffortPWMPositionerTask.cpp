/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "EffortPWMPositionerTask.hpp"

using namespace joint_tools;

EffortPWMPositionerTask::EffortPWMPositionerTask(std::string const& name)
    : EffortPWMPositionerTaskBase(name)
{
}

EffortPWMPositionerTask::EffortPWMPositionerTask(std::string const& name, RTT::ExecutionEngine* engine)
    : EffortPWMPositionerTaskBase(name, engine)
{
}

EffortPWMPositionerTask::~EffortPWMPositionerTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See EffortPWMPositionerTask.hpp for more detailed
// documentation about them.

bool EffortPWMPositionerTask::configureHook()
{
    if (! EffortPWMPositionerTaskBase::configureHook())
        return false;

    mSettings = _settings.get();
    // Pre-allocate. We're going to re-do it in startHook to reinitialize
    mPositioner = PWMJointPositioner(mSettings);

    size_t size = mSettings.size();
    mTargets.elements.resize(size);
    mState.elements.resize(size);
    mCommand.elements.resize(size);
    mErrors.joints.resize(size);
    return true;
}
bool EffortPWMPositionerTask::startHook()
{
    if (! EffortPWMPositionerTaskBase::startHook())
        return false;

    mPositioner = PWMJointPositioner(mSettings);
    return true;
}
void EffortPWMPositionerTask::updateHook()
{
    EffortPWMPositionerTaskBase::updateHook();

    if (_joints_target.read(mTargets, false) == RTT::NoData)
        return;
    if (_joints_position.read(mState, false) == RTT::NoData)
        return;

    if (mTargets.elements.size() != mSettings.size())
    {
        exception(UNEXPECTED_TARGET_SIZE);
        return;
    }
    else if (mState.elements.size() != mSettings.size())
    {
        exception(UNEXPECTED_POSITION_SIZE);
        return;
    }

    for (auto const& j : mTargets.elements)
    {
        if (!j.hasPosition())
        {
            exception(NO_POSITION_IN_TARGET);
            return;
        }
    }
    for (auto const& j : mState.elements)
    {
        if (!j.hasPosition())
        {
            exception(NO_POSITION_IN_POSITION);
            return;
        }
    }

    base::Time now = mState.time;

    auto const& commands = mPositioner.update(now, mTargets, mState);
    for (size_t i = 0; i < mSettings.size(); ++i)
        mCommand.elements[i].effort = commands[i];
    mCommand.time = now;
    _joints_cmd.write(mCommand);

    mErrors.time = now;
    mErrors.joints = mPositioner.getErrors();
    _error.write(mErrors);
}
void EffortPWMPositionerTask::errorHook()
{
    EffortPWMPositionerTaskBase::errorHook();
}
void EffortPWMPositionerTask::stopHook()
{
    EffortPWMPositionerTaskBase::stopHook();
}
void EffortPWMPositionerTask::cleanupHook()
{
    EffortPWMPositionerTaskBase::cleanupHook();
}
