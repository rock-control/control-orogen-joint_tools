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
    mPeriod = _cycle_duration.get();

    size_t size = mSettings.size();
    mTargets.elements.resize(size);
    mState.elements.resize(size);
    mCommand.elements.resize(size);
    mError.joints.resize(size);
    mOff.resize(size);
    return true;
}
bool EffortPWMPositionerTask::startHook()
{
    if (! EffortPWMPositionerTaskBase::startHook())
        return false;

    for (auto& time : mOff)
        time = base::Time();
    for (auto& joint : mCommand.elements)
        joint = base::JointState::Effort(0);

    mNextCycle = base::Time();
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
    if (mNextCycle.isNull()) {
        mNextCycle = now;
    }
    mCommand.time = now;

    if (now < mNextCycle)
    {
        for (size_t i = 0; i < mOff.size(); ++i)
            if (mOff[i] < now)
                mCommand.elements[i].effort = 0;

        _joints_cmd.write(mCommand);
        return;
    }

    while (now >= mNextCycle) {
        mNextCycle = mNextCycle + mPeriod;
    }
    for (size_t i = 0; i < mOff.size(); ++i)
    {
        double error = mTargets.elements[i].position -
            mState.elements[i].position;

        auto const& settings = mSettings[i];
        double cycle;
        double effort;
        if (error > 0) {
            cycle = std::fabs(settings.Kpositive * error);
            effort = settings.Epositive;
        }
        else {
            cycle = std::fabs(settings.Knegative * error);
            effort = settings.Enegative;
        }
        mOff[i] = now + mPeriod * cycle;
        if (mOff[i] > now)
            mCommand.elements[i].effort = effort;
        else
            mCommand.elements[i].effort = 0;

        mError.joints[i].error = error;
        mError.joints[i].duty = cycle;
    }
    mError.time = now;
    _error.write(mError);
    _joints_cmd.write(mCommand);
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
