/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_GAZEBOWORLDINTERFACE_HPP
#define GAZEBOCOSIM_GAZEBOWORLDINTERFACE_HPP

#include <stdint.h>

namespace robotics {
namespace gazebotransport {
/// Abstract interface for Gazebo world
class GazeboWorldInterface {
  public:
    /// Destructor
    virtual ~GazeboWorldInterface() = default;

    /// set gazebo pause state
    virtual void setPaused(bool state) = 0;

    /// Virtual function of Gazebo world Step
    virtual void step(uint32_t numSteps) = 0;

    /// Virtual function of Gazebo world IsPaused
    virtual bool isPaused() = 0;

    /// Virtual function of Gazebo world Reset
    virtual void reset() = 0;

    /// Virtual function of Gazebo world ResetTime
    virtual void resetTime() = 0;
};
} // namespace gazebotransport
} // namespace robotics
#endif
