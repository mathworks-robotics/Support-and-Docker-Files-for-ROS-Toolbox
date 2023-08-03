/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_GAZEBOWORLDIMPL_HPP
#define GAZEBOCOSIM_GAZEBOWORLDIMPL_HPP

#include "gazebotransport/gazeboserver/GazeboWorldInterface.hpp"
#include <gazebo/physics/physics.hh>

namespace robotics {
namespace gazebotransport {
/// concrete implementation for Gazebo world interface
class GazeboWorldImpl : public GazeboWorldInterface {
  public:
    /// Constructor
    /**
    @param WorldPtr                Gazebo pointer of Gazebo class World
    */
    GazeboWorldImpl(gazebo::physics::WorldPtr world);

    /// Change Gazebo simulator pause state
    void setPaused(bool state) override;

    /// Start Gazebo simulation for numSteps steps ( for gmock function )
    /**
    @param numSteps                 Number of steps for simulation
    */
    void step(uint32_t numSteps) override;

    /// Checks Gazebo simulation is paused ( for gmock function )
    bool isPaused() override;

    /// Reset Scene and Time of Gazebo simulation ( for gmock function )
    void reset() override;

    /// Reset Scene and Time of Gazebo simulation ( for gmock function )
    void resetTime() override;

  private:
    /// Gazebo class World pointer
    gazebo::physics::WorldPtr m_world;
};
} // namespace gazebotransport
} // namespace robotics
#endif
