/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/GazeboWorldImpl.hpp"

namespace robotics {
namespace gazebotransport {
GazeboWorldImpl::GazeboWorldImpl(gazebo::physics::WorldPtr world)
    : m_world(world) {
}

void GazeboWorldImpl::setPaused(bool state) {
    m_world->SetPaused(state);
}

void GazeboWorldImpl::step(uint32_t numSteps) {
    m_world->Step(numSteps);
}

bool GazeboWorldImpl::isPaused() {
    return m_world->IsPaused();
}

void GazeboWorldImpl::reset() {
    return m_world->Reset();
}

void GazeboWorldImpl::resetTime() {
    return m_world->ResetTime();
}

} // namespace gazebotransport
} // namespace robotics
