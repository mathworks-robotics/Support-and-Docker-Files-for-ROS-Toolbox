/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/gazeboserver/GazeboServerCallback.hpp"
#include "gazebotransport/gazeboserver/MsgDispatcher.hpp"

#include "boost/date_time.hpp"
#include <cmath>

namespace robotics {
namespace gazebotransport {


GazeboServer::CoSimulationStatus::CoSimulationStatus()
    : m_clientID("")
    , m_isCoSimulationRunning(false)
    , m_mutex()
    , m_service()
    , m_work(std::make_shared<boost::asio::io_service::work>(m_service))
    , m_timer(m_service)
    , m_result() {
    // launch the service queue in a different thread to keep the deadline timer ticking
    m_result = std::async(std::launch::async, [this]() { this->m_service.run(); });
}

GazeboServer::CoSimulationStatus::~CoSimulationStatus() {
    // reset timer
    boost::system::error_code ignored;
    m_timer.cancel(ignored);
    // reset work place holder
    m_work.reset();
    // wait the run to finish
    m_result.get();
}

bool GazeboServer::CoSimulationStatus::start(std::string const& id, double duration) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (m_isCoSimulationRunning.load()) {
        // if co-simulation is already running, check whether we need to extend
        return extend(id, duration);
    } else {
        // grant co-simulation and record co-simulation client id
        m_clientID = id;

        if (std::isfinite(duration)) {
            // start a deadline timer if duration is finite
            // stop the co-simulation after duration
            boost::system::error_code operationCode;
            m_timer.expires_from_now(
                boost::posix_time::milliseconds(static_cast<int64_t>(duration)), operationCode);
            if (!operationCode) {
                // if timer is set, change co-simulation status
                m_isCoSimulationRunning = true;
                m_timer.async_wait([this](const boost::system::error_code& ec) {
                    if (ec != boost::asio::error::operation_aborted) {
                        std::lock_guard<std::recursive_mutex> handleLock(m_mutex);
                        // once timeout is reached, reset co-simulation status
                        m_isCoSimulationRunning = false;
                    }
                });
                return true;
            } else {
                // if timer cannot be set, don't accept co-simulation
                return false;
            }
        } else {
            // if duration is infinite, just turn into co-simulation mode
            m_isCoSimulationRunning = true;
            return true;
        }
    }
}

bool GazeboServer::CoSimulationStatus::extend(std::string const& id, double duration) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (id == m_clientID) {
        // reset the timer for stopping co-simulation
        boost::system::error_code operationCode;
        m_timer.cancel(operationCode);
        if (std::isfinite(duration)) {
            // if timer is only extended with finite value, then prepare for next timer
            m_timer.expires_from_now(
                boost::posix_time::milliseconds(static_cast<int64_t>(duration)), operationCode);
            m_timer.async_wait([this](const boost::system::error_code& ec) {
                if (ec != boost::asio::error::operation_aborted) {
                    std::lock_guard<std::recursive_mutex> handleLock(m_mutex);
                    // once timeout is reached, reset co-simulation status
                    m_isCoSimulationRunning = false;
                }
            });
        }
        return operationCode.value() == 0;
    } else {
        return false;
    }
}

bool GazeboServer::CoSimulationStatus::stop(std::string const& id) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (!m_isCoSimulationRunning.load() || m_clientID == id) {
        // If co-simulation is not running
        // Or co-simulation is running with the given client
        // Allow co-simulation to stop
        boost::system::error_code ignored;
        m_timer.cancel(ignored);
        m_isCoSimulationRunning = false;
        return true;
    } else {
        return false;
    }
}

std::pair<bool, std::string> GazeboServer::CoSimulationStatus::getStatus() {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    return std::make_pair(m_isCoSimulationRunning.load(), m_clientID);
}

GazeboServer::GazeboServer(uint16_t portNumber)
    : m_dispatcher(std::make_shared<MsgDispatcher>())
    , m_server(std::make_shared<Server>(portNumber,
                                        std::make_shared<GazeboServerCallback>(m_dispatcher)))
    , m_coSimulationStatus() {
}

void GazeboServer::registerHandler(MsgHandlerPtr handler) {
    m_dispatcher->registerHandler(handler);
}

void GazeboServer::run() {
    m_server->run();
}

std::string GazeboServer::getPortName() {
    return std::to_string(m_server->getPortNumber());
}

void GazeboServer::stop() {
    m_server->shutdown();
}

std::pair<bool, std::string> GazeboServer::getCoSimulationStatus() {
    return m_coSimulationStatus.getStatus();
}

bool GazeboServer::startCoSimulation(std::string const& clientID, double duration) {
    return m_coSimulationStatus.start(clientID, duration);
}

bool GazeboServer::stopCoSimulation(std::string const& clientID) {
    return m_coSimulationStatus.stop(clientID);
}

} // namespace gazebotransport
} // namespace robotics
