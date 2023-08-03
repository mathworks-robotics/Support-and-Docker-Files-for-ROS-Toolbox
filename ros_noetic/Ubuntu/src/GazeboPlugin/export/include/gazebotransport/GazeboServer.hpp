/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOSERVER_HPP
#define GAZEBOSERVER_HPP

#include "gazebotransport/gazebotransport_util.hpp"
#include "gazebotransport/Server.hpp"

#include "boost/asio.hpp"

#include <memory>
#include <cstdint>
#include <string>
#include <mutex>
#include <utility>
#include <future>
#include <atomic>

namespace robotics {
namespace gazebotransport {
/// TCP ip server implementation class
class Server;

/// initiates message-handler based on message ID
class MsgDispatcher;

/// handles and process message
class MsgHandler;

class GAZEBOTRANSPORT_EXPORT_CLASS GazeboServer {

  public:
    /// constructor
    /**
    @param portNumber            server port number

    This class access Server interface file and establish connection with Client. Further, it
    initiates all message handlers at Server end.
    */
    GazeboServer(uint16_t portNumber);


    /// The message handler described in the input gets initiates
    /**
    @param handler      Message handler

    Respective message handler is evoked by Server after connection established with client
    */
    void registerHandler(std::shared_ptr<MsgHandler> handler);

    /// Starts connection with client
    void run();

    /// Check whether co-simulation is running in the server
    std::pair<bool, std::string> getCoSimulationStatus();

    /// Start co-simulation with particular client
    /**
    @param clientID                 ID of the client that requested co-simulation
    @return                         True if co-simulation is granted

    This call is thread safe
    */
    bool startCoSimulation(std::string const& clientID, double duration);

    /// End co-simulation with particular client
    /**
    @param clientID                 Identification for the client that requested co-simulation
    @return                         True if co-simulation is stopped

    This call is thread safe
    */
    bool stopCoSimulation(std::string const& clientID);

    /// get server port number
    std::string getPortName();

    /// Ends connection with client
    void stop();

  private:
    /// Co-simulation status identifies with which client server is co-simulating
    class CoSimulationStatus {
      public:
        /// default constructor
        CoSimulationStatus();

        ~CoSimulationStatus();

        /// start co-simulation for a given duration
        /**
        @param id                       Identification for the client that runs co-simulation
        @param duration                 Duration to set in milliseconds
        */
        bool start(std::string const& id, double duration);

        /// stop co-simulation
        /**
        @param id                       Identification for the client that runs co-simulation
        */
        bool stop(std::string const& id);

        /// Check whether co-simulation is running
        std::pair<bool, std::string> getStatus();

      private:
        /// reset co-simulation stop timer to given duration
        /**
        @param id                       Identification for the client that runs co-simulation
        @param duration                 Duration to set in milliseconds
        */
        bool extend(std::string const& id, double duration);

        /// Co-simulation client ID
        std::string m_clientID;

        /// Indicates whether server is running co-simulation
        std::atomic<bool> m_isCoSimulationRunning;

        /// Lock guard for multi-thread access
        std::recursive_mutex m_mutex;

        /// Boost asio queue
        boost::asio::io_service m_service;

        /// Place holder to keep service running
        std::shared_ptr<boost::asio::io_service::work> m_work;

        /// Boost deadline timer
        boost::asio::deadline_timer m_timer;

        /// Boost asio queue run result
        std::future<void> m_result;
    };

    /// Message dispatcher instance
    std::shared_ptr<MsgDispatcher> m_dispatcher;

    /// Server instance
    std::shared_ptr<Server> m_server;

    /// Server co-simulation status
    CoSimulationStatus m_coSimulationStatus;
};

} // namespace gazebotransport
} // namespace robotics

#endif
