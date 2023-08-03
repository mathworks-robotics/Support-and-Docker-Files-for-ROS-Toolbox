/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_SUBSCRIBEMSGHANDLER_HPP
#define GAZEBOCOSIM_SUBSCRIBEMSGHANDLER_HPP

#include <map>
#include <mutex>

namespace robotics {
namespace gazebotransport {
class SubscribeMsgManager {
  public:
    /// Constructor
    /// It take care of repetitive retrieving data of Gazebo callback in Paused state
    SubscribeMsgManager();

    /// Destructor
    ~SubscribeMsgManager();
    /// Stores input topic that subscribed
    void registerTopic(std::string const& topic);
    /// Informs that new updates available for input topic
    bool doUpdate(std::string const& topic);
    /// Provides status to stored topics
    void enableUpdate();

  protected:
    /// Map that stores topic name and status
    std::map<std::string, bool> m_update;
    std::mutex m_lock;
};
} // namespace gazebotransport
} // namespace robotics
#endif
