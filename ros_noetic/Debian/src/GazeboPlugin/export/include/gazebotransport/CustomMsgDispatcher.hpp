/* Copyright 2019-2022 The MathWorks, Inc. */
#ifndef CUSTOMMSGDISPATCHER_HPP
#define CUSTOMMSGDISPATCHER_HPP
#include "gazebotransport/gazebotransport_util.hpp"
#include "gazebotransport/CustomMsgHandler.hpp"
#include <map>

namespace robotics {
namespace gazebotransport {
/// The custom message needs four actions
/// 1] initPublisher      : Initialize publisher on Gazebo
/// 2] initSubscriber  	  : Initialize subscriber on Gazebo
/// 3] publishCustomMsg   : Publish custom message on Gazebo
/// 4] subscribeCustomMsg : Subscribe custom from Gazebo
///
/// The CustomMsgDispatcher registers custom message handler created on gazebogenmsg() call.
/// In the registration process, the customMsgHandler is stored based on it message type name.
/// Further, CustomMsgDispatcher calls respective function based on the Packet ID.

class GAZEBOTRANSPORT_EXPORT_CLASS CustomMsgDispatcher {
  public:
    /**
     * @param handler      Message handler
     * @param msgName      Custom message type
     *
     * Stores Initiated custom message handler in map based on custom message type
     */
    void registerCustomHandler(CustomMsgHandlerPtr handler,
                               std::string const& msgName); // Initiates message respective handler

    /**
     * @param msgName     Custom message type
     * @param msg		  Packet message
     *
     * The CustomMsgHandler map contains number of CustomMsgHandler Initiated at server. Retrieves
     * CustomMsgHandler based on messageType (msgName) and calls corresponding initPublisher of
     * custom message handler. Further, returns error message based on presence of CustomMsgHandler
     * in the map.
     */
    std::string initPublisher(std::string const& msgName,
                              mw::internal::robotics::gazebotransport::Packet const& msg) const;

    /**
     * @param msgName     Custom message type
     * @param msg		  Packet message
     *
     * The CustomMsgHandler map contains number of CustomMsgHandler Initiated at server. Retrieves
     * CustomMsgHandler based on messageType (msgName) and calls corresponding publishCustomMsg of
     * custom message handler. Further, returns error message based on presence of CustomMsgHandler
     * in the map.
     */
    std::string publishCustomMsg(std::string const& msgName,
                                 mw::internal::robotics::gazebotransport::Packet const& msg) const;

    /**
     * @param msgName     Custom message type
     * @param msg		  Packet message
     *
     * The CustomMsgHandler map contains number of CustomMsgHandler Initiated at server. Retrieves
     * CustomMsgHandler based on messageType (msgName) and calls corresponding initSubscriber of
     * custom message handler. Further, returns error message based on presence of CustomMsgHandler
     * in the map.
     */
    std::string initSubscriber(std::string const& msgName,
                               mw::internal::robotics::gazebotransport::Packet const& msg) const;

    /**
     * @param msgName     Custom message type
     * @param msg		  Packet message
     *
     * The CustomMsgHandler map contains number of CustomMsgHandler Initiated at server. Retrieves
     * CustomMsgHandler based on messageType (msgName) and calls corresponding subscribeCustomMsg of
     * custom message handler. Further, returns serialized string or empty string based on presence
     * of CustomMsgHandler in the map.
     */
    std::pair<bool, std::string> subscribeCustomMsg(
        std::string const& msgName,
        mw::internal::robotics::gazebotransport::Packet const& msg) const;

    /// Clears CustomMsgHandler from map
    void reset();

  private:
    /// Stores multiple CustomMsgHandler initiated
    std::map<std::string, CustomMsgHandlerPtr> m_handlers;
};
typedef std::shared_ptr<CustomMsgDispatcher> CustomMsgDispatcherPtr;
} // namespace gazebotransport
} // namespace robotics

#endif
