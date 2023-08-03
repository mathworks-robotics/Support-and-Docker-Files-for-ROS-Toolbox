/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_SUBSCRIBEIMAGEMSGHANDLER_HPP
#define GAZEBOCOSIM_SUBSCRIBEIMAGEMSGHANDLER_HPP

#include "gazebotransport/gazeboserver/gazebomsghandler/SubscribeMsgHandler.hpp"

namespace robotics {
namespace gazebotransport {
class SubscribeImageMsgHandler : public SubscribeMsgHandler {
  public:
    /// Constructor
    /// This class initiates message handler to exchange SubscribeImage message
    /*
      @param world                         Pointer to Gazebo simulation world
      @param node                          Pointer to transport node
    */
    explicit SubscribeImageMsgHandler(gazebo::physics::WorldPtr world,
                                      gazebo::transport::NodePtr node);

    /// Destructor
    ~SubscribeImageMsgHandler();

    /// Returns Message ID
    uint32_t getAcceptID() const override;

  private:
    /// returns topic name for the subscribe image request
    virtual std::string getTopicName(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) override;

    /// do subscribe to image message
    void doSubscribe(gazebo::physics::WorldPtr world,
                     gazebo::transport::NodePtr node,
                     mw::internal::robotics::gazebotransport::Packet const& msgContent) override;
};
} // namespace gazebotransport
} // namespace robotics
#endif
