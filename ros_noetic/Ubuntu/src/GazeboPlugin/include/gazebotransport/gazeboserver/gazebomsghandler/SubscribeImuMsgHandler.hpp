/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_SUBSCRIBEIMUMSGHANDLER_HPP
#define GAZEBOCOSIM_SUBSCRIBEIMUMSGHANDLER_HPP

#include "gazebotransport/gazeboserver/gazebomsghandler/SubscribeMsgHandler.hpp"

namespace robotics {
namespace gazebotransport {
class SubscribeImuMsgHandler : public SubscribeMsgHandler {
  public:
    /// Constructor
    /// This class initiated message handler to exchange SubscribeImu message
    /*
      @param world                         Pointer to Gazebo simulation world
      @param node                          Pointer to transport node
    */
    explicit SubscribeImuMsgHandler(gazebo::physics::WorldPtr world,
                                    gazebo::transport::NodePtr node);

    /// Destructor
    ~SubscribeImuMsgHandler();

    /// Returns Message ID
    uint32_t getAcceptID() const override;

  private:
    /// returns topic name for the subscribe IMU request
    virtual std::string getTopicName(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) override;

    /// do subscribe to IMU message
    void doSubscribe(gazebo::physics::WorldPtr world,
                     gazebo::transport::NodePtr node,
                     mw::internal::robotics::gazebotransport::Packet const& msgContent) override;
};
} // namespace gazebotransport
} // namespace robotics
#endif
