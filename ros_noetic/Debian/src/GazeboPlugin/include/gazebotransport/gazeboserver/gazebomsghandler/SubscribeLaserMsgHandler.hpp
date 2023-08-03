/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_SUBSCRIBELASERMSGHANDLER_HPP
#define GAZEBOCOSIM_SUBSCRIBELASERMSGHANDLER_HPP

#include "gazebotransport/gazeboserver/gazebomsghandler/SubscribeMsgHandler.hpp"

namespace robotics {
namespace gazebotransport {
class SubscribeLaserMsgHandler : public SubscribeMsgHandler {
  public:
    /// Constructor
    /// This class initiated message handler to exchange SubscribeLaser message
    /*
      @param world                         Pointer to Gazebo simulation world
      @param node                          Pointer to transport node
    */
    explicit SubscribeLaserMsgHandler(gazebo::physics::WorldPtr world,
                                      gazebo::transport::NodePtr node);

    /// Destructor
    ~SubscribeLaserMsgHandler();

    /// Returns Message ID
    uint32_t getAcceptID() const override;

  private:
    /// returns topic name for the subscribe laser request
    virtual std::string getTopicName(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) override;

    /// do subscribe to laser message
    void doSubscribe(gazebo::physics::WorldPtr world,
                     gazebo::transport::NodePtr node,
                     mw::internal::robotics::gazebotransport::Packet const& msgContent) override;
};
} // namespace gazebotransport
} // namespace robotics
#endif
