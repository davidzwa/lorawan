/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 University of Padova
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Davide Magrin <magrinda@dei.unipd.it>
 *         Martina Capuzzo <capuzzom@dei.unipd.it>
 *
 * Modified by: Peggy Anderson <peggy.anderson@usask.ca>
 *              Yonatan Woldeleul Shiferaw <yoniwt@gmail.com>
 */

#ifndef END_DEVICE_LORAWAN_MAC_H
#define END_DEVICE_LORAWAN_MAC_H

#include "ns3/lorawan-mac.h"
#include "ns3/lorawan-mac-header.h"
#include "ns3/lora-frame-header.h"
#include "ns3/random-variable-stream.h"
#include "ns3/lora-device-address.h"
#include "ns3/traced-value.h"
#include "ns3/aes.h"

namespace ns3 {
namespace lorawan {

/**
 * Class representing the MAC layer of a LoRaWAN device.
 */
class EndDeviceLorawanMac : public LorawanMac
{
public:
  static TypeId GetTypeId (void);

  EndDeviceLorawanMac ();
  virtual ~EndDeviceLorawanMac ();

  /////////////////////
  // Sending methods //
  /////////////////////

  /**
   * Send a packet.
   *
   * The MAC layer of the ED will take care of using the right parameters.
   *
   * \param packet the packet to send
   */
  virtual void Send (Ptr<Packet> packet);

  /**
   * Checking if we are performing the transmission of a new packet or a retransmission, and call SendToPhy function.
   *
   * \param packet the packet to send
   */
  virtual void DoSend (Ptr<Packet> packet);

  /**
  * Add headers and send a packet with the sending function of the physical layer.
  *
  * \param packet the packet to send
  */
  virtual void SendToPhy (Ptr<Packet> packet);

  /**
   * Postpone transmission to the specified time and delete previously scheduled transmissions if present.
   *
   * \param nextTxDelay Delay at which the transmission will be performed.
   */
  virtual void postponeTransmission (Time nextTxDelay, Ptr<Packet>);


  ///////////////////////
  // Receiving methods //
  ///////////////////////

  /**
   * Receive a packet.
   *
   * This method is typically registered as a callback in the underlying PHY
   * layer so that it's called when a packet is going up the stack.
   *
   * \param packet the received packet.
   */
  virtual void Receive (Ptr<Packet const> packet);

  virtual void FailedReception (Ptr<Packet const> packet);

  /**
   * Perform the actions that are required after a packet send.
   *
   * This function handles opening of the first receive window.
   */
  virtual void TxFinished (Ptr<const Packet> packet);

  /////////////////////////
  // Getters and Setters //
  /////////////////////////

  /**
  * Reset retransmission parameters contained in the structure LoraRetxParams
  */
  virtual void resetRetransmissionParameters ();

  /**
   * Enable data rate adaptation in the retransmitting procedure.
   *
   * \param adapt If the data rate adaptation is enabled or not.
   */
  void SetDataRateAdaptation (bool adapt);

  /**
   * Get if data rate adaptation is enabled or not.
   */
  bool GetDataRateAdaptation (void);

  /**
   * Set the maximum number of transmissions allowed.
   *
   * \param maxNumbTx The maximum number of transmissions allowed
   */
  void SetMaxNumberOfTransmissions (uint8_t maxNumbTx);

  /**
   * Set the maximum number of transmissions allowed.
   */
  uint8_t GetMaxNumberOfTransmissions (void);

  /**
   * Set the data rate this end device will use when transmitting. For End
   * Devices, this value is assumed to be fixed, and can be modified via MAC
   * commands issued by the GW.
   *
   * \param dataRate The dataRate to use when transmitting.
   */
  void SetDataRate (uint8_t dataRate);

  /**
   * Get the data rate this end device is set to use.
   *
   * \return The data rate this device uses when transmitting.
   */
  uint8_t GetDataRate (void);

  /**
   * Get the transmission power this end device is set to use.
   *
   * \return The transmission power this device uses when transmitting.
   */
  virtual uint8_t GetTransmissionPower (void);

  /**
   * Set the network address of this device.
   *
   * \param address The address to set.
   */
  void SetDeviceAddress (LoraDeviceAddress address);

  /**
   * Get the network address of this device.
   *
   * \return This device's address.
   */
  LoraDeviceAddress GetDeviceAddress (void);

  /**
   * Set the multicast network address of this device.
   * 
   * This has to be set and different from `1` before calling ``EnableMulticast``.
   *
   * \param address The multicast address to set.
   */
  void SetMulticastDeviceAddress (LoraDeviceAddress address);

  /**
   * Get the multicast network address of this device.
   *
   * \return This device's multicast address.
   */
  LoraDeviceAddress GetMulticastDeviceAddress (void);
  
  /**
   * Set a value for the RX1DROffset parameter.
   *
   * This value decides the offset to use when deciding the DataRate of the
   * downlink transmission during the first receive window from the
   * replyDataRateMatrix.
   *
   * \param rx1DrOffset The value to set for the offset.
   */
  // void SetRx1DrOffset (uint8_t rx1DrOffset);

  /**
   * Get the value of the RX1DROffset parameter.
   *
   * \return The value of the RX1DROffset parameter.
   */
  // uint8_t GetRx1DrOffset (void);

  /**
   * Get the aggregated duty cycle.
   *
   * \return A time instance containing the aggregated duty cycle in fractional
   * form.
   */
  double GetAggregatedDutyCycle (void);

  /////////////////////////
  // MAC command methods //
  /////////////////////////

  /**
   * Add the necessary options and MAC commands to the LoraFrameHeader.
   *
   * \param frameHeader The frame header on which to apply the options.
   */
  void ApplyNecessaryOptions (LoraFrameHeader &frameHeader);

  /**
   * Add the necessary options and MAC commands to the LorawanMacHeader.
   *
   * \param macHeader The mac header on which to apply the options.
   */
  void ApplyNecessaryOptions (LorawanMacHeader &macHeader);

  /**
   * Set the message type to send when the Send method is called.
   */
  void SetMType (LorawanMacHeader::MType mType);

  /**
   * Get the message type to send when the Send method is called.
   */
  LorawanMacHeader::MType GetMType (void);

  /**
   * Parse and take action on the commands contained on this FrameHeader.
   */
  void ParseCommands (LoraFrameHeader frameHeader);

  /**
   * Perform the actions that need to be taken when receiving a LinkCheckAns command.
   *
   * \param margin The margin value of the command.
   * \param gwCnt The gateway count value of the command.
   */
  void OnLinkCheckAns (uint8_t margin, uint8_t gwCnt);

  /**
   * Perform the actions that need to be taken when receiving a LinkAdrReq command.
   *
   * \param dataRate The data rate value of the command.
   * \param txPower The transmission power value of the command.
   * \param enabledChannels A list of the enabled channels.
   * \param repetitions The number of repetitions prescribed by the command.
   */
  void OnLinkAdrReq (uint8_t dataRate, uint8_t txPower,
                     std::list<int> enabledChannels, int repetitions);

  /**
   * Perform the actions that need to be taken when receiving a DutyCycleReq command.
   *
   * \param dutyCycle The aggregate duty cycle prescribed by the command, in
   * fraction form.
   */
  void OnDutyCycleReq (double dutyCycle);

  /**
   * Perform the actions that need to be taken when receiving a RxParamSetupReq command.
   *
   * \param rxParamSetupReq The Parameter Setup Request
   */
  void OnRxParamSetupReq (Ptr<RxParamSetupReq> rxParamSetupReq);

  /**
   * Perform the actions that need to be taken when receiving a RxParamSetupReq
   * command based on the Device's Class Type.
   *
   * \param rxParamSetupReq The Parameter Setup Request
   */
  virtual void OnRxClassParamSetupReq (Ptr<RxParamSetupReq> rxParamSetupReq);

  /**
   * Perform the actions that need to be taken when receiving a DevStatusReq command.
   */
  void OnDevStatusReq (void);

  /**
   * Perform the actions that need to be taken when receiving a NewChannelReq command.
   */
  void OnNewChannelReq (uint8_t chIndex, double frequency, uint8_t minDataRate,
                        uint8_t maxDataRate);

  ////////////////////////////////////
  // Logical channel administration //
  ////////////////////////////////////

  /**
   * Add a logical channel to the helper.
   *
   * \param frequency The channel's center frequency.
   */
  void AddLogicalChannel (double frequency);

  /**
   * Set a new logical channel in the helper.
   *
   * \param chIndex The channel's new index.
   * \param frequency The channel's center frequency.
   * \param minDataRate The minimum data rate allowed on the channel.
   * \param maxDataRate The maximum data rate allowed on the channel.
   */
  void SetLogicalChannel (uint8_t chIndex, double frequency,
                          uint8_t minDataRate, uint8_t maxDataRate);

  /**
   * Add a logical channel to the helper.
   *
   * \param frequency The channel's center frequency.
   */
  void AddLogicalChannel (Ptr<LogicalLoraChannel> logicalChannel);

  /**
   * Add a subband to the logical channel helper.
   *
   * \param startFrequency The SubBand's lowest frequency.
   * \param endFrequency The SubBand's highest frequency.
   * \param dutyCycle The SubBand's duty cycle, in fraction form.
   * \param maxTxPowerDbm The maximum transmission power allowed on the SubBand.
   */
  void AddSubBand (double startFrequency, double endFrequency, double dutyCycle,
                   double maxTxPowerDbm);

  /**
   * Add a MAC command to the list of those that will be sent out in the next
   * packet.
   */
  void AddMacCommand (Ptr<MacCommand> macCommand);

  //////////////////////////////////////////////////
  //  LoRaWAN Class B Related enums and typdefs  //
  ////////////////////////////////////////////////
  
  /// The Mac state the end device is currently at
  enum MacState
  {
    MAC_TX, ///< Device Mac is transmitting 
    MAC_RX1, ///< Device Mac has currently opened the Rx1 window
    MAC_RX2, ///< Device Mac has currently opened the Rx2 window
    MAC_BEACON_GUARD, ///< Device Mac is in Beacon_guard state
    MAC_RX_BEACON_GUARD, ///< Device Mac is entering beacon guard during Rx1/Rx2 reception
    MAC_PING_SLOT_BEACON_GUARD, ///< Device Mac is entering beacon guard during ping slot reception
    MAC_BEACON_RESERVED, ///< Device Mac is in the Beacon_reserved state
    MAC_PING_SLOT,  ///< Device Mac has opened the ping slot receiver window
    MAC_IDLE, ///< Device Mac is free. You can transmit only in this state.
  };
  
  /// The LoRaWAN Device Class that the end device is operating on
  enum DeviceClass
  {
    CLASS_A, ///< device currently operating on Class A
    CLASS_B, ///< device currently operating on Class B
    CLASS_C  ///< device currently operating on Class C. NOT IMPLIMENTED YET.
  };
  
  /// The state of the device with respect to receiving beacon
  enum BeaconState
  {
    BEACON_UNLOCKED, ///< Beacon is not locked
    BEACON_SEARCH, ///< Mac is searching for beacon
    BEACON_LOCKED, ///< Beacon is locked
    BEACONLESS, ///< Beacon not received but device is still in the beaconless operation mode
  };
  
  /// Kind of service for which we are receiving a downlink in the ping slots
  enum ServiceType
  {
    UNICAST, ///< receives messages in the unicast address
    MULTICAST ///< receives messages in the multicast address
  };
  
  /**
   * \param the kind of service that the packet is intended for based on whether
   * the packet is received on multicast or unicast address
   * \param the received packet
   * \param the ping number in which the packet is received
   * \param the slot index through which we received the downlink
   */
  typedef Callback<void, enum ServiceType, Ptr<const Packet>, uint8_t > ClassBDownlinkCallback;
  
  //////////////////////////////////////////////////////
  // Conflict resolution Between Class A and Class B //
  ////////////////////////////////////////////////////
  
  /**
   * \brief Uses defined algorithm to give the next time by when you can transmit 
   * this packet 
   * 
   * Tx requests come from the application layer asynchronously. Therefore,
   * conditions arise where either the Tx it self or its corresponding Rx 
   * window conflict with the beacon guard, beacon reserved, or ping slots.
   * We can change this algorithm so as to try different techniques and compare
   * performance. 
   * \TODO Therefore additional API could be provided for selecting from 
   * available algorithms.   
   * 
   * \param packet The packet which you want to transmit. This can be used to 
   * calculate air-time and find out whether it fits or not. 
   * \return The next time by which you can attempt to transmit.  
   */
  Time ResolveWithClassBAndGetTime ( Ptr<const Packet> packet );
  
  ////////////////////////////////////////
  //  LoRaWAN Class B Beacon Receive   //
  //////////////////////////////////////
  
  /**
   * \brief It begins the beacon search process to switch to class B
   * 
   * The end-device application requests the LoRaWAN layer to switch to Class B mode.
   * The LoRaWAN layer in the end-device searches for a beacon and invokes the
   * m_beaconLockedCallback of the application if beacon was found and locked or 
   * a m_beaconLostCallback (to inform beacon is not locked). 
   */
  void SwitchToClassB (void);
  
  /**
   * \brief It cancels all pending ping slots and beacon guard and switches back
   * to Class A
   */
  void SwitchFromClassB (void); 
  
  /**
   * \brief Reserves the beacon guard time so that no transmission is during this
   * time
   * 
   */
  void StartBeaconGuard (void);
  
  /**
   * \brief Releases the beacon guard and schedules the beacon reserved.
   * 
   */
  void EndBeaconGuard (void);
  
  /**
   * \brief Perform operations needed to open the beacon receive window and 
   * also reserves the beacon reserved time for beacon only
   * 
   */
  void StartBeaconReserved (void);
  
  /**
   * \brief Releases the beacon reserved duration and performs operation to be
   * done at the end of the beacon reserved.
   * 
   * It schedules the coming ping slots, it schedules the next StartBeaconGuard
   * and it calls the beaconLockedCallback if m_beaconInfo.bcnTime is still 
   * 0 seconds. 
   * 
   */
  void EndBeaconReserved (void);
  
  /**
   * \brief Perform operations needed to close the beacon receive window
   * 
   * If the m_beaconInfo.bcnTime has a difference more than 128 minute 
   * it will call the  m_beaconLostCallback and do operations to switch the 
   * device back to Class A.
   */
  void CloseBeaconReceiveWindow (void);
  
  /**
   * \brief Do procedure that need to be done when you miss a beacon
   * 
   * If no beacon is locked yet it doesn't do anything.If the minimal beacon less 
   * operation mode is exceeded it switches back to class A.
   */
  void BeaconMissed (void);
  
  /**
   * \brief When beacon is received we reset the beacon paramters back to default
   * and  update beacon related information
   * 
   * This function should be called from the Receive function when a beacon 
   * payload is received (which is during the m_beaconState == BEACON_RESERVED).
   * 
   * \param packet the packet that contains the beacon payload
   */
  void BeaconReceived (Ptr<Packet const> packet);
  
  ////////////////////////////////////////
  // LoRaWAN Class B PingSlot Receive  //
  //////////////////////////////////////
  
  /**
   * \brief Perform operations needed to open the ping slot receive window 
   * 
   * \param slotIndex the slot index (N) to which this ping slot corresponds to
   * which ranges from 0 to pingNb-1 
   * 
   */
  void OpenPingSlotReceiveWindow (uint8_t slotIndex);
  
  /**
   * \brief Perform operations needed to close the ping slot receive window
   */
  void ClosePingSlotRecieveWindow (void);
  
  /**
   * \brief Process a ping payload and fire traces and invoke the classBDownlinkCallback
   * callback with the slot index it is received at.
   * 
   * This is called by the Receive method when a packet is received while the 
   * EndDeviceLorawanMac is in the PINGSLOT context.
   * 
   * \param packet the packet that is received in the ping slot
   */
  void PingReceived (Ptr<Packet const> packet);

  
  ///////////////////////////////////////////////////
  //  LoRaWAN Class B Related Getters and Setters //
  /////////////////////////////////////////////////
  
  /**
   * \brief Set the device class that the end device is operating on
   *
   * This method switches the device state from one class to another. 
   * To switch to Class B, it checks the current DeviceClass and BeaconState.
   *  
   * \param deviceClass the device class to change to
   * 
   * \return true when devices class is successfully set  
   */
  bool SetDeviceClass (enum DeviceClass deviceClass);
  
  /**
   * \brief Get the device class that the end device is operating on
   * 
   * \return device class that the end device is operating on will be returned
   */
  enum DeviceClass GetDeviceClass (void);
  
  /**
   * \brief Set the data rate corresponding to the ping slot windows
   * 
   * \param pingSlotDr the data rate to be used for the ping slots
   */
  void SetPingSlotReceiveWindowDataRate (uint8_t pingSlotDr);
  
  /**
   * \brief Get the data rate which the ping slots are operating on
   * 
   * \return the data rate the ping slots are operating on
   */
  uint8_t GetPingSlotReceiveWindowDataRate (void);

  /**
   * \brief Set the frequency that correspond to the ping slot windows
   * 
   * The ping slots by default operates on 869.525MHz for EU Region. 
   * 
   * \param frequency the frequency in MHz that corresponds to the ping slots
   */
  void SetPingSlotReceiveWindowFrequency (double frequency); 
  
  /**
   * \brief Get the frequency that the ping slots are operating on
   * 
   * \return the frequency in MHz on which the ping slots are operating on
   */
  double GetPingSlotRecieveWindowFrequency (void);
  
  /**
   * \brief Set the frequency that we expect to receive beacons
   * 
   * The beacon by default operates on 869.525MHz for EU Region. This is method
   * is provided just for experiment and the default should be left as it is for
   * EU Region
   * 
   * \param frequency the frequency in MHz to be used by the end devices to
   * listen to beacons. 
   */
  void SetBeaconReceiveWindowFrequency (double frequency); 
  
  /**
   * \brief Get the frequency that we expect to receive the beacons
   * 
   * \return the frequency in MHz on which we receive beacons
   */
  double GetBeaconRecieveWindowFrequency (void);
 
    /**
   * \brief Set the DR that we expect to receive beacons
   * 
   * The beacon by default uses DR3 for EU Region. This is method
   * is provided just for experiment and the default should be left as it is for
   * EU Region
   * 
   * \param beaconDr the DR used by the end devices to listen to beacons. 
   */
  void SetBeaconReceiveWindowDataRate (uint8_t beaconDr); 
  
  /**
   * \brief Get the DR that we expect to receive the beacons
   * 
   * \return the DR used to receive beacons
   */
  uint8_t GetBeaconRecieveWindowDataRate (void);
  
  /**
   * \brief Set the ping-slot-periodicity for which to open the ping slots 
   * 
   * This will set the ping-slot-periodicity for opening ping slots. It will   
   * additionally drive and set the PingNb and the PingPeriod. Periodicity 
   * should be between 0 to 7. 
   * It resizes and void m_pingSlotInfo.pendingPingSlotEvents.
   * 
   * \param periodicity the ping-slot-periodicity for opening the ping slots 
   * per beacon period.
   */
  void SetPingSlotPeriodicity (uint8_t periodicity);
  
  /**
   * \brief Get the ping-slot-periodicity which the end-device is opening the
   * ping slots
   * 
   * \return the ping-slot-periodicity which the end-device is opening its ping
   * slots 
   */
  uint8_t GetPingSlotPeriodicity (void);
  
  /**
   * \brief Get the number of ping slots to open in a beacon period
   *
   * This will set the pingNb which is the number of ping slots per beacon period. 
   * It will also drive and set the PingSlotPeriodicity and the PingPeriod. It 
   * will give an error for invalid PingNb.
   * It resizes and void m_pingSlotInfo.pendingPingSlotEvents.
   * 
   * \param pingNb the number of ping slots per beacon period 
   */
  void SetPingNb (uint8_t pingNb);
  
  /**
   * \brief Get the number of ping slots the device is opening per beacon period.
   * 
   * \return the number of ping slots per beacon period
   */
  uint8_t GetPingNb (void);
  
  /**
   * \brief Set the ping period between ping slots
   * 
   * This will set the ping period between ping slots. Additionally, it will 
   * drive and set the PingNb and the PingPeriod. It will give an error for
   * invalid ping periods.
   * It resizes and void m_pingSlotInfo.pendingPingSlotEvents.
   * 
   * \param pingPeriod the ping period between ping slots
   */
  void SetPingPeriod (uint pingPeriod); 
  
  /**
   * \brief Get the ping period between ping slots
   * 
   * \return the ping period between ping slots
   */
  uint GetPingPeriod (void);

  /**
   * \brief Used to set a callback for which to receive the classB downlinks.
   * 
   * \param classBDownlinkCallback a callback to be invoked when receiving
   * downlinks in the ping slots. 
   */
  void SetClassBDownlinkCallback (ClassBDownlinkCallback classBDownlinkCallback);
  
  /**
   * \brief Used to set a callback to be invoked when beacon is locked after 
   * the BeaconSearch method is called.
   * 
   * \param beaconLockedCallback the callback to be invoked when beacon is locked.
   */
  void SetBeaconLockedCallback (Callback<void> beaconLockedCallback);
  
  /**
   * \brief Used to set a callback to be invoked when beacon is lost after
   * the minimal beaconless operation mode, or after invoking the function
   * BeaconSearch.
   * 
   * \param beaconLostCallback the callback to be invoked when beacon is not locked. 
   */
  void SetBeaconLostCallback (Callback<void> beaconLostCallback);
  
  
  /**
   * \brief Get the mac state that end device is currently at
   * 
   * The SetMacState should be private as the state should be only modified 
   * internally when conditions are meet.
   * 
   * \return the m_macState that the end device is currently at
   */
  enum MacState GetMacState (void);  
  
  ///////////////////////////////////////////
  // Enabling and Disabling Multicasting  //
  /////////////////////////////////////////
  
  /**
   * \brief Enabling Class B multicast reception
   * 
   * The multicast address has to be set in-order to enable multicast. If it is 
   * enabled, Switching to Class B will automatically use the class B parameters
   * for multicasting. Furthermore, you will now receive in the application 
   * layer callback the multicast packets.
   * 
   * //\NOTE coexistence of multicast and unicast with different parameter is not 
   * implemented in this module. Furthermore, there can only be one multicast 
   * service per device in the current implementation.
   *  
   */
  void EnableMulticast (void);
  
  /**
   * \brief Disable Class B multicast reception
   * 
   * If disabled although the packet will still be received, it won't be passed
   * to the application layer.
   * 
   */
  void DisableMulticast (void);
  
  /**
   * \brief Check whether the end device is enabled for multicast service.
   *
   * \return true if the end device is enabled for multicast transmission.
   */
  bool IsMulticastEnabled  (void);
  
  ////////////////////////////
  // Tracesource callbacks //
  //////////////////////////
  
  /**
   * Tracecallback for receiving a packet that has been received via ping slot
   * 
   * \param mcAddress the multicast address of this device, 0 if it is a unicast device
   * \param ucAddress the unicast address of this device
   * \param packet the packet received
   * \param slotIndex from 0 to N-1 where N is the PingNb which is the number of 
   *  ping slots in a beacon period
   */
  typedef void (*ReceivedPingPacket) (LoraDeviceAddress mcAddress, LoraDeviceAddress ucAddress, Ptr<const Packet> packet, uint8_t slotIndex);
  
  
  /**
   * Custom traced value that will include the unicast and the multicast address along
   * with the new traced value
   * 
   * \param mcAddress the multicast address of this device, 0 if it is a unicast device
   * \param ucAddress the unicast address of this device
   * \param beaconMissedCount number of beacons missed
   */
  typedef void (*CustomTracedValue) (LoraDeviceAddress mcAddress, LoraDeviceAddress ucAddress, uint32_t newValue);
  
  /**
   * Custom traced value that will include the unicast and the multicast address along
   * with the new traced value
   * 
   * \param mcAddress the multicast address of this device, 0 if it is a unicast device
   * \param ucAddress the unicast address of this device
   * \param numberOfOverheardPacket number of overheard packet
   */ 
  typedef void (*NumberOfOverhearedPackets) (LoraDeviceAddress mcAddress, LoraDeviceAddress ucAddress, uint32_t numberOfOverheardPacket);
  
  //////////////////////////////////////////////////////////////
  // MAC Layer modification to enabling cooprative relaying 
  // 
  // Note: This modification is just to relay the packet received
  // on the next ping slot if a received packet has a hop count of
  // 0. The transmission power that is used in-order to relay 
  // the packet is the 27db which is the maximum power allowed in
  // 869.525 band + margin power (like 10 db) devided by the 
  // number of devices in the multicast group
  //
  ////////////////////////////////////////////////////////////
  
  void EnableCoordinatedRelaying (uint32_t numberOfEndDeviceInMcGroup, uint8_t relayingAlgorithm); 
  
  /**
   * To get the power by which to relay based on the algorithm selected
   */
  double GetRelayingPower (void);
  
protected:
  /**
   * Structure representing the parameters that will be used in the
   * retransmission procedure.
   */
  struct LoraRetxParameters
  {
    Time firstAttempt;
    Ptr<Packet> packet = 0;
    bool waitingAck = false;
    uint8_t retxLeft;
  };

  /**
   * Enable Data Rate adaptation during the retransmission procedure.
   */
  bool m_enableDRAdapt;

  /**
   * Maximum number of transmission allowed.
   */
  uint8_t m_maxNumbTx;

  /**
   * The DataRate this device is using to transmit.
   */
  TracedValue<uint8_t> m_dataRate;

  /**
   * The transmission power this device is using to transmit.
   */
  TracedValue<double> m_txPower;

  /**
   * The coding rate used by this device.
   */
  uint8_t m_codingRate;

  /**
   * Whether or not the header is disabled for communications by this device.
   */
  bool m_headerDisabled;

  /**
   * The address of this device.
   */
  LoraDeviceAddress m_address;

  /**
   * Find the minimum waiting time before the next possible transmission based
   * on End Device's Class Type.
   */
  virtual Time GetNextClassTransmissionDelay (Time waitingTime);

  /**
   * Find a suitable channel for transmission. The channel is chosen among the
   * ones that are available in the ED's LogicalLoraChannel, based on their duty
   * cycle limitations.
   */
  Ptr<LogicalLoraChannel> GetChannelForTx (void);

  /**
   * The duration of a receive window in number of symbols. This should be
   * converted to time based or the reception parameter used.
   *
   * The downlink preamble transmitted by the gateways contains 8 symbols.
   * The receiver requires 5 symbols to detect the preamble and synchronize.
   * Therefore there must be a 5 symbols overlap between the receive window
   * and the transmitted preamble.
   * (Ref: Recommended SX1272/76 Settings for EU868 LoRaWAN Network Operation )
   */
  uint8_t m_receiveWindowDurationInSymbols;

  /**
   * List of the MAC commands that need to be applied to the next UL packet.
   */
  std::list<Ptr<MacCommand> > m_macCommandList;

  /* Structure containing the retransmission parameters
   * for this device.
   */
  struct LoraRetxParameters m_retxParams;

  /**
   * An uniform random variable, used by the Shuffle method to randomly reorder
   * the channel list.
   */
  Ptr<UniformRandomVariable> m_uniformRV;

  /////////////////
  //  Callbacks  //
  /////////////////

  /**
   * The trace source fired when the transmission procedure is finished.
   *
   * \see class CallBackTraceSource
   */
  TracedCallback<uint8_t, bool, Time, Ptr<Packet> > m_requiredTxCallback;

private:
  /**
   * Randomly shuffle a Ptr<LogicalLoraChannel> vector.
   *
   * Used to pick a random channel on which to send the packet.
   */
  std::vector<Ptr<LogicalLoraChannel> > Shuffle (std::vector<Ptr<LogicalLoraChannel> > vector);

  /**
   * Find the minimum waiting time before the next possible transmission.
   */
  Time GetNextTransmissionDelay (void);

  /**
   * Whether this device's data rate should be controlled by the NS.
   */
  bool m_controlDataRate;

  /**
   * The event of retransmitting a packet in a consecutive moment if an ACK is not received.
   *
   * This Event is used to cancel the retransmission if the ACK is found in ParseCommand function and
   * if a newer packet is delivered from the application to be sent.
   */
  EventId m_nextTx;

  /**
   * The event of transmitting a packet in a consecutive moment, when the duty cycle let us transmit.
   *
   * This Event is used to cancel the transmission of this packet if a newer packet is delivered from the application to be sent.
   */
  EventId m_nextRetx;

  /**
   * The last known link margin.
   *
   * This value is obtained (and updated) when a LinkCheckAns Mac command is
   * received.
   */
  TracedValue<double> m_lastKnownLinkMargin;

  /**
   * The last known gateway count (i.e., gateways that are in communication
   * range with this end device)
   *
   * This value is obtained (and updated) when a LinkCheckAns Mac command is
   * received.
   */
  TracedValue<int> m_lastKnownGatewayCount;

  /**
   * The aggregated duty cycle this device needs to respect across all sub-bands.
   */
  TracedValue<double> m_aggregatedDutyCycle;

  /**
   * The message type to apply to packets sent with the Send method.
   */
  LorawanMacHeader::MType m_mType;

  uint16_t m_currentFCnt;
};


} /* namespace ns3 */

}
#endif /* END_DEVICE_LORAWAN_MAC_H */
