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
 *              qiuyukang <b612n@qq.com>
 */

#include "ns3/class-b-end-device-lorawan-mac.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/hop-count-tag.h"
#include "ns3/bcn-payload.h"
#include "ns3/log.h"
#include <algorithm>

namespace ns3
{
    namespace lorawan
    {

        NS_LOG_COMPONENT_DEFINE("ClassBEndDeviceLorawanMac");

        NS_OBJECT_ENSURE_REGISTERED(ClassBEndDeviceLorawanMac);

        TypeId
        ClassBEndDeviceLorawanMac::GetTypeId(void)
        {
            static TypeId tid = TypeId("ns3::ClassBEndDeviceLorawanMac")
                                    .SetParent<EndDeviceLorawanMac>()
                                    .SetGroupName("lorawan")
                                    .AddTraceSource("MacState",
                                                    "The current Mac state of the device",
                                                    MakeTraceSourceAccessor(&ClassBEndDeviceLorawanMac::m_macState),
                                                    "ns3::TracedValueCallback::EndDeviceLorawanMac::MacState")
                                    .AddTraceSource("DeviceClass",
                                                    "The current device class of the device",
                                                    MakeTraceSourceAccessor(&ClassBEndDeviceLorawanMac::m_deviceClass),
                                                    "ns3::TracedValueCallback::EndDeviceLorawanMac::DeviceClass")
                                    .AddTraceSource("BeaconState",
                                                    "The current beacon state of the device",
                                                    MakeTraceSourceAccessor(&ClassBEndDeviceLorawanMac::m_beaconState),
                                                    "ns3::TracedValueCallback::EndDeviceLorawanMac::BeaconState")
                                    .AddTraceSource("ReceivedPingMessages",
                                                    "The packet received via ping slot",
                                                    MakeTraceSourceAccessor(&ClassBEndDeviceLorawanMac::m_receivedPingPacket),
                                                    "ns3::EndDeviceLorawanMac::ReceivedPingPacket")
                                    .AddTraceSource("FailedPings",
                                                    "Number of packets failed while receiving in the ping slots",
                                                    MakeTraceSourceAccessor(&ClassBEndDeviceLorawanMac::m_failedPings),
                                                    "ns3::TracedValueCallback::Uint32")
                                    .AddTraceSource("TotalSuccessfulBeaconPackets",
                                                    "Number of beacons successfully received during the simulation time",
                                                    MakeTraceSourceAccessor(&ClassBEndDeviceLorawanMac::m_totalSuccessfulBeaconPackets),
                                                    "ns3::TracedValueCallback::Uint32")
                                    .AddTraceSource("TotalSuccessfulBeaconPacketsTracedCallback",
                                                    "Number of beacons successfully received during the simulation time",
                                                    MakeTraceSourceAccessor(&ClassBEndDeviceLorawanMac::m_totalSuccessfulBeaconPacketsTracedCallback),
                                                    "ns3::EndDeviceLorawanMac::CustomTracedValue")
                                    .AddTraceSource("MissedBeaconCount",
                                                    "Number of beacons missed throughout the simulation period including during switch to class B attempts",
                                                    MakeTraceSourceAccessor(&ClassBEndDeviceLorawanMac::m_missedBeaconCount),
                                                    "ns3::TracedValueCallback::Uint32")
                                    .AddTraceSource("MissedBeaconTracedCallback",
                                                    "Number of beacons missed throughout the simulation period including during switch to class B attempts",
                                                    MakeTraceSourceAccessor(&ClassBEndDeviceLorawanMac::m_missedBeaconTracedCallback),
                                                    "ns3::EndDeviceLorawanMac::CustomTracedValue")
                                    .AddTraceSource("MaximumConsecutiveBeaconsMissed",
                                                    "The maximum number of beacons missed consecutively",
                                                    MakeTraceSourceAccessor(&ClassBEndDeviceLorawanMac::m_maximumConsecutiveBeaconsMissed),
                                                    "ns3::TracedValueCallback::Uint8")
                                    .AddTraceSource("CurrentConsecutiveBeaconsMissed",
                                                    "The number of beacons missed until now consecutively if the device is in minimal beaconless operation mode",
                                                    MakeTraceSourceAccessor(&ClassBEndDeviceLorawanMac::m_currentConsecutiveBeaconsMissed),
                                                    "ns3::TracedValueCallback::Uint8")
                                    .AddTraceSource("CurrentConsecutiveBeaconsMissedTracedCallback",
                                                    "The number of beacons missed until now consecutively if the device is in minimal beaconless operation mode",
                                                    MakeTraceSourceAccessor(&ClassBEndDeviceLorawanMac::m_currentConsecutiveBeaconsMissedTracedCallback),
                                                    "ns3::EndDeviceLorawanMac::CustomTracedValue")
                                    .AddTraceSource("AttemptToClassB",
                                                    "The number of attempt in the simulation time to switch to class B",
                                                    MakeTraceSourceAccessor(&ClassBEndDeviceLorawanMac::m_attemptToClassB),
                                                    "ns3::TracedValueCallback::Uint32")
                                    .AddTraceSource("TotalBytesReceived",
                                                    "The number of downlink bytes received by the device",
                                                    MakeTraceSourceAccessor(&ClassBEndDeviceLorawanMac::m_totalBytesReceived),
                                                    "ns3::TracedValueCallback::Uint32")
                                    .AddTraceSource("NumberOfOverhearedPackets",
                                                    "The packet that are overheard in the ping slots",
                                                    MakeTraceSourceAccessor(&ClassBEndDeviceLorawanMac::m_numberOfOverhearedPackets),
                                                    "ns3::EndDeviceLorawanMac::NumberOfOverhearedPackets")
                                    .AddConstructor<ClassBEndDeviceLorawanMac>();
            return tid;
        }

        ClassBEndDeviceLorawanMac::ClassBEndDeviceLorawanMac()
            : // LoraWAN default
              m_receiveDelay1(Seconds(1)),
              // LoraWAN default
              m_receiveDelay2(Seconds(2)),
              m_rx1DrOffset(0),
              m_mcAddress(LoraDeviceAddress(1)),
              m_macState(ClassBEndDeviceLorawanMac::MAC_IDLE),
              m_deviceClass(ClassBEndDeviceLorawanMac::CLASS_A),
              m_beaconState(ClassBEndDeviceLorawanMac::BEACON_UNLOCKED),
              m_slotIndexLastOpened(255),
              m_failedPings(0),
              m_totalSuccessfulBeaconPackets(0),
              m_missedBeaconCount(0),
              m_maximumConsecutiveBeaconsMissed(0),
              m_currentConsecutiveBeaconsMissed(0),
              m_attemptToClassB(0),
              m_totalBytesReceived(0),
              m_overheardPacketCount(0),
              m_enableMulticast(false),
              m_relayActivated(false),
              m_relayPending(false),
              maxHop(2)
        {
            NS_LOG_FUNCTION(this);

            // Void the two receiveWindow events
            m_closeFirstWindow = EventId();
            m_closeFirstWindow.Cancel();
            m_closeSecondWindow = EventId();
            m_closeSecondWindow.Cancel();
            m_secondReceiveWindow = EventId();
            m_secondReceiveWindow.Cancel();

            //Initializing structure for class B beacon and ping
            m_beaconInfo = ClassBEndDeviceLorawanMac::BeaconInfo();
            m_pingSlotInfo = ClassBEndDeviceLorawanMac::PingSlotInfo();
            m_classBReceiveWindowInfo = ClassBEndDeviceLorawanMac::ClassBReceiveWindowInfo();

            //Initialize and void ping slot Events
            //used to cancel events when device Class is switched from Class B to A
            m_pingSlotInfo.pendingPingSlotEvents.resize(m_pingSlotInfo.pingNb);
            for (EventId &ping : m_pingSlotInfo.pendingPingSlotEvents)
            {
                ping = EventId();
                ping.Cancel();
            }

            //Initializing relay power structure
            m_relayPower = ClassBEndDeviceLorawanMac::RelayPower();
        }

        ClassBEndDeviceLorawanMac::~ClassBEndDeviceLorawanMac()
        {
            NS_LOG_FUNCTION_NOARGS();
        }

        /////////////////////
        // Sending methods //
        /////////////////////

        void ClassBEndDeviceLorawanMac::SendToPhy(Ptr<Packet> packetToSend)
        {
            /////////////////////////////////////////////////////////
            // Add headers, prepare TX parameters and send the packet
            /////////////////////////////////////////////////////////

            NS_LOG_DEBUG("PacketToSend: " << packetToSend);

            // Data Rate Adaptation as in LoRaWAN specification, V1.0.2 (2016)
            if (m_enableDRAdapt && (m_dataRate > 0) && (m_retxParams.retxLeft < m_maxNumbTx) && (m_retxParams.retxLeft % 2 == 0))
            {
                m_txPower = 14; // Reset transmission power
                m_dataRate = m_dataRate - 1;
            }

            // Craft LoraTxParameters object
            LoraTxParameters params;
            params.sf = GetSfFromDataRate(m_dataRate);
            params.headerDisabled = m_headerDisabled;
            params.codingRate = m_codingRate;
            params.bandwidthHz = GetBandwidthFromDataRate(m_dataRate);
            params.nPreamble = m_nPreambleSymbols;
            params.crcEnabled = 1;
            params.lowDataRateOptimizationEnabled = LoraPhy::GetTSym(params) > MilliSeconds(16) ? true : false;

            // Wake up PHY layer and directly send the packet

            Ptr<LogicalLoraChannel> txChannel = GetChannelForTx();

            NS_LOG_DEBUG("PacketToSend: " << packetToSend);
            m_phy->Send(packetToSend, params, txChannel->GetFrequency(), m_txPower);

            //////////////////////////////////////////////
            // Register packet transmission for duty cycle
            //////////////////////////////////////////////

            // Compute packet duration
            Time duration = m_phy->GetOnAirTime(packetToSend, params);

            // Register the sent packet into the DutyCycleHelper
            m_channelHelper.AddEvent(duration, txChannel);

            // Mac is now busy transmitting
            SetMacState(MAC_TX);

            //////////////////////////////
            // Prepare for the downlink //
            //////////////////////////////

            // Switch the PHY to the channel so that it will listen here for downlink
            m_phy->GetObject<EndDeviceLoraPhy>()->SetFrequency(txChannel->GetFrequency());

            // Instruct the PHY on the right Spreading Factor to listen for during the window
            // create a SetReplyDataRate function?
            uint8_t replyDataRate = GetFirstReceiveWindowDataRate();
            NS_LOG_DEBUG("m_dataRate: " << unsigned(m_dataRate) << ", m_rx1DrOffset: " << unsigned(m_rx1DrOffset) << ", replyDataRate: " << unsigned(replyDataRate) << ".");

            m_phy->GetObject<EndDeviceLoraPhy>()->SetSpreadingFactor(GetSfFromDataRate(replyDataRate));
        }

        //////////////////////////
        //  Receiving methods   //
        //////////////////////////
        void
        ClassBEndDeviceLorawanMac::Receive(Ptr<Packet const> packet)
        {
            NS_LOG_FUNCTION(this << packet);

            // Work on a copy of the packet
            Ptr<Packet> packetCopy = packet->Copy();

            BcnPayload bcnPayload;
            packetCopy->PeekHeader(bcnPayload);

            NS_LOG_DEBUG("MacState while receiving " << m_macState);

            if (m_macState == MAC_BEACON_RESERVED)
            {
                if (bcnPayload.GetBcnTime() != 0)
                {
                    NS_LOG_DEBUG("BeaconRecieved!");
                    BeaconReceived(packetCopy);
                }
                else
                {
                    //We received a non-beacon packet so basically beacon is missed
                    BeaconMissed();
                }
            }

            if (m_macState == MAC_PING_SLOT || m_macState == MAC_PING_SLOT_BEACON_GUARD)
            {
                if (bcnPayload.GetBcnTime() != 0)
                {
                    NS_LOG_DEBUG("Dropping packet! BcnPacket received in a wrong slot (Ping Slot)");
                    //\TODO may be fire trace source here

                    //Once the packet has failed free the MAC
                    if (m_macState == MAC_PING_SLOT_BEACON_GUARD)
                    {
                        //If beacon guard started before end of the packet then switch back
                        //to the beacon reserved
                        NS_LOG_DEBUG("Switching back to Beacon Guard!");
                        SetMacState(MAC_BEACON_GUARD);
                    }
                    else if (m_macState == MAC_PING_SLOT)
                    {
                        NS_LOG_DEBUG("Switching to IDLE!");
                        SetMacState(MAC_IDLE);
                    }
                }
                else
                {
                    //\TODO When ping and rx2 window parameter matches: what if we receive a
                    // class A downlink during the ping slot

                    PingReceived(packetCopy);
                }
            }

            if (m_macState == MAC_RX1 || m_macState == MAC_RX2 || m_macState == MAC_RX_BEACON_GUARD)
            {
                if (bcnPayload.GetBcnTime() != 0)
                {
                    //This happens if we configure the Rx2 SF and the Beacon SF with same value
                    NS_LOG_DEBUG("Dropping packet! BcnPacket received in a wrong slot (Class A Rx2-slot)");
                    //\TODO may be fire trace source here
                }
                else
                {
                    // Remove the Mac Header to get some information
                    LorawanMacHeader mHdr;
                    packetCopy->RemoveHeader(mHdr);

                    NS_LOG_DEBUG("Mac Header: " << mHdr);

                    // Only keep analyzing the packet if it's downlink
                    if (!mHdr.IsUplink())
                    {
                        NS_LOG_INFO("Found a downlink packet.");

                        // Remove the Frame Header
                        LoraFrameHeader fHdr;
                        fHdr.SetAsDownlink();
                        packetCopy->RemoveHeader(fHdr);

                        NS_LOG_DEBUG("Frame Header: " << fHdr);

                        // Determine whether this packet is for us
                        bool messageForUs = (m_address == fHdr.GetAddress());

                        if (messageForUs)
                        {
                            NS_LOG_INFO("The message is for us!");

                            // If it exists, cancel the second receive window event
                            // THIS WILL BE GetReceiveWindow()
                            Simulator::Cancel(m_secondReceiveWindow);

                            // Parse the MAC commands
                            ParseCommands(fHdr);

                            // TODO Pass the packet up to the NetDevice

                            // Call the trace source
                            m_receivedPacket(packet);
                        }
                        else
                        {
                            NS_LOG_DEBUG("The message is intended for another recipient.");

                            // In this case, we are either receiving in the first receive window
                            // and finishing reception inside the second one, or receiving a
                            // packet in the second receive window and finding out, after the
                            // fact, that the packet is not for us. In either case, if we no
                            // longer have any retransmissions left, we declare failure.
                            if (m_retxParams.waitingAck && m_secondReceiveWindow.IsExpired())
                            {
                                if (m_retxParams.retxLeft == 0)
                                {
                                    uint8_t txs = m_maxNumbTx - (m_retxParams.retxLeft);
                                    m_requiredTxCallback(txs, false, m_retxParams.firstAttempt, m_retxParams.packet);
                                    NS_LOG_DEBUG("Failure: no more retransmissions left. Used " << unsigned(txs) << " transmissions.");

                                    // Reset retransmission parameters
                                    resetRetransmissionParameters();
                                }
                                else // Reschedule
                                {
                                    this->Send(m_retxParams.packet);
                                    NS_LOG_INFO("We have " << unsigned(m_retxParams.retxLeft) << " retransmissions left: rescheduling transmission.");
                                }
                            }
                        }
                    }
                    else if (m_retxParams.waitingAck && m_secondReceiveWindow.IsExpired())
                    {
                        NS_LOG_INFO("The packet we are receiving is in uplink.");
                        if (m_retxParams.retxLeft > 0)
                        {
                            this->Send(m_retxParams.packet);
                            NS_LOG_INFO("We have " << unsigned(m_retxParams.retxLeft) << " retransmissions left: rescheduling transmission.");
                        }
                        else
                        {
                            uint8_t txs = m_maxNumbTx - (m_retxParams.retxLeft);
                            m_requiredTxCallback(txs, false, m_retxParams.firstAttempt, m_retxParams.packet);
                            NS_LOG_DEBUG("Failure: no more retransmissions left. Used " << unsigned(txs) << " transmissions.");

                            // Reset retransmission parameters
                            resetRetransmissionParameters();
                        }
                    }
                }
                if (m_macState == MAC_RX_BEACON_GUARD)
                {
                    //packet finished being received during guard, put it back to guard
                    SetMacState(MAC_BEACON_GUARD);
                }
                else
                {
                    SetMacState(MAC_IDLE);
                }
            }

            m_phy->GetObject<EndDeviceLoraPhy>()->SwitchToSleep();
        }

        void
        ClassBEndDeviceLorawanMac::FailedReception(Ptr<Packet const> packet)
        {
            NS_LOG_FUNCTION(this << packet);

            // Switch to sleep after a failed reception
            m_phy->GetObject<EndDeviceLoraPhy>()->SwitchToSleep();

            if (m_macState == MAC_BEACON_RESERVED)
            {
                BeaconMissed();
            }

            if (m_macState == MAC_PING_SLOT || m_macState == MAC_PING_SLOT_BEACON_GUARD)
            {
                m_failedPings++;

                //Once the packet has failed free the MAC
                if (m_macState == MAC_PING_SLOT_BEACON_GUARD)
                {
                    //If beacon guard started before end of the packet then switch back
                    //to the beacon reserved
                    NS_LOG_DEBUG("Ping failed! Switching back to Beacon Guard");
                    SetMacState(MAC_BEACON_GUARD);
                }
                else if (m_macState == MAC_PING_SLOT)
                {
                    NS_LOG_DEBUG("Ping failed! Switching to IDLE");
                    SetMacState(MAC_IDLE);
                }
                else
                {
                    NS_LOG_ERROR("Invalid MAC State at the End of failed ping!");
                }
            }

            if (m_macState == MAC_RX1 || m_macState == MAC_RX2 || m_macState == MAC_RX_BEACON_GUARD)
            {
                if (m_secondReceiveWindow.IsExpired() && m_retxParams.waitingAck)
                {
                    if (m_retxParams.retxLeft > 0)
                    {
                        this->Send(m_retxParams.packet);
                        NS_LOG_INFO("We have " << unsigned(m_retxParams.retxLeft) << " retransmissions left: rescheduling transmission.");
                    }
                    else
                    {
                        uint8_t txs = m_maxNumbTx - (m_retxParams.retxLeft);
                        m_requiredTxCallback(txs, false, m_retxParams.firstAttempt, m_retxParams.packet);
                        NS_LOG_DEBUG("Failure: no more retransmissions left. Used " << unsigned(txs) << " transmissions.");

                        // Reset retransmission parameters
                        resetRetransmissionParameters();
                    }
                }
            }
        }

        void
        ClassBEndDeviceLorawanMac::TxFinished(Ptr<const Packet> packet)
        {
            NS_LOG_FUNCTION_NOARGS();

            //If it was transmitting during the ping slot, it is cooprative relaying
            if (m_macState == MAC_PING_SLOT)
            {
                NS_LOG_DEBUG("Relaying done!");

                // Switch the PHY to sleep
                m_phy->GetObject<EndDeviceLoraPhy>()->SwitchToSleep();

                NS_LOG_DEBUG("Switching to idle!");

                // Mac state is free now
                SetMacState(MAC_IDLE);

                //Don't schedule receive windows
                return;
            }
            else if (m_macState == MAC_PING_SLOT_BEACON_GUARD)
            {
                NS_LOG_DEBUG("Relaying done!");

                // Switch the PHY to sleep
                m_phy->GetObject<EndDeviceLoraPhy>()->SwitchToSleep();

                NS_LOG_DEBUG("Switching to beacon guard!");

                // Mac state is free now
                SetMacState(MAC_BEACON_GUARD);

                //Don't schedule receive windows
                return;
            }

            // Schedule the opening of the first receive window
            Simulator::Schedule(m_receiveDelay1,
                                &ClassBEndDeviceLorawanMac::OpenFirstReceiveWindow, this);

            // Schedule the opening of the second receive window
            m_secondReceiveWindow = Simulator::Schedule(m_receiveDelay2,
                                                        &ClassBEndDeviceLorawanMac::OpenSecondReceiveWindow,
                                                        this);
            // // Schedule the opening of the first receive window
            // Simulator::Schedule (m_receiveDelay1,
            //                      &ClassBEndDeviceLorawanMac::OpenFirstReceiveWindow, this);
            //
            // // Schedule the opening of the second receive window
            // m_secondReceiveWindow = Simulator::Schedule (m_receiveDelay2,
            //                                              &ClassBEndDeviceLorawanMac::OpenSecondReceiveWindow,
            //                                              this);

            // Switch the PHY to sleep
            m_phy->GetObject<EndDeviceLoraPhy>()->SwitchToSleep();

            // Mac state is free now
            SetMacState(MAC_IDLE);
        }

        void
        ClassBEndDeviceLorawanMac::OpenFirstReceiveWindow(void)
        {
            NS_LOG_FUNCTION_NOARGS();

            // Set Phy in Standby mode
            m_phy->GetObject<EndDeviceLoraPhy>()->SwitchToStandby();

            //Calculate the duration of a single symbol for the first receive window DR
            double tSym = pow(2, GetSfFromDataRate(GetFirstReceiveWindowDataRate())) / GetBandwidthFromDataRate(GetFirstReceiveWindowDataRate());

            // Schedule return to sleep after "at least the time required by the end
            // device's radio transceiver to effectively detect a downlink preamble"
            // (LoraWAN specification)
            m_closeFirstWindow = Simulator::Schedule(Seconds(m_receiveWindowDurationInSymbols * tSym),
                                                     &ClassBEndDeviceLorawanMac::CloseFirstReceiveWindow, this); //m_receiveWindowDuration

            // Mac state is serving first receive window
            SetMacState(MAC_RX1);
        }

        void
        ClassBEndDeviceLorawanMac::CloseFirstReceiveWindow(void)
        {
            NS_LOG_FUNCTION_NOARGS();

            Ptr<EndDeviceLoraPhy> phy = m_phy->GetObject<EndDeviceLoraPhy>();

            // Check the Phy layer's state:
            // - RX -> We are receiving a preamble.
            // - STANDBY -> Nothing was received.
            // - SLEEP -> We have received a packet.
            // We should never be in TX or SLEEP mode at this point
            switch (phy->GetState())
            {
            case EndDeviceLoraPhy::TX:
                NS_ABORT_MSG("PHY was in TX mode when attempting to "
                             << "close a receive window.");
                break;
            case EndDeviceLoraPhy::RX:
                // PHY is receiving: let it finish. The Receive method will switch it back to SLEEP.
                break;
            case EndDeviceLoraPhy::SLEEP:
                // PHY has received, and the MAC's Receive already put the device to sleep
                break;
            case EndDeviceLoraPhy::STANDBY:
                // Turn PHY layer to SLEEP
                phy->SwitchToSleep();
                // Mac state is free now
                SetMacState(MAC_IDLE);
                break;
            }
        }

        void
        ClassBEndDeviceLorawanMac::OpenSecondReceiveWindow(void)
        {
            NS_LOG_FUNCTION_NOARGS();

            // Check for receiver status: if it's locked on a packet, don't open this
            // window at all.
            if (m_phy->GetObject<EndDeviceLoraPhy>()->GetState() == EndDeviceLoraPhy::RX)
            {
                NS_LOG_INFO("Won't open second receive window since we are in RX mode.");

                return;
            }

            // Set Phy in Standby mode
            m_phy->GetObject<EndDeviceLoraPhy>()->SwitchToStandby();

            // Switch to appropriate channel and data rate
            NS_LOG_INFO("Using parameters: " << m_secondReceiveWindowFrequency << "Hz, DR"
                                             << unsigned(m_secondReceiveWindowDataRate));

            m_phy->GetObject<EndDeviceLoraPhy>()->SetFrequency(m_secondReceiveWindowFrequency);
            m_phy->GetObject<EndDeviceLoraPhy>()->SetSpreadingFactor(GetSfFromDataRate(m_secondReceiveWindowDataRate));

            //Calculate the duration of a single symbol for the second receive window DR
            double tSym = pow(2, GetSfFromDataRate(GetSecondReceiveWindowDataRate())) / GetBandwidthFromDataRate(GetSecondReceiveWindowDataRate());

            // Schedule return to sleep after "at least the time required by the end
            // device's radio transceiver to effectively detect a downlink preamble"
            // (LoraWAN specification)
            m_closeSecondWindow = Simulator::Schedule(Seconds(m_receiveWindowDurationInSymbols * tSym),
                                                      &ClassBEndDeviceLorawanMac::CloseSecondReceiveWindow, this);

            // Mac state is serving second receive window
            SetMacState(MAC_RX2);
        }

        void
        ClassBEndDeviceLorawanMac::CloseSecondReceiveWindow(void)
        {
            NS_LOG_FUNCTION_NOARGS();

            Ptr<EndDeviceLoraPhy> phy = m_phy->GetObject<EndDeviceLoraPhy>();

            // NS_ASSERT (phy->m_state != EndDeviceLoraPhy::TX &&
            // phy->m_state != EndDeviceLoraPhy::SLEEP);

            // Check the Phy layer's state:
            // - RX -> We have received a preamble.
            // - STANDBY -> Nothing was detected.
            switch (phy->GetState())
            {
            case EndDeviceLoraPhy::TX:
                break;
            case EndDeviceLoraPhy::SLEEP:
                break;
            case EndDeviceLoraPhy::RX:
                // PHY is receiving: let it finish
                NS_LOG_DEBUG("PHY is receiving: Receive will handle the result.");
                return;
            case EndDeviceLoraPhy::STANDBY:
                // Turn PHY layer to sleep
                phy->SwitchToSleep();
                // Mac state is free now
                SetMacState(MAC_IDLE);
                break;
            }

            if (m_retxParams.waitingAck)
            {
                NS_LOG_DEBUG("No reception initiated by PHY: rescheduling transmission.");
                if (m_retxParams.retxLeft > 0)
                {
                    NS_LOG_INFO("We have " << unsigned(m_retxParams.retxLeft) << " retransmissions left: rescheduling transmission.");
                    this->Send(m_retxParams.packet);
                }

                else if (m_retxParams.retxLeft == 0 && m_phy->GetObject<EndDeviceLoraPhy>()->GetState() != EndDeviceLoraPhy::RX)
                {
                    uint8_t txs = m_maxNumbTx - (m_retxParams.retxLeft);
                    m_requiredTxCallback(txs, false, m_retxParams.firstAttempt, m_retxParams.packet);
                    NS_LOG_DEBUG("Failure: no more retransmissions left. Used " << unsigned(txs) << " transmissions.");

                    // Reset retransmission parameters
                    resetRetransmissionParameters();
                }

                else
                {
                    NS_ABORT_MSG("The number of retransmissions left is negative ! ");
                }
            }
            else
            {
                uint8_t txs = m_maxNumbTx - (m_retxParams.retxLeft);
                m_requiredTxCallback(txs, true, m_retxParams.firstAttempt, m_retxParams.packet);
                NS_LOG_INFO("We have " << unsigned(m_retxParams.retxLeft) << " transmissions left. We were not transmitting confirmed messages.");

                // Reset retransmission parameters
                resetRetransmissionParameters();
            }
        }

        /////////////////////////
        // Getters and Setters //
        /////////////////////////

        Time
        ClassBEndDeviceLorawanMac::GetNextClassTransmissionDelay(Time waitingTime)
        {
            NS_LOG_FUNCTION_NOARGS();

            // This is a new packet from APP; it can not be sent until the end of the
            // second receive window (if the second recieve window has not closed yet)
            if (!m_retxParams.waitingAck)
            {
                if (!m_closeFirstWindow.IsExpired() ||
                    !m_closeSecondWindow.IsExpired() ||
                    !m_secondReceiveWindow.IsExpired())
                {
                    NS_LOG_WARN("Attempting to send when there are receive windows:"
                                << " Transmission postponed.");
                    // Compute the duration of a single symbol for the second receive window DR
                    double tSym = pow(2, GetSfFromDataRate(GetSecondReceiveWindowDataRate())) / GetBandwidthFromDataRate(GetSecondReceiveWindowDataRate());
                    // Compute the closing time of the second receive window
                    Time endSecondRxWindow = Time(m_secondReceiveWindow.GetTs()) + Seconds(m_receiveWindowDurationInSymbols * tSym);

                    NS_LOG_DEBUG("Duration until endSecondRxWindow for new transmission:" << (endSecondRxWindow - Simulator::Now()).GetSeconds());
                    waitingTime = std::max(waitingTime, endSecondRxWindow - Simulator::Now());
                }
            }
            // This is a retransmitted packet, it can not be sent until the end of
            // ACK_TIMEOUT (this timer starts when the second receive window was open)
            else
            {
                double ack_timeout = m_uniformRV->GetValue(1, 3);
                // Compute the duration until ACK_TIMEOUT (It may be a negative number, but it doesn't matter.)
                Time retransmitWaitingTime = Time(m_secondReceiveWindow.GetTs()) - Simulator::Now() + Seconds(ack_timeout);

                NS_LOG_DEBUG("ack_timeout:" << ack_timeout << " retransmitWaitingTime:" << retransmitWaitingTime.GetSeconds());
                waitingTime = std::max(waitingTime, retransmitWaitingTime);
            }

            return waitingTime;
        }

        void ClassBEndDeviceLorawanMac::SetMulticastDeviceAddress(LoraDeviceAddress address)
        {
            NS_LOG_FUNCTION(this << address);

            if (address.Get() == 1)
            {
                NS_LOG_ERROR("multicast address has to be different than 1");
                return;
            }

            NS_ASSERT_MSG(address != m_address, "Multicast and Unicast can not have same Address");

            m_mcAddress = address;
        }

        LoraDeviceAddress
        ClassBEndDeviceLorawanMac::GetMulticastDeviceAddress()
        {
            NS_LOG_FUNCTION(this);

            return m_mcAddress;
        }

        uint8_t
        ClassBEndDeviceLorawanMac::GetFirstReceiveWindowDataRate(void)
        {
            return m_replyDataRateMatrix.at(m_dataRate).at(m_rx1DrOffset);
        }

        void
        ClassBEndDeviceLorawanMac::SetSecondReceiveWindowDataRate(uint8_t dataRate)
        {
            m_secondReceiveWindowDataRate = dataRate;
        }

        uint8_t
        ClassBEndDeviceLorawanMac::GetSecondReceiveWindowDataRate(void)
        {
            return m_secondReceiveWindowDataRate;
        }

        void
        ClassBEndDeviceLorawanMac::SetSecondReceiveWindowFrequency(double frequencyMHz)
        {
            m_secondReceiveWindowFrequency = frequencyMHz;
        }

        double
        ClassBEndDeviceLorawanMac::GetSecondReceiveWindowFrequency(void)
        {
            return m_secondReceiveWindowFrequency;
        }

        /////////////////////////
        // MAC command methods //
        /////////////////////////

        void
        ClassBEndDeviceLorawanMac::OnRxClassParamSetupReq(Ptr<RxParamSetupReq> rxParamSetupReq)
        {
            NS_LOG_FUNCTION(this << rxParamSetupReq);

            bool offsetOk = true;
            bool dataRateOk = true;

            uint8_t rx1DrOffset = rxParamSetupReq->GetRx1DrOffset();
            uint8_t rx2DataRate = rxParamSetupReq->GetRx2DataRate();
            double frequency = rxParamSetupReq->GetFrequency();

            NS_LOG_FUNCTION(this << unsigned(rx1DrOffset) << unsigned(rx2DataRate) << frequency);

            // Check that the desired offset is valid
            if (!(0 <= rx1DrOffset && rx1DrOffset <= 5))
            {
                offsetOk = false;
            }

            // Check that the desired data rate is valid
            if (GetSfFromDataRate(rx2DataRate) == 0 || GetBandwidthFromDataRate(rx2DataRate) == 0)
            {
                dataRateOk = false;
            }

            // For now, don't check for validity of frequency
            m_secondReceiveWindowDataRate = rx2DataRate;
            m_rx1DrOffset = rx1DrOffset;
            m_secondReceiveWindowFrequency = frequency;

            // Craft a RxParamSetupAns as response
            NS_LOG_INFO("Adding RxParamSetupAns reply");
            m_macCommandList.push_back(CreateObject<RxParamSetupAns>(offsetOk,
                                                                     dataRateOk, true));
        }

        //////////////////////////////////////////////////////
        // Conflict resolution Between Class A and Class B //
        ////////////////////////////////////////////////////

        Time
        ClassBEndDeviceLorawanMac::ResolveWithClassBAndGetTime(Ptr<const Packet> packet)
        {
            NS_LOG_FUNCTION_NOARGS();
            // Here we can put different algorithms with if-else so that another API
            // could set what algorithm to use for certain end device from the MAC
            // Helper function

            //Experiment with your algorithms here

            // Algorithm 1
            {
                if (m_macState == MAC_BEACON_GUARD)
                {
                    NS_LOG_LOGIC("Collision with beacon guard! implement Algorithm 1");
                    return Simulator::GetDelayLeft(m_beaconInfo.endBeaconGuardEvent) +
                           Simulator::GetDelayLeft(m_beaconInfo.endBeaconReservedEvent) +
                           Seconds(m_uniformRV->GetValue(0, (m_pingSlotInfo.pingOffset) * m_pingSlotInfo.slotLen.GetSeconds()));
                }
                else if (m_macState == MAC_BEACON_RESERVED)
                {
                    NS_LOG_LOGIC("Collision with beacon reserved! implement Algorithm 1");
                    return Simulator::GetDelayLeft(m_beaconInfo.endBeaconReservedEvent) +
                           Seconds(m_uniformRV->GetValue(0, (m_pingSlotInfo.pingOffset) * m_pingSlotInfo.slotLen.GetSeconds()));
                }
                else if (m_macState == MAC_PING_SLOT)
                {
                    NS_LOG_LOGIC("Collision with ping slot! implement Algorithm 1");
                    return Seconds(m_uniformRV->GetValue(0, (m_pingSlotInfo.pingOffset) * m_pingSlotInfo.slotLen.GetSeconds()));
                }
                else if (m_deviceClass == CLASS_A)
                {
                    NS_LOG_LOGIC("No Collision with Class B! Device operating in Class A");
                    // If we are in idle or the class is even not Class B
                    return Seconds(0);
                }
                else if (m_deviceClass == CLASS_B)
                {
                    //Make sure the transmission and the rx windows do not pass beyond the
                    //beacon guard. Now for easy computation take the worst case
                    //Variation of this with perfect airtime calculation could be tried.

                    //Calculating the time when the beacon will start
                    Time nextGuard = Simulator::GetDelayLeft(m_beaconInfo.nextBeaconGuardEvent);

                    //Calculating the total time required for TX+RX1+RX2 time
                    //The Longest Tx time on SF 12 is 2,465.79ms (2.457seconds)
                    Time longestTx = Seconds(2.5);
                    //Either of the rx1 or the rx2 window will be used each opened for 5 symbols
                    //worst case rx1
                    Time rx1 = MilliSeconds(163.84);
                    //default rx2
                    Time rx2 = MilliSeconds(163.84);

                    if (nextGuard < longestTx + m_receiveDelay1 + rx1 + m_receiveDelay2 + rx2)
                    {
                        //If the tx+rx1+rx2 don't fit postpone at the end of the transmission
                        return nextGuard + Seconds(5.12) + Seconds(m_uniformRV->GetValue(0, (m_pingSlotInfo.pingOffset) * m_pingSlotInfo.slotLen.GetSeconds()));
                    }
                    else
                    {
                        return Seconds(0);
                    }
                }
                else
                {
                    NS_LOG_LOGIC("No immediate collision with Class B!");
                }
            }

            //Algorithm 2 (Smart device solution): Have an if else statement control the API
            {
                // When collision happens with beacon guard or reserved we can end until the
                // end and randomly schedule

                // When the device is already in receive mode

                // When the tx and its rx windows fit with in a ping by calculating air time

                // When only the tx fits within the ping

                // When nothing fits

            }

            // Algorithm 3
            // With zero effect on Class A. Class A always gets priority
            {

            }

            //Algorithm 4
            // With zero effect on Class B. Class B always gets priority
            {
            }

            return Seconds(0);
        }

        ///////////////////////////////////////////////////
        //  LoRaWAN Class B related  procedures         //
        /////////////////////////////////////////////////

        void
        ClassBEndDeviceLorawanMac::SwitchToClassB(void)
        {
            NS_LOG_FUNCTION_NOARGS();

            if (m_deviceClass == CLASS_C)
            {
                NS_LOG_ERROR("Can't switch to Class B from Class C! You need to switch to class A first");
                return;
            }

            if (m_deviceClass == CLASS_B)
            {
                NS_LOG_INFO("Device already operating in Class B!");
                return;
            }

            if (m_beaconState != BEACON_UNLOCKED)
            {
                NS_LOG_INFO("Invalid request to SwitchToclassB! Check previous request made to SwitchToClassB");
                return;
            }

            //\todo Class B Mac exchanges before beacon searching
            //For now assume the DeviceTimeAns/BeaconTimingAns is already received from

            // k is the smallest integer for which k * 128 > T, where
            // T = seconds since 00:00:00, Sunday 5th of January 1980 (start of the GPS time)
            // which is the Simulator::Now in this ns3 simulation */
            NS_LOG_DEBUG("SwitchToClassb requested at" << Simulator::Now());

            int k = 0;

            while (Simulator::Now() >= Seconds(k * 128))
            {
                k++;
            }
            NS_LOG_DEBUG("k(" << k << ")"
                              << "*128>T(" << Simulator::Now() << ")");

            // The time when beacon is sent
            Time bT = Seconds(k * 128) + m_beaconInfo.tBeaconDelay;

            // Calculate the beacon guard start time from the beacon time
            Time nextAbsoluteBeaconGuardTime = bT - m_beaconInfo.beaconGuard;

            // Calculate the relative time to schedule the beacon guard
            Time nextBeaconGuard = nextAbsoluteBeaconGuardTime - Simulator::Now();

            // Schedule the beaconGuard
            Simulator::Schedule(nextBeaconGuard,
                                &ClassBEndDeviceLorawanMac::StartBeaconGuard,
                                this);

            // Update beaconState
            m_beaconState = BEACON_SEARCH;

            NS_LOG_DEBUG("BeaconGuard scheduled at " << nextAbsoluteBeaconGuardTime);

            //Reset beacon and class b window to default
            m_classBReceiveWindowInfo.beaconReceiveWindowDurationInSymbols = ClassBEndDeviceLorawanMac::ClassBReceiveWindowInfo().beaconReceiveWindowDurationInSymbols;
            m_classBReceiveWindowInfo.pingReceiveWindowDurationInSymbols = ClassBEndDeviceLorawanMac::ClassBReceiveWindowInfo().pingReceiveWindowDurationInSymbols;

            NS_LOG_DEBUG("Beacon Receive Window Reset to " << (int)(m_classBReceiveWindowInfo.beaconReceiveWindowDurationInSymbols));
            NS_LOG_DEBUG("Ping Receive Window Reset to " << (int)(m_classBReceiveWindowInfo.pingReceiveWindowDurationInSymbols));

            m_attemptToClassB++;
        }

        void
        ClassBEndDeviceLorawanMac::SwitchFromClassB(void)
        {
            NS_LOG_FUNCTION_NOARGS();

            if (m_deviceClass != CLASS_B)
            {
                //Nothing to do if device is not in class B
                NS_LOG_DEBUG("The device is not in Class A");
                return;
            }

            m_beaconState = BEACON_UNLOCKED;

            m_deviceClass = CLASS_A;

            // Cancel all pending ping slots if any
            for (EventId &ping : m_pingSlotInfo.pendingPingSlotEvents)
            {
                Simulator::Cancel(ping);
            }

            // Cancel upcoming beacon guard if any
            Simulator::Cancel(m_beaconInfo.nextBeaconGuardEvent);

            NS_LOG_DEBUG("Beacon unlocked, ping slot canceled, beaconGuard canceled and device switched to class A");
            //\todo Schedule an uplink with class B field set
        }

        void
        ClassBEndDeviceLorawanMac::StartBeaconGuard(void)
        {

            NS_LOG_FUNCTION_NOARGS();

            if (m_deviceClass == CLASS_A)
            {
                if (m_macState == MAC_IDLE)
                {
                    SetMacState(MAC_BEACON_GUARD);
                }
                else
                {
                    NS_LOG_INFO("Can not start the beacon guard; device mac is not in IDLE state");
                    return;
                }
            }
            //where is beacon guard from beacon less
            // If m_deviceClass == CLASS _B, device could be receiving in the beacon guard
            // So check the state and put intermediate stqte accordingly
            if (m_deviceClass == CLASS_B)
            {
                switch (m_macState)
                {
                case MAC_IDLE:
                    SetMacState(MAC_BEACON_GUARD);
                    break;
                case MAC_RX1:
                    SetMacState(MAC_RX_BEACON_GUARD);
                    NS_LOG_DEBUG("Finsihing RX1 reception during the guard period!");
                    break;
                case MAC_RX2:
                    SetMacState(MAC_RX_BEACON_GUARD);
                    NS_LOG_DEBUG("Finsihing RX2 reception during the guard period!");
                    break;
                case MAC_PING_SLOT:
                    SetMacState(MAC_PING_SLOT_BEACON_GUARD);
                    NS_LOG_DEBUG("Finsihing ping-slot reception during the guard period!");
                    break;
                default:
                    NS_LOG_INFO("Can not start the beacon guard; mac is in an improper state");
                    return;
                }
            }

            m_beaconInfo.endBeaconGuardEvent = Simulator::Schedule(m_beaconInfo.beaconGuard,
                                                                   &ClassBEndDeviceLorawanMac::EndBeaconGuard,
                                                                   this);
        }

        void
        ClassBEndDeviceLorawanMac::EndBeaconGuard(void)
        {
            NS_LOG_FUNCTION_NOARGS();
            //Now Start the Beacon_reserved period as the beacon guard is done
            Simulator::Schedule(Seconds(0),
                                &ClassBEndDeviceLorawanMac::StartBeaconReserved,
                                this);
        }

        void
        ClassBEndDeviceLorawanMac::StartBeaconReserved(void)
        {

            NS_LOG_FUNCTION_NOARGS();

            NS_ASSERT_MSG(m_macState == MAC_BEACON_GUARD, "Macstate = " << m_macState << "! Beacon guard should be right before the beacon reserved");

            //Open the beacon slot receive window to detect preamble for the beacon

            SetMacState(MAC_BEACON_RESERVED);

            NS_LOG_DEBUG("Beacon receive window opened at " << Simulator::Now().GetSeconds() << "Seconds");

            // Set Phy in Standby mode
            m_phy->GetObject<EndDeviceLoraPhy>()->SwitchToStandby();

            // Switch to appropriate channel and data rate
            NS_LOG_INFO("Beacon parameters: " << m_classBReceiveWindowInfo.beaconReceiveWindowFrequency << "Hz, DR"
                                              << unsigned(m_classBReceiveWindowInfo.beaconReceiveWindowDataRate));

            m_phy->GetObject<EndDeviceLoraPhy>()->SetFrequency(m_classBReceiveWindowInfo.beaconReceiveWindowFrequency);
            m_phy->GetObject<EndDeviceLoraPhy>()->SetSpreadingFactor(GetSfFromDataRate(m_classBReceiveWindowInfo.beaconReceiveWindowDataRate));

            //Calculate the duration of a single symbol for the beacon slot receive window
            double tSym = pow(2, GetSfFromDataRate(m_classBReceiveWindowInfo.beaconReceiveWindowDataRate)) /
                          GetBandwidthFromDataRate(m_classBReceiveWindowInfo.beaconReceiveWindowDataRate);

            // Schedule return to sleep after current beacon slot receive window duration
            Simulator::Schedule(Seconds(m_classBReceiveWindowInfo.beaconReceiveWindowDurationInSymbols * tSym),
                                &ClassBEndDeviceLorawanMac::CloseBeaconReceiveWindow,
                                this);

            NS_LOG_DEBUG("The receive window opened for : " << Seconds(m_classBReceiveWindowInfo.beaconReceiveWindowDurationInSymbols * tSym));

            //Schedule release from beacon reserved so that the mac could start using the
            //device for transmission and also schedule ping slots
            m_beaconInfo.endBeaconReservedEvent = Simulator::Schedule(m_beaconInfo.beaconReserved,
                                                                      &ClassBEndDeviceLorawanMac::EndBeaconReserved,
                                                                      this);

            NS_LOG_DEBUG("The beacon reserved finishes at: " << (Simulator::Now() + m_beaconInfo.beaconReserved));
        }

        void
        ClassBEndDeviceLorawanMac::CloseBeaconReceiveWindow(void)
        {
            NS_LOG_FUNCTION_NOARGS();
            //Beacon preamble not detected

            NS_ASSERT_MSG(m_macState == MAC_BEACON_RESERVED, "Beacon receive window should reside in beacon reserved");

            Ptr<EndDeviceLoraPhy> phy = m_phy->GetObject<EndDeviceLoraPhy>();

            // Check the Phy layer's state:
            // - TX -> Its an error, transmission can't happen in beacon reserved
            // - RX -> We have received a preamble.
            // - STANDBY -> Nothing was detected.
            switch (phy->GetState())
            {
            case EndDeviceLoraPhy::TX:
                NS_LOG_ERROR("TX can't happen in beacon reserved");
                break;
            case EndDeviceLoraPhy::SLEEP:
                break;
            case EndDeviceLoraPhy::RX:
                // PHY is receiving: let it finish
                NS_LOG_DEBUG("PHY is receiving: Receive will handle the result.");
                return;
            case EndDeviceLoraPhy::STANDBY:
                // Turn PHY layer to sleep
                phy->SwitchToSleep();
                // Report beacon is missed to execute procedure that matches the device state
                BeaconMissed();
                break;
            }
        }

        void
        ClassBEndDeviceLorawanMac::EndBeaconReserved(void)
        {
            NS_LOG_FUNCTION_NOARGS();

            NS_ASSERT_MSG(m_macState == MAC_BEACON_RESERVED, "Error happened in the beacon reserved time!");

            // Turn back mac to idle
            SetMacState(MAC_IDLE);

            // If beacon is unlocked there is nothing to do
            if (m_beaconState == BEACON_UNLOCKED)
            {
                return;
            }

            Ptr<EndDeviceLoraPhy> phy = m_phy->GetObject<EndDeviceLoraPhy>();

            // If beacon is locked or beacon less
            if (m_beaconState == BEACON_LOCKED || m_beaconState == BEACONLESS)
            {
                // If device locked beacon or is operating in beacon less operation and if
                // radio is still in receive mode error as beacon payload can't exceed beacon
                // reserved
                if (phy->GetState() == EndDeviceLoraPhy::RX)
                {
                    NS_LOG_ERROR("Beacon payload can't exceed the beacon reserved time!");
                    return;
                }

                // If beacon is locked we reset the beacon information
                if (m_beaconState == BEACON_LOCKED)
                {
                    //Beacon information, beacon receive window, ping slot receive window
                    //will be reseted immediately after the receiving the packet

                    //Check and update for the maximum number of beacons missed in the
                    //minimal beaconless operation mode if the device was in the mode
                    if (m_currentConsecutiveBeaconsMissed > m_maximumConsecutiveBeaconsMissed)
                        m_maximumConsecutiveBeaconsMissed = m_currentConsecutiveBeaconsMissed;

                    //device is not in minimal beaconless operation mode anymore if it was
                    //therefore, reset
                    m_currentConsecutiveBeaconsMissed = 0;

                    //Fire traced-callback for informing the run length has ended
                    m_currentConsecutiveBeaconsMissedTracedCallback(GetMulticastDeviceAddress(),
                                                                    GetDeviceAddress(),
                                                                    m_currentConsecutiveBeaconsMissed);
                }

                switch (m_deviceClass)
                {
                case CLASS_A:
                    // If device is still in class A switch to Class B and continue to /
                    // scheduling the beacon and the ping slots
                    // \todo Think if we have to leave this procedure to the Application layer
                    //       for the m_beaconLockedCallback to do this.
                    m_deviceClass = CLASS_B;
                case CLASS_B:
                    // If device is already in Class B then schedule the coming ping slots
                    SchedulePingSlots();

                    // The time gap between the end of Beacon_reserved and the start of
                    // the next Beacon_guard is after Beacon_window.
                    // Need to store the event to cancel incase a switch to class A is requested in the middle
                    m_beaconInfo.nextBeaconGuardEvent = Simulator::Schedule(m_beaconInfo.beaconWindow,
                                                                            &ClassBEndDeviceLorawanMac::StartBeaconGuard,
                                                                            this);

                    break;
                case CLASS_C:
                    //\todo Class C not implimented yet
                    NS_LOG_ERROR("A switch to Class B is possible only from Class A!");
                    break;
                }
            }
        }

        // This function must be called at the end of BeaconResered which is
        // ClassBEndDeviceLorawanMac::EndBeaconReserved as it assumes the current time to be the
        // end of the BeaconReserved time.
        void
        ClassBEndDeviceLorawanMac::SchedulePingSlots(void)
        {
            NS_LOG_FUNCTION_NOARGS();
            // AES part taken with modification from Joseph Finnegan
            // <https://github.com/ConstantJoe/ns3-lorawan-class-B/blob/master/lorawan/model/lorawan-enddevice-application.cc>

            //Key = 16 times 0x00 (4x4 block)
            uint8_t key[16] = {0}; //  AES encryption with a fixed key of all zeros is used to randomize

            //Rand is also a 4x4 block (16 byte)
            uint8_t rand[16] = {0};

            //Serialize the beaconPayload
            uint32_t bcnPayload = (uint32_t)(m_beaconInfo.deviceBcnTime.GetSeconds());
            uint8_t *beaconTime = (uint8_t *)&bcnPayload;
            rand[0] = beaconTime[0];
            rand[1] = beaconTime[1];
            rand[2] = beaconTime[2];
            rand[3] = beaconTime[3];

            // Serialize the devAddr
            // Serialize method doesn't have any side effect so could be directly used
            uint8_t devAddr[4] = {0};

            //Give priority to MC
            if (IsMulticastEnabled())
            {
                m_mcAddress.Serialize(devAddr);
            }
            else
            {
                m_address.Serialize(devAddr);
            }

            rand[4] = devAddr[0];
            rand[5] = devAddr[1];
            rand[6] = devAddr[2];
            rand[7] = devAddr[3];

            //Rand = aes128_encrypt (key, beaconTime(4byte)|DevAddr(4byte)|Pad16)
            AES aes;
            aes.SetKey(key, 16);
            aes.Encrypt(rand, 16);

            // pingOffset = (Rand[0] + Rand[1]*256) modulo pingPeriod
            m_pingSlotInfo.pingOffset = (rand[0] + rand[1] * 256) % m_pingSlotInfo.pingPeriod;
            // m_pingSlotInfo.pingOffset = m_pingSlotInfo.pingPeriod-1; //Use this one to test for the max pingOffset

            // For all the slotIndex = [0 ... PingNb-1] and schedule them on
            // (BeaconReserved + (pingOffset+ slotIndex*pingPeriod)*slotLen)
            uint8_t slotIndex = 0;

            NS_LOG_DEBUG("Number of pings scheduled per beacon period " << m_pingSlotInfo.pendingPingSlotEvents.size());

            for (EventId &ping : m_pingSlotInfo.pendingPingSlotEvents)
            {
                //When switch back to class A is requested cancel all the non expired events

                Time slotTime = (m_pingSlotInfo.pingOffset + slotIndex * m_pingSlotInfo.pingPeriod) * m_pingSlotInfo.slotLen;
                ping = Simulator::Schedule(slotTime,
                                           &ClassBEndDeviceLorawanMac::OpenPingSlotReceiveWindow,
                                           this,
                                           slotIndex);
                NS_ASSERT_MSG(slotTime < m_beaconInfo.beaconWindow, "A slot should only be placed within a beaconWindow duration!");
                NS_LOG_DEBUG("Ping scheduled for index " << (int)slotIndex << " after " << slotTime.GetSeconds() << " seconds");
                slotIndex++;
            }
        }

        void
        ClassBEndDeviceLorawanMac::BeaconMissed(void)
        {
            NS_LOG_FUNCTION_NOARGS();

            // Do nothing if beacon has not been locked yet
            if (m_beaconState == BEACON_SEARCH)
            {
                m_beaconState = BEACON_UNLOCKED;

                NS_LOG_INFO("No beacon found!");

                // Notify the Application layer that beacon is not locked
                if (!m_beaconLostCallback.IsNull())
                {
                    m_beaconLostCallback();
                }

                // increment missed beacons
                m_missedBeaconCount++;
                // fire trace-callback
                m_missedBeaconTracedCallback(GetMulticastDeviceAddress(), GetDeviceAddress(), m_missedBeaconCount);

                return;
            }

            // increment current beacon missed
            m_currentConsecutiveBeaconsMissed++;
            // Fire trace-callback on currentConsecutiveBeaconsMissed
            m_currentConsecutiveBeaconsMissedTracedCallback(GetMulticastDeviceAddress(),
                                                            GetDeviceAddress(),
                                                            m_currentConsecutiveBeaconsMissed);

            // increment missed beacons
            m_missedBeaconCount++;
            // fire trace-callback
            m_missedBeaconTracedCallback(GetMulticastDeviceAddress(), GetDeviceAddress(), m_missedBeaconCount);

            // If we exceed minimal beacon less operation time switch to class A and reset context
            // and invoke beacon lost call back
            if (((Simulator::Now() - m_beaconInfo.gwBcnTime) > m_beaconInfo.minimalBeaconLessOperationTime) && (m_beaconState == BEACONLESS))
            {
                m_deviceClass = CLASS_A;

                m_beaconState = BEACON_UNLOCKED;
                // Notify the Application layer that beacon is not locked
                if (!m_beaconLostCallback.IsNull())
                {
                    m_beaconLostCallback();
                }

                NS_LOG_INFO("Beacon lost! switching back to class A.");

                // Optional: update the bcnTimes to default
                m_beaconInfo.gwBcnTime = Seconds(0);
                m_beaconInfo.deviceBcnTime = Seconds(0);

                // Reset the consecutiveBeaconMissed state and callback

                // reset current beacon missed
                m_currentConsecutiveBeaconsMissed = 0;

                // Fire trace-callback on currentConsecutiveBeaconsMissed
                m_currentConsecutiveBeaconsMissedTracedCallback(GetMulticastDeviceAddress(),
                                                                GetDeviceAddress(),
                                                                m_currentConsecutiveBeaconsMissed);
                return;
            }

            // Expand the beacon window if we missed a beacon before minimal beacon less operation time
            if (m_beaconState == BEACON_LOCKED || m_beaconState == BEACONLESS)
            {
                m_beaconState = BEACONLESS;

                NS_LOG_INFO("minimal beacon less operation mode");

                m_classBReceiveWindowInfo.beaconReceiveWindowDurationInSymbols *= m_classBReceiveWindowInfo.symbolToExpantionFactor;

                if (m_classBReceiveWindowInfo.beaconReceiveWindowDurationInSymbols > m_classBReceiveWindowInfo.maxBeaconReceiveWindowDurationInSymbols)
                {
                    m_classBReceiveWindowInfo.beaconReceiveWindowDurationInSymbols = m_classBReceiveWindowInfo.maxBeaconReceiveWindowDurationInSymbols;
                }

                m_classBReceiveWindowInfo.pingReceiveWindowDurationInSymbols *= m_classBReceiveWindowInfo.symbolToExpantionFactor;

                if (m_classBReceiveWindowInfo.pingReceiveWindowDurationInSymbols > m_classBReceiveWindowInfo.maxPingReceiveWindowDurationInSymbols)
                {
                    m_classBReceiveWindowInfo.pingReceiveWindowDurationInSymbols = m_classBReceiveWindowInfo.maxPingReceiveWindowDurationInSymbols;
                }

                NS_LOG_DEBUG("Beacon expanded to " << (int)m_classBReceiveWindowInfo.beaconReceiveWindowDurationInSymbols << " and ping slot expanded to " << (int)m_classBReceiveWindowInfo.pingReceiveWindowDurationInSymbols);

                // Advance the time stamp of the beacon payload stored by 128 second as the NS will not know whether the end device has lost the beacon or not.
                // This will enable a more precise calculation of pingOffset when beacon is not received.
                m_beaconInfo.deviceBcnTime += m_beaconInfo.beaconPeriod;
            }

            //\TODO Also include the slot movement together with the slot expansion
        }

        //\TODO the bcnPayload has to be replaced from the packet received, otherwise
        // there will be a difference in the time calculated by the gateway and end-device
        void
        ClassBEndDeviceLorawanMac::BeaconReceived(Ptr<Packet const> packet)
        {
            NS_LOG_FUNCTION_NOARGS();

            // If beacon is unlocked, then you should first do beacon search
            // So do nothing.
            // SwitchFromClassB can simply change m_beaconState to UNLOCKED to
            // stop receiving beacon.
            if (m_beaconState == BEACON_UNLOCKED)
            {
                return;
            }

            //Extract the beaconTime from the bcnPayload
            // Work on a copy of the packet
            Ptr<Packet> packetCopy = packet->Copy();

            BcnPayload bcnPayload;
            packetCopy->RemoveHeader(bcnPayload);

            //\TODO check CRC in the bcnPayload using bcnPayload.CrcCheck, which also need to be written

            // remove the time field of the bcnPayload and update the time
            m_beaconInfo.gwBcnTime = Seconds(bcnPayload.GetBcnTime());
            // update the device beacon time
            m_beaconInfo.deviceBcnTime = m_beaconInfo.gwBcnTime;

            NS_LOG_DEBUG("Last Beacon Received Time Updated to " << m_beaconInfo.deviceBcnTime);

            //Reset beacon and class b window to default
            m_classBReceiveWindowInfo.beaconReceiveWindowDurationInSymbols = ClassBEndDeviceLorawanMac::ClassBReceiveWindowInfo().beaconReceiveWindowDurationInSymbols;
            m_classBReceiveWindowInfo.pingReceiveWindowDurationInSymbols = ClassBEndDeviceLorawanMac::ClassBReceiveWindowInfo().pingReceiveWindowDurationInSymbols;

            NS_LOG_DEBUG("Beacon Receive Window Reset to " << (int)m_classBReceiveWindowInfo.beaconReceiveWindowDurationInSymbols);
            NS_LOG_DEBUG("Ping Receive Window Reset to " << (int)m_classBReceiveWindowInfo.pingReceiveWindowDurationInSymbols);

            // If beacon is locked for the first time
            if (m_beaconState == BEACON_SEARCH)
            {
                // At the end of the beacon reserved the end device will be switched to
                // Class B

                //Invoke the callback to notify the Application layer about the locked beacon
                if (!m_beaconLockedCallback.IsNull())
                {
                    m_beaconLockedCallback();
                }
                NS_LOG_INFO("beacon locked!");
            }

            m_beaconState = BEACON_LOCKED;
            m_totalSuccessfulBeaconPackets++;

            //Fire traced callback
            m_totalSuccessfulBeaconPacketsTracedCallback(GetMulticastDeviceAddress(),
                                                         GetDeviceAddress(),
                                                         m_totalSuccessfulBeaconPackets);
        }

        void
        ClassBEndDeviceLorawanMac::OpenPingSlotReceiveWindow(uint8_t slotIndex)
        {
            //Change Mac state to ping slot

            //\TODO: No actual solution in spec for dealing with of tx, rx1, rx2 overlap
            // with ping. Different choices will lead to different network performance.
            // The Tx knows about the ping slot and so can make this decision by referring
            // the ResolveWithClassBAndGetTime () function for this.

            NS_LOG_FUNCTION(this << slotIndex);

            // Check for receiver status: if it's locked on a packet or overlap with other
            // reception window refrain from transmission.
            if (m_phy->GetObject<EndDeviceLoraPhy>()->GetState() == EndDeviceLoraPhy::RX)
            {
                if (m_macState == MAC_RX1 || m_macState == MAC_RX2)
                {
                    NS_LOG_INFO("Collision with Rx1 and Rx2 window!");
                }
                NS_LOG_INFO("Won't open ping window since we are in RX mode.");

                return;
            }

            if (!(m_beaconState == BEACONLESS || m_beaconState == BEACON_LOCKED))
            {
                NS_LOG_INFO("Beacon has to be locked or device should be in minimal beaconless operation mode to open ping slots.");

                return;
            }

            if (!(m_deviceClass == CLASS_B))
            {
                NS_LOG_INFO("Device is not in class B. Can't open ping slot!");

                return;
            }

            if (!(m_macState == MAC_IDLE))
            {
                NS_LOG_INFO("Mac is busy!");

                return;
            }

            if (m_relayActivated && m_packetToRelay.size() > 0)
            {
                // Relay
                NS_LOG_DEBUG("Relaying Packet!");

                Ptr<Packet> packetToRelay = m_packetToRelay.front()->Copy();
                m_packetToRelay.pop_front();

                // Craft LoraTxParameters object
                LoraTxParameters params;
                params.sf = GetSfFromDataRate(m_classBReceiveWindowInfo.pingSlotReceiveWindowDataRate);
                params.headerDisabled = m_headerDisabled;
                params.codingRate = m_codingRate;
                params.bandwidthHz = GetBandwidthFromDataRate(m_classBReceiveWindowInfo.pingSlotReceiveWindowDataRate);
                params.nPreamble = m_nPreambleSymbols;
                params.crcEnabled = 1;
                params.lowDataRateOptimizationEnabled = 0;

                // Wake up PHY layer and directly send the packet

                double relayPower = GetRelayingPower();

                //Only transmit if relay power is greater than 0
                if (relayPower >= 0)
                {
                    NS_LOG_DEBUG("Packet to relay: " << packetToRelay << "& UID :" << packetToRelay->GetUid());
                    m_phy->Send(packetToRelay, params, m_classBReceiveWindowInfo.pingSlotReceiveWindowFrequency, GetRelayingPower());

                    //\TODO For future you need to register packet transmission for duty cycle
                    // For now only one gateway is used. Therefore, if the gateway respects the duty cycle, the end-device will also respect.=
                    // There will be a problem for multiple gateway case

                    // Mac is now busy transmitting in the ping slot,
                    // The Tx finished will put it back to IDLE
                    // Therefore only the purpose of the ping is changed the rest stays the same
                    SetMacState(MAC_PING_SLOT);

                    return;
                }
                else
                {
                    NS_LOG_DEBUG("Packet not relayed!");
                }
            }

            SetMacState(MAC_PING_SLOT);

            // Set Phy in Standby mode
            m_phy->GetObject<EndDeviceLoraPhy>()->SwitchToStandby();

            // Switch to appropriate channel and data rate
            NS_LOG_INFO("Using parameters: " << m_classBReceiveWindowInfo.pingSlotReceiveWindowFrequency << "Hz, DR"
                                             << unsigned(m_classBReceiveWindowInfo.pingSlotReceiveWindowDataRate));

            m_phy->GetObject<EndDeviceLoraPhy>()->SetFrequency(m_classBReceiveWindowInfo.pingSlotReceiveWindowFrequency);
            m_phy->GetObject<EndDeviceLoraPhy>()->SetSpreadingFactor(GetSfFromDataRate(m_classBReceiveWindowInfo.pingSlotReceiveWindowDataRate));

            //Calculate the duration of a single symbol for the ping slot DR
            double tSym = pow(2, GetSfFromDataRate(m_classBReceiveWindowInfo.pingSlotReceiveWindowDataRate)) / GetBandwidthFromDataRate(m_classBReceiveWindowInfo.pingSlotReceiveWindowDataRate);

            // Schedule return to sleep after current beacon slot receive window duration
            m_pingSlotInfo.closeOpenedPingSlot = Simulator::Schedule(Seconds(m_classBReceiveWindowInfo.pingReceiveWindowDurationInSymbols * tSym),
                                                                     &ClassBEndDeviceLorawanMac::ClosePingSlotRecieveWindow, this);

            //update slot index of the opened ping slot
            m_slotIndexLastOpened = slotIndex;
        }

        void
        ClassBEndDeviceLorawanMac::ClosePingSlotRecieveWindow()
        {
            NS_LOG_FUNCTION_NOARGS();

            NS_ASSERT_MSG((m_macState == MAC_PING_SLOT || m_macState == MAC_PING_SLOT_BEACON_GUARD),
                          "Mac should has stayed in MAC_PING_SLOT!");

            Ptr<EndDeviceLoraPhy> phy = m_phy->GetObject<EndDeviceLoraPhy>();

            // Check the Phy layer's state:
            // - TX -> Its an error, transmission during ping slot
            // - RX -> We have received a preamble.
            // - STANDBY -> Nothing was detected.
            switch (phy->GetState())
            {
            case EndDeviceLoraPhy::TX:
                NS_LOG_ERROR("TX can't happen while ping is opened! resolve conflict");
                break;
            case EndDeviceLoraPhy::SLEEP:
                NS_LOG_ERROR("Device can't sleep before the duration of the preamble we opened finishes!");
                break;
            case EndDeviceLoraPhy::RX:
                // PHY is receiving: let it finish
                NS_LOG_DEBUG("PHY is receiving: Receive will handle the result.");
                return;
            case EndDeviceLoraPhy::STANDBY:
                // Turn PHY layer to sleep
                phy->SwitchToSleep();
                break;
            }

            // Update Mac state
            if (m_macState == MAC_PING_SLOT_BEACON_GUARD)
            {
                // If the Ping Slot Periodicity is 0 and the DR is bellow and equal to
                // 3, then the last ping window will cross the beacon-guard boarder, as the
                // last ping will start just 30ms before the beacon guard.
                NS_LOG_DEBUG("Ping crossed the beacon guard boundary! Switching back to Beacon Guard");
                SetMacState(MAC_BEACON_GUARD);
            }
            else if (m_macState == MAC_PING_SLOT)
            {
                //Once the packet has been recieved via ping slot free the MAC
                NS_LOG_DEBUG("Ping finished! Switching to IDLE");
                SetMacState(MAC_IDLE);
            }
            else
            {
                NS_ASSERT_MSG(false, "Invalid MAC State at the End of receiving ping!");
            }
        }

        void
        ClassBEndDeviceLorawanMac::PingReceived(Ptr<Packet const> packet)
        {

            NS_LOG_FUNCTION(this << packet);

            NS_ASSERT_MSG((m_macState == MAC_PING_SLOT || m_macState == MAC_PING_SLOT_BEACON_GUARD),
                          "Mac should has stayed in MAC_PING_SLOT!");

            //Working on the copy of the packet
            Ptr<Packet> packetCopy = packet->Copy();

            // Remove the Mac Header to get some information
            LorawanMacHeader mHdr;
            packetCopy->RemoveHeader(mHdr);

            NS_LOG_DEBUG("Mac Header: " << mHdr);

            // Only keep analyzing the packet if it's downlink
            if (mHdr.IsUplink())
            {
                NS_LOG_DEBUG("Uplink data received via the ping slot! Packet Dropped!");
            }
            else
            {
                NS_LOG_INFO("Found a downlink packet.");

                // Remove the Frame Header
                LoraFrameHeader fHdr;
                fHdr.SetAsDownlink();
                packetCopy->RemoveHeader(fHdr);

                NS_LOG_DEBUG("Frame Header: " << fHdr);

                // Determine whether this packet is for us
                // Is it a unicasat message
                bool unicastMessage = (m_address == fHdr.GetAddress());
                // Is it a multicast message
                bool multicastMessage = (m_mcAddress == fHdr.GetAddress());

                if (unicastMessage)
                {
                    NS_LOG_INFO("Unicast Ping Message!");

                    //\TODO Pass the packet up to the NetDevice

                    // For now call the callback
                    if (!m_classBDownlinkCallback.IsNull())
                    {
                        m_classBDownlinkCallback(ClassBEndDeviceLorawanMac::UNICAST, packetCopy, m_slotIndexLastOpened);
                    }

                    // Call the trace source
                    m_receivedPingPacket(0, m_address, packetCopy, m_slotIndexLastOpened);
                }
                else if (multicastMessage)
                {
                    NS_LOG_INFO("Multicast Ping Message!");

                    //\TODO pass the packet up to the NetDevice

                    // For now call the callback
                    if (m_enableMulticast)
                    {
                        HopCountTag hopCountTag;
                        packetCopy->RemovePacketTag(hopCountTag);

                        NS_ASSERT_MSG(hopCountTag.GetHopCount() > 0, "Hop count should be one or greater for a packet received!");

                        NS_LOG_DEBUG("Packet hop count is " << (int)hopCountTag.GetHopCount());

                        if (maxHop > hopCountTag.GetHopCount() && m_relayActivated)
                        {
                            NS_ASSERT_MSG(m_packetToRelay.size() == 0, "Device should relay " << m_packetToRelay.size() << " packet before receiving!");

                            NS_LOG_DEBUG("Preparing to relay Packet!");
                            hopCountTag.IncreamentHopCount();
                            NS_LOG_DEBUG("Hop incremented to " << (int)hopCountTag.GetHopCount());
                            Ptr<Packet> packetToRelay = packetCopy->Copy();
                            packetToRelay->AddPacketTag(hopCountTag);
                            packetToRelay->AddHeader(fHdr);
                            packetToRelay->AddHeader(mHdr);
                            m_packetToRelay.push_back(packetToRelay);
                        }

                        if (!m_classBDownlinkCallback.IsNull())
                        {
                            m_classBDownlinkCallback(ClassBEndDeviceLorawanMac::MULTICAST, packetCopy, m_slotIndexLastOpened);
                        }
                        // Call the trace source
                        m_receivedPingPacket(m_mcAddress, m_address, packetCopy, m_slotIndexLastOpened);
                    }
                    else
                    {
                        NS_LOG_INFO("MC packet received but device not MC enabled!");
                    }
                }
                else
                {
                    // Device overheared a packet not meant for it
                    m_overheardPacketCount++;

                    // Fire trace source
                    m_numberOfOverhearedPackets(m_mcAddress, m_address, m_overheardPacketCount);
                }
            }

            // Update Mac state
            if (m_macState == MAC_PING_SLOT_BEACON_GUARD)
            {
                //If beacon guard started before end of the packet then switch back
                //to the beacon reserved
                NS_LOG_DEBUG("Ping Received! Switching back to beacon guard");
                SetMacState(MAC_BEACON_GUARD);
            }
            else if (m_macState == MAC_PING_SLOT)
            {
                //Once the packet has been recieved via ping slot free the MAC
                NS_LOG_DEBUG("Ping Received! Switching to IDLE");
                SetMacState(MAC_IDLE);
            }
            else
            {
                NS_ASSERT_MSG(false, "Invalid MAC State at the End of receiving ping!");
            }
        }

        ///////////////////////////////////////////////////
        //  LoRaWAN Class B Related Getters and Setters //
        /////////////////////////////////////////////////

        ///////////////////////////////////
        // Device Class and Mac State   //
        /////////////////////////////////

        bool
        ClassBEndDeviceLorawanMac::SetDeviceClass(DeviceClass deviceClass)
        {
            NS_LOG_FUNCTION(this << deviceClass);

            if (deviceClass == CLASS_A)
            {
                if (m_deviceClass == CLASS_A)
                {
                    NS_LOG_DEBUG("Device already in class A!");
                    return true;
                }
                else if (m_deviceClass == CLASS_B)
                {
                    NS_LOG_DEBUG("Switch device class from B to A");
                    SwitchFromClassB();
                    return true;
                }
                else
                {
                    NS_LOG_DEBUG("Device was on invalid Class! Only Class A and Class B are implemented currently");
                    return false;
                }
            }
            else if (deviceClass == CLASS_B)
            {
                if (m_deviceClass == CLASS_A)
                {
                    NS_LOG_DEBUG(" Use SwitchToClassB () instead as it will need to search and lock the beacon first");
                    return false;
                }
                else if (m_deviceClass == CLASS_B)
                {
                    NS_LOG_DEBUG("Device Already in Class B!");
                    return true;
                }
                else
                {
                    NS_LOG_DEBUG("Device was on invalid Class! Only Class A and Class B are implemented currently");
                    return false;
                }
            }
            else if (deviceClass == CLASS_C)
            {
                NS_LOG_ERROR("Device currently don't implement Class C");
                return false;
            }
            else
            {

                NS_LOG_ERROR(" Unknown Device Class" << deviceClass);
                return false;
            }
        }

        ClassBEndDeviceLorawanMac::DeviceClass
        ClassBEndDeviceLorawanMac::GetDeviceClass()
        {
            return m_deviceClass;
        }

        void
        ClassBEndDeviceLorawanMac::SetMacState(MacState macState)
        {
            NS_LOG_FUNCTION(this << macState);
            //Check the logic before making a switch to other state
            //Mac State machine
            switch (m_macState)
            {
            case MAC_TX:
                if (macState == MAC_IDLE)
                {
                    m_macState = macState;
                }
                else
                {
                    NS_LOG_ERROR("Only IDLE is a possible macState after Tx!");
                }
                break;

            case MAC_RX1:
                if (macState == MAC_IDLE)
                {
                    m_macState = macState;
                }
                else if (macState == MAC_RX_BEACON_GUARD)
                {
                    m_macState = macState;
                }
                else
                {
                    NS_LOG_ERROR("Can not switch from " << m_macState << " to " << macState);
                }
                break;

            case MAC_RX2:
                if (macState == MAC_IDLE)
                {
                    m_macState = macState;
                }
                else if (macState == MAC_RX_BEACON_GUARD)
                {
                    m_macState = macState;
                }
                else
                {
                    NS_LOG_ERROR("Can not switch from " << m_macState << " to " << macState);
                }
                break;

            case MAC_RX_BEACON_GUARD:
                if (macState == MAC_BEACON_GUARD)
                {
                    m_macState = macState;
                }
                else
                {
                    NS_LOG_ERROR("Only BEACON_GUARD is a possible macState after RX_BEACON_GUARD");
                }
                break;

            case MAC_BEACON_GUARD:
                if (macState == MAC_BEACON_RESERVED)
                {
                    m_macState = macState;
                }
                else
                {
                    NS_LOG_ERROR("Only BEACON_RESERVED is a possible macState after BEACON_GUARD");
                }
                break;

            case MAC_BEACON_RESERVED:
                if (macState == MAC_IDLE)
                {
                    m_macState = macState;
                }
                else
                {
                    NS_LOG_ERROR("Only IDLE is a possible macState after BEACON_RESERVED");
                }
                break;

            case MAC_PING_SLOT:
                if (macState == MAC_IDLE)
                {
                    m_macState = macState;
                }
                else if (macState == MAC_PING_SLOT_BEACON_GUARD)
                {
                    m_macState = macState;
                }
                else
                {
                    NS_LOG_ERROR("Can not switch from " << m_macState << " to " << macState);
                }
                break;

            case MAC_PING_SLOT_BEACON_GUARD:
                if (macState == MAC_BEACON_GUARD)
                {
                    m_macState = macState;
                }
                else
                {
                    NS_LOG_ERROR("Only BEACON_GUARD is a possible macState after PING_SLOT_BEACON_GUARD");
                }
                break;

            case MAC_IDLE:
                if (macState == MAC_BEACON_RESERVED ||
                    macState == MAC_PING_SLOT_BEACON_GUARD ||
                    macState == MAC_RX_BEACON_GUARD)
                {
                    NS_LOG_ERROR("Can not switch from " << m_macState << " to " << macState << " without an intermediate state");
                }
                else
                {
                    m_macState = macState;
                }
                break;

            default:
                NS_LOG_ERROR("Unknown Mac State");
                break;
            }
        }

        ClassBEndDeviceLorawanMac::MacState
        ClassBEndDeviceLorawanMac::GetMacState()
        {
            return m_macState;
        }

        //////////////
        // Channel //
        ////////////

        void
        ClassBEndDeviceLorawanMac::SetPingSlotReceiveWindowDataRate(uint8_t pingSlotDr)
        {
            m_classBReceiveWindowInfo.pingSlotReceiveWindowDataRate = pingSlotDr;
        }

        uint8_t
        ClassBEndDeviceLorawanMac::GetPingSlotReceiveWindowDataRate()
        {
            return m_classBReceiveWindowInfo.pingSlotReceiveWindowDataRate;
        }

        void
        ClassBEndDeviceLorawanMac::SetPingSlotReceiveWindowFrequency(double frequency)
        {
            m_classBReceiveWindowInfo.pingSlotReceiveWindowFrequency = frequency;
        }

        double
        ClassBEndDeviceLorawanMac::GetPingSlotRecieveWindowFrequency()
        {
            return m_classBReceiveWindowInfo.pingSlotReceiveWindowFrequency;
        }

        void
        ClassBEndDeviceLorawanMac::SetBeaconReceiveWindowDataRate(uint8_t beaconDr)
        {
            m_classBReceiveWindowInfo.beaconReceiveWindowDataRate = beaconDr;
        }

        uint8_t
        ClassBEndDeviceLorawanMac::GetBeaconRecieveWindowDataRate()
        {
            return m_classBReceiveWindowInfo.pingSlotReceiveWindowDataRate;
        }

        void
        ClassBEndDeviceLorawanMac::SetBeaconReceiveWindowFrequency(double frequency)
        {
            m_classBReceiveWindowInfo.beaconReceiveWindowFrequency = frequency;
        }

        double
        ClassBEndDeviceLorawanMac::GetBeaconRecieveWindowFrequency()
        {
            return m_classBReceiveWindowInfo.beaconReceiveWindowFrequency;
        }

        /////////////////////////
        // Class B parameters //
        ///////////////////////

        void
        ClassBEndDeviceLorawanMac::SetPingSlotPeriodicity(uint8_t periodicity)
        {

            NS_ASSERT_MSG(m_deviceClass != CLASS_B, "Error! Ping slot periodicity can't be set while operation");

            if (periodicity < 8 || periodicity >= 0)
            {
                m_pingSlotInfo.pingSlotPeriodicity = periodicity;
                m_pingSlotInfo.pingNb = std::pow(2, (7 - periodicity));
                m_pingSlotInfo.pingPeriod = 4096 / (m_pingSlotInfo.pingNb);

                // Set number of pings per beacon period
                m_pingSlotInfo.pendingPingSlotEvents.resize(m_pingSlotInfo.pingNb);
            }
            else
            {
                NS_LOG_ERROR("Invalid Ping Slot periodicity");
            }
        }

        uint8_t
        ClassBEndDeviceLorawanMac::GetPingSlotPeriodicity()
        {
            return m_pingSlotInfo.pingSlotPeriodicity;
        }

        void
        ClassBEndDeviceLorawanMac::SetPingNb(uint8_t pingNb)
        {

            NS_ASSERT_MSG(m_deviceClass != CLASS_B, "Error! Ping slot periodicity can't be set while operation");

            if (pingNb <= 128 || pingNb >= 1)
            {
                m_pingSlotInfo.pingNb = pingNb;
                m_pingSlotInfo.pingSlotPeriodicity = 7 - std::log2(pingNb);
                m_pingSlotInfo.pingPeriod = 4096 / pingNb;

                // Set number of pings per beacon period
                m_pingSlotInfo.pendingPingSlotEvents.resize(m_pingSlotInfo.pingNb);
            }
            else
            {
                NS_LOG_ERROR("Invalid PingNb");
            }
        }

        uint8_t
        ClassBEndDeviceLorawanMac::GetPingNb()
        {
            return m_pingSlotInfo.pingNb;
        }

        void
        ClassBEndDeviceLorawanMac::SetPingPeriod(uint pingPeriod)
        {

            NS_ASSERT_MSG(m_deviceClass != CLASS_B, "Error! Ping slot periodicity can't be set while operation");

            if (pingPeriod <= 4096 || pingPeriod >= 32)
            {
                m_pingSlotInfo.pingPeriod = pingPeriod;
                m_pingSlotInfo.pingNb = 4096 / pingPeriod;
                m_pingSlotInfo.pingSlotPeriodicity = 7 - std::log2(m_pingSlotInfo.pingNb);

                // Set number of pings per beacon period
                m_pingSlotInfo.pendingPingSlotEvents.resize(m_pingSlotInfo.pingNb);
            }
            else
            {
                NS_LOG_ERROR("Invalid pingPeriod");
            }
        }

        uint
        ClassBEndDeviceLorawanMac::GetPingPeriod()
        {
            return m_pingSlotInfo.pingPeriod;
        }

        ////////////////
        //Callbacks  //
        //////////////

        void
        ClassBEndDeviceLorawanMac::SetBeaconLockedCallback(Callback<void> beaconLockedCallback)
        {
            m_beaconLockedCallback = beaconLockedCallback;
        }

        void
        ClassBEndDeviceLorawanMac::SetBeaconLostCallback(Callback<void> beaconLostCallback)
        {
            m_beaconLostCallback = beaconLostCallback;
        }

        void
        ClassBEndDeviceLorawanMac::SetClassBDownlinkCallback(ClassBDownlinkCallback classBDownlinkCallback)
        {
            m_classBDownlinkCallback = classBDownlinkCallback;
        }

        ///////////////////
        // Multicasting //
        /////////////////

        void
        ClassBEndDeviceLorawanMac::EnableMulticast(void)
        {
            if (m_mcAddress.Get() == 1)
            {
                NS_LOG_ERROR("Set the multicast Address before enabling multicast!");
            }
            else
            {
                m_enableMulticast = true;
            }
        }

        void
        ClassBEndDeviceLorawanMac::DisableMulticast()
        {
            m_enableMulticast = false;
        }

        bool
        ClassBEndDeviceLorawanMac::IsMulticastEnabled(void)
        {
            return m_enableMulticast;
        }

        //////////////////////////////////////////////
        // Related to coordinated relaying         //
        ////////////////////////////////////////////

        void
        ClassBEndDeviceLorawanMac::EnableCoordinatedRelaying(uint32_t numberOfEndDeviceInMcGroup, uint8_t relayingAlgorithm)
        {
            NS_LOG_FUNCTION(this << numberOfEndDeviceInMcGroup << relayingAlgorithm);

            if (relayingAlgorithm == 0)
                return; // relayingAlgorithm 0 means not using relaying at all

            NS_ASSERT_MSG(numberOfEndDeviceInMcGroup > 1, "You can not activate coordinated relaying with only one node!");

            //Register number of member nodes in a multicast group
            m_relayPower.numberOfEndDeviceInMcGroup = numberOfEndDeviceInMcGroup;

            //Select the algorithm to use for coordinated relaying
            m_relayPower.algorithm = relayingAlgorithm;

            //should we add one hop as far as the network-server is in the off-time as well?

            // \TODO May be further decrease the m_relayPower according to device energy consumption

            m_relayActivated = true;
        }

        double
        ClassBEndDeviceLorawanMac::GetRelayingPower()
        {
            NS_LOG_FUNCTION_NOARGS();

            // relay power to be used, negative number shows do not relay for now
            double relayPower = 0;

            switch (m_relayPower.algorithm)
            {
            case 0:
                NS_ASSERT_MSG(false, "Relying power not activated, yet invoked!");
                break;
            case 1: // relaying with max power
                relayPower = m_relayPower.m_maxTxPower;
                break;
            case 2: // this node transmit with the probability of 1/numberOfEndDeviceInMcGroup
            {
                //random number generation.
                int relayIndex = m_relayPower.decisionForRelayingRandomNumber->GetInteger(1, (uint32_t)m_relayPower.numberOfEndDeviceInMcGroup); // may be assume 50% has received packet

                //Only transmit if relayIndex is 1, that means you will transmit with the probability of 1/numberOfEndDevicesInMcGroup
                if (relayIndex == 1)
                {
                    relayPower = m_relayPower.m_maxTxPower;
                }
                else
                {
                    relayPower = -2; // 20dbm is the maximumpower for semtech sx1272 with 31/41mA
                }

                break;
            }
            case 3: //  the power to be used is 14dbm/(randomselection between 1 and numberOfEndDeviceInMcGroup)
            {
                //Randomely select a power to transmit with
                int relayPowerDevider = m_relayPower.relayPowerRandomNumber->GetInteger(1, (uint32_t)(m_relayPower.numberOfEndDeviceInMcGroup / 2));
                // Divide the maximum power with the number of nodes in the multicast group
                relayPower = m_relayPower.m_maxTxPower / (relayPowerDevider); //std::min ((uint32_t)10, (numberOfEndDeviceInMcGroup-1)); // 2 db is the minimum power for Semtech SX1272 with 26mA and
                break;
            }
            case 4: //  the power to be used is 14dbm/(randomselection between 1 and numberOfEndDeviceInMcGroup)
            {
                //random number generation.
                int relayIndex = m_relayPower.decisionForRelayingRandomNumber->GetInteger(1, (uint32_t)m_relayPower.numberOfEndDeviceInMcGroup); // may be assume 50% has received packet

                //Only transmit if relayIndex is 1, that means you will transmit with the probability of 1/numberOfEndDevicesInMcGroup
                if (relayIndex == 1)
                {
                    //Randomely select a power to transmit with
                    int relayPowerDevider = m_relayPower.relayPowerRandomNumber->GetInteger(1, (uint32_t)(m_relayPower.numberOfEndDeviceInMcGroup / 2));
                    // Divide the maximum power with the number of nodes in the multicast group
                    relayPower = m_relayPower.m_maxTxPower / (relayPowerDevider); //std::min ((uint32_t)10, (numberOfEndDeviceInMcGroup-1)); // 2 db is the minimum power for Semtech SX1272 with 26mA and
                }
                else
                {
                    relayPower = -2; // 20dbm is the maximumpower for semtech sx1272 with 31/41mA
                }
                break;
            }
            case 5: //Decrease transmit power with number of nodes
            {
                // Divide the maximum power with the number of nodes in the multicast group
                relayPower = m_relayPower.m_maxTxPower / (m_relayPower.numberOfEndDeviceInMcGroup - 1); //std::min ((uint32_t)10, (numberOfEndDeviceInMcGroup-1)); // 2 db is the minimum power for Semtech SX1272 with 26mA and
                break;
            }
            default:
                NS_ASSERT_MSG(false, "Relaying algorithm not implimented yet!");
                break;
            }

            return relayPower;
        }

    } /* namespace lorawan */
} /* namespace ns3 */