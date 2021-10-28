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
 */

#include "ns3/gateway-lorawan-mac.h"
#include "ns3/lorawan-mac-header.h"
#include "ns3/lora-net-device.h"
#include "ns3/lora-frame-header.h"
#include "ns3/log.h"
#include "ns3/bcn-payload.h"

namespace ns3 {
namespace lorawan {

NS_LOG_COMPONENT_DEFINE ("GatewayLorawanMac");

NS_OBJECT_ENSURE_REGISTERED (GatewayLorawanMac);

TypeId
GatewayLorawanMac::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::GatewayLorawanMac")
    .SetParent<LorawanMac> ()
    .AddConstructor<GatewayLorawanMac> ()
    .SetGroupName ("lorawan");
  return tid;
}

GatewayLorawanMac::GatewayLorawanMac ()
{
  NS_LOG_FUNCTION (this);
}

GatewayLorawanMac::~GatewayLorawanMac ()
{
  NS_LOG_FUNCTION (this);
}

void
GatewayLorawanMac::Send (Ptr<Packet> packet)
{
  NS_LOG_FUNCTION (this << packet);

  // Get DataRate to send this packet with
  LoraTag tag;
  packet->RemovePacketTag (tag);
  uint8_t dataRate = tag.GetDataRate ();
  double frequency = tag.GetFrequency ();
  NS_LOG_DEBUG ("DR: " << unsigned (dataRate));
  NS_LOG_DEBUG ("SF: " << unsigned (GetSfFromDataRate (dataRate)));
  NS_LOG_DEBUG ("BW: " << GetBandwidthFromDataRate (dataRate));
  NS_LOG_DEBUG ("Freq: " << frequency << " MHz");
  packet->AddPacketTag (tag);

  //If it a beacon packet and the device is not enabled to transmit beacon 
  //then we need to skip it
  bool beaconPacket = tag.IsBeaconPacket ();
  if (beaconPacket && !m_beaconTransmission)
  {
    NS_LOG_DEBUG ("This Gateway is not enabled to transmit beacons!");
    return;
  }
  else if (beaconPacket)
  {
    NS_LOG_DEBUG ("Transmitting beacon packet!");
  }

  // Make sure we can transmit this packet
  if (m_channelHelper.GetWaitingTime(CreateObject<LogicalLoraChannel> (frequency)) > Time(0))
    {
      // We cannot send now!
      NS_LOG_WARN ("Trying to send a packet but Duty Cycle won't allow it. Aborting.");
      return;
    }

  LoraTxParameters params;
  params.sf = GetSfFromDataRate (dataRate);
  params.headerDisabled = false;
  params.codingRate = 1;
  params.bandwidthHz = GetBandwidthFromDataRate (dataRate);
    //Beacon Packet uses longer preamble (10) in-order to allow low power duty cycling for the end-nodes
  params.nPreamble = (beaconPacket ? 10 : 8);
  params.crcEnabled = 1;
  params.lowDataRateOptimizationEnabled = LoraPhy::GetTSym (params) > MilliSeconds (16) ? true : false;

  // Get the duration
  Time duration = m_phy->GetOnAirTime (packet, params);

  NS_LOG_DEBUG ("Duration: " << duration.GetSeconds ());

  // Find the channel with the desired frequency
  double sendingPower = m_channelHelper.GetTxPowerForChannel
      (CreateObject<LogicalLoraChannel> (frequency));

  // Add the event to the channelHelper to keep track of duty cycle
  m_channelHelper.AddEvent (duration, CreateObject<LogicalLoraChannel>
                              (frequency));

  // Send the packet to the PHY layer to send it on the channel
  m_phy->Send (packet, params, frequency, sendingPower);

  m_sentNewPacket (packet);
}

bool
GatewayLorawanMac::IsTransmitting (void)
{
  return m_phy->IsTransmitting ();
}

void
GatewayLorawanMac::Receive (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (this << packet);

  // Make a copy of the packet to work on
  Ptr<Packet> packetCopy = packet->Copy ();

  BcnPayload bcnPayload;
  packetCopy->PeekHeader (bcnPayload);
  
  //Drop right here if it is a beacon packet
  if (bcnPayload.GetBcnTime() != 0)
  {
    NS_LOG_DEBUG ("Not forwarding beacon packet");
    return;
  }
  
  // Only forward the packet if it's uplink
  LorawanMacHeader macHdr;
  packetCopy->PeekHeader (macHdr);

  if (macHdr.IsUplink ())
    {
      m_device->GetObject<LoraNetDevice> ()->Receive (packetCopy);

      NS_LOG_DEBUG ("Received packet: " << packet);

      m_receivedPacket (packet);
    }
  else
    {
      NS_LOG_DEBUG ("Not forwarding downlink message to NetDevice");
    }
}

void
GatewayLorawanMac::FailedReception (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (this << packet);
}

void
GatewayLorawanMac::TxFinished (Ptr<const Packet> packet)
{
  NS_LOG_FUNCTION_NOARGS ();
}

Time
GatewayLorawanMac::GetWaitingTime (double frequency)
{
  NS_LOG_FUNCTION_NOARGS ();

  return m_channelHelper.GetWaitingTime (CreateObject<LogicalLoraChannel>
                                           (frequency));
}

void
GatewayLorawanMac::EnableBeaconTransmission (void)
{
  NS_LOG_FUNCTION_NOARGS ();
  
  m_beaconTransmission = true;
}

void
GatewayLorawanMac::DisableBeaconTransmission (void)
{
  NS_LOG_FUNCTION_NOARGS ();
  
  m_beaconTransmission = false;
}

bool
GatewayLorawanMac::IsBeaconTransmissionEnabled ()
{
  NS_LOG_FUNCTION_NOARGS ();  
  
  return m_beaconTransmission;
}

void
GatewayLorawanMac::EnableClassBTransmission ()
{
  NS_LOG_FUNCTION_NOARGS ();
  
  m_classBTransmission = true;
}

void
GatewayLorawanMac::DisableClassBTransmission ()
{
  NS_LOG_FUNCTION_NOARGS ();
  
  m_classBTransmission = false;
}

bool
GatewayLorawanMac::IsClassBTransmissionEnabled ()
{
  NS_LOG_FUNCTION_NOARGS ();
  
  return m_classBTransmission;
}

void
GatewayLorawanMac::AddMulticastGroup (LoraDeviceAddress mcAddress)
{
  NS_LOG_FUNCTION_NOARGS ();
// Check if the address already exists
std::list<LoraDeviceAddress>::iterator it;
it = std::find(m_mcAddressList.begin(), m_mcAddressList.end(), mcAddress);
 
// Add to the list if not
if(it == m_mcAddressList.end())
  {
    m_mcAddressList.push_back (mcAddress);
  }

}

std::list<LoraDeviceAddress>
GatewayLorawanMac::GetMulticastGroups ()
{
  return m_mcAddressList;
}

bool
GatewayLorawanMac::CheckMulticastGroup (LoraDeviceAddress mcAddress)
{
  // Check if the address exists
  std::list<LoraDeviceAddress>::iterator it;
  it = std::find(m_mcAddressList.begin(), m_mcAddressList.end(), mcAddress);
 
  // Return true if the address if found otherwise return false
  return (it != m_mcAddressList.end());
}

}
}
