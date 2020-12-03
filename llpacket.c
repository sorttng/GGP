/*
 *  Copyright 2007 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) TCP/IP_Network_Developers_Kit 1.92.00.22 01-10-2007 (ndk-b22)
 */
//--------------------------------------------------------------------------
// IP Stack
//--------------------------------------------------------------------------
// llPacket.c
//
// Ethernet Packet Driver
//
// Author: Michael A. Denio
// Copyright 1998, 2003 by Texas Instruments Inc.
//-------------------------------------------------------------------------
#include <stkmain.h>
#include "llpacket.h"

#ifndef _INCLUDE_NIMU_CODE

//--------------------------------------------------------------------
// PUBLIC FUNCTIONS USED FOR INTIALIZATION AND EVENTS
//--------------------------------------------------------------------
#define MAX_INSTANCE    2

static uint     PktDevCount = 0;        // Number of Net Devices
static PDINFO   pdi[MAX_INSTANCE];      // Private Info about devices

//
// Queue for Received Packets
//
PBMQ    PBMQ_rx;

//--------------------------------------------------------------------
// PUBLIC FUNCTIONS USED BY NETCTRL
//--------------------------------------------------------------------

//--------------------------------------------------------------------
// _llPacketInit()
//
// Opens the packet driver environment and enumerates devices
//--------------------------------------------------------------------
uint _llPacketInit(STKEVENT_Handle hEvent)
{
    int i;

    // Initialize RX Queue
    PBMQ_init(&PBMQ_rx);

    //
    // Initialize the packet driver(s)
    //
    PktDevCount = HwPktInit();

    if( PktDevCount > MAX_INSTANCE )
        PktDevCount = MAX_INSTANCE;

    //
    // Initialize PDINFO for each driver instance
    //
    for(i=0; i<PktDevCount; i++)
    {
        mmZeroInit( &pdi[i], sizeof( PDINFO ) );

        // Set physical index
        pdi[i].PhysIdx     = i;

        // Copy Event Handle
        pdi[i].hEvent      = hEvent;

        // MCast List
        pdi[i].MCastCnt    = 0;

        // Default MAC Address (can be overwritten by HwPktOpen())
        pdi[i].bMacAddr[0] = 0x08;
        pdi[i].bMacAddr[1] = 0x00;
        pdi[i].bMacAddr[2] = 0x28;
        pdi[i].bMacAddr[3] = 0xFF;
        pdi[i].bMacAddr[4] = 0xFF;
        pdi[i].bMacAddr[5] = (UINT8)i;
    }

    printf ("My LIBRARY\n");

    return( PktDevCount );
}

//--------------------------------------------------------------------
// _llPacketShutdown()
//
// Called to shutdown packet driver environment
//--------------------------------------------------------------------
void _llPacketShutdown()
{
    PktDevCount = 0;
    HwPktShutdown();

    // Flush out our pending queue
    while( PBMQ_count(&PBMQ_rx) )
        PBM_free( PBMQ_deq(&PBMQ_rx) );
}

//--------------------------------------------------------------------
// _llPacketServiceCheck()
//
// Called to check for packet activity
//--------------------------------------------------------------------
void _llPacketServiceCheck( uint fTimerTick )
{
    uint dev;

    for( dev=0; dev<PktDevCount; dev++ )
        _HwPktPoll( &(pdi[dev]), fTimerTick );
}

//--------------------------------------------------------------------
// PUBLIC FUNCTIONS USED BY THE STACK
//--------------------------------------------------------------------

//--------------------------------------------------------------------
// llPacketOpen()
//
// Opens the packet driver, and request packets of our desired type.
//--------------------------------------------------------------------
uint llPacketOpen( uint dev, HANDLE hEther )
{
    // Our device index is "1" based to the upper layers
    dev--;
    if( dev >= PktDevCount )
        return(0);

    // Init Logical Device
    pdi[dev].hEther = hEther;

    // Call low-level open function
    return( HwPktOpen( &pdi[dev] ) );
}

//--------------------------------------------------------------------
// PacketClose()
//
// Called to shutdown packet driver.
//--------------------------------------------------------------------
void llPacketClose( uint dev )
{
    // Our device index is "1" based to the upper layers
    dev--;
    if( dev >= PktDevCount )
        return;

    // Call low-level close function
    HwPktClose( &pdi[dev] );
}

//--------------------------------------------------------------------
// llPacketSend()
//
// Called to send data to a device. The buffer has been completely
// formatted by the time this call is made, and is ready for
// transmit. The dev parameter is the physical destination device.
//--------------------------------------------------------------------
void llPacketSend( uint dev, PBM_Handle hPkt )
{
    // Our device index is "1" based to the upper layers
    dev--;

    if( PBM_getValidLen(hPkt) < 60 )
        PBM_setValidLen( hPkt, 60 );

    if( dev < PktDevCount && PBM_getValidLen(hPkt) <= 1514 )
    {
        PBMQ_enq( &(pdi[dev].PBMQ_tx), hPkt );

        if( pdi[dev].TxFree )
            HwPktTxNext( &(pdi[dev]) );
    }
    else
        PBM_free( hPkt );
}

//--------------------------------------------------------------------
// llPacketGetMacAddr()
//
// Called to the MAC address of the specified device.
//--------------------------------------------------------------------
void llPacketGetMacAddr( uint dev, UINT8 *pbData )
{
    // Our device index is "1" based to the upper layers
    dev--;

    if( dev < PktDevCount )
        mmCopy( pbData, pdi[dev].bMacAddr, 6 );
}

//--------------------------------------------------------------------
// llPacketGetMCastMax()
//
// Called to the maxumim multicast addresses that the device can hold.
//--------------------------------------------------------------------
uint llPacketGetMCastMax( uint dev )
{
    // Our device index is "1" based to the upper layers
    dev--;

    if( dev < PktDevCount )
        return( PKT_MAX_MCAST );
    else
        return( 0 );
}

//--------------------------------------------------------------------
// llPacketSetRxFilter()
//
// Called to set the Rx Filter mode of the device. The legal modes
// are defined in ETHER.H
//--------------------------------------------------------------------
void llPacketSetRxFilter( uint dev, uint Filter )
{
    // Our device index is "1" based to the upper layers
    dev--;

    if( dev < PktDevCount && Filter <= ETH_PKTFLT_ALL )
    {
        pdi[dev].Filter = Filter;
        HwPktSetRx( &(pdi[dev]) );
    }
}

//--------------------------------------------------------------------
// llPacketSetMCast()
//
// Called to set the Multicast list for the device.
//--------------------------------------------------------------------
void llPacketSetMCast( uint dev, uint addrcnt, UINT8 *bAddr )
{
    // Our device index is "1" based to the upper layers
    dev--;

    // Copy the multicast list from ETH and update HW driver
    if( dev < PktDevCount && addrcnt <= PKT_MAX_MCAST )
    {
        pdi[dev].MCastCnt = addrcnt;
        mmCopy( pdi[dev].bMCast, bAddr, 6 * addrcnt );
        HwPktSetRx( &(pdi[dev]) );
    }
}

//--------------------------------------------------------------------
// llPacketGetMCast()
//
// Called to get the Multicast list for the device.
//--------------------------------------------------------------------
uint llPacketGetMCast( uint dev, uint max, UINT8 *bAddr )
{
    // Our device index is "1" based to the upper layers
    dev--;

    // Copy the multicast list to ETH
    if( dev < PktDevCount && max >= pdi[dev].MCastCnt && bAddr  )
    {
        mmCopy( bAddr, pdi[dev].bMCast, pdi[dev].MCastCnt * 6 );
        return( pdi[dev].MCastCnt );
    }
    return(0);
}

//--------------------------------------------------------------------
// llPacketService()
//
// Called to service packet activity.
//--------------------------------------------------------------------
void llPacketService()
{
    // Give all queued packets to the Ether module
    while( PBMQ_count(&PBMQ_rx) )
        EtherRxPacket( PBMQ_deq(&PBMQ_rx) );
}

#endif /* _INCLUDE_NIMU_CODE */
