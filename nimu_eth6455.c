/**************************************************************************
 * FILE PURPOSE	:  	NIMU Interface for the Ethernet Driver (DSK6455)
 **************************************************************************
 * FILE NAME	:   nimu_eth6455.c
 *
 * DESCRIPTION	:
 *  Ethernet Packet Driver for the DSK6455 rewritten using the NIMU Packet
 *  Architecture guidelines. 

 *	CALL-INs:
 *
 *	CALL-OUTs:
 *
 *	User-Configurable Items:
 *
 *	(C) Copyright 2008, Texas Instruments, Inc.
 *************************************************************************/

#include <stkmain.h>
#include "llpacket.h"

#ifdef _INCLUDE_NIMU_CODE

/* The DSK6455 EMAC Initialization Function. */
int DSK6455EmacInit (STKEVENT_Handle hEvent);

/* This is the NIMU Device Table for the DSK6455 Platform. 
 * This should be defined for each platform. Since the DSK6455 platform
 * has a single network Interface; this has been defined here. If the 
 * platform supports more than one network interface this should be 
 * defined to have a list of "initialization" functions for each of the
 * interfaces. */
NIMU_DEVICE_TABLE_ENTRY NIMUDeviceTable[] = 
{
    DSK6455EmacInit,
    NULL
};

/*********************************************************************
 * STRUCTURE NAME : DSK6455_EMAC_DATA
 *********************************************************************
 * DESCRIPTION   :
 *  The structure is used to store the private data for the DSK6455 
 *  EMAC controller.
 *********************************************************************/
typedef struct DSK6455_EMAC_DATA
{
    PDINFO      pdi;        /* Private Information  */
}DSK6455_EMAC_DATA;

/*********************************************************************
 * FUNCTION NAME : DSK6455EmacStart
 *********************************************************************
 * DESCRIPTION   :
 *  The function is used to initialize and start the DSK6455 EMAC
 *  controller and device.
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 *********************************************************************/
static int DSK6455EmacStart (NETIF_DEVICE* ptr_net_device)
{
    DSK6455_EMAC_DATA*  ptr_pvt_data;

    /* Get the pointer to the private data */
    ptr_pvt_data = (DSK6455_EMAC_DATA *)ptr_net_device->pvt_data;

    /* Call low-level open function */
    if (HwPktOpen(&ptr_pvt_data->pdi) == 1)
    {
        /* Copy the MAC Address into the network interface object here. */
        mmCopy(&ptr_net_device->mac_address[0], &ptr_pvt_data->pdi.bMacAddr[0], 6);

        /* Set the 'initial' Receive Filter */
        ptr_pvt_data->pdi.Filter = ETH_PKTFLT_MULTICAST;
        HwPktSetRx(&ptr_pvt_data->pdi);

        /* Inform the world that we are operational. */
        printf ("DSK6455 EMAC has been started successfully\n");
        return 0;
    }

    /* Error: DSK6455 EMAC failed to start. */
    return -1;
}

/*********************************************************************
 * FUNCTION NAME : DSK6455EmacStop
 *********************************************************************
 * DESCRIPTION   :
 *  The function is used to de-initialize and stop the DSK6455 EMAC
 *  controller and device.
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 *********************************************************************/
static int DSK6455EmacStop (NETIF_DEVICE* ptr_net_device)
{
    DSK6455_EMAC_DATA*  ptr_pvt_data;

    /* Get the pointer to the private data */
    ptr_pvt_data = (DSK6455_EMAC_DATA *)ptr_net_device->pvt_data;

    /* Call low-level close function */
    HwPktClose (&ptr_pvt_data->pdi);

    /* Shut down the Ethernet controller. */    
    HwPktShutdown();

    /* Flush out our pending queue */
    while( PBMQ_count(&ptr_pvt_data->pdi.PBMQ_rx) )
        PBM_free( PBMQ_deq(&ptr_pvt_data->pdi.PBMQ_rx) );
       
    /* EMAC Controller has been stopped. */ 
    return 0;
}

/*********************************************************************
 * FUNCTION NAME : DSK6455EmacPoll
 *********************************************************************
 * DESCRIPTION   :
 *  The function is used to poll the DSK6455 EMAC controller to check
 *  if there has been any activity.
 *********************************************************************/
static void DSK6455EmacPoll (NETIF_DEVICE* ptr_net_device, uint timer_tick)
{
    DSK6455_EMAC_DATA*  ptr_pvt_data;

    /* Get the pointer to the private data */
    ptr_pvt_data = (DSK6455_EMAC_DATA *)ptr_net_device->pvt_data;
    
    /* Poll the driver. */
    _HwPktPoll (&ptr_pvt_data->pdi, timer_tick);
    return;
}

/*********************************************************************
 * FUNCTION NAME : DSK6455EmacSend
 *********************************************************************
 * DESCRIPTION   :
 *  The function is the interface routine invoked by the NDK stack to
 *  pass packets to the driver. 
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 *********************************************************************/
static int DSK6455EmacSend (NETIF_DEVICE* ptr_net_device, PBM_Handle hPkt)
{
    DSK6455_EMAC_DATA*  ptr_pvt_data;

    /* Get the pointer to the private data */
    ptr_pvt_data = (DSK6455_EMAC_DATA *)ptr_net_device->pvt_data;

    /* Make sure the driver does not transmit packet less than min. as per the
     * Ethernet standards. */
    if( PBM_getValidLen(hPkt) < 60 )
        PBM_setValidLen (hPkt, 60 );

    /* Transmit the packet only if does not exceed the MTU */
    if(PBM_getValidLen(hPkt) <= 1518 )
    {
        /* Enqueue the packet and send it for transmission. */
        PBMQ_enq (&ptr_pvt_data->pdi.PBMQ_tx, hPkt);

        /* Pass the packet to the controller if the transmitter is free. */
        if(ptr_pvt_data->pdi.TxFree )
            HwPktTxNext(&ptr_pvt_data->pdi);

        /* Packet has been successfully transmitted. */
        return 0;
    }

    /* NOTE: This is a violation and should never occur as the NDK stack should have 
     * fragmented the packet. In this case though we drop and clean the packet we will 
     * still return success. */
    PBM_free (hPkt);
    return 0;
}

/*********************************************************************
 * FUNCTION NAME : DSK6455EmacPktService
 *********************************************************************
 * DESCRIPTION   :
 *  The function is called by the NDK core stack to receive any packets
 *  from the driver.
 *********************************************************************/
static void DSK6455EmacPktService (NETIF_DEVICE* ptr_net_device)
{
    DSK6455_EMAC_DATA*  ptr_pvt_data;
    PBM_Handle          hPacket;

    /* Get the pointer to the private data */
    ptr_pvt_data = (DSK6455_EMAC_DATA *)ptr_net_device->pvt_data;
        
    /* Give all queued packets to the Ether module */
    while (PBMQ_count(&ptr_pvt_data->pdi.PBMQ_rx))
    {
        /* Dequeue a packet from the driver receive queue. */
        hPacket = PBMQ_deq(&ptr_pvt_data->pdi.PBMQ_rx);

        /* Prepare the packet so that it can be passed up the networking stack. 
         * If this 'step' is not done the fields in the packet are not correct
         * and the packet will eventually be dropped.  */
        PBM_setIFRx (hPacket, ptr_net_device);
        
        /* Pass the packet to the NDK Core stack. */    
        NIMUReceivePacket(hPacket);
    }

    /* Work has been completed; the receive queue is empty... */ 
    return;
}

/*********************************************************************
 * FUNCTION NAME : DSK6455Emacioctl
 *********************************************************************
 * DESCRIPTION   :
 *  The function is called by the NDK core stack to configure the 
 *  driver
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 *********************************************************************/
static int DSK6455Emacioctl (NETIF_DEVICE* ptr_net_device, uint cmd, void* pBuf, uint size)
{
    DSK6455_EMAC_DATA*  ptr_pvt_data;
    int                 count = 0;
    UINT8*              mac_address_add_del = (UINT8 *)pBuf;   
    UINT16              index;

    /* Get the pointer to the private data */
    ptr_pvt_data = (DSK6455_EMAC_DATA *)ptr_net_device->pvt_data;

    /* Process the command. */    
    switch (cmd)
    {
        case NIMU_ADD_MULTICAST_ADDRESS:
        {
            /* Validate the arguments. */
            if ((pBuf == NULL) || (size != 6))
                return -EINVAL;

            /* We need to add an address to the NIMU Network Interface Object's multicast 
             * list. Check for duplicate addresses. */
            while (count < ptr_pvt_data->pdi.MCastCnt)
            {
                UINT8* mac_address_list = (UINT8 *)&ptr_pvt_data->pdi.bMCast[count*6];

                /* Match the MAC Addresses  */
                for (index=0; index<6; index++)
                {
                    if( *(mac_address_add_del+index) != *(mac_address_list+index))
                        break;
                }

                /* Check if there is a hit or not? */
                if (index == 6)
                {
                    /* Duplicate MAC address; the address was already present in the list. 
                     * This is not an error we will still return SUCCESS here */
                    return 0;                        
                }
                else
                {
                    /* No HIT! Goto the next entry in the device multicast list. */
                    count++;                        
                }
            }

            /* Control comes here implies that the MAC Address needs to be added to the
             * device list. The variable 'count' is pointing to the free location available
             * in which the multicast address can be added. But before we do so check if 
             * we dont exceed the upper limit? */
            if (count >= PKT_MAX_MCAST)
                return -ENOMEM;

            /* Add the multicast address to the end of the list. */
            mmCopy (&ptr_pvt_data->pdi.bMCast[count*6], mac_address_add_del, 6);
            ptr_pvt_data->pdi.MCastCnt++;

            /* Configure the DSK6455 Ethernet controller with the new multicast list. */
            HwPktSetRx (&ptr_pvt_data->pdi);
            break;
        }
        case NIMU_DEL_MULTICAST_ADDRESS:
        {
            /* Validate the arguments. */
            if ((pBuf == NULL) || (size != 6))
                return -EINVAL;

            /* We need to delete an address from the NIMU Network Interface Object's multicast 
             * list. First cycle through and make sure the entry exists. */
            while (count < ptr_pvt_data->pdi.MCastCnt)
            {
                UINT8* mac_address_list = (UINT8 *)&ptr_pvt_data->pdi.bMCast[count*6];

                /* Match the MAC Addresses  */
                for ( index=0; index<6; index++)
                {
                    if( *(mac_address_add_del+index) != *(mac_address_list+index))
                        break;
                }

                /* Check if there is a hit or not? */
                if (index == 6)
                {
                    /* Found the matching entry. We can now delete this! */
                    break;
                }
                else
                {
                    /* No HIT! Goto the next entry in the device multicast list. */
                    count++;
                }
            }

            /* Did we find a match or not? If not then report the error back */
            if (count == ptr_pvt_data->pdi.MCastCnt)
                return -EINVAL;

            /* At this time 'count' points to the entry being deleted. We now need to copy all 
             * the entries after the 'del' entry back one space in the multicast array. */
            for (index = count; index < (ptr_pvt_data->pdi.MCastCnt - 1); index++)
                mmCopy (&ptr_pvt_data->pdi.bMCast[index*6], &ptr_pvt_data->pdi.bMCast[(index+1)*6], 6);

            /* Decrement the multicast entries. */
            ptr_pvt_data->pdi.MCastCnt--;

            /* Configure the DSK6455 Ethernet controller with the new multicast list. */
            HwPktSetRx (&ptr_pvt_data->pdi);
            break;
        }
        default:
        {
            /* The command is NOT handled by the driver. */
            return -1;
        } 
    }
    return 0;
}

/*********************************************************************
 * FUNCTION NAME : DSK6455EmacInit
 *********************************************************************
 * DESCRIPTION   :
 *  The function is used to initialize and register the EMAC for the
 *  DSK6455 with the Network Interface Management Unit (NIMU)
 *********************************************************************/
int DSK6455EmacInit (STKEVENT_Handle hEvent)
{
    NETIF_DEVICE*       ptr_device;
    DSK6455_EMAC_DATA*  ptr_pvt_data;

    /* Allocate memory for the private data */
    ptr_pvt_data = mmAlloc(sizeof(DSK6455_EMAC_DATA));
    if (ptr_pvt_data == NULL)
    {
        printf ("Error: Unable to allocate private memory data\n");
        return -1;
    }

    /* Initialize the allocated memory block. */
    mmZeroInit (ptr_pvt_data, sizeof(DSK6455_EMAC_DATA));

    /* Initialize the RX Queue */
    PBMQ_init(&ptr_pvt_data->pdi.PBMQ_rx);

    /* Initialize the packet drivers. */
    HwPktInit();

    /* Initialize the private data */
    mmZeroInit(&ptr_pvt_data->pdi, sizeof(PDINFO));

    /* Set physical index */
    ptr_pvt_data->pdi.PhysIdx = 0;
    ptr_pvt_data->pdi.hEvent  = hEvent;

    /* MCast List is EMPTY */
    ptr_pvt_data->pdi.MCastCnt    = 0;

    /* Default MAC Address (can be overwritten by HwPktOpen()) */
    ptr_pvt_data->pdi.bMacAddr[0] = 0x08;
    ptr_pvt_data->pdi.bMacAddr[1] = 0x00;
    ptr_pvt_data->pdi.bMacAddr[2] = 0x28;
    ptr_pvt_data->pdi.bMacAddr[3] = 0xFF;
    ptr_pvt_data->pdi.bMacAddr[4] = 0xFF;
    ptr_pvt_data->pdi.bMacAddr[5] = 0x20;

    /* Init Logical Device */
    /* ptr_pvt_data->pdi.hEther = hEther; */

    /* Allocate memory for the DSK6455 EMAC. */
    ptr_device = mmAlloc(sizeof(NETIF_DEVICE));
    if (ptr_device == NULL)
    {
        printf ("Error: Unable to allocate memory for the DSK6455 EMAC\n");
        return -1;
    }

    /* Initialize the allocated memory block. */
    mmZeroInit (ptr_device, sizeof(NETIF_DEVICE));

    /* Populate the Network Interface Object. */
    strcpy (ptr_device->name, "eth0");
    ptr_device->mtu         = ETH_MAX_PAYLOAD - ETHHDR_SIZE;
    ptr_device->pvt_data    = (void *)ptr_pvt_data;

    /* Populate the Driver Interface Functions. */
    ptr_device->start       = DSK6455EmacStart;
    ptr_device->stop        = DSK6455EmacStop;
    ptr_device->poll        = DSK6455EmacPoll;
    ptr_device->send        = DSK6455EmacSend;
    ptr_device->pkt_service = DSK6455EmacPktService;
    ptr_device->ioctl       = DSK6455Emacioctl;
    ptr_device->add_header  = NIMUAddEthernetHeader;

    /* Register the device with NIMU */
    if (NIMURegister (ptr_device) < 0)
    {
        printf ("Error: Unable to register the DSK6455 EMAC\n");
        return -1;
    }

    printf ("Registeration of the DSK6455 EMAC Successful\n");
    return 0;
}

#endif /* _INCLUDE_NIMU_CODE */

