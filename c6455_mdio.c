/*
 *  Copyright 2007 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) TCP/IP_Network_Developers_Kit 1.92.00.22 01-10-2007 (ndk-b22)
 */
/*****************************************************************************\
*           Copyright (C) 1999-2003 Texas Instruments Incorporated.
*                           All Rights Reserved
*------------------------------------------------------------------------------
* FILENAME...... c6455_mdio.c
* DATE CREATED.. 08/09/2005
* LAST MODIFIED.
*------------------------------------------------------------------------------
* NOTE:
*   When used in an multitasking environment, no MDIO function may be
*   called while another MDIO function is operating on the same device
*   handle in another thread. It is the responsibility of the application
*   to assure adherence to this restriction.
*
\******************************************************************************/

/* Include the MDIO header file */
#include "c6455_mdio.h"
#include "usertype.h"

/*
 * Standard defines/assumptions for MDIO interface
 */
#define VBUSCLK     165

/*-----------------------------------------------------------------------*\
* PHY Control Registers
*
* Used by MDIO to configure a MII compliant PHY
\*-----------------------------------------------------------------------*/
#define PHYREG_CONTROL              0
#define PHYREG_CONTROL_RESET        (1<<15)
#define PHYREG_CONTROL_LOOPBACK     (1<<14)
#define PHYREG_CONTROL_SPEEDLSB     (1<<13)
#define PHYREG_CONTROL_AUTONEGEN    (1<<12)
#define PHYREG_CONTROL_POWERDOWN    (1<<11)
#define PHYREG_CONTROL_ISOLATE      (1<<10)
#define PHYREG_CONTROL_AUTORESTART  (1<<9)
#define PHYREG_CONTROL_DUPLEXFULL   (1<<8)
#define PHYREG_CONTROL_SPEEDMSB     (1<<6)

#define PHYREG_STATUS               1
#define PHYREG_STATUS_FD100         (1<<14)
#define PHYREG_STATUS_HD100         (1<<13)
#define PHYREG_STATUS_FD10          (1<<12)
#define PHYREG_STATUS_HD10          (1<<11)
#define PHYREG_STATUS_EXTSTATUS     (1<<8)
#define PHYREG_STATUS_NOPREAMBLE    (1<<6)
#define PHYREG_STATUS_AUTOCOMPLETE  (1<<5)
#define PHYREG_STATUS_REMOTEFAULT   (1<<4)
#define PHYREG_STATUS_AUTOCAPABLE   (1<<3)
#define PHYREG_STATUS_LINKSTATUS    (1<<2)
#define PHYREG_STATUS_JABBER        (1<<1)
#define PHYREG_STATUS_EXTENDED      (1<<0)

#define PHYREG_ID1                  2

#define PHYREG_ID2                  3

#define PHYREG_ADVERTISE            4
#define PHYREG_ADVERTISE_NEXTPAGE   (1<<15)
#define PHYREG_ADVERTISE_FAULT      (1<<13)
#define PHYREG_ADVERTISE_PAUSE      (1<<10)
#define PHYREG_ADVERTISE_FD100      (1<<8)
#define PHYREG_ADVERTISE_HD100      (1<<7)
#define PHYREG_ADVERTISE_FD10       (1<<6)
#define PHYREG_ADVERTISE_HD10       (1<<5)
#define PHYREG_ADVERTISE_MSGMASK    (0x1F)
#define PHYREG_ADVERTISE_MSG        (1)

#define PHYREG_PARTNER              5
#define PHYREG_PARTNER_NEXTPAGE     (1<<15)
#define PHYREG_PARTNER_ACK          (1<<14)
#define PHYREG_PARTNER_FAULT        (1<<13)
#define PHYREG_PARTNER_PAUSE        (1<<10)
#define PHYREG_PARTNER_FD100        (1<<8)
#define PHYREG_PARTNER_HD100        (1<<7)
#define PHYREG_PARTNER_FD10         (1<<6)
#define PHYREG_PARTNER_HD10         (1<<5)
#define PHYREG_PARTNER_MSGMASK      (0x1F)

#define PHYREG_1000CONTROL          9
#define PHYREG_ADVERTISE_FD1000     (1<<9)

#define PHYREG_1000STATUS           0xA
#define PHYREG_PARTNER_FD1000       (1<<11)

#define PHYREG_EXTSTATUS            0x0F
#define PHYREG_EXTSTATUS_FD1000     (1<<13)

#define PHYREG_SHADOW               0x18
#define PHYREG_SHADOW_EXTLOOPBACK       0x8400
#define PHYREG_SHADOW_RGMIIMODE         0xF080
#define PHYREG_SHADOW_INBAND            0xF1C7

#define PHYREG_ACCESS                           0x1C
#define PHYREG_ACCESS_COPPER            0xFC00


/*-----------------------------------------------------------------------*\
* PHY Control Register Macros
*
* These MACROS provide an easy way to read/write PHY registers
\*-----------------------------------------------------------------------*/
#define PHYREG_read(regadr, phyadr)                             \
            MDIO_REGS->USERACCESS0 =                            \
                    CSL_FMK(MDIO_USERACCESS0_GO,1u)         |   \
                    CSL_FMK(MDIO_USERACCESS0_REGADR,regadr) |   \
                    CSL_FMK(MDIO_USERACCESS0_PHYADR,phyadr)

#define PHYREG_write(regadr, phyadr, data)                      \
            MDIO_REGS->USERACCESS0 =                            \
                    CSL_FMK(MDIO_USERACCESS0_GO,1u)         |   \
                    CSL_FMK(MDIO_USERACCESS0_WRITE,1)       |   \
                    CSL_FMK(MDIO_USERACCESS0_REGADR,regadr) |   \
                    CSL_FMK(MDIO_USERACCESS0_PHYADR,phyadr) |   \
                    CSL_FMK(MDIO_USERACCESS0_DATA, data)

#define PHYREG_wait()                                           \
            while( CSL_FEXT(MDIO_REGS->USERACCESS0,MDIO_USERACCESS0_GO) )

#define PHYREG_waitResults( results ) {                                                \
            while( CSL_FEXT(MDIO_REGS->USERACCESS0,MDIO_USERACCESS0_GO) );             \
            results = CSL_FEXT(MDIO_REGS->USERACCESS0,MDIO_USERACCESS0_DATA); }

#define PHYREG_waitResultsAck( results, ack ) {                                        \
            while( CSL_FEXT(MDIO_REGS->USERACCESS0,MDIO_USERACCESS0_GO) );             \
            results = CSL_FEXT( MDIO_REGS->USERACCESS0,MDIO_USERACCESS0_DATA );            \
            ack = CSL_FEXT( MDIO_REGS->USERACCESS0, MDIO_USERACCESS0_ACK); }


/*-----------------------------------------------------------------------*\
* PHY State Machine
*
* When using auto-negotiation, the software must keep the MAC in
* sync with the PHY (for duplex). This module will also attempt to
* "auto-select" the PHY from a potential list of 32 based on which is
* first to get a link.
*
* On detection of a good link, the link speed and duplex settings will be
* used to update the EMAC configuration (done external to this module).
\*-----------------------------------------------------------------------*/

/* States in the PHY State Machine */
#define PHYSTATE_MDIOINIT       0
#define PHYSTATE_RESET          1
#define PHYSTATE_NWAYSTART      2
#define PHYSTATE_NWAYWAIT       3
#define PHYSTATE_LINKWAIT       4
#define PHYSTATE_LINKED         5

/*
// Tick counts for timeout of each state
// Note that NWAYSTART falls through to NWAYWAIT which falls through
// to LINKWAIT. The timeout is not reset progressing from one state
// to the next, so the system has 5 seconds total to find a link.
*/
static uint PhyStateTimeout[] = { 2,  /* PHYSTATE_MDIOINIT   - min-delay */
                                  6,  /* PHYSTATE_RESET      - 0.5 sec max */
                                  41, /* PHYSTATE_NWAYSTART  - 4 seconds */
                                  41, /* PHYSTATE_NWAYWAIT   - 4 seconds */
                                  51, /* PHYSTATE_LINKWAIT   - 5 seconds */
                                  0 };/* PHYSTATE_LINKED     - no timeout*/


typedef struct _MDIO_Device {
    uint            ModeFlags;      /* User specified configuration flags */
    uint            phyAddr;        /* Current (or next) PHY addr (0-31) */
    uint            phyState;       /* PHY State                         */
    uint            phyStateTicks;  /* Ticks elapsed in this PHY state   */
    uint            PendingStatus;  /* Pending Link Status               */
    uint            LinkStatus;     /* Link State PHYREG_STATUS_LINKSTATUS  */
} MDIO_Device;

static void MDIO_initStateMachine( MDIO_Device *pd );
static uint MDIO_initContinue( MDIO_Device *pd );

Uint32          macsel = 0;

/*-----------------------------------------------------------------------*\
* MDIO_initStateMachine()
*
* Internal function to initialize the state machine. It is referred to
* often in the code as it is called in case of a PHY error
\*-----------------------------------------------------------------------*/
static void MDIO_initStateMachine( MDIO_Device *pd )
{
    /* Setup the state machine defaults */
    pd->phyAddr       = 0;                  /* The next PHY to try  */
    pd->phyState      = PHYSTATE_MDIOINIT;  /* PHY State            */
    pd->phyStateTicks = 0;                  /* Ticks elapsed        */
    pd->LinkStatus    = MDIO_LINKSTATUS_NOLINK;
}

/*-----------------------------------------------------------------------*\
* MDIO_open()
*
* Opens the MDIO peripheral and start searching for a PHY device.
*
* It is assumed that the MDIO module is reset prior to calling this
* function.
\*-----------------------------------------------------------------------*/
Handle MDIO_open( uint mdioModeFlags )
{
    /*
    // Note: In a multi-instance environment, we'd have to allocate "localDev"
    */
    static MDIO_Device localDev;

        /* Find out what interface we are working with */
        macsel = CSL_FEXT(DEV_REGS->DEVSTAT, DEV_DEVSTAT_MACSEL);

    /* Get the mode flags from the user - clear our reserved flag */
    localDev.ModeFlags = mdioModeFlags & ~MDIO_MODEFLG_NWAYACTIVE;

    /* Setup the MDIO state machine */
    MDIO_initStateMachine( &localDev );

    /* Enable MDIO and setup divider */
    MDIO_REGS->CONTROL = CSL_FMKT(MDIO_CONTROL_ENABLE,YES) |
                         CSL_FMK(MDIO_CONTROL_CLKDIV,VBUSCLK) ;

    /* We're done for now - all the rest is done via MDIO_event() */
    return( &localDev );
}


/*-----------------------------------------------------------------------*\
* MDIO_close()
*
* Close the  MDIO peripheral and disable further operation.
\*-----------------------------------------------------------------------*/
void MDIO_close( Handle hMDIO )
{
    Uint32         ltmp1;
    uint           i;

    (void)hMDIO;

    /*
    // We really don't care what state anything is in at this point,
    // but to be safe, we'll isolate all the PHY devices.
    */
    ltmp1 = MDIO_REGS->ALIVE;
    for( i=0; ltmp1; i++,ltmp1>>=1 )
    {
        if( ltmp1 & 1 )
        {
            PHYREG_write( PHYREG_CONTROL, i, PHYREG_CONTROL_ISOLATE |
                                             PHYREG_CONTROL_POWERDOWN );
            PHYREG_wait();
        }
    }
}


/*-----------------------------------------------------------------------*\
* MDIO_getStatus()
*
* Called to get the status of the MDIO/PHY
\*-----------------------------------------------------------------------*/
void MDIO_getStatus( Handle hMDIO, uint *pPhy, uint *pLinkStatus )
{
    MDIO_Device *pd = (MDIO_Device *)hMDIO;

    if( pPhy )
        *pPhy = pd->phyAddr;
    if( pLinkStatus )
        *pLinkStatus = pd->LinkStatus;
}


/*-----------------------------------------------------------------------*\
* MDIO_timerTick()
*
* Called to signify that approx 100mS have elapsed
*
* Returns an MDIO event code (see MDIO Events in C6455_MDIO.H).
\*-----------------------------------------------------------------------*/
uint MDIO_timerTick( Handle hMDIO )
{
    MDIO_Device *pd = (MDIO_Device *)hMDIO;
    Uint16      tmp1,tmp2,tmp1gig = 0, tmp2gig = 0, ack;
    Uint32      ltmp1 = 0;
    uint        RetVal = MDIO_EVENT_NOCHANGE;

    /*
    // If we are linked, we just check to see if we lost link. Otherwise;
    // we keep treking through our state machine.
    */
    if( pd->phyState == PHYSTATE_LINKED )
    {
        /*
        // Here we check for a "link-change" status indication or a link
        // down indication.
        */
        ltmp1 = MDIO_REGS->LINKINTRAW & 1;
        MDIO_REGS->LINKINTRAW = ltmp1;
        if( ltmp1 || !(MDIO_REGS->LINK)&(1<<pd->phyAddr) )
        {
            /*
            // There has been a change in link (or it is down)
            // If we do not auto-neg, then we just wait for a new link
            // Otherwise, we enter NWAYSTART or NWAYWAIT
            */

            pd->LinkStatus = MDIO_LINKSTATUS_NOLINK;
            RetVal = MDIO_EVENT_LINKDOWN;
            pd->phyStateTicks = 0;  /* Reset timeout */

            /* If not NWAY, just wait for link */
            if( !(pd->ModeFlags & MDIO_MODEFLG_NWAYACTIVE) )
                pd->phyState = PHYSTATE_LINKWAIT;
            else
            {
                /* Handle NWAY condition */

                /* First see if link is really down */
                PHYREG_read( PHYREG_STATUS, pd->phyAddr );
                PHYREG_wait();
                PHYREG_read( PHYREG_STATUS, pd->phyAddr );
                PHYREG_waitResultsAck( tmp1, ack );
                if( !ack )
                {
                    /* No PHY response, maybe it was unplugged */
                    MDIO_initStateMachine( pd );
                }
                else if( !(tmp1 & PHYREG_STATUS_LINKSTATUS) )
                {
                    /* No Link - restart NWAY */
                    pd->phyState = PHYSTATE_NWAYSTART;

                    PHYREG_write( PHYREG_CONTROL, pd->phyAddr,
                                  PHYREG_CONTROL_AUTONEGEN |
                                  PHYREG_CONTROL_AUTORESTART );
                    PHYREG_wait();
                }
                else
                {
                    /* We have a Link - re-read NWAY params  */
                    pd->phyState = PHYSTATE_NWAYWAIT;
                }
            }
        }
    }

    /*
    // If running in a non-linked state, execute the next
    // state of the state machine.
    */
    if( pd->phyState != PHYSTATE_LINKED )
    {
        /* Bump the time counter */
        pd->phyStateTicks++;

        /* Process differently based on state */
        switch( pd->phyState )
        {
        case PHYSTATE_RESET:
            /* Don't try to read reset status for the first 100 to 200 ms */
            if( pd->phyStateTicks < 2 )
                break;

            /* See if the PHY has come out of reset */
            PHYREG_read( PHYREG_CONTROL, pd->phyAddr );
            PHYREG_waitResultsAck( tmp1, ack );
            if( ack && !(tmp1 & PHYREG_CONTROL_RESET) )
            {
                /* PHY is not reset. If the PHY init is going well, break out */
                if( MDIO_initContinue( pd ) )
                    break;
                /* Else, this PHY is toast. Manually trigger a timeout */
                pd->phyStateTicks = PhyStateTimeout[pd->phyState];
            }

            /* Fall through to timeout check */

        case PHYSTATE_MDIOINIT:
CheckTimeout:
            /* Here we just check timeout and try to find a PHY */
            if( pd->phyStateTicks >= PhyStateTimeout[pd->phyState] )
            {
                // Try the next PHY if anything but a MDIOINIT condition
                if( pd->phyState != PHYSTATE_MDIOINIT )
                    if( ++pd->phyAddr == 32 )
                        pd->phyAddr = 0;
                ltmp1 = MDIO_REGS->ALIVE;
                for( tmp1=0; tmp1<32; tmp1++ )
                {
                    if( ltmp1 & (1<<pd->phyAddr) )
                    {
                        if( MDIO_initPHY( pd, pd->phyAddr ) )
                            break;
                    }

                    if( ++pd->phyAddr == 32 )
                        pd->phyAddr = 0;
                }

                // If we didn't find a PHY, try again
                if( tmp1 == 32 )
                {
                    pd->phyAddr       = 0;
                    pd->phyState      = PHYSTATE_MDIOINIT;
                    pd->phyStateTicks = 0;
                    RetVal = MDIO_EVENT_PHYERROR;
                }
            }
            break;

        case PHYSTATE_NWAYSTART:
            /*
            // Here we started NWAY. We check to see if NWAY is done.
            // If not done and timeout occured, we find another PHY.
            */

            /* Read the CONTROL reg to verify "restart" is not set */
            PHYREG_read( PHYREG_CONTROL, pd->phyAddr );
            PHYREG_waitResultsAck( tmp1, ack );
            if( !ack )
            {
                MDIO_initStateMachine( pd );
                break;
            }
            if( tmp1 & PHYREG_CONTROL_AUTORESTART )
                goto CheckTimeout;

            /* Flush latched "link status" from the STATUS reg */
            PHYREG_read( PHYREG_STATUS, pd->phyAddr );
            PHYREG_wait();

            pd->phyState = PHYSTATE_NWAYWAIT;

            /* Fallthrough */

        case PHYSTATE_NWAYWAIT:
            /*
            // Here we are waiting for NWAY to complete.
            */

            /* Read the STATUS reg to check for "complete" */
            PHYREG_read( PHYREG_STATUS, pd->phyAddr );
            PHYREG_waitResultsAck( tmp1, ack );
            if( !ack )
            {
                MDIO_initStateMachine( pd );
                break;
            }
            if( !(tmp1 & PHYREG_STATUS_AUTOCOMPLETE) )
                goto CheckTimeout;

            /* We can now check the negotiation results */

                        if ( (macsel == CSL_DEV_DEVSTAT_MACSEL_GMII) || (macsel == CSL_DEV_DEVSTAT_MACSEL_RGMII) )
                        {
                                PHYREG_read( PHYREG_1000CONTROL, pd->phyAddr );
                                PHYREG_waitResults( tmp1gig );
                                PHYREG_read( PHYREG_1000STATUS, pd->phyAddr );
                                PHYREG_waitResults( tmp2gig );
                        }

            PHYREG_read( PHYREG_ADVERTISE, pd->phyAddr );
            PHYREG_waitResults( tmp1 );
            PHYREG_read( PHYREG_PARTNER, pd->phyAddr );
            PHYREG_waitResults( tmp2 );
            /*
            // Use the "best" results
            */
            tmp2 &= tmp1;

            /* Check first for 1 Gigabit */
                        if( (tmp1gig & PHYREG_ADVERTISE_FD1000) && (tmp2gig & PHYREG_PARTNER_FD1000) )
                                pd->PendingStatus = MDIO_LINKSTATUS_FD1000;
            else if( tmp2 & PHYREG_ADVERTISE_FD100 )
                pd->PendingStatus = MDIO_LINKSTATUS_FD100;
            else if( tmp2 & PHYREG_ADVERTISE_HD100 )
                pd->PendingStatus = MDIO_LINKSTATUS_HD100;
            else if( tmp2 & PHYREG_ADVERTISE_FD10 )
                pd->PendingStatus = MDIO_LINKSTATUS_FD10;
            else if( tmp2 & PHYREG_ADVERTISE_HD10 )
                pd->PendingStatus = MDIO_LINKSTATUS_HD10;
            /*
            // If we get here the negotiation failed
            // We just use HD 100 or 10 - the best we think we can do
            */
            else if( tmp1 & PHYREG_ADVERTISE_HD100 )
                pd->PendingStatus = MDIO_LINKSTATUS_HD100;
            else
                pd->PendingStatus = MDIO_LINKSTATUS_HD10;

            pd->phyState = PHYSTATE_LINKWAIT;

            /* Fallthrough */

        case PHYSTATE_LINKWAIT:
            /*
            // Here we are waiting for LINK
            */

            /* Read the STATUS reg to check for "link" */
            PHYREG_read( PHYREG_STATUS, pd->phyAddr );
            PHYREG_waitResultsAck( tmp1, ack );
            if( !ack )
            {
                MDIO_initStateMachine( pd );
                break;
            }
            if( !(tmp1 & PHYREG_STATUS_LINKSTATUS) )
                goto CheckTimeout;

            /* Make sure we're linked in the MDIO module as well */
            ltmp1 = MDIO_REGS->LINK;
            if( !(ltmp1&(1<<pd->phyAddr)) )
                goto CheckTimeout;

            /* Start monitoring this PHY */
            MDIO_REGS->USERPHYSEL0 = pd->phyAddr;

            /* Clear the link change flag so we can detect a "re-link" later */
            MDIO_REGS->LINKINTRAW = 1;

            /* Setup our linked state */
            pd->phyState   = PHYSTATE_LINKED;
            pd->LinkStatus = pd->PendingStatus;
            RetVal = MDIO_EVENT_LINKUP;

            break;
        }
    }

    return( RetVal );
}


/*-----------------------------------------------------------------------*\
* MDIO_initPHY()
*
* Force a switch to the specified PHY, and start the negotiation process.
*
* Returns 1 if the PHY selection completed OK, else 0
\*-----------------------------------------------------------------------*/
uint MDIO_initPHY( Handle hMDIO, volatile uint phyAddr )
{
    MDIO_Device *pd = (MDIO_Device *)hMDIO;
    Uint32         ltmp1;
    uint           i,ack;

    /* Switch the PHY */
    pd->phyAddr = phyAddr;

    /* There will be no link when we're done with this PHY */
    pd->LinkStatus = MDIO_LINKSTATUS_NOLINK;

    /* Shutdown all other PHYs */
    ltmp1 = MDIO_REGS->ALIVE ;
    for( i=0; ltmp1; i++,ltmp1>>=1 )
    {
        if( (ltmp1 & 1) && (i != phyAddr) )
        {
            PHYREG_write( PHYREG_CONTROL, i, PHYREG_CONTROL_ISOLATE |
                                             PHYREG_CONTROL_POWERDOWN );
            PHYREG_wait();
        }
    }

    /* Reset the PHY we plan to use */
    PHYREG_write( PHYREG_CONTROL, phyAddr, PHYREG_CONTROL_RESET );
    PHYREG_waitResultsAck( i, ack );

    /* If the PHY did not ACK the write, return zero */
    if( !ack )
        return(0);

        /* Settings for Broadcom phys */

        if ( macsel == CSL_DEV_DEVSTAT_MACSEL_RGMII )
        {
                //Put phy in copper mode
                PHYREG_write( PHYREG_ACCESS, phyAddr, PHYREG_ACCESS_COPPER );
                PHYREG_waitResultsAck( i, ack );

                /* If the PHY did not ACK the write, return zero */
                if( !ack )
                        return(0);

                PHYREG_write( 0x10, phyAddr, 0x0000 );  //GMII Interface
                PHYREG_wait();

                // Put phy in RGMII mode/in-band status data for PG 2.0
                if (EMAC_REGS->TXIDVER != 0x000C1207) {
                        PHYREG_write(PHYREG_SHADOW, phyAddr, PHYREG_SHADOW_INBAND);
                        PHYREG_waitResultsAck( i, ack );

                        /* If the PHY did not ACK the write, return zero */
                        if( !ack )
                                return(0);

                }
        }

        if ( macsel == CSL_DEV_DEVSTAT_MACSEL_GMII )
        {
                //Put phy in copper mode
        PHYREG_write( PHYREG_ACCESS, phyAddr, PHYREG_ACCESS_COPPER );
        PHYREG_wait();

                /* If the PHY did not ACK the write, return zero */
                if( !ack )
                        return(0);
        }

    /* Setup for our next state */
    pd->phyState = PHYSTATE_RESET;
    pd->phyStateTicks = 0;  /* Reset timeout */

    return(1);
}


/*-----------------------------------------------------------------------*\
* MDIO_initContinue()
*
* Continues the initialization process started in MDIO_initPHY()
*
* Returns 0 on an error, 1 on success
\*-----------------------------------------------------------------------*/
static uint MDIO_initContinue( MDIO_Device *pd )
{
    Uint16              tmp1,tmp2;
    Uint16              tmp1gig = 0;

    /* Read the STATUS reg to check autonegotiation capability */
    PHYREG_read( PHYREG_STATUS, pd->phyAddr );
    PHYREG_waitResults( tmp1 );

        if ( (macsel == CSL_DEV_DEVSTAT_MACSEL_GMII) || (macsel == CSL_DEV_DEVSTAT_MACSEL_RGMII) )
        {
                PHYREG_read( PHYREG_EXTSTATUS, pd->phyAddr );
                PHYREG_waitResults( tmp1gig );
        }

    /* See if we auto-neg or not */
    if( (pd->ModeFlags & MDIO_MODEFLG_AUTONEG) &&
                                     (tmp1 & PHYREG_STATUS_AUTOCAPABLE) )
    {
        /* We will use NWAY */

                /* Advertise 1000 for supported interfaces */
                if ( (macsel == CSL_DEV_DEVSTAT_MACSEL_GMII) || (macsel == CSL_DEV_DEVSTAT_MACSEL_RGMII) )
                {
                        tmp1gig >>= 4;
                        tmp1gig &= PHYREG_ADVERTISE_FD1000;

                        PHYREG_write( PHYREG_1000CONTROL, pd->phyAddr, tmp1gig );
                }

        /* Shift down the capability bits */
        tmp1 >>= 6;

        /* Mask with the capabilities */
        tmp1 &= ( PHYREG_ADVERTISE_FD100 | PHYREG_ADVERTISE_HD100 |
                  PHYREG_ADVERTISE_FD10 | PHYREG_ADVERTISE_HD10 );

        /* Set Ethernet message bit */
        tmp1 |= PHYREG_ADVERTISE_MSG;

        /* Write out advertisement */
        PHYREG_write( PHYREG_ADVERTISE, pd->phyAddr, tmp1 );
        PHYREG_wait();

        /* Start NWAY */
        PHYREG_write( PHYREG_CONTROL, pd->phyAddr, PHYREG_CONTROL_AUTONEGEN );
        PHYREG_wait();

        PHYREG_write( PHYREG_CONTROL, pd->phyAddr,
                      PHYREG_CONTROL_AUTONEGEN|PHYREG_CONTROL_AUTORESTART );
        PHYREG_wait();

        /* Setup current state */
        pd->ModeFlags |= MDIO_MODEFLG_NWAYACTIVE;
        pd->phyState = PHYSTATE_NWAYSTART;
        pd->phyStateTicks = 0;  /* Reset timeout */
    }
    else
    {
        /* We will use a fixed configuration */

        /* Shift down the capability bits */
        tmp1 >>= 10;

        /* Mask with possible modes */
        tmp1 &= ( MDIO_MODEFLG_HD10 | MDIO_MODEFLG_FD10 |
                  MDIO_MODEFLG_HD100 | MDIO_MODEFLG_FD100 );

        if ( (macsel == CSL_DEV_DEVSTAT_MACSEL_GMII) || (macsel == CSL_DEV_DEVSTAT_MACSEL_RGMII) )
        {
                        tmp1gig >>= 8;
                        tmp1gig&= MDIO_MODEFLG_FD1000;

                        /* Mask with what the User wants to allow */
                        tmp1gig &= pd->ModeFlags;

                }

        /* Mask with what the User wants to allow */
        tmp1 &= pd->ModeFlags;

        /* If nothing if left, move on */
        if( (!tmp1) && (!tmp1gig) )
            return(0);

        /* Setup Control word and pending status */
        if( tmp1gig ) {
                                tmp2 = PHYREG_CONTROL_SPEEDMSB | PHYREG_CONTROL_DUPLEXFULL;
                                pd->PendingStatus = MDIO_LINKSTATUS_FD1000;
                }
        else if( tmp1 & MDIO_MODEFLG_FD100 )
        {
            tmp2 = PHYREG_CONTROL_SPEEDLSB | PHYREG_CONTROL_DUPLEXFULL;
            pd->PendingStatus = MDIO_LINKSTATUS_FD100;
        }
        else if( tmp1 & MDIO_MODEFLG_HD100 )
        {
            tmp2 = PHYREG_CONTROL_SPEEDLSB;
            pd->PendingStatus = MDIO_LINKSTATUS_HD100;
        }
        else if( tmp1 & MDIO_MODEFLG_FD10 )
        {
            tmp2 = PHYREG_CONTROL_DUPLEXFULL;
            pd->PendingStatus = MDIO_LINKSTATUS_FD10;
        }
        else
        {
            tmp2 = 0;
            pd->PendingStatus = MDIO_LINKSTATUS_HD10;
        }

        /* Add in internal phy loopback if user wanted it */
        if( pd->ModeFlags & MDIO_MODEFLG_LOOPBACK )
            tmp2 |= PHYREG_CONTROL_LOOPBACK;

        /* Configure PHY */
        PHYREG_write( PHYREG_CONTROL, pd->phyAddr, tmp2 );
        PHYREG_wait();

        /* Add in external phy loopback with plug if user wanted it */
        if( pd->ModeFlags & MDIO_MODEFLG_EXTLOOPBACK ) {
                        PHYREG_write( PHYREG_SHADOW, pd->phyAddr, PHYREG_SHADOW_EXTLOOPBACK );
                        PHYREG_wait();
                }

        /* Setup current state */
        pd->ModeFlags &= ~MDIO_MODEFLG_NWAYACTIVE;
        pd->phyState = PHYSTATE_LINKWAIT;
        pd->phyStateTicks = 0;  /* Reset timeout */
    }

    return(1);
}


/*-----------------------------------------------------------------------*\
* MDIO_phyRegRead()
*
* Raw data read of a PHY register.
*
* Returns 1 if the PHY ACK'd the read, else 0
\*-----------------------------------------------------------------------*/
uint MDIO_phyRegRead( volatile uint phyIdx, volatile uint phyReg, Uint16 *pData )
{
    uint data,ack;

    PHYREG_read( phyReg, phyIdx );
    PHYREG_waitResultsAck( data, ack );
    if( !ack )
        return(0);
    if( pData )
        *pData = data;
    return(1);
}


/*-----------------------------------------------------------------------*\
* MDIO_phyRegWrite()
*
* Raw data write of a PHY register.
*
* Returns 1 if the PHY ACK'd the write, else 0
\*-----------------------------------------------------------------------*/
uint MDIO_phyRegWrite( volatile uint phyIdx, volatile uint phyReg, Uint16 data )
{
    uint ack;

    PHYREG_write( phyReg, phyIdx, data );
    PHYREG_waitResultsAck( data, ack );
    if( !ack )
        return(0);
    return(1);
}

/******************************************************************************\
* End of c6455_mdio.c
\******************************************************************************/

