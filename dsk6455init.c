/*
 *  Copyright 2007 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) TCP/IP_Network_Developers_Kit 1.92.00.22 01-10-2007 (ndk-b22)
 */
//--------------------------------------------------------------------------
// IP Stack Client Demo
//--------------------------------------------------------------------------
// dsk6455init.c
//
// Private initialization functions for DSK6455
//
// Author: Michael A. Denio
//                 Magdalena B. Iovescu
//
// Copyright 2006 by Texas Instruments Inc.
//--------------------------------------------------------------------------
#include <stdio.h>
#include <netmain.h>
#include "dsk6455.h"
#include "macro.h"
#include "videosrv.h"
// MDIO phy access functions
uint MDIO_phyRegWrite( uint phyIdx, uint phyReg, Uint16 data );
uint MDIO_phyRegRead( uint, uint, UINT16 * );

// This string array corresponds to link state as defined in c6455_mdio.h
static char *LinkStr[] = { "No Link",
                           "10Mb/s Half Duplex",
                           "10Mb/s Full Duplex",
                           "100Mb/s Half Duplex",
                           "100Mb/s Full Duplex",
                           "1000Mb/s Full Duplex" };
/*********************************/
// Main Thread
// We changed our TCF file to point call this private init
// function. Here we initialize our board and read in our
// MAC address.
/*********************************/
/* Initialize the board */
void DSK6455_init()
{
	int i;
    int PLLM_val =    20; 
    int PREDIV_val =  1;
    int PLLDIV4_val = 6;//5;//8;//
//    int PLLDIV5_val = 4;
	CSR &=~(0x1);      
	/* In PLLCTL, write PLLENSRC = 0 (enable PLLEN bit).*/
	PLLCTL_1 &= ~(0x00000020);
	//*(int *)PLLCTL_1 =0x48;
	//*(int *)0x029A0100=0x48;
	/* In PLLCTL, write PLLEN = 0 (bypass mode).*/
	PLLCTL_1 &= ~(0x00000001);
	//*(int *)0x029A0100 &= ~(0x00000001);
   // *(int *)PLLCTL_1 =0x0000000b;
	/* Wait 4 cycles of the slowest of PLLOUT or reference clock source (CLKIN).*/
	for (i=0 ; i<100*20 ; i++);
	/*In PLLCTL, write PLLRST = 1 (PLL is reset).*/
    PLLCTL_1 |= 0x00000008;
	//*(int *)0x029A0100 |= 0x00000008;
   // *(int *)PLLCTL_1 = 0x00000009;
	/*If necessary, program PREDIV and PLLM.*/
	PLLM_1 = PLLM_val - 1;
	//*(int *)0x029A0110 = PLLM_val - 1;
	PREDIV_1 = (PREDIV_val - 1) | 0x8000;	/* set PLLDIV0 */
	//*(int *)0x029A0114 = (PREDIV_val - 1) | 0x8000;	/* set PLLDIV0 */
	/*If necessary, program PLLDIV1n. Note that you must apply the GO operation
		to change these dividers to new ratios.*/

		/* Check that the GOSTAT bit in PLLSTAT is cleared to show that no GO 
			operation is currently in progress.*/
		while( (PLLSTAT_1) & 0x00000001);
	//	while( (*(int *)0x029A013C) & 0x00000001);
	/* Program the RATIO field in PLLDIVn to the desired new divide-down rate. 
			If the RATIO field changed, the PLL controller will flag the change
			in the corresponding bit of DCHANGE.*/
		PLLDIV4_1 = (PLLDIV4_val - 1) | 0x8000;	/* set PLLDIV4 */
		//*(int *)0x029A0160 = (PLLDIV4_val - 1) | 0x8000;	/* set PLLDIV4 */
		/* Set the GOSET bit in PLLCMD to initiate the GO operation to change
			the divide values and align the SYSCLKs as programmed.*/
		PLLCMD_1 |= 0x00000001;
		//*(int *)0x029A0138 |= 0x00000001;
		/* Read the GOSTAT bit in PLLSTAT to make sure the bit returns to 0
			to indicate that the GO operation has completed.*/
		while( (PLLSTAT_1) & 0x00000001);
		//	while( (*(int *)0x029A013C) & 0x00000001);
	/* Wait for PLL to properly reset.(128 CLKIN1 cycles).*/
	for (i=0 ; i<1000*20 ; i++);
	/* In PLLCTL, write PLLRST = 0 to bring PLL out of reset.*/
	//*(int *)PLLCTL_1 &= ~(0x00000008);
	*(int *)0x029A0100 &= ~(0x00000008);
	/* Wait for PLL to lock (2000 CLKIN1 cycles). */
	for (i=0 ; i<4000*20 ; i++);
	/* In PLLCTL, write PLLEN = 1 to enable PLL mode. */
	PLLCTL_1 |= (0x00000001);
	//*(int *)0x029A0100 |= (0x00000001);
//	ISTP = 0x00900000;	//interruput vector table location
//	INTMUX1 = 0x33;//0x18; 	//*INTMUX1[6:0]=011000	INT4 map to edma3cc_gint*/
//	IER |= 0x0013;	/*enable interrupt: reset, NMI,INT4*/
//	IER |= 0x0083;
	PERLOCK = 0x0f0a0b00; /* Unlock PERCFG0 through PERLOCK */
//	PERCFG0 = 0xC0015555; /* Enable EMAC, Timers, McBSPs, I2C, GPIO in PERCFG0 */
	PERCFG0 = 0x00001550;
	PERCFG1 = 0x3; /* Enable DDR and EMIFA in PERCFG1 */

	GP_DIR &= 0xffffffeb;
//	GP_OUT_DATA &= 0xffffffef; 
//    GP_SET_DATA |= 0x10;
    GP_CLR_DATA  = 0x14;
	for (i=0 ; i<1000 ; i++);
//	GP_OUT_DATA |= 0x10;
	GP_SET_DATA = 0x14;

	GP_BINTEN |= 0x1 ;	//enable gpio as souce of interruput
	GP_SET_RIS_TRIG |= 0x1 ;	//the rising edge of gpio trige the interruput or event 
//    GP_SET_RIS_TRIG |= 0xff ; 	
	EMIFA_STAT &= ~(0x80000000) ;	//config emifa work in little endian mode 

	//EMIFA_CE2CFG = 0x00240120; /* 8-bit async, 10 cycle read/write strobe */0x8000030C
    EMIFA_CE2CFG = 0x8000030E;
	EMIFA_CE3CFG = 0x00240120; /* 8-bit async, 10 cycle read/write strobe */
//    EMIFA_CE2CFG = 0x8000030A;//sync mode read letance = 2,write letance = 0 32bits wide//0x00240120; /* 8-bit async, 10 cycle read/write strobe */
//    EMIFA_CE3CFG = 0x00240120; /* 8-bit async, 10 cycle read/write strobe */
//   EMIFA_CE4CFG = 0x8000030C;//0x80000308;//sync mode read letance = 3,write letance = 0 8bits wides//0x00240122; /* 32-bit async, 10 cycle read/write strobe */
//   EMIFA_CE4CFG = 0x8000030E;//32
    EMIFA_CE4CFG = 0x8000030d;//16
    //EMIFA_CE5CFG = 0x8000030A;//sync mode read letance = 2,write letance = 0 32bits wide//0x00240122; /* 32-bit async, 10 cycle read/write strobe */
    EMIFA_CE5CFG = 0x8000030E;//sync mode read letance = 3,write letance = 0 32bits wide//0x00240122; /* 32-bit async, 10 cycle read/write strobe */
    EMIFA_BPRIO  = 0x000000FE; /* Enable priority based starvation control SPRU971A sec. 7.2 */
	//EMIFA_CONFIGREG_CE4CFG |= 0x8000010b ;	//emifa works in sysnc mode; read letance =2; 64bits wide
	/* Configure DDR for 500MHz operation (sequence is order dependent) */
    DDR_SDCFG    = 0x00D38832; /* Unlock boot + timing, CAS4, 8 banks, 10 bit column */
    DDR_SDRFC    = 0x000007A2; /* Refresh */
    DDR_SDTIM1   = 0x3EDB4B91; /* Timing 1 */
    DDR_SDRIM2   = 0x00A2C722; /* Timing 2 */
    DDR_DDRPHYC   = 0x00000005; /* PHY read latency for CAS 4 is 4 + 2 - 1 */
    DDR_SDCFG    = 0x00538832; /* Lock, CAS4, 8 banks, 10 bit column, lock timing */
    /* Wait at least 128 CPU cycles */
    DSK6455_wait(128*10);
    DSK6455_I2C_init();
   // CSR	|=	0x1;	//enable intertuput
}
/* Spin in a delay loop for delay iterations */
void DSK6455_wait(Uint32 delay)
{
    volatile Uint32 i, n;
    n = 0;
    for (i = 0; i < delay; i++)
    {
        n = n + 1;
    }
}
/* Spin in a delay loop for delay microseconds */
void DSK6455_waitusec(Uint32 delay)
{
    DSK6455_wait(delay * 28);
}
/* ------------------------------------------------------------------------ *
 *                                                                          *
 *  DSK6455_I2C_init( )                                                  *
 *                                                                          *
 *      Enable and initalize the I2C module                                 *
 *                                                                          *
 * ------------------------------------------------------------------------ */
Int16 DSK6455_I2C_init( )
{
    I2C_ICMDR   = 0;                                // Reset I2C
    I2C_ICPSC   = 125;                              // Run I2C module at 1MHz (input SYSCLK2/6)
    I2C_ICCLKL  = 5;                                // Config clk LOW for 50kHz
    I2C_ICCLKH  = 5;                                // Config clk HIGH for 50kHz
    I2C_ICMDR  |= ICMDR_IRS;                        // Release I2C from reset
    return 0;
}
/* ------------------------------------------------------------------------ *
 *                                                                          *
 *  DSK6455_I2C_write( i2caddr, data, len )                              *
 *                                                                          *
 *      I2C write in Master mode                                            *
 *                                                                          *
 *      i2caddr <- I2C slave address                                        *
 *      data    <- I2C data ptr                                             *
 *      len     <- # of bytes to write                                      *
 *                                                                          *
 * ------------------------------------------------------------------------ */
Int16 DSK6455_I2C_write( Uint16 i2caddr, Uint8* data, Uint16 len )
{
    Uint16 i;
    I2C_ICCNT = len;                                // Set len
    I2C_ICSAR = i2caddr;                            // Set I2C slave address
    I2C_ICMDR = ICMDR_STT                           // Config for master write
              | ICMDR_TRX
              | ICMDR_MST
              | ICMDR_IRS
              | ICMDR_FREE
              ;
    DSK6455_wait( 10 );                                  // Short delay
    for ( i = 0 ; i < len ; i++ )
    {
        I2C_ICDXR = data[i];                        // Write
        while ( ( I2C_ICSTR & ICSTR_ICXRDY ) == 0 );// Wait for Tx Ready
    }
    I2C_ICMDR |= ICMDR_STP;                         // Generate STOP
    return 0;
}
/* ------------------------------------------------------------------------ *
 *                                                                          *
 *  DSK6455_I2C_read( i2caddr, data, len )                               *
 *                                                                          *
 *      I2C read in Master mode                                             *
 *                                                                          *
 *      i2caddr <- I2C slave address                                        *
 *      data    <- I2C data ptr                                             *
 *      len     <- # of bytes to write                                      *
 *                                                                          *
 * ------------------------------------------------------------------------ */
Int16 DSK6455_I2C_read( Uint16 i2caddr, Uint8* data, Uint16 len )
{
    Uint16 i;

    I2C_ICCNT = len;                                // Set len
    I2C_ICSAR = i2caddr;                            // Set I2C slave address
    I2C_ICMDR = ICMDR_STT                           // Config for master read
              | ICMDR_MST
              | ICMDR_IRS
              | ICMDR_FREE
              ;
    for ( i = 0 ; i < len ; i++ )
    {
        while ( ( I2C_ICSTR & ICSTR_ICRRDY ) == 0 );// Wait for Rx Ready
        data[i] = I2C_ICDRR;                        // Read
    }
    I2C_ICMDR |= ICMDR_STP;                         // Generate STOP
    return 0;
}
/* ------------------------------------------------------------------------ *
 *                                                                          *
 *  DSK6455_EEPROM_read( src, dst, length )                              *
 *                                                                          *
 *      Read from the I2C EEPROM                                            *
 *                                                                          *
 * ------------------------------------------------------------------------ */
Int16 DSK6455_EEPROM_read( Uint32 src, Uint32 dst, Uint32 length )
{
    Int16 retcode = 0;
    Uint8 addr[2];
    addr[0] = src >> 8;         // HIGH address
    addr[1] = src & 0xFF;       // LOW address
    /* Send 16-bit address */
    retcode |= DSK6455_I2C_write( EEPROM_I2C_ADDR, addr, 2 );
    DSK6455_waitusec( 10000 );
    /* Read data */
    retcode |= DSK6455_I2C_read ( EEPROM_I2C_ADDR, ( Uint8* )dst, length );
    /* Need to wait at least 10ms */
    DSK6455_waitusec(10000);
    return retcode;
}
/*********************************/
void c6455_init()
{
    DSK6455_init();
}
/*********************************/
// C6455EMAC_getConfig()
// This is a callback from the Ethernet driver. This function
// is used by the driver to get its 6 byte MAC address, and
// to determine which DSP interrupt the EMAC should be mapped to.
/*********************************/
void C6455EMAC_getConfig( UINT8 *pMacAddr, uint *pIntVector )
{
    printf("Using MAC Address: %02x-%02x-%02x-%02x-%02x-%02x\n",
            bMacAddr[0], bMacAddr[1], bMacAddr[2],
            bMacAddr[3], bMacAddr[4], bMacAddr[5]);
    // We fill in the two pointers here. We'll use int 5 for EMAC
    mmCopy( pMacAddr, bMacAddr, 6 );
    *pIntVector = 5;
	//*pIntVector = 4;
}
/*********************************/
// C6455EMAC_linkStatus()
// This is a callback from the Ethernet driver. This function
// is called whenever there is a change in link state. The
// current PHY and current link state are passed as parameters.
/*********************************/
void C6455EMAC_linkStatus( uint phy, uint linkStatus )
{
    printf("Link Status: %s on PHY %d\n",LinkStr[linkStatus],phy);
}
/*********************************/

