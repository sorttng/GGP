/*
 *  Copyright 2007 by Texas Instruments Incorporated.
 *  All rights reserved. Property of Texas Instruments Incorporated.
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 *
 *  @(#) TCP/IP_Network_Developers_Kit 1.92.00.22 01-10-2007 (ndk-b22)
 */
/*H***************************************************************************
*
* DESCRIPTION: DSK6455 board initializion implementation.
*
* NOTES :  BSL Version# 3.24
*
* (C) Copyright 2005 by Spectrum Digital Incorporated
* All rights reserved
*
*H***************************************************************************/

#include "dsk6455.h"

/* Initialize the board */
void DSK6455_init()
{
    volatile Uint32 test;
        volatile Uint32 *pcfg;

    /* This section is coded such that the next two statements will execute in */
        /* under 16 SYSCLK3 cycles */
        pcfg = (volatile Uint32 *)0x2ac0004;

    /* Unlock PERCFG0 through PERLOCK */
        *pcfg++ = 0x0f0a0b00;

    /* Enable EMAC, Timers, McBSPs, I2C, GPIO in PERCFG0 */
    *pcfg = 0xC0015555;

    /* Read back PERSTAT0 */
    test = *((Uint32 *)0x02AC0014);

    /* Wait at least 128 CPU cycles */
    DSK6455_wait(128);

        DSK6455_I2C_init();
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

