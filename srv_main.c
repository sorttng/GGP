//--------------------------------------------------------------------------
#include <stdio.h>
#include <netmain.h>
#include <_stack.h>
#include "servers.h"
#include "client.h"
//#include "macro.h"
#include "videosrv.h"
#include <csl.h>
#include <csl_cache.h>
//---------------------------------------------------------------------------
// Title String
char *VerStr = "\nTCP/IP Stack Example Client\n";
extern unsigned short   first_fifo_1[1280];
static void   NetworkOpen();
static void   NetworkClose();
static void   NetworkIPAddr( IPN IPAddr, uint IfIdx, uint fAdd );
extern Uint8 AOS_Enable;
/********************************************/
// Main Entry Point
// Configuration
/********************************************/
int main()
{
/*	
    CACHE_setL2Mode(CACHE_256KCACHE);
	CACHE_enableCaching(CACHE_EMIFB_CE08);
	CACHE_enableCaching(CACHE_EMIFB_CE09);
	CACHE_enableCaching(CACHE_EMIFB_CE010);
	CACHE_enableCaching(CACHE_EMIFB_CE011);
	CACHE_enableCaching(CACHE_EMIFB_CE012);
	CACHE_enableCaching(CACHE_EMIFB_CE013);
	CACHE_enableCaching(CACHE_EMIFB_CE014);
	CACHE_enableCaching(CACHE_EMIFB_CE015);
*/
/*
	while(1)
	{
		memcpy((void *)first_fifo_1, (void *)(FPGA_TO_DSP_CHN2+8), 8);
	}
*/
}
/********************************************/
// Main Thread
/********************************************/
int StackTest()
{
    int               rc;
    HANDLE            hCfg;
	C62_disableIER(1<<9);
#if 1	
   rc = NC_SystemOpen( NC_PRIORITY_HIGH, NC_OPMODE_INTERRUPT );
    if( rc )
    {
        printf("NC_SystemOpen Failed (%d)\n",rc);
        for(;;);
    }
    // Print out our banner
    printf(VerStr);
/********************************************/
    // Enable the EDMA interrupt - since the EDMA interrupt
    // is configurable, this one line of code has no home.
    // NOTE: This code is not required for DM642, nor any
    //       driver environment that does not use EMDA sharing.
    //       However, it doesn't hurt to turn it on.
    // Create and build the system configuration from scratch.
     // Create a new configuration
/********************************************/
    hCfg = CfgNew();
    if( !hCfg )
    {
        printf("Unable to create configuration\n");
       // goto main_exit;
	   main_exit();
    }
    // We better validate the length of the supplied names
    if( strlen( DomainName ) >= CFG_DOMAIN_MAX ||
        strlen( HostName ) >= CFG_HOSTNAME_MAX )
    {
        printf("Names too long\n");
        //goto main_exit;
		main_exit();
    }
    // Add our global hostname to hCfg (to be claimed in all connected domains)
    CfgAddEntry( hCfg, CFGTAG_SYSINFO, CFGITEM_DHCP_HOSTNAME, 0,
                 strlen(HostName), (UINT8 *)HostName, 0 );
    // If the IP address is specified, manually configure IP
    if( inet_addr(LocalIpAddr) )
    {
        CI_IPNET NA;
        // Setup manual IP address
        bzero( &NA, sizeof(NA) );
        NA.IPAddr  = inet_addr(LocalIpAddr);
        NA.IPMask  = inet_addr(LocalIpMask);
        strcpy( NA.Domain, DomainName );
        NA.NetType = CFG_NETTYPE_DHCPS;
        NA.NetType |= CFG_NETTYPE_VIRTUAL;
        CfgAddEntry( hCfg, CFGTAG_IPNET, 1, 0,
                           sizeof(CI_IPNET), (UINT8 *)&NA, 0 );
		IPFilterSet(NA.IPAddr,NA.IPMask);
    }
    // If the IP gateway is specified, manually Gateway
    if( inet_addr(GatewayIP) )
    {
        CI_ROUTE RT;
        // Add the default gateway. Since it is the default, the
        // destination address and mask are both zero (we go ahead
        // and show the assignment for clarity).
        bzero( &RT, sizeof(RT) );
        RT.IPDestAddr = 0;
        RT.IPDestMask = 0;
        RT.IPGateAddr = inet_addr(GatewayIP);
        // Add the route
        CfgAddEntry( hCfg, CFGTAG_ROUTE, 0, 0,
                           sizeof(CI_ROUTE), (UINT8 *)&RT, 0 );

    }
    // We don't want to see debug messages less than WARNINGS
    rc = DBG_WARN;
    CfgAddEntry( hCfg, CFGTAG_OS, CFGITEM_OS_DBGPRINTLEVEL,
                 CFG_ADDMODE_UNIQUE, sizeof(uint), (UINT8 *)&rc, 0 );
/********************************************/
    // This code sets up the TCP and UDP buffer sizes
    // (Note 8192 is actually the default. This code is here to
    // illustrate how the buffer and limit sizes are configured.)
/********************************************/
    // TCP Transmit buffer size
    rc = 8192;
    CfgAddEntry( hCfg, CFGTAG_IP, CFGITEM_IP_SOCKTCPTXBUF,
                 CFG_ADDMODE_UNIQUE, sizeof(uint), (UINT8 *)&rc, 0 );
    // TCP Receive buffer size (copy mode)
    rc = 8192;
    CfgAddEntry( hCfg, CFGTAG_IP, CFGITEM_IP_SOCKTCPRXBUF,
                 CFG_ADDMODE_UNIQUE, sizeof(uint), (UINT8 *)&rc, 0 );
    // TCP Receive limit (non-copy mode)
    rc = 8192;
    CfgAddEntry( hCfg, CFGTAG_IP, CFGITEM_IP_SOCKTCPRXLIMIT,
                 CFG_ADDMODE_UNIQUE, sizeof(uint), (UINT8 *)&rc, 0 );
    // UDP Receive limit
    rc = 1024*32;
    CfgAddEntry( hCfg, CFGTAG_IP, CFGITEM_IP_SOCKUDPRXLIMIT,
                 CFG_ADDMODE_UNIQUE, sizeof(uint), (UINT8 *)&rc, 0 );
/********************************************/
    // Boot the system using this configuration
    // We keep booting until the function returns 0. This allows
    // us to have a "reboot" command.
/********************************************/
    do
    {
        rc = NC_NetStart( hCfg, NetworkOpen, NetworkClose, NetworkIPAddr );
    } while( rc > 0 );
    // Free the WEB files
    // Delete Configuration
    CfgFree( hCfg );//
#endif
   TaskCreate( videosrv,   "videosrv", OS_TASKPRINORM, 0x2000, 0, 0, 0 );
    // Close the OS
   return main_exit();
}
/********************************************/
int main_exit()
{
 	NC_SystemClose();
    return(0);
}
/********************************************/
// System Task Code [ Traditional Servers ]
/********************************************/
static HANDLE hvideo=0;
/********************************************/
// NetworkOpen
// This function is called after the configuration has booted
/********************************************/
static void NetworkOpen()
{
    // Create our local servers
      hvideo = TaskCreate( videosrv,   "videosrv", OS_TASKPRINORM, 0x2000, 0, 0, 0 );
}
/********************************************/
// NetworkClose
// This function is called when the network is shutting down,
// or when it no longer has any IP addresses assigned to it.
/********************************************/
static void NetworkClose()
{
    // If we opened NETCTRL as NC_PRIORITY_HIGH, we can't
    // kill our task threads until we've given them the
    // opportunity to shutdown. We do this by manually
    // setting our task priority to NC_PRIORITY_LOW.
    fdCloseSession( hvideo );
    TaskSetPri( TaskSelf(), NC_PRIORITY_LOW );
    TaskDestroy( hvideo );
}
/********************************************/
// NetworkIPAddr
// This function is called whenever an IP address binding is
// added or removed from the system.
/********************************************/
static void NetworkIPAddr( IPN IPAddr, uint IfIdx, uint fAdd )
{
    static uint fAddGroups = 0;
    IPN IPTmp;

    if( fAdd )
        printf("Network Added: ");
    else
        printf("Network Removed: ");

    // Print a message
    IPTmp = ntohl( IPAddr );
    printf("If-%d:%d.%d.%d.%d\n", IfIdx,
            (UINT8)(IPTmp>>24)&0xFF, (UINT8)(IPTmp>>16)&0xFF,
            (UINT8)(IPTmp>>8)&0xFF, (UINT8)IPTmp&0xFF );

    // This is a good time to join any multicast group we require
    if( fAdd && !fAddGroups )
    {
        fAddGroups = 1;
        // IGMPJoinHostGroup( inet_addr("224.0.0.2"), 1 );
    }
}
/********************************************/
