
#include <stkmain.h>
#include "lli.h"


static HANDLE hRtCache = 0;
static IPN    IPCache;
#define SEND_IP_DATA_BUFLEN  3000
extern Uint32 g_u32IpSendIndex;
extern SEM_Handle sem1;
extern UINT8  IP_DATA[SEND_IP_DATA_BUFLEN][1600];
HANDLE MY_IPGetRoute( uint RtCallFlags, IPN IPDst );
HANDLE MY_Two_IPGetRoute( uint RtCallFlags, IPN IPDst );
static RT   *prtFirstExp  = 0;    // First route in Exp list

void MyRtFlush()
{
    uint   wKilled;
    RT     *prt;

    // Kill Everything in the Timeout List
    prt = prtFirstExp;
   
    while( prt )
    {
        // Timeout Entry
        prt->Flags |= FLG_RTE_EXPIRED;

        // Get it out of the list
        prtFirstExp = prt->pNextExp;

        // Clear the expiration time
        prt->dwTimeout = 0;

        // Remove it from the node chain
        _RtNodeRemove( prt );

        // DeRef the expired node (was ref'd when timeout set)
        RtDeRef( prt );

        // Go to next entry
        prt = prtFirstExp;
    }

    //
    // Kill all STATIC routes
    //
    do
    {
        // Start Walking
        prt = RtWalkBegin();
        wKilled = 0;
        while( prt && !wKilled )
        {
            // Look for ANY static route
            if( prt->Flags & FLG_RTE_STATIC )
            {
                // Found a static route

                // Make it non-static
                prt->Flags &= ~FLG_RTE_STATIC;

                // Remove it from the node chain
                _RtNodeRemove( prt );

                // DeRef it
                RtDeRef( prt );

                // Flag that we killed a route
                wKilled = 1;
            }
            else
                prt = RtWalkNext(prt);
        }
        // We killed a route or ran out of routes. Either way
        // we need to end the walk (and potentially start over).
        RtWalkEnd( prt );
    } while( wKilled );
}

void IP_SEND_TEST()
{

	 LLI *plli;
	 int  val,retVal,test;
 	 SOCKET  s; 
     HANDLE  hRt;
	 HANDLE  hRt_bak;

	  char   *testIpAddr  = "172.22.8.100";
	   char   *testIpAddr2  = "172.22.255.255";
	  char   *testIpAddr1  = "172.22.8.150";
    struct   sockaddr_in  to ;//,sin1;
    bzero( &to, sizeof(struct sockaddr_in));
    to.sin_family       = AF_INET;
    to.sin_len          = sizeof( to );
    to.sin_addr.s_addr  = inet_addr(testIpAddr2);
	fdOpenSession( TaskSelf() );
	s = socket(AF_RAWETH, SOCK_RAWETH, 0x300);
	val = 1;
	retVal = setsockopt(s, SOL_SOCKET, SO_IFDEVICE, &val, sizeof(val));
	if(retVal)
		printf("error in setsockopt \n");

	val = 3;
	retVal = setsockopt(s, SOL_SOCKET, SO_PRIORITY, &val, sizeof(val));
	if(retVal)
		printf("error in setsockopt \n");


    while(1)
	{

			if(hRt= MY_IPGetRoute(3,to.sin_addr.s_addr))
			{  
			  
			    
				hRt_bak=hRt;
               //IPDst=RtGetIPAddr(hRt); //建立RT与ipdst的映射 //	IPN RtGetIPAddr( HANDLE hRt )  //从RT中取出IPDst  route.c  
			    if(to.sin_addr.s_addr  == inet_addr(testIpAddr2))
				{
					to.sin_addr.s_addr  = inet_addr(testIpAddr2);
				}
				else
				{
					to.sin_addr.s_addr  = inet_addr(testIpAddr2);
				}

	            plli = (LLI *)RtGetLLI( hRt ); 

	            if( plli->Status == LLI_STATUS_IDLE )
	            {
	                plli->Status = LLI_STATUS_ARP1;
	                _LLIExpListInsert( plli, llTimerGetTime(0) + 2 );
	                LLIGenArpPacket( RtGetIF(plli->hRt), RtGetIPAddr(plli->hRt) );
	            }
				else if( plli->Status == LLI_STATUS_DOWN )
	            {
	                plli->Status = LLI_STATUS_IDLE;
	                RtSetFailure( plli->hRt, FLG_RTF_REPORT, 0 );
	            }
           	        
				if ((plli->Status >= LLI_STATUS_ARP0 && plli->Status < LLI_STATUS_ARP5) || 
                (plli->Status >= LLI_STATUS_REVALID1 && plli->Status <= LLI_STATUS_REVALID2))
	            {
	                plli->Status++;
	                _LLIExpListInsert( plli, llTimerGetTime(0) + 1 );
	                LLIGenArpPacket( RtGetIF(plli->hRt), RtGetIPAddr(plli->hRt) );
	            }
					mmCopy(&IP_DATA[g_u32IpSendIndex][0],&plli->MacAddr[0],6);
					test=send(s, &IP_DATA[g_u32IpSendIndex][0], 100, 0);
		  
		  	   _RtNodeRemove( hRt );
               RtDeRef( hRt );  // DeRef the expired node (was ref'd when timeout set)
		     //	 RtRemove( hRt, FLG_RTF_REPORT, RTC_NETUNREACH );
			 //    MyRtFlush();
			}
			else
			{
			//	val = 1;
  
			//	to.sin_addr.s_addr  = inet_addr(testIpAddr1);
				hRt=hRt_bak;
				mmCopy((void *)((int)hRt+28),	&to.sin_addr.s_addr,4 );
				//(int *)(hRt+16)=to.sin_addr.s_addr;
				val = 1;

			
			}
		   
      
		TaskSleep(5);
	}

}
void IP_SEND_TEST1()
{

	 LLI *plli;
	 int  val,retVal,test;
 	 SOCKET  s; 
     HANDLE  hRt;

	  char   *testIpAddr  = "172.22.8.200";
	  char   *testIpAddr2  = "172.22.255.255";
	  char   *testIpAddr1  = "172.22.8.250";
    struct   sockaddr_in  to ;//,sin1;
    bzero( &to, sizeof(struct sockaddr_in));
    to.sin_family       = AF_INET;
    to.sin_len          = sizeof( to );
    to.sin_addr.s_addr  = inet_addr(testIpAddr2);
	fdOpenSession( TaskSelf() );
	s = socket(AF_RAWETH, SOCK_RAWETH, 0x300);
	val = 1;
	retVal = setsockopt(s, SOL_SOCKET, SO_IFDEVICE, &val, sizeof(val));
	if(retVal)
		printf("error in setsockopt \n");

	val = 3;
	retVal = setsockopt(s, SOL_SOCKET, SO_PRIORITY, &val, sizeof(val));
	if(retVal)
		printf("error in setsockopt \n");


    while(1)
	{

 /*      if( !(hRt = IPGetRoute(3, to.sin_addr.s_addr)) )
       {
            ips.Cantforward++;
            ICMPGenPacket(pIpHdr,hIFRx,ICMP_UNREACH,ICMP_UNREACH_NET,0);
         
        }*/

			if(hRt= MY_Two_IPGetRoute(3,to.sin_addr.s_addr))
			{  
			    if(to.sin_addr.s_addr  == inet_addr(testIpAddr2))
				{
					to.sin_addr.s_addr  = inet_addr(testIpAddr2);
				}
				else
				{
					to.sin_addr.s_addr  = inet_addr(testIpAddr2);
				}

	            plli = (LLI *)RtGetLLI( hRt ); 

	            if( plli->Status == LLI_STATUS_IDLE )
	            {
	                plli->Status = LLI_STATUS_ARP1;
	                _LLIExpListInsert( plli, llTimerGetTime(0) + 2 );
	                LLIGenArpPacket( RtGetIF(plli->hRt), RtGetIPAddr(plli->hRt) );
	            }
				else if( plli->Status == LLI_STATUS_DOWN )
	            {
	                plli->Status = LLI_STATUS_IDLE;
	                RtSetFailure( plli->hRt, FLG_RTF_REPORT, 0 );
	            }
           	        
				if ((plli->Status >= LLI_STATUS_ARP0 && plli->Status < LLI_STATUS_ARP5) || 
                (plli->Status >= LLI_STATUS_REVALID1 && plli->Status <= LLI_STATUS_REVALID2))
	            {
	                plli->Status++;
	                _LLIExpListInsert( plli, llTimerGetTime(0) + 1 );
	                LLIGenArpPacket( RtGetIF(plli->hRt), RtGetIPAddr(plli->hRt) );
	            }
					mmCopy(&IP_DATA[g_u32IpSendIndex][0],&plli->MacAddr[0],6);
					test=send(s, &IP_DATA[g_u32IpSendIndex][0], 100, 0);
				
			}
			else
			{
				to.sin_addr.s_addr  = inet_addr(testIpAddr2);
			}
		 TaskSleep(3);
	}

}


HANDLE MY_RtFind( uint wCallFlags, IPN IP )
{
    RT     *prt,*prtClone;
    int    Search = 1;
    HANDLE hNode = 0;

    while( Search )
    {

	    SEM_pend(sem1, SYS_FOREVER);
        // The following is guaranteed to find a node
        hNode = NodeFind( IP, hNode );

        // Now we need to find the "best" route on this node.
        // Note: Our search can be affected by the call flags

        // Get the route
        prt = (RT *)NodeGetRt( hNode );

        // We'll set the following flag to TRUE if we pass over a match
        Search = 0;

        while( prt )
        {
            // Try this route
            if( (IP & prt->IPMask) != (prt->IPAddr & prt->IPMask) )
                goto RouteNotAccepted;

            // Quit now if there are no matching conditions
            if( !(wCallFlags & FLG_RTF_CONDITIONAL) )
                break;

            // If we discard the match due to search criteria, first
            // mark if we can or can not continue. We don't continue
            // if we are at the far left node.
            if( prt->IPAddr & prt->IPMask )
                Search = 1;

            // Host only search
            if( wCallFlags & FLG_RTF_HOST )
            {
                // We never match gateway routes here - even if they are
                // "host" routes (host re-directs)
                if( prt->Flags & FLG_RTE_GATEWAY )
                    goto RouteNotAccepted;

                // If this is not a host route, if the CLONING flags is set,
                // we will still accept it if the route can clone into a
                // host route.
                if( !(prt->Flags & FLG_RTE_HOST) )
                {
                    // We will reject this route unless it is a viable clone
                    // route and the caller wants to clone.
                    if( !(wCallFlags & FLG_RTF_CLONE) ||
                        !(prt->Flags & FLG_RTE_CLONING) )
                        goto RouteNotAccepted;
                }
                break;
            }

            // Proxy only search
            if( wCallFlags & FLG_RTF_PROXY )
            {
                if( !(prt->Flags & FLG_RTE_PROXY) )
                    goto RouteNotAccepted;
                break;
            }

            // ProxyPub only search
            if( wCallFlags & FLG_RTF_PROXYPUB )
            {
                if( !(prt->Flags & FLG_RTE_PROXYPUB) )
                    goto RouteNotAccepted;
                break;
            }

            // This route is acceptable
            break;

RouteNotAccepted:
            // No match yet
            prt = prt->pNext;
        }

        // Reference the matched route (if any)
        if( prt )
        {
            RtRef( prt );
            Search = 0;
        }
    }

    // We no longer need the node
    NodeDeRef( hNode );

    // If we have a route, we may need to clone it
    if( prt && (prt->Flags & FLG_RTE_CLONING) && (wCallFlags & FLG_RTF_CLONE) )
    {
        uint   NewFlags;

        // Yep, we have to clone ...

        // Copy the flags
        NewFlags = prt->Flags;

        // Clear the flags that don't carry over
        NewFlags &= ~(FLG_RTE_CLONING|FLG_RTE_STATIC|FLG_RTE_DYNAMIC);

        // Make us a host
        NewFlags |= FLG_RTE_HOST;

        if( !(prtClone = RtCreate( wCallFlags, NewFlags, IP, 0xffffffff,
                                   prt->hIF, prt->IPGate, 0 )) )
            DbgPrintf(DBG_WARN,"RtFind: Clone creation failed!");
        else
        {
            // Mark as "keep-alive" route
            prtClone->Flags |= FLG_RTE_KEEPALIVE;

            // Set Default Timeout for Clone Node
            RtSetTimeout( prtClone, ROUTE_CLONE_TIMEOUT );
        }

        // DeRef the source node
        RtDeRef( prt );

        // We now use the clone node
        prt = prtClone;
    }

    // If we have no match, this is a "miss"
    if( !prt )
    {
        // Miss! Post a report if needed
        if( wCallFlags & FLG_RTF_REPORT )
        {
            // Send Routing Report
            RTCReport( MSG_RTC_MISS, IP, 0xffffffff );
        }
    }

    // NOTE: prt contains one of three things:
    //  1. NULL for a "miss"
    //  2. The referenced original route found
    //  3. The referenced clone route (original unreferenced above).

    return( prt );
}
HANDLE MY_Two_RtFind( uint wCallFlags, IPN IP )
{
    RT     *prt,*prtClone;
    int    Search = 1;
    HANDLE hNode = 0;

    while( Search )
    {

	   
        // The following is guaranteed to find a node
        hNode = NodeFind( IP, hNode );
	   	SEM_post(sem1);
        // Now we need to find the "best" route on this node.
        // Note: Our search can be affected by the call flags

        // Get the route
        prt = (RT *)NodeGetRt( hNode );

        // We'll set the following flag to TRUE if we pass over a match
        Search = 0;

        while( prt )
        {
            // Try this route
            if( (IP & prt->IPMask) != (prt->IPAddr & prt->IPMask) )
                goto RouteNotAccepted;

            // Quit now if there are no matching conditions
            if( !(wCallFlags & FLG_RTF_CONDITIONAL) )
                break;

            // If we discard the match due to search criteria, first
            // mark if we can or can not continue. We don't continue
            // if we are at the far left node.
            if( prt->IPAddr & prt->IPMask )
                Search = 1;

            // Host only search
            if( wCallFlags & FLG_RTF_HOST )
            {
                // We never match gateway routes here - even if they are
                // "host" routes (host re-directs)
                if( prt->Flags & FLG_RTE_GATEWAY )
                    goto RouteNotAccepted;

                // If this is not a host route, if the CLONING flags is set,
                // we will still accept it if the route can clone into a
                // host route.
                if( !(prt->Flags & FLG_RTE_HOST) )
                {
                    // We will reject this route unless it is a viable clone
                    // route and the caller wants to clone.
                    if( !(wCallFlags & FLG_RTF_CLONE) ||
                        !(prt->Flags & FLG_RTE_CLONING) )
                        goto RouteNotAccepted;
                }
                break;
            }

            // Proxy only search
            if( wCallFlags & FLG_RTF_PROXY )
            {
                if( !(prt->Flags & FLG_RTE_PROXY) )
                    goto RouteNotAccepted;
                break;
            }

            // ProxyPub only search
            if( wCallFlags & FLG_RTF_PROXYPUB )
            {
                if( !(prt->Flags & FLG_RTE_PROXYPUB) )
                    goto RouteNotAccepted;
                break;
            }

            // This route is acceptable
            break;

RouteNotAccepted:
            // No match yet
            prt = prt->pNext;
        }

        // Reference the matched route (if any)
        if( prt )
        {
            RtRef( prt );
            Search = 0;
        }
    }

    // We no longer need the node
    NodeDeRef( hNode );

    // If we have a route, we may need to clone it
    if( prt && (prt->Flags & FLG_RTE_CLONING) && (wCallFlags & FLG_RTF_CLONE) )
    {
        uint   NewFlags;

        // Yep, we have to clone ...

        // Copy the flags
        NewFlags = prt->Flags;

        // Clear the flags that don't carry over
        NewFlags &= ~(FLG_RTE_CLONING|FLG_RTE_STATIC|FLG_RTE_DYNAMIC);

        // Make us a host
        NewFlags |= FLG_RTE_HOST;

        if( !(prtClone = RtCreate( wCallFlags, NewFlags, IP, 0xffffffff,
                                   prt->hIF, prt->IPGate, 0 )) )
            DbgPrintf(DBG_WARN,"RtFind: Clone creation failed!");
        else
        {
            // Mark as "keep-alive" route
            prtClone->Flags |= FLG_RTE_KEEPALIVE;

            // Set Default Timeout for Clone Node
            RtSetTimeout( prtClone, ROUTE_CLONE_TIMEOUT );
        }

        // DeRef the source node
        RtDeRef( prt );

        // We now use the clone node
        prt = prtClone;
    }

    // If we have no match, this is a "miss"
    if( !prt )
    {
        // Miss! Post a report if needed
        if( wCallFlags & FLG_RTF_REPORT )
        {
            // Send Routing Report
            RTCReport( MSG_RTC_MISS, IP, 0xffffffff );
        }
    }

    // NOTE: prt contains one of three things:
    //  1. NULL for a "miss"
    //  2. The referenced original route found
    //  3. The referenced clone route (original unreferenced above).

    return( prt );
}
HANDLE MY_IPGetRoute( uint RtCallFlags, IPN IPDst )
{
    HANDLE hRt, hRtGate;

    // First, try the cached route
    if( hRtCache && IPCache == IPDst )
    {
        ips.CacheHit++;
        RtRef( hRtCache );
        return( hRtCache );
    }

    // Find the route
    if( !(hRt = MY_Two_RtFind( RtCallFlags, IPDst )) )
        return(0);
    
    // If this is a GATEWAY route, we need to get the route to
    // the GateIP
    if( RtGetFlags( hRt ) & FLG_RTE_GATEWAY )
    {
        // Get a route to the Gateway IP
        IPDst = RtGetGateIP( hRt );
        RtDeRef( hRt );

        // First, try the cached route
        if( hRtCache && IPCache == IPDst )
        {
            ips.CacheHit++;
            RtRef( hRtCache );
            return( hRtCache );
        }

        // Find the route to the gateway
        // Note that this time, only HOST routes are allowed
        if( !(hRtGate = RtFind( RtCallFlags|FLG_RTF_HOST, IPDst )) )
            return(0);

        // Switch to the Gateway route
        hRt = hRtGate;
    }

    // We've officially missed the cache
    ips.CacheMiss++;

    // As this is not the cached route, if it is a host route, we'll
    // make it the new cache route.
    if( RtGetFlags( hRt ) & FLG_RTE_HOST )
    {
        if( hRtCache )
            RtDeRef( hRtCache );
        RtRef( hRt );
        hRtCache = hRt;
        IPCache  = IPDst;
    }

    // Return whatever route we found
    return( hRt );
}
HANDLE MY_Two_IPGetRoute( uint RtCallFlags, IPN IPDst )
{
    HANDLE hRt, hRtGate;

    // First, try the cached route
    if( hRtCache && IPCache == IPDst )
    {
        ips.CacheHit++;
        RtRef( hRtCache );
        return( hRtCache );
    }

    // Find the route
    if( !(hRt = MY_RtFind( RtCallFlags, IPDst )) )
        return(0);

    // If this is a GATEWAY route, we need to get the route to
    // the GateIP
    if( RtGetFlags( hRt ) & FLG_RTE_GATEWAY )
    {
        // Get a route to the Gateway IP
        IPDst = RtGetGateIP( hRt );
        RtDeRef( hRt );

        // First, try the cached route
        if( hRtCache && IPCache == IPDst )
        {
            ips.CacheHit++;
            RtRef( hRtCache );
            return( hRtCache );
        }

        // Find the route to the gateway
        // Note that this time, only HOST routes are allowed
        if( !(hRtGate = RtFind( RtCallFlags|FLG_RTF_HOST, IPDst )) )
            return(0);

        // Switch to the Gateway route
        hRt = hRtGate;
    }

    // We've officially missed the cache
    ips.CacheMiss++;

    // As this is not the cached route, if it is a host route, we'll
    // make it the new cache route.
    if( RtGetFlags( hRt ) & FLG_RTE_HOST )
    {
        if( hRtCache )
            RtDeRef( hRtCache );
        RtRef( hRt );
        hRtCache = hRt;
        IPCache  = IPDst;
    }

    // Return whatever route we found
    return( hRt );
}