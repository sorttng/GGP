#include <stdint.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include "evmdm6467_timer.h"
#include "dm6467_hw.h"
#include "Mip_Stack.h"
#include "rtp.h"


struct udp_pcb * g_pcRtpPcb = NULL;
CRtpHeader g_cRtpHeader;

#pragma DATA_SECTION ( g_s8StreamBuf, ".ddr2_udp" );
#pragma DATA_ALIGN(g_s8StreamBuf, 128);
Uint8 g_s8StreamBuf[INPUT_BUF_NUM][INPUT_BUF_SIZE]; //输入数据缓冲区
Uint32 g_u32RevFrmInx = 0;
Uint32 g_u32DecFrmInx = 0;

const struct ip_addr ip_addr_local       = { 0xA8C0, 0x0A80 };
const struct ip_addr ip_addr_remote       = { 0xA8C0, 0x6480 };
const struct ip_addr ip_addr_remote1       = { 0xA8C0, 0x6580 };
#define IP_ADDR_LOCAL ((struct ip_addr *)&ip_addr_local)
#define IP_ADDR_REMOTE ((struct ip_addr *)&ip_addr_remote)
#define IP_ADDR_REMOTE1 ((struct ip_addr *)&ip_addr_remote1)
#define H264_PAYLOAD_TYPE 105

static void RTP_rcv(void * pArg, 
                     struct udp_pcb *upcb, 
                     struct pbuf *p, 
                     struct ip_addr *addr, 
                     u16_t port);

//------------------------------------------------------------------------------------
Uint32 g_u32FrameSize[INPUT_BUF_NUM] = {0};
extern int g_iWaitForIFrame;

static void RTP_rcv(void * pArg, 
                     struct udp_pcb *upcb, 
                     struct pbuf *p, 
                     struct ip_addr *addr, 
                     u16_t port)
{
	Uint32 u32ValLen = 0;
	static Uint32 s_u32SumFrmLen=0;
	uint8_t  uiPreHead[4] = {0x00,0x00,0x00,0x01};
	uint8_t u8NRI;
    uint8_t u8Type;
	uint8_t u8RtpMark;
	uint8_t u8FuHeader;
	uint8_t u8FuIndic;
	uint8_t u8StartFlg;
	uint8_t u8EndFlg;

	// Receive packet from encoder.
	if (sizeof(g_cRtpHeader) >= p->len)
	{
		return;
	}

	u32ValLen = p->len - sizeof(g_cRtpHeader);
	memcpy(&g_cRtpHeader, p->payload, sizeof(g_cRtpHeader));
	u8RtpMark = g_cRtpHeader.m_u8Type & 0x80;
	
	if(H264_PAYLOAD_TYPE == (g_cRtpHeader.m_u8Type & 0x7F))//判断是否为H264类型的数据包
	{
		if((0x67 == p->payload[sizeof(g_cRtpHeader)]) || (0x68 == p->payload[sizeof(g_cRtpHeader)]))
		{

			memcpy(g_s8StreamBuf[g_u32RevFrmInx & INPUT_BUF_NUM_MASK] + s_u32SumFrmLen, uiPreHead, 4);
			memcpy(g_s8StreamBuf[g_u32RevFrmInx & INPUT_BUF_NUM_MASK] + s_u32SumFrmLen + 4, p->payload + sizeof(g_cRtpHeader), u32ValLen);
			s_u32SumFrmLen += u32ValLen + 4;				
		}
		else //非参数集数据
		{
			u8FuIndic = p->payload[sizeof(g_cRtpHeader)];
        	u8Type    = u8FuIndic & 0x1F;
			u8NRI     = u8FuIndic & 0xE0;
			if(28 != u8Type) //单片模式
			{
				memcpy(g_s8StreamBuf[g_u32RevFrmInx & INPUT_BUF_NUM_MASK] + s_u32SumFrmLen, uiPreHead, 4);
				memcpy(g_s8StreamBuf[g_u32RevFrmInx & INPUT_BUF_NUM_MASK] + s_u32SumFrmLen + 4, p->payload + sizeof(g_cRtpHeader), u32ValLen);
				s_u32SumFrmLen += u32ValLen + 4;
				u8EndFlg = 0x40;
			}
			else //FU-A分片模式
			{
				u8FuHeader = p->payload[sizeof(g_cRtpHeader)+1];
				u8StartFlg = u8FuHeader & 0x80;
				u8EndFlg   = u8FuHeader & 0x40;
				u8Type     = u8FuHeader & 0x1F;
				if(u8StartFlg)
				{
					memcpy(g_s8StreamBuf[g_u32RevFrmInx & INPUT_BUF_NUM_MASK] + s_u32SumFrmLen, uiPreHead, 4);
					g_s8StreamBuf[g_u32RevFrmInx & INPUT_BUF_NUM_MASK][s_u32SumFrmLen+4] = u8NRI | u8Type;
					memcpy(g_s8StreamBuf[g_u32RevFrmInx & INPUT_BUF_NUM_MASK] + s_u32SumFrmLen + 5, p->payload + sizeof(g_cRtpHeader)+2, u32ValLen-2);
					s_u32SumFrmLen += u32ValLen-2 + 5;	
				}
				else
				{
					memcpy(g_s8StreamBuf[g_u32RevFrmInx & INPUT_BUF_NUM_MASK] + s_u32SumFrmLen, p->payload + sizeof(g_cRtpHeader)+2, u32ValLen-2);
					s_u32SumFrmLen += u32ValLen-2;
				}					
			}
		}
	}
	else
	{		
		printf("The received frame is not the H264 type.\n");
		return;
	}
	

	if( (0x40 == u8EndFlg) )//一帧的结尾
	{
		if(   (g_iWaitForIFrame == 0)
		   && 0x00 == g_s8StreamBuf[g_u32RevFrmInx & INPUT_BUF_NUM_MASK][0]
		   && 0x00 == g_s8StreamBuf[g_u32RevFrmInx & INPUT_BUF_NUM_MASK][1]
		   && 0x00 == g_s8StreamBuf[g_u32RevFrmInx & INPUT_BUF_NUM_MASK][2]
		   && 0x01 == g_s8StreamBuf[g_u32RevFrmInx & INPUT_BUF_NUM_MASK][3]
		   && 0x67 == g_s8StreamBuf[g_u32RevFrmInx & INPUT_BUF_NUM_MASK][4]) //wait for I frame
		{
			g_iWaitForIFrame = 1;
			g_u32DecFrmInx   = g_u32RevFrmInx;					
		}

		g_u32FrameSize[g_u32RevFrmInx & INPUT_BUF_NUM_MASK] = s_u32SumFrmLen;
		s_u32SumFrmLen = 0;
		g_u32RevFrmInx++;
	
	}	
} 


//------------------------------------------------------------------------------------
int32_t RTP_init()
{
	if (NULL != g_pcRtpPcb)
	{
		return -1;
	}
	g_pcRtpPcb = udp_new();
	if (NULL == g_pcRtpPcb)
	{
		TRACE_0("Create RTP socket.[failed]");
	}
	else
	{
		TRACE_0("Create RTP socket.[ok]");
	}
    if (ERR_OK != udp_bind(g_pcRtpPcb, IP_ADDR_LOCAL, RTP_PORT))
	{
		TRACE_0("Bind to rtp port error\n");
		udp_remove(g_pcRtpPcb);
		g_pcRtpPcb = NULL;
	}

	udp_connect(g_pcRtpPcb,IP_ADDR_REMOTE,6000);
	udp_recv(g_pcRtpPcb, RTP_rcv, NULL);
	return 0;
} // RTP_init()



