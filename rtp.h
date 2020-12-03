#ifndef __RTP_H__
#define __RTP_H__
#include <stdint.h>
#include <time.h>

#define INPUT_BUF_SIZE 256 * 256
#define INPUT_BUF_NUM  128
#define INPUT_BUF_NUM_MASK INPUT_BUF_NUM-1
#define RTP_PORT 5600

typedef struct _RTP_HEADER_TAG_ {
    Uint8  m_u8Ver;
	Uint8  m_u8Type;
	Uint16 m_u16SeqNum;
	Uint32 m_u32TimeStamp;
	Uint32 m_u32SSRC;
//	Uint32 m_u32CSRC;
} CRtpHeader;

extern uint32_t g_uLocalTimeSec;
extern uint32_t g_uLocalTimeMs;

int32_t RTP_init();


#endif // __SNTP_H__


