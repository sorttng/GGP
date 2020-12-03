/*********************************/
/*20161206**************/
/*ADS_B**************/
/*********************************/
//-------------------------------------------------------------------------
#include <stdio.h>
#include <netmain.h>
#include <csl_cache.h>
#include "videosrv.h"
#include <_stack.h>
#include <tsk.h>
#include <_oskern.h>
#include <csl_emac.h>
#include "lli.h"
#include "ip.h"
#include "macro.h"
#include "Address_of_Reg.h"
#include "EDMA_init.h"
#include "EMIF_init.h"
#include "data_function.h"
#include "data_macro.h"
unsigned short edma_finished;//edma
UINT32 g_u32phyIdx;
UINT32 g_u32phyReg;
UINT32 g_u32phyVal;
UINT32 * g_u32Reg;
UINT32 g_u32Val;
unsigned int fpga_cnt_bak[20]={0};
unsigned int fpga_cnt[20]={0};
unsigned int fpga_err_cnt=0;
Uint8 cmd_buf_test2[1500];
unsigned int data_test_index=0;
unsigned char dma_test0[1000],dma_test1[1000];
Uint8 DSTIP[160]={0};
unsigned int   fpga_SetIP;
unsigned short fpga_SetPort;
Uint8 fpga_SetCH;
Uint8 fpga_SetFlag=0;
Uint8 flag=0;
Uint8 test_flag =0; //ÓĞ²âÊÔÊı¾İÖÃ³É1£¬Ê²Ã´Ê±ºò½áÊøÄØ£¿
Uint8 sum_cnt=0;
Uint8 zero_cnt=0;
Uint8 XSH_Array[288]={0};//88 
Uint8 TestArray[160]={0};


Uint8 ICAOArray[800]={0};	//ÓÃÀ´½ÓÊÕFPGA´«¹ıÀ´µÄICAOºÅ£¬30*4B
Uint8 ICAOArray_real[200]={0};	//µ÷ÕûË³ĞòºóµÄ´æ´¢Î»ÖÃ
Uint16 check_sum_1,check_sum_2;

//zhangfulong add start
Uint8	yaokongcmd[140] = {0};//Ò£¿ØÖ¸ÁîĞÅºÅÊı¾İ¸ñÊ½
Uint32 ICAO_number[30] = {0};//ÓÃÀ´½«ÊÕµ½µÄICAOºÅĞ´µ½32Î»µÄÊı×éÀïÃæ
int taokong_workmode = 0;// 0±íÊ¾Ã»ÓĞ·¢£¬1£º×ÔÓÉÄ¿±êËÑË÷ £»2£ºÄ¿±ê¸ú×Ù
int yaokong_jingdu = 0.0;
int yaokong_weidu = 0.0;
int yaokong_gaodu = 0.0;
int yaokong_banzhuijiao = 0;//0-90

struct yaokongcmd_all
{
  unsigned int mode;
  float jingdu;
  float weidu;
  float gaodu;
  int   banzhuijiao;
  Uint8 chelue_1;
  Uint8 chelue_2;
  Uint8 chelue_3;
  Uint8 chelue_new;
  Uint32 ICAO[30]; 							
}yaokongcmd_all_data;

//Uint8 pbuf_yindao[36] = {0};//Òıµ¼Êı¾İ¸ñÊ½
Uint8 pbuf_yindao[150] = {0};
Uint8 up_commend_count = 0;//ÉÏ×¢°ü¼ÆÊı±äÁ¿
//zhangfulong add end

//Uint8 XSH_Array2[188]={0};//?
//EDMA_Handle hEdma,hEdma_samp;     /* Handle for the EDMA channel  */
#if 0
	unsigned int int_cnt_1=0;//ÖĞ¶Ï¸öÊı
	unsigned int int_cnt_2=0;//CRCÈ«¶ÔµÄ
	unsigned int int_cnt_3=0;//·¢ËÍ°ü¸öÊı
	unsigned int int_cnt_4=0;//¿É¾À´íµÄ
	unsigned int int_cnt_5=0;
	unsigned int int_cnt_6=0;
	unsigned int int_cnt_7=0;  //ºÍÂëÔ´²»Ò»ÖÂµÄ
	unsigned int int_cnt_8=0;   //ËÑÍ·¶ÔµÄ
	unsigned int int_cnt_9=0;  //yc
	unsigned int int_cnt_10=0;  //xsh
	unsigned int int_cnt_11=0;  //ykip
	unsigned int int_cnt_12=0;  //test
#endif
unsigned int int_cnt_1=0;//ÖĞ¶Ï¸öÊı
unsigned int int_cnt_3=0;//·¢ËÍ°ü¸öÊı
Uint8  int_cnt[22]={0x0};   //
unsigned int pos_2_cnt=0; //½âËãÎ»ÖÃ2¼ÆÊı
unsigned int pos_3_cnt=0; //½âËãÎ»ÖÃ3¼ÆÊı(ÓĞĞ§)
unsigned int vel_4_cnt=0; //½âËãËÙ¶È4¼ÆÊı
unsigned int vel_5_cnt=0; //½âËãËÙ¶È5¼ÆÊı£¨ÓĞĞ§£©
#if 0
	unsigned int LOW_num=0;
	unsigned int LOW_num_15=0;
	unsigned int LOW_num_20=0; 
	unsigned int Error_num=0;
	unsigned int cf_1_num=0;
	unsigned int cf_2_num=0;
	unsigned int p_1_num=0;
	unsigned int p_0_num=0;
#endif
	unsigned int test_time=0;
	unsigned int  t1,t2,t3,t4,t5;
//190611
#define FIND_LOST 50
//#define FIND_RANGE 20
//190625
#define FIND_RANGE 25
//5000
#define ICAO_BUFLEN 500

Uint8 start_pos[FIND_RANGE]; //
Uint8 end_pos[FIND_RANGE];//
unsigned int g_32_ICAO_new_cnt=0;
unsigned int g_32_ICAO_match_cnt=0;
unsigned char g_8_ICAO_NAME[ICAO_BUFLEN][4]={0};//int *
#define SEND_IP_DATA_BUFLEN 150
#pragma DATA_SECTION ( IP_DATA, ".DATA_IP" );
#pragma DATA_ALIGN(IP_DATA, 128);
unsigned short  IP_DATA[SEND_IP_DATA_BUFLEN][1400];  //
volatile Uint32 SND_Catch_Program_Full = 0;  //
volatile Uint32 IP_Pkg_index=0;//1;  //Ğ´Ö¸Õë //0//170904 //?¨®¨º?????
#pragma DATA_SECTION ( SEND_ADDR_CACHE, ".DATA_IP" );
//#pragma DATA_ALIGN(SEND_ADDR_CACHE, 128);
IP_ADDR_CACHE  SEND_ADDR_CACHE[SEND_IP_DATA_BUFLEN];
Uint32 g_u32IpSendIndex=1;   //¶ÁÖ¸Õë
//------------------------
#define pulse_BUFLEN 150
#pragma DATA_SECTION ( pulse, ".DATA_IP" );
unsigned short  pulse[pulse_BUFLEN][112];  // 
Uint32 pulse_time[pulse_BUFLEN];  //190521
volatile Uint32 pulse_cnt=0;//
volatile Uint32 pulse_wr=0;
Uint32 pulse_rd =0;
//------------------------
//#define pulse_BUFLEN_IP 1500
#define pulse_BUFLEN_IP 15000
//190521
//#define pulse_BUFLEN_IP 7500
//¸Ğ¾õ¸øÍøÂç×¼±¸µÄ»º³åÇøºÜÈİÒ×Âú£¬ÊÇ·ñÓĞ±ØÒª¿ª´ó
#pragma DATA_SECTION ( pulse_IP, ".DATA_IP" );
//unsigned short  pulse_IP[pulse_BUFLEN_IP][11];  //   112
Uint8  pulse_IP[pulse_BUFLEN_IP][11];  //190412
volatile Uint32 pulse_cnt_IP=0;//
volatile Uint32 pulse_wr_IP=0;
Uint32 pulse_rd_IP =0;
//------------------------
//position
#define position_BUFLEN 1500
#pragma DATA_SECTION ( position, ".DATA_IP" );
Uint8 position[position_BUFLEN][20];  // 
//unsigned short  position[position_BUFLEN][20];  // 
volatile Uint32 position_cnt=0;//
volatile Uint32 position_wr=0;
Uint32 position_rd =0;

//zhangfulong add 
volatile Uint32 ronghe_cnt_p = 0;
volatile Uint32 ronghe_cnt_v = 0;

volatile Uint32 ronghe_position_p = 0;
volatile Uint32 ronghe_position_v = 0;
//zhangfulong add
//------------------------
//position_xsh
//#define position_BUFLEN_xsh 1500  //190611
#define position_BUFLEN_xsh 500  //190611
#pragma DATA_SECTION ( position_xsh, ".DATA_IP" );
//unsigned short  position_xsh[position_BUFLEN_xsh][20];  // 
Uint8  position_xsh[position_BUFLEN_xsh][20];    //190418

struct location_struct airplane_location_XSH_static[position_BUFLEN_xsh];  //ÕÅ¸»Â¡ add XSHÏ¡Êè»¯µÄÎ»ÖÃÊı×é
struct speed_struct_three airplane_velocity_three_XSH_static[position_BUFLEN_xsh];  //ÕÅ¸»Â¡ add XSHÏ¡Êè»¯µÄËÙ¶ÈÊı×é

Uint32 position_xsh_ICAO[position_BUFLEN_xsh];
volatile Uint32 position_cnt_xsh=0;//
volatile Uint32 position_wr_xsh=0;
volatile Uint32 position_wr_xsh_v=0;//zhangfulong add Ï¡Êè»¯Êı¾İ£¬
Uint32 position_rd_xsh =0;
//------------------------
//velocity
#define velocity_BUFLEN 1500
#pragma DATA_SECTION ( velocity, ".DATA_IP" );
Uint8 velocity[velocity_BUFLEN][20];  //
//unsigned short  velocity[velocity_BUFLEN][20];  // 
volatile Uint32 velocity_cnt=0;//
volatile Uint32 velocity_wr=0;
Uint32 velocity_rd =0;
FPGA_YC s_FPGA_YC;  //16×Ö½ÚÒ£²â
Uint8 UTC_time_all[20] = 0;//½ÓÊÕµÄÊı¾İ
Uint8 UTC_time_real[6] = 0;//¼ÇÂ¼UTCÊ±¼ä£¬ÓÃÀ´¸øÆäËûµÄÖµ
#pragma DATA_ALIGN(s_FPGA_YC, 8);
//ÓÃÓÚ±¨ÎÄĞÅÏ¢µÄÉÏ±¨µÄÈ«¾Ö½á¹¹Ìå
//extern void gpio7_set_1();
//extern void gpio7_set_0();
#if 0
void gpio7_set_0()
{  
   (GP_DIR)=(GP_DIR)&0xffffff7f;           //GPIO direction register    
   (GP_OUT_DATA)=(GP_OUT_DATA)|0x80;//&0xffffff7f;        //GPIO value register 
   (GP_CLR_DATA)=(GP_SET_DATA)&0x80;  
}
void gpio7_set_1()
{  
   (GP_DIR)=(GP_DIR)&0xffffff7f;           //GPIO direction register    
   (GP_OUT_DATA)=(GP_OUT_DATA)|0x80;        //GPIO value register 
   (GP_SET_DATA)=(GP_SET_DATA)&0x80;  
}
#endif
//***************************************************************************************************************************
struct message_struct
{
  unsigned int time;
  unsigned char data_demodulator[11];  //ADSBµÄÔ­Ê¼88bitÏûÏ¢±¨ÎÄ£¬11×Ö½Ú							
}ADSB_message; 
//******************************************************************************************************************************
//½á¹¹Ìå±äÁ¿µÄÉùÃ÷±ØĞë°üº¬½á¹¹ÌåÀàĞÍ¶¨Òå±¾Éí,ÓÃÓÚÎ»ÖÃĞÅÏ¢µÄ¼ÆËã
//´Ë½á¹¹ÌåÊı×éºÍ½á¹¹Ìå±äÁ¿È«¾ÖÊ¹ÓÃ£¬ÔÚdecode_positionº¯Êı¸üĞÂ¸Ã½á¹¹Ìådata_save[]Êı×é£¬new_data½á¹¹Ìå±äÁ¿ÔÚdemodulatorº¯ÊıÖĞ¸üĞÂ
//¸Ã½á¹¹Ìå±ØĞëÈ«¾Ö±£´æÊı¾İ£¬ÏÂÃæÊÇ¸÷¸öÔªËØµÄ¶¨Òå¡£
//timeÊÇ±¨ÎÄµÄÊ±¼ä±êÇ©£¬¾«È·µ½Ãë£»
//ICAO_adress_with_markÓÃÓÚ´æ·ÅICAOµØÖ·£¬ÓÃµÍ24Î»£¬µÚ25Î»ÊÇ±êÖ¾Î»£¬Îª0Ö¸Ê¾¹²ÓÃÌådataÖĞ´æµÄÊÇ¾­Î³¶ÈĞÅÏ¢£¬Îª1Ö¸Ê¾Ö¸Ê¾¹²ÓÃÌådataÖĞ´æµÄÊÇÇ°Ò»´ÎÊÕµ½µÄCPR±àÂëĞÅÏ¢
//position¹²ÓÃÌå±£´æ¾­Î³¶ÈĞÅÏ¢coordinate[0]ÊÇÎ³¶È£¬coordinate[1]ÊÇ¾­¶È£¬ÎªdoubleÀàĞÍ¸÷Õ¼8×Ö½Ú£»
//position»òÕß±£´æCPR±àÂëĞÅÏ¢CPR_code[0]ÎªÎ³¶È±àÂë£¬CPR_code[1]Îª¾­¶È±àÂë£¬CPR_code[2]ÎªÆæÅ¼±àÂëÖ¸Ê¾£¬ÓÃ×îµÍÎ»£¬CPR_code[3]Î´Ê¹ÓÃ£¬unsigned intÀàĞÍ£¬¸÷Õ¼4¸ö×Ö½Ú
//****************************************************************************************************************************
struct data_struct
{
	  unsigned int ICAO_adress_with_mark;  //190315
	  unsigned int time;
	  union
	  {
	   double coordinate[2];
	   unsigned int CPR_code[4];
	  }position;
}data_save[1000],new_data;
struct location_struct
{
	  unsigned int ICAO_address;
	  unsigned int time;
	  int coordinate[2];//ÕıÊ½ÉÏ±¨Ê±ºòÊ¹ÓÃintĞÍ£¬ÕıÊ½ÉÏ±¨Ê±ºòÊ¹ÓÃint(1E-7Îªµ¥Î»)¡£
	  int altitude;
};
struct location_struct airplane_location;//´Ë½á¹¹ÌåÊı×éºÍ½á¹¹Ìå±äÁ¿È«¾ÖÊ¹ÓÃ,airplane_location;

//zhangfulong add start ¶¨ÒåÒ»½á¹¹ÊÇlocation_struct ICAO Ê±¼ä ¾­Î³¶È ¸ß¶È 1500¸ö
struct location_struct airplane_location_static[position_BUFLEN];
struct location_struct airplane_location_static_ronghe[position_BUFLEN];//Î»ÖÃ¸øÈÚºÏÓÃ
//zhangfulong add end

#pragma DATA_ALIGN(data_save,32);//¶ÔÆë
//****************************************************************************************************************************
//½á¹¹Ìå±äÁ¿µÄÉùÃ÷±ØĞë°üº¬½á¹¹ÌåÀàĞÍ¶¨Òå±¾Éí,ÓÃÓÚËÙ¶ÈĞÅÏ¢µÄ¼ÆËã
//timeÊÇËÙ¶ÈĞÅÏ¢µÄÊ±¼ä±êÇ©£¬¾«È·µ½Ãë£»
//ICAO_adressÊÇICAOµØÖ·
//E_W_velocityÊÇ¶«Î÷·½ÏòËÙ¶È£¬¶«ÎªÕı£¬Î÷Îª¸º£¬µ¥Î»km/h£»N_S_velocityÊÇÄÏ±±·½ÏòËÙ¶È£¬±±ÎªÕı£¬ÄÏÎª¸º£¬µ¥Î»km/h£»
//VERT_velocityÊÇ´¹Ö±·½ÏòËÙ¶È£¬ÉÏÉıÎªÕı£¬ÏÂ½µÎª¸º£¬µ¥Î»m/sdisu£»

/*airplane_velocity.ICAO_addressµÄµÍ24bitÓÃÓÚ´æ·ÅICAOµØÖ·£¬µÚ25-27Î»ÓÃÓÚÖ¸Ê¾ËÙ¶È×´Ì¬£¨´Ó0Î»±àºÅ£¬µÚ24-26Î»£©
ÈıÎ»µÄ×éºÏ¶¨ÒåÎª001,010,011,100£¬ËÄÖÖ×´Ì¬£¬ÆäËü×´Ì¬ÎŞĞ§£»ÈıÎ»Ë³Ğò°´ÕÕÎª27£¬26£¬25´Ó¸ßµ½µÍµÄ·½Ê½¡£

µ±27-25Îª001Ê±£ºÖ¸Ê¾µ±Ç°¸ø³öµÄÊÇ"µØËÙ"£¬·Ç³¬ÒôËÙÄ£Ê½£¨¸ú·ÉĞĞÆ÷Êµ¼ÊÊÇ·ñ³¬ÒôËÙÎŞ¹Ø£©
´ËÊ±£º¶«Î÷»òÄÏ±±·½ÏòµÄËÙ¶È¾ø¶ÔÊıÖµÈçµÈÓÚ1891.818£¬±íÊ¾ÔÚ¶«Î÷»òÄÏ±±·½ÏòËÙ¶ÈÖµ³¬¹ı1891.818¹«Àï/Ğ¡Ê±£¬¾ßÌåÊıÖµÎ´¿ÉÖª¡££¨¾ø¶ÔÖµĞ¡ÓÚ1891.818Ê±£¬ÊıÖµ¶ÔÓ¦¸÷¸ö·½ÏòµÄËÙ¶È£©
      ÔÚ´¹Ö±·½ÏòµÄËÙ¶È¾ø¶ÔÖµÈçµÈÓÚ9938.9184£¬±íÊ¾´¹Ö±·½ÏòµÄËÙ¶È´óÓÚ9938.9184Ã×/·ÖÖÓ£¬¾ßÌåÊıÖµÎ´¿ÉÖª¡££¨¾ø¶ÔÖµĞ¡9938.9184Ê±£¬ÊıÖµ¶ÔÓ¦´¹Ö±·½ÏòµÄËÙ¶È?

µ±27-25Îª010Ê±£ºÖ¸Ê¾µ±Ç°¸øöµÄÊ?µØËÙ"£¬³¬ÒôËÙÄ£Ê½£¨¸ú·ÉĞĞÆ÷Êµ¼ÊÊÇ·ñ³¬ÒôËÙÎŞ¹Ø£©
´Ë±£º¶«Î÷»òÄÏ±±·½ÏòµÄËÙ¶È¾ø¶ÔÊıÖµÈçµÈÓ?567.272£¬±íÊ¾ÔÚ¶«Î÷»òÄÏ±±·½ÏòËÙ¶ÈÖµ³¬?567.272¹«Àï/Ğ¡Ê±£¬¾ßÌåÊıÖµÎ´¿ÉÖª¡££¨¾ø¶ÔÖµĞ¡ÓÚ7567.272Ê±£¬ÊıÖµ¶ÔÓ¦¸÷¸ö·½ÏòµÄËÙ¶È£©
      ÔÚ´¹Ö±·½ÏòµÄËÙ¶È¾ø¶ÔÖµÈçµÈÓÚ9938.9184£¬±íÊ¾´¹Ö±·½ÏòµÄËÙ¶È´óÓÚ9938.9184Ã×/·ÖÖÓ£¬¾ßÌåÊıÖµÎ´¿ÉÖª¡££¨¾ø¶ÔÖµĞ¡9938.9184Ê±£¬ÊıÖµ¶ÔÓ¦´¹Ö±·½ÏòµÄËÙ¶È£©

µ±27-25Îª011Ê±£ºÖ¸Ê¾µ±Ç°¸ø³öµÄÊÇ"¿ÕËÙ"£¬·Ç³¬ÒôËÙÄ£Ê½£¨¸ú·ÉĞĞÆ÷Êµ¼ÊÊÇ·ñ³¬ÒôËÙÎŞ¹Ø
´ËÊ±£ºÆ½Ãæ·½ÏòËÙ¶ÈÒÔ¼«×ø±ê·½Ê½¸ø³ö£¬ÎªÍ³Ò»±íÊ¾·½·¨£¬½«Æä·Ö½âÎª¶«Î÷ºÍÄÏ±±·½Ïò¸ø³öËÙ¶È¡£
      Èç«Î÷ºÍÄÏ±±·½Ïò¸ø³öµÄËÙ¶ÈÇ?ÏòÁ¿ºÍ"ºó£¬¾ø¶ÔÊıÖµµÈÓÚ1891.818£¬Ö¸Ê¾ÔÚÆ½ÃæÉÏµÄËÙ¶È´óÓÚ1891.818¹«Àï/Ğ¡Ê±£¬¾ßÌåÊıÖµÎ´¿ÉÖª¡££¨"ÏòÁ¿ºÍ"Ğ¡ÓÚ1891.818Ê±£¬ÊıÖµ¶ÔÓ¦Æ½ÃæÉÏµÄËÙ¶È£©      
      ÔÚ´¹Ö±·½ÏòµÄËÙ¶È¾ø¶ÔÖµÈçµÈÓÚ9938.9184£¬±íÊ¾´¹Ö±·½ÏòµÄËÙ¶È´óÓÚ9938.9184Ã×/·ÖÖÓ£¬¾ßÌåÊıÖµÎ´¿É¡££¨¾ø¶ÔÖµĞ¡9938.9184Ê±£¬ÊıÖµ¶ÔÓ¦´¹Ö±·½ÏòµÄËÙ¶È£©

µ±27-25Îª100Ê±£ºÖ¸Ê¾µ±Ç°¸ø³öµÄÊÇ"¿ÕËÙ"£¬³¬ÒôËÙÄ£Ê½£¨¸ú·ÉĞĞÆ÷Êµ¼ÊÊÇ·ñ³¬ÒôËÙÎŞ¹Ø£©
´ËÊ±£ºÆ½Ãæ·½ÏòËÙ¶ÈÒÔ¼«×ø±ê·½Ê½¸ø³ö£¬ÎªÍ³Ò»±íÊ¾·½·¨£¬½«Æä·Ö½âÎª¶«Î÷ºÍÄÏ±±·½Ïò¸ø³öËÙ¶È¡£
      Èç¹û¶«Î÷ºÍÄÏ±±·½Ïò¸ø³öµÄËÙ¶ÈÇó"ÏòÁ¿ºÍ"ºó£¬¾ø¶ÔÊıÖµµÈÓÚ7567.272£¬Ö¸Ê¾ÔÚÆ½ÃæÉÏµÄËÙ¶È´óÓÚ7567.272¹«Àï/Ğ¡Ê±£¬¾ßÌåÊıÖµÎ´¿ÉÖª¡££¨"ÏòÁ¿ºÍ"Ğ¡ÓÚ7567.272Ê±£¬ÊıÖµ¶ÔÓ¦Æ½ÃæÉÏµÄËÙ¶È£©        
      ÔÚ´¹Ö±·½ÏòµÄËÙ¶È¾ø¶ÔÖµÈçµÈÓÚ9938.9184£¬±íÊ¾´¹Ö±·½ÏòµÄËÙ¶È´óÓÚ9938.9184Ã×/·ÖÖÓ£¬¾ßÌåÊıÖµÎ´¿ÉÖª¡££¨¾ø¶ÔÖµĞ¡9938.9184Ê±£¬ÊıÖµ¶ÔÓ¦´¹Ö±·½òµÄËÙÈ£©*/
//****************************************************************************************************************************
struct speed_struct_three
{
	  unsigned int ICAO_address;
	  unsigned int time;
      int  N_S_velocity;      
	  int  E_W_velocity;  	  
      int  VERT_velocity;  
}airplane_velocity_three;

//zhangulong add start
struct speed_struct_three airplane_velocity_three_static[position_BUFLEN];  //1500
struct speed_struct_three airplane_velocity_three_static_ronghe[position_BUFLEN];  //ËÙ¶È¸øÈÚºÏÓÃ1500

struct location_struct airplane_location_tmp = {0};//ÓÃÀ´¼ÇÂ¼ÁÙÊ±µÄÃüÖĞµÄICAOºÅµÈĞÅÏ¢ Î»ÖÃ
struct speed_struct_three airplane_speed_tmp = {0};//ÓÃÀ´¼ÇÂ¼ÁÙÊ±µÄÃüÖĞµÄICAOºÅµÈĞÅÏ¢ ËÙ¶È
//zhangfulong add end

unsigned char velocity_subtype=0;//ËÙ¶È½âËãÈ«¾Ö±äÁ¿£¬ÓÃÓÚ´«µİĞÅÏ¢Ê¹ÓÃ£¬ÓÃ»§²»ÓÃ¹ØĞÄ
struct speed_code
{
	  unsigned char bit46;
	  unsigned short bit47_56;
	  unsigned char bit57;
	  unsigned short bit58_67;
	  unsigned char bit69;
	  unsigned short bit70_78;
}velocity_code;              //ËÙ¶È½âËãÈ«¾Ö½á¹¹Ìå±äÁ¿£¬ÓÃÓÚ´«µİĞÅÏ¢Ê¹ÓÃ£¬ÓÃ»§²»ÓÃ¹ØĞÄ
struct trans_Record
{
  unsigned int ICAO_adress_with_mark;  //190315
  unsigned int time;
  unsigned int flag;
}trans_Record[FIND_RANGE];
//}trans_Record[20];
//****************************************************************************************************************************
/**¾ÉÄ£ÄâÔ´**/
/*-----------------------------------------------------------------------------------------------**
** unsigned short int data_yuan[112]={1,0,0,0,  1,1,0,1,  0,1,0,0,  0,0,0,0,  0,1,1,0,  0,0,1,0, **
**                                    //8          D        4         0          6         2     **
**                                    0,0,0,1,  1,1,0,1,  0,1,0,1,  1,0,0,0,  1,1,0,0,  0,0,1,1, **
**                                    //1          D        5         8          C         3     **
**                                    1,0,0,0,  0,0,1,0,  1,1,0,1,  0,1,1,0,  1,0,0,1,  0,0,0,0, **
**                                    //8          2        D         6          9         0     **
**                                    1,1,0,0,  1,0,0,0,  1,0,1,0,  1,1,0,0,  0,0,1,0,  1,0,0,0, **
**                                    //C          8        A         C          2         8     **
**                                    0,1,1,0,  0,0,1,1,  1,0,1,0,  0,1,1,1 };                   **
**                                    //6          3        A         7                          **
**-----------------------------------------------------------------------------------------------*/
							   
/**ĞÂÄ£ÄâÔ´**/
unsigned short int data_yuan[112]={1,0,0,0,  1,1,0,0,  0,0,0,0,  0,0,0,1,  0,1,1,0,  0,0,1,0,
                                   //8          C        0         1          6         2
                                   0,0,0,1,  1,1,0,1,  0,1,0,1,  1,0,0,0,  1,1,0,0,  0,0,1,1,
                                   //1          D        5         8          C         3  
                                   1,0,0,0,  0,0,1,0,  1,1,0,1,  0,1,1,0,  1,0,0,1,  0,0,0,0,
                                   //8          2        D         6          9         0
                                   1,1,0,0,  1,0,0,0,  1,0,1,0,  1,1,0,0,  1,0,1,1,  0,1,1,0,
                                   //C          8        A         C          B         6
                                   0,1,0,1,  0,0,1,0,  1,0,0,1,  0,1,0,1 };
                                   //5          2        9         5  
    
/*****************************************/	  
/*****************************************/	
SEM_Handle sem0;
Uint8 AOS_Enable=0;
extern UINT8 bMacAddr[8]; //macµØÖ·
extern void watchdog();
extern unsigned short int decode_position();
extern unsigned int data_pro(unsigned short  *);
/*****************************************/
struct EDMA3CC_PaRAM
{
	unsigned int OPT;
	unsigned char *SRC;
	unsigned int BCNT_ACNT;
	unsigned char *DST;
	unsigned int DSTBIDX_SRCBIDX;
	unsigned int BCNTRLD_LINK;
	unsigned int DSTCIDX_SRCCIDX;
	unsigned int RSVD_CCNT;  
}*pEDMA3CC_PaRAM;
/*****************************************/
void EDMA_init()                    //initialize C6455 EDMA3                                
{
	EDMA3CC_DCHMAP48 = 0x00000020; // map channel 48(tied to gpio0 event) to PaRAM set 1 
	pEDMA3CC_PaRAM = EDMA3CC_PaRAM1;
	pEDMA3CC_PaRAM->OPT = 0x00130204;
//	pEDMA3CC_PaRAM->OPT = 0x00130004;  //170627
//	pEDMA3CC_PaRAM->OPT = 0x00100004; //TCINTEN=1;TCC=0;SYNCDIM=1;
	pEDMA3CC_PaRAM->SRC = (unsigned char *)&dma_test0;//EMIFA_CE4_BASE_ADDR;
	pEDMA3CC_PaRAM->BCNT_ACNT = 0x00000000; //ACNT=128;BCNT=1;
	pEDMA3CC_PaRAM->DST = (unsigned char *)&dma_test1;//FPGA_DATA_PING;
	pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00080008; //DSTBIDX=8;SRCBIDX=8;
	pEDMA3CC_PaRAM->BCNTRLD_LINK = 0x00804020; //LINK=4060, PaRMA SET 3
	pEDMA3CC_PaRAM->DSTCIDX_SRCCIDX = 0x0;
	pEDMA3CC_PaRAM->RSVD_CCNT = 0x00000001;
}
/*****************************************/
void ErrorPro(SOCKET sudp)
{
    if( sudp >= 0 )
        fdClose( sudp );
    printf("EchoSrv Fatal Error\n");
    TaskBlock( TaskSelf() );
}
/*****************************************/
void MEM_initial() //ÄÚ´æ¼°È«¾Ö±äÁ¿³õÊ¼»¯
{
    int  i=0;
//initials data rcv buffer from FPGA  
	for(i=0;i<1280;i++)
	{
	  	  first_fifo_1[i]=0;
		  first_fifo_2[i]=0;
	}
    for(i=0;i<112;i++)
    {
   		  pulse_amp[i]=0;
	 	  confi[i]=0;
    }
	for(i=0;i<24;i++)
    {	
  		  correct[i]=1;
    }
  	refer_amp=0;
  	adsb_message_counter=0;//ADSBÏûÏ¢µÄ¼ÆÊıÆ÷£¬ĞèÒªÇåÁã
	Yaoce_counter=0;//Ò£²âÆÊ
	edma_finished=0;
    int_cnt[0]=0x5a;
	int_cnt[1]=0;//
	int_cnt[2]=0;
	int_cnt[3]=0;
	int_cnt[4]=0;
	int_cnt[5]=0;
	int_cnt[6]=0;
 //   LOW_num=0;
 //   LOW_num_15=0;
 //   LOW_num_20=0; 
//¶Ô¸üĞÂÎ»ÖÃ±¨ÎÄĞÅÏ¢ÓÃµÄnew_dataÇå0£¬¶ÔÈ«¾Ö±£´æÎ»ÖÃĞÅÏ¢µÄ½á¹¹ÌåÊı×édata_save[]³õÊ¼»¯´¦Àí£¬×¢Òâ£ºÎªÁËÂú×ã¸üĞÂ²ßÂÔµÄÒªÇó
//data_save[i].ICAO_adress_with_markµÄµÚ25Î»ĞèÒªÉèÖÃÎª0£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡
//**************************************************************************
   new_data.time=0;
   new_data.ICAO_adress_with_mark=0;//
   new_data.position.coordinate[0] = 0.0;
   new_data.position.coordinate[1] = 0.0;
   for(i=0;i<1000;i++)
   {
     data_save[i].time=0;
     data_save[i].ICAO_adress_with_mark=0;
     data_save[i].position.coordinate[0] = 0.0;
     data_save[i].position.coordinate[1] = 0.0;
   }
//***************************************************************************
//³õÊ¼»¯¹À¼Æ²»È« £¨»¹ÓĞºÜ¶à±äÁ¿ĞèÒª³õÊ¼»¯£©
//**************************************************************************
   memset(pulse,0,sizeof(pulse)) ;
   memset(&new_data, 0, sizeof(new_data));     //190305     
   memset(data_save, 0, sizeof(data_save));
   memset(&velocity_code, 0, sizeof(velocity_code));              
   memset(&airplane_location, 0, sizeof(airplane_location));      //ÉÏ±¨¾­Î³Î»ÖÃÓÃ½á¹¹Ìå
   memset(&airplane_velocity_three, 0, sizeof(airplane_velocity_three));      //ÉÏ±¨ËÙ¶ÈÓÃ½á¹¹Ìå
   memset(&ADSB_message, 0, sizeof(ADSB_message));                //ÉÏ±¨Ô­Ê¼88bit±¨ÄĞÅÏ¢ÓÃ½á¹Ìå
  // memset(data_sample, 0, sizeof(data_sample)); 
   memset(g_8_ICAO_NAME,0,sizeof(g_8_ICAO_NAME));
   memset(velocity,0,sizeof(velocity));
   for(i=0;i<20;i++)
   {
     trans_Record[i].ICAO_adress_with_mark=0;
	 trans_Record[i].time=0;
	 trans_Record[i].flag=0;
   }
//***************************************************************************
}
/*****************************************/
void c_int07()    //int7 start   hhh
{
	int i,j,k;
  //  Uint8 int_cnt=0;//ÖĞ¶Ï¼ÆÊı
  	//	C62_disableIER(1<<9);  //1?¡À??D??
#if 1   
    CHAN_LENTH_FLAG = DSP_INIT_ADDR_2;   //ÒªĞ´2´Î
    CHAN_LENTH_FLAG = DSP_INIT_ADDR;//
    FPGA_CHAN_FLAG = CHAN_LENTH_FLAG & 0xff;//oxf
    if(FPGA_CHAN_FLAG==0x02) //2  //5  //1280 ADSB AD data****************
#endif
	{
	    flag=1;  //ÓÃÓÚêÊ¾ÊÇÊ²Ã´ÀàĞÍµÄÖĞ¶?
		int_cnt_1++;
		int_cnt[1]++;
		TestArray[0]=0xff;
		test_flag =0;  //190415
		EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32			
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_CHN2;  //ce4	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x05000004; //1280*4		
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;		
        if(SND_Catch_Program_Full<SEND_IP_DATA_BUFLEN-2)
		{    //»º³åÎ´Âú
	//		int_cnt = 0;
		//	while(1)
			{
			   //int_cnt++;
			   if(IP_Pkg_index<SEND_IP_DATA_BUFLEN-2)
			   {
					IP_Pkg_index++;
			   }
			   else
			   {
					IP_Pkg_index=0;
			   }
		/*	   if((SEND_ADDR_CACHE[IP_Pkg_index].pwrite == 0)||(int_cnt==SEND_IP_DATA_BUFLEN-1))
			   {
					break;
			   }
			   else
			   {
				    ;//tmp++;
			   }*/
			}//while
			for (i = 0; i < 1400; i++)
			{
				IP_DATA[IP_Pkg_index][i] = 0;
			}
	        pEDMA3CC_PaRAM->DST = (unsigned char *)&IP_DATA[IP_Pkg_index][0];
		//	t1=CLK_getltime();
			EDMA3CC_ESRH = 0x00010000;  //
		#if 0
			SEND_ADDR_CACHE[IP_Pkg_index].pwrite = 1; //
			SEND_ADDR_CACHE[IP_Pkg_index].IP_HDADDR=(Uint32)&IP_DATA[IP_Pkg_index][0];
			SND_Catch_Program_Full++; //190201
		#endif
			//SND_Catch_Program_Full++;
         }
         else
		 {   //Èç¹û»º³åÂúÁË   £¨²»Ó¦¸ÃÂú£©
			 pEDMA3CC_PaRAM->DST = (unsigned char *)&IP_DATA[SEND_IP_DATA_BUFLEN-1][12];
		     EDMA3CC_ESRH = 0x00010000;
			 SEND_ADDR_CACHE[SEND_IP_DATA_BUFLEN-1].pwrite = 0;//
			 SEND_ADDR_CACHE[SEND_IP_DATA_BUFLEN-1].IP_HDADDR=(Uint32)&IP_DATA[SEND_IP_DATA_BUFLEN-1][0];
		 }
	}
#if 1
	else if(FPGA_CHAN_FLAG==0x04) //1 // utc time04********
	{
        flag=2;
        int_cnt[9]++;
     	EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32			
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_UTC;  //ce4	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x00100004; //1280*4		
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	  //  pEDMA3CC_PaRAM->DST = (unsigned char *)UTC_Time;
	    //pEDMA3CC_PaRAM->DST = (unsigned char *)(&s_FPGA_YC); //16×Ö½ÚÒ£²â
		pEDMA3CC_PaRAM->DST = (unsigned char *)(UTC_time_all);	
		EDMA3CC_ESRH = 0x00010000;  //???¡¥

#if 0		
		//zhangfulong add Ê±¼ä¸üĞÂ  Ã»ÓĞÓÃ ÒªÔÚÆäËûµØ·½Ê¹ÓÃ¸üĞÂ£¬±ÜÃâÃ»´«Íê¾ÍÓÃ
		UTC_time_real[0] = UTC_time_all[13];
		UTC_time_real[1] = UTC_time_all[12];
		UTC_time_real[2] = UTC_time_all[15];
		UTC_time_real[3] = UTC_time_all[14];
		UTC_time_real[4] = UTC_time_all[17];
		UTC_time_real[5] = UTC_time_all[16];
		s_FPGA_YC.time = 0;
		//zhangfulong add Ê±¼ä¸üĞÂ
#endif


		s_FPGA_YC.UTCtime = 0;
		return;
	}
#if 0
	else if(FPGA_CHAN_FLAG==0x04)  //4  // IP
	{
        flag=3;
        int_cnt[11]++;  //
    	EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32			
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_DSTIP;  //ce4	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x00100004; //1280*4		
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	   	pEDMA3CC_PaRAM->DST = (unsigned char *)DSTIP;
		EDMA3CC_ESRH = 0x00010000;  //???¡¥
		return;
	}
#endif
    else if(FPGA_CHAN_FLAG==0x08)  //4  // test
	{
	    flag=4;
		test_flag =1; //½øÈë²âÊÔÄ£Ê½
	    int_cnt[12]++;//
    	EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32			
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_TEST;  //ce4	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x006a0004; //104*4	 	
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)TestArray;
		EDMA3CC_ESRH = 0x00010000;  //???¡¥
		return;
	} 
    else if(FPGA_CHAN_FLAG==0x0a)  //5  // test  
	{
	    flag=5;
		test_flag =0 ; //ÍË³ö²âÊÔÄ£Ê½
	    int_cnt[12]++;//
    	EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32			
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_TEST;  //ce4	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x006a0004; //104*4	 	
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)TestArray;
		EDMA3CC_ESRH = 0x00010000;  //???¡¥
		return;
	} 
	else if(FPGA_CHAN_FLAG==0xff)
	{
	   FPGA_CHAN_FLAG=0;
	}
#endif

	else if(FPGA_CHAN_FLAG==0x0b)  //20200811 ÕÅ¸»Â¡Ìí¼Ó£¬¶ÀÁ¢²¿·Ö½ÓÊÕICAOºÅ
	{
	    flag=4;
		test_flag =1; //½øÈë²âÊÔÄ£Ê½
	    int_cnt[12]++;//
    	EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32			
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_ICAO;  //ce4 ºÍIP¹²ÓÃÒ»¸öµØÖ·¿Õ¼ä	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x006a0004; //104*4	 	
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)ICAOArray;	//³¤¶È30*4B = 120 ¸öUINTĞÍ
		EDMA3CC_ESRH = 0x00010000;  //???¡¥
		return;
	} 
	else if(FPGA_CHAN_FLAG==0x55)//0x55×ÔÓÉÄ¿±êÊÕË÷Ä£Ê½
	{
		//
		EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32	
				
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_ICAO;  //ce4 ºÍIP¹²ÓÃÒ»¸öµØÖ·¿Õ¼ä	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x00860004; //	 0x86 = 134	
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)(&yaokongcmd_all_data.mode);;	//³¤¶È30*4B = 120 ¸öUINTĞÍ
		EDMA3CC_ESRH = 0x00010000;
		//
		/*
		Uint32 ICAO_number[30] = {0};//ÓÃÀ´½«ÊÕµ½µÄICAOºÅ´µ?2Î»µÄÊı×éÀïÃæ
		int taokong_workmode = 0;// 0±íÊ¾Ã»ÓĞ·¢£¬1£º×ÔÓÉÄ¿±êËÑË÷ £»2£ºÄ¿±ê¸ú×Ù
		int yaokong_jingdu = 0.0;
		int yaokong_weidu = 0.0;
		int yaokong_gaodu = 0.0;
		int yaokong_banzhuijiao = 0;//0-90
		*/
		taokong_workmode = 1;
	}
	else if(FPGA_CHAN_FLAG==0xAA)//0x55Ä¿±ê¸ú×ÙÄ£Ê½
	{
		//
		EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32	
				
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_ICAO;  //ce4 ºÍIP¹²ÓÃÒ»¸öµØÖ·¿Õ¼ä	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x00860004; //	 0x86 = 134	
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)(&yaokongcmd_all_data.mode);;	//³¤¶È30*4B = 120 ¸öUINTĞÍ
		EDMA3CC_ESRH = 0x00010000;
		//
		/*
		Uint32 ICAO_number[30] = {0};//ÓÃÀ´½«ÊÕµ½µÄICAOºÅĞ´µ½32Î»µÄÊı×éÀïÃæ
		int taokong_workmode = 0;// 0±íÊ¾Ã»ÓĞ·¢£¬1£º×ÔÓÉÄ¿±êËÑË÷ £»2£ºÄ¿±ê¸ú×Ù
		int yaokong_jingdu = 0.0;
		int yaokong_weidu = 0.0;
		int yaokong_gaodu = 0.0;
		int yaokong_banzhuijiao = 0;//0-90
		*/
		taokong_workmode = 2;
	}
	else if(FPGA_CHAN_FLAG==0x03)//0x03 Ò£¿ØÄ£Ê½
	{
		//flag=4;
		//test_flag =1; //½øÈë²âÊÔÄ£Ê½
	    //int_cnt[12]++;//
		//for (i = 0; i < 200; i++)
		//{
		//	ICAOArray[i] = 0;
		//}
    	EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32			
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_ICAO;  //ce4 ºÍIP¹²ÓÃÒ»¸öµØÖ·¿Õ¼ä	
		//pEDMA3CC_PaRAM->BCNT_ACNT =0x006a0004; //104*4
		pEDMA3CC_PaRAM->BCNT_ACNT =0x00C80004;	 //0xC8 = 200	
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)ICAOArray;	//³¤¶È30*4B = 120 ¸öUINTĞÍ
		EDMA3CC_ESRH = 0x00010000;  //???¡¥
#if 0		
		//µ÷ÕûË³Ğò£¬Êı¾İ´¦Àí²¿·Ö·ÅÔÚleadÈÎÎñforÑ­»·¿ªÍ·×´Ì¬£¬Õâ¶ÎÃ»ÓÃ£¬Ó¦¸ÃÒÆ¶¯×ß
		for (i = 0; i < 200; i = i + 2)
		{
			ICAOArray_real[i] = ICAOArray[i + 1];
			ICAOArray_real[i + 1] = ICAOArray[i];
		}
		
		if ((ICAOArray_real[12] = 0xbb) && (ICAOArray_real[13] = 0x77) && (ICAOArray_real[14] = 0x3D) )
		{
			//°üÍ·ÕıÈ· £¬ÏÂÃæ½øĞĞĞ£ÑéºÍ¼ÆËã ´Ó 12 µÄ00 04 ¿ªÊ¼  Uint16 check_sum_1,check_sum_2;
			check_sum_1 = 0;
			check_sum_2 = 0;
			for (i = 12; i < 151; i = i +2 )
			{
				check_sum_1 = (check_sum_1 ^ ( (Uint16)(ICAOArray_real[i] << 8)+(Uint16)(ICAOArray_real[i]) ) );
			}
			check_sum_2 = ( (Uint16)(ICAOArray_real[152] << 8)+(Uint16)(ICAOArray_real[153]) );
//			if (check_sum_1 != check_sum_2)
			{
				//ÄÜ½øÀ´ËµÃ÷Ğ£ÑéºÍÕıÈ·£¬ÏÂÃæ¿ªÊ¼½«Öµ¸üĞÂµ½ÎÒÃÇµÄ½á¹¹ÌåÀïÃæ
				if (ICAOArray_real[15] == 0x55)
				{
					yaokongcmd_all_data.mode = 1;
				}
				else if (ICAOArray_real[15] == 0xAA)
				{
					yaokongcmd_all_data.mode = 2;
				}
				else
				{
					yaokongcmd_all_data.mode = 0;
				}
				memcpy(&(yaokongcmd_all_data.jingdu),&(ICAOArray_real[16]),4);//¾­¶È
				memcpy(&(yaokongcmd_all_data.weidu),&(ICAOArray_real[20]),4);//Î³¶È
				memcpy(&(yaokongcmd_all_data.gaodu),&(ICAOArray_real[24]),4);//¸ß¶È
				yaokongcmd_all_data.banzhuijiao = ICAOArray_real[28];
				yaokongcmd_all_data.chelue_1 = ICAOArray_real[29];
				yaokongcmd_all_data.chelue_2 = ICAOArray_real[30];
				yaokongcmd_all_data.chelue_3 = ICAOArray_real[31];
				for (k = 0; k < 30; k++)
				{
					memcpy(&(yaokongcmd_all_data.ICAO[k]),&(ICAOArray[36 + k*4]),4);;
				}
				;
			}			
		}//µ½´Ë£¬ÄÃ×ß
#endif
/*
		if (ICAOArray[1] == 0x55)//×ÔÓÉÄ¿±êËÑË÷
		{
			yaokongcmd_all_data.mode = 1;
			memcpy(&(yaokongcmd_all_data.jingdu),&(ICAOArray[2]),4);//¾­¶È
			memcpy(&(yaokongcmd_all_data.weidu),&(ICAOArray[6]),4);//Î³¶È
			memcpy(&(yaokongcmd_all_data.gaodu),&(ICAOArray[10]),4);//¸ß¶È
			yaokongcmd_all_data.banzhuijiao = ICAOArray[14];
			yaokongcmd_all_data.chelue_1 = ICAOArray[15];
			yaokongcmd_all_data.chelue_2 = ICAOArray[16];
			yaokongcmd_all_data.chelue_3 = ICAOArray[17];
			for (k = 0; k < 30; k++)
			{
				memcpy(&(yaokongcmd_all_data.ICAO[k]),&(ICAOArray[18 + k*4]),4);;
			}
		}
		else if (ICAOArray[1] == 0xAA)//¸ú×ÙÄ£Ê½
		{
			yaokongcmd_all_data.mode = 2;
			memcpy(&(yaokongcmd_all_data.jingdu),&(ICAOArray[2]),4);//¾­¶È
			memcpy(&(yaokongcmd_all_data.weidu),&(ICAOArray[6]),4);//Î³¶È
			memcpy(&(yaokongcmd_all_data.gaodu),&(ICAOArray[10]),4);//¸ß¶È
			yaokongcmd_all_data.banzhuijiao = ICAOArray[14];
			yaokongcmd_all_data.chelue_1 = ICAOArray[15];
			yaokongcmd_all_data.chelue_2 = ICAOArray[16];
			yaokongcmd_all_data.chelue_3 = ICAOArray[17];
			for (k = 0; k < 30; k++)
			{
				memcpy(&(yaokongcmd_all_data.ICAO[k]),&(ICAOArray[18 + k*4]),4);;
			}
		}
		else
		{
			yaokongcmd_all_data.mode = 0;
		}
		//taokong_workmode = 2;
*/
		return;
	}
	else
	{
		;
	}

}
/**********************************************/
void EDMA_int()  //EDMAÖĞ¶Ï
{
	int_cnt[5]++;
 	EDMA3CC_ICRH=0x00010000;   //ÇåedmaÖĞ¶Ï   //???170718
  //  C62_clearIFR(1<<8); 
 //     *(CIPRL)=0x10;//Çå³ıDMA4ÖĞ¶Ï
 //     *(ECRL)=0x10;//Çå³ıDMA4ÖĞ¶Ï
//	   *(GPVAL)=*(GPVAL)&0xfffffdff;  //GPIO
//----------------------------------------------------
// Ä¿Ç°Æô¶¯EDMAµÄÓĞ4ÀàÊı¾İ £¨ÊÇ·ñ¶¼ĞèÒªÖĞ¶Ï£©
//EDMA ÖĞ¶ÏĞèÒª¸ù¾İ±êÖ¾´¦Àí£¿
   if(flag==1){
			SEND_ADDR_CACHE[IP_Pkg_index].pwrite = 1; //
			SEND_ADDR_CACHE[IP_Pkg_index].IP_HDADDR=(Uint32)&IP_DATA[IP_Pkg_index][0];
			SND_Catch_Program_Full++; //190201
	        edma_finished = 1;
   }
   else if(flag==2)
   {
		//zhangfulong add Ê±¼ä¸üĞÂ
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				
		//
    edma_finished = 1;
   }
   else if(flag==4)
   {
       test_time=(Uint32)(TestArray[11]<<24)+(Uint32)(TestArray[10]<<16)
            +(Uint32)(TestArray[13]<<8)+(Uint32)(TestArray[12]);
       flag=0;
   }
   else if(flag==3) //190420
   {
   		if((DSTIP[0]==0xe5)&&(DSTIP[1]==0x8A))
		{
		  fpga_SetIP   = (Uint32)(DSTIP[4]<<24)+ (Uint32)(DSTIP[5]<<16)
		                  + (Uint32)(DSTIP[2]<<8)+ (Uint32)(DSTIP[3]) ; //IPµØÖ·
		  fpga_SetPort =  (Uint16)(DSTIP[6]<<8)+ DSTIP[9]; //Êı¾İÍ¨´ï
		  fpga_SetCH   =  DSTIP[7];//¶Ë¿ÚºÅ
		  fpga_SetFlag =1;
		}
   }
#if 0
   else if(flag==4){//
      if((TestArray[0]==0x5a)&&(TestArray[1]==0x5a))
	  {
      	test_flag =1;
	  }
	  else
	  {
		test_flag =0;
	  }
   }
#endif
   flag=0;
 // fpga_cnt[fpga_err_cnt]=data_fifo[3];
//    C62_enableIER( 1<<9 );//4
//	DSP_TO_FPGA_INT_EN = 0x1234;  //CE4 +0x1c
}
/**********************************************/
//·µ»ØÖµ0£¬Ö¸Ê¾¸ÃÖ¡²ÉÑùÊı¾İ²»Õı³£/»òÊÇ½âµ÷ÖÆºóÊı¾İ²»ÕıÈ·/»ò²»ÊÇDF17µÄ±¨ÎÄ, ADSB_message±äÁ¿²»»á±»¸üĞÂ£»
  //·µ»ØÖµ1£¬Ö¸Ê¾¸ÃÖ¡²ÉÑùÊı¾İ´¦ÀíÕı³££¬µ«¸ÃÖ¡²»ÊÇÎ»ÖÃ±¨ÎÄ»òËÙ¶È±¨ÎÄ£¬½ö¸üĞÂADSB_message±äÁ¿ÓÃÓÚÉÏ´«Ô­Ê¼±¨ÎÄ£»
  //        £¨Ğø£©ÓÃ»§¿ÉÔÚADSB_messageÊı×éÖĞÌáÈ¡ĞèÒªÉÏ±¨µÄÔ­Ê¼88bit±¨ÎÄ£¬°´ÕÕ×Ö½Ú´æ·Å

  //·µ»ØÖµ2£¬Ö¸Ê¾¸ÃÖ¡²ÉÑùÊı¾İ´¦ÀíÕı³££¬²¢ÇÒ¸ÃÖ¡ÊÇÎ»ÖÃ±¨ÎÄ£¬µ«ÊÇ½âËã²»³É¹¦£¬½ö¸üĞÂÁËADSB_message±äÁ¿£¬
  //        £¨Ğø£©ÓÃ»§¿ÉÔÚADSB_messageÊı×éÖĞÌáÈ¡ĞèÒªÉÏ±¨µÄÔ­Ê¼88bit±¨ÎÄ£¬°´ÕÕ×Ö½Ú´æ·Å
  //·µ»ØÖµ3£¬Ö¸Ê¾¸ÃÖ¡²ÉÑùÊı¾İ´¦ÀíÕı³££¬²¢ÇÒ¸ÃÖ¡ÊÇÎ»ÖÃ±¨ÎÄ£¬½âËã³É¹¦£¬¸üĞÂÁËADSB_message±äÁ¿£¬¸üĞÂÁËÊı¾İ¿â£¬¸üĞÂÁËÉÏ±¨Î»ÖÃÓÃµÄairplane_location½á¹¹Ìå±äÁ¿
  //	    £¨Ğø£©ÓÃ»§¿ÉÔÚADSB_messageÊı×éÖĞÌáÈ¡ĞèÒªÉÏ±¨µÄÔ­Ê¼88bit±¨ÎÄ£¬°´ÕÕ×Ö½Ú´æ·Å
   //	    £¨Ğø£©ÓÃ»§¿ÉÔÚairplane_location½á¹¹Ìå±äÁ¿ÖĞÌáÈ¡ICAOºÍÎ»ÖÃĞÅÏ¢ÉÏ±¨å¶¨Òå¼ûÈ«¾Ö±äÁ¿¶¨Òå£    
  //·µ»ØÖµ4£¬Ö¸Ê¾¸ÃÖ¡²ÉÑù´¦ÀíÊÇËÙ¶È±¨ÎÄ£¬ÓÉÓÚÎ´ÖªµÄÔ­òÎ´½âËã³öÓĞĞ§µÄËÙ¶ÈĞÅÏ¢£¬½ö¸üĞÂÁËADSB_message±äÁ¿£¬£¨È±£©
  //·µ»ØÖµ5£¬Ö¸Ê¾¸ÃÖ¡´¦ÀíÊÇËÙ¶È¨ÎÄ£¬½âËã³É¹¦£¬¸üĞÂÁËADSB_message±äÁ¿¸üĞÂÁËÉÏ±¨Î»ÖÃÓÃµÄXXXÊı×é£¬ÉÏ±¨ËÙ¶ÈĞÅÏ¢£¨È±£©
  
//message_type·µ»ØÖµ0£¬Ö¸Ê¾¸ÃÖ¡²ÉÑùÊı¾İ²»Õı³£,»òÊÇ½âµ÷ÖÆºóÊı¾İ²»ÕıÈ·,»ò²»ÊÇDF17µÄ±¨ÎÄ, ADSB_message±äÁ¿²»»á±»¸üĞÂ;´ËÊ±ÎŞÈÎºÎĞÅÏ¢ĞèÒªÉÏ´«»ò±£´æ£¬µÈ´ı½øÈëÏÂÒ»´Î²ÉÑùÊı¾İµÄ´¦í¡?

//message_type·µ»ØÖµ1£¬Ö¸Ê¾¸ÃÖ¡²ÉÑùÊı¾İ´¦ÀíÕı³££¬µ«¸ÃÖ¡²»ÊÇÎ»ÖÃ±¨ÎÄ»òËÙ¶È±¨ÎÄ£¬½ö¸üĞÂADSB_message½á¹¹Ìå±äÁ¿ÓÃÓÚÉÏ´«Ô­Ê¼±¨ÎÄ£»
//                   £¨ø©¿ÉÔÚADSB_message½á¹¹ÌåÖĞÌáÈ¡ĞèÒªÉÏ±¨µÄÔ­Ê¼88bit±¨ÎÄ£¬ADSB_messageµÄ¶¨Òå·½Ê½¼û½á¹Ìå±äÁ¿¶¨Òå¡?

//message_type·µ»ØÖµ2£¬Ö¸Ê¾¸ÃÖ¡²ÉÑùÊı¾İ´¦ÀíÕı³££¬²¢ÇÒ¸ÃÖ¡ÊÇÎ»ÖÃ±¨ÎÄ£¬µ«ÊÇ¾­Î³Î»ÖÃ½âËã²»³É¹¦£¬½ö¸üĞÂADSB_message½á¹¹Ìå±äÁ¿ÓÃÓÚÉÏ´«Ô­Ê¼±¨ÎÄ£»
//                   £¨Ğø£©ÓÃ»§¿ÉÔÚADSB_message½á¹¹ÌåÖĞÌáÈ¡ĞèÒªÉÏ±¨µÄÔ­Ê¼88bit±¨ÎÄ£¬ADSB_messageµÄ¶¨Òå·½Ê½¼û½á¹¹Ìå±äÁ¿¶¨Òå¡£
//message_type·µ»ØÖµ3£¬Ö¸Ê¾¸ÃÖ¡²ÉÑùÊı¾İ´¦ÀíÕı³££¬²¢ÇÒ¸ÃÖ¡ÊÇÎ»ÖÃ±¨ÎÄ£¬¾­Î³Î»ÖÃ½âËãADSB_message½á¹¹Ìå±äÁ¿£¬¸üĞÂËÉÏ±¨¾­Î³Î»ÖÃÓÃµÄairplane_location½á¹¹Ìå±äÁ¿£»
//	                 £¨Ğø£©ÓÃ»§¿ÉÔÚADSB_message½á¹¹ÌåÖĞÌáÈ¡ĞèÒªÉÏ±¨µÄÔ­Ê¼88bit±¨ÎÄ£¬ADSB_messageµÄ¶¨Òå·½Ê½¼û½á¹¹Ìå±äÁ¿¶¨Òå¡£
//	                 £¨Ğø£©ÓÃ»§¿ÉÔÚairplane_location½á¹¹Ìå±äÁ¿ÖĞÌáÈ¡ICAO¡¢Ê±¼äºÍ¾­Î³Î»ÖÃĞÅÏ¢ÉÏ±¨, airplane_locationµÄ¶¨Òå·½Ê½¼û½á¹¹Ìå±äÁ¿¶¨Òå¡£
   
//message_type·µ»ØÖµ4£¬Ö¸Ê¾¸ÃÖ¡²ÉÑùÊı¾İ´¦ÀíÕı³££¬²¢ÇÒ¸ÃÖ¡ÊÇËÙ¶È¨ÎÄ£¬ÓÉÓÚ¸÷ÖÖÔ­ÒòÎ´½âËã³öÓĞĞ§µÄËÙ¶ÈĞÅÏ¢£¬½ö¸üĞÂADSB_message½á¹¹Ìå±äÁ¿ÓÃÓÚÉÏ´«Ô­Ê¼±¨ÎÄ£»
//	                 £¨Ğø£©ÓÃ»§¿ÉÔÚADSB_message½á¹¹ÌåÖĞÌáÈ¡ĞèÒªÉÏ±¨µÄÔ­Ê¼88bit±¨ÎÄ£¬ADSB_messageµÄ¶¨Òå·½Ê½¼û½á¹¹Ìå±äÁ¿¶¨Òå¡£
//message_type·µ»ØÖµ5£¬Ö¸Ê¾¸ÃÖ¡²ÉÑùÊı¾İ´¦ÀíÕı³££¬²¢Ò¸ÃÖ¡ÊÇËÙ¶È±¨ÎÄ£¬ËÙ¶È½âËã³É¹¦£¬¸üĞÂÁËADSB_message±äÁ¿£¬¸üĞÂÁËÉÏ±¨ËÙ¶ÈĞÅÏ¢ÓÃµÄairplane_velocity½á¹¹Ìå±äÁ¿£»
//	                 £¨Ğø£©ÓÃ»§¿ÉÔÚADSB_message½á¹¹ÌåÖĞÌáÈ¡ĞèÒª±¨µÄÔ­Ê¼88bit±¨ÎÄ£¬ADSB_messageµÄ¶¨Òå·½Ê½¼û½á¹¹Ìå±äÁ¿¶¨Òå¡£
//	                 £¨Ğø£©ÓÃ»§¿ÉÔÚairplane_velocity½á¹¹Ìå±äÁ¿ÖĞÌáÈ¡ICAO¡¢Ê±¼äºÍËÙ¶ÈÎ»ÖÃĞÅÏ¢ÉÏ±¨, airplane_velocityµÄ¶¨Òå·½Ê½¼û½á¹¹Ìå±äÁ¿¶¨Òå¡£
                       

/*******************************/
#if 1
//±¨ÎÄ1.2 1400
//Í·(6)+ÀàĞÍ(2)+ÈÎÎñ(2)+°üÄÚ¼ÆÊı(2)+Ê±¼ä(4)+ÓĞĞ§Ä¿±ê(2)+ICAO£¨16*86£©+Ìî³ä(6)

//±¨ÎÄ3  1400
//Í·(6)+¼ÆÊı(1)+Ò£²â(13)+ÓĞĞ§Êı(2)+ads(75*17)+icao(5*20)+Ìî³ä(3)
void udp_sndPacket()  //ÈÎÎñ4 ·¢ËÍÁ½ÖÖ±¨ÎÄºÍ²âÊÔÊı¾İ 
{
	SOCKET   							sudp = INVALID_SOCKET;
  	struct   sockaddr_in 				sin1,sin;
	int 	 							reuse = 1;
	int    								i=0,j=0;
    unsigned int 						recvsize = 1024*8;
	Uint8 pbuf1[1432];//²âÊÔÊı¾İ
	Uint8 pbuf2[1432];//ÈÚºÏÊı¾İ
	Uint8 pbuf3[1432];//adsÊı¾İ
    Uint32 total_cnt=0;  //×Ü¼ÆÊı ÔÚ
	Uint32 total_cnt_2=0;  //×Ü¼ÆÊı ÔÚ
    Uint16 test_cnt=0;   // °üÄÚ¼ÆÊı
	Uint16 join_cnt=0;  //°üÄÚ¼ÆÊı
    Uint8 ads_cnt=0 ;   //°üÄÚ¼ÆÊı
    Uint8 ads_snd_t=0;    //ÅĞÊÇ·ñĞèÒªÌî³ä
    Uint8 rh_snd_t=0;    //ÅĞÊÇ·ñĞèÒªÌî³ä
	Uint32 hhtmp;
#if 0
	Uint32 udp_snd_lost=0;
	Uint32 udp_snd_lost_1=0;
	Uint32 udp_snd_lost_2=0;
#endif
	const Uint8 pack32[32]={
		0x0,0x0,0x0,0x0,0x0,0x55,//
		0x55,0x55,0x55,0x55,0x55,0x55,	//190610
		0x55,0x55,0x55,0x55,0x55,0x55,
		0x55,0x55,0x55,0x55,0x55,0x55,
		0x55,0x55,0x55,0x55,0x55,0x55,
	    0x55,0x55
	};//32×Ö½Ú--ÈÎÎñºÅ(1)--Êı¾İÀàĞÍ(1)--Ö¡¼ÆÊı£¨3£©--Êı¾İÍ¨µÀ£¨1£©--Ô¤Áô£¨26£©

	const Uint8 pack_samp1[6]={
	0x9e,0xda,0xa7,0x8b,0xF4,0xc5
    } ;///²âÊÔÊı¾İ
    const Uint8 pack_samp2[6]={
	0x9e,0xda,0xa7,0x8b,0xF4,0xc5
    } ;//ÈÚºÏÊı¾İ
    const Uint8 pack_samp3[6]={
	0x61,0x25,0x58,0x74,0x0b,0x3a
    } ;//adsÊı¾İ

    const Uint8 pack_reserve[6]={
    0x5a,0x5a,0x5a,0xff,0xff,0xff
    } ;

    const Uint8 pack_ads_nodata[20]={0
    } ;
  //---32
    memcpy(&(pbuf1[0]), pack32, 32);//
    memcpy(&(pbuf2[0]), pack32, 32);//
    memcpy(&(pbuf3[0]), pack32, 32);// 
  //--6
    memcpy(&(pbuf1[32]), pack_samp1, 6);//
    memcpy(&(pbuf2[32]), pack_samp2, 6);//
    memcpy(&(pbuf3[32]), pack_samp3, 6);//
//--------------------1
    pbuf1[38] =0xdd; 
    pbuf1[39] =0xdd;  //190418

	for(i=50;i<1426;i++)
	{
	  	pbuf1[i]=0;
	}
	pbuf1[1426]=0xaa;//Î²²¿Ìî³ä
	pbuf1[1427]=0xaa;
	pbuf1[1428]=0xaa;
	pbuf1[1429]=0xaa;
	pbuf1[1430]=0xaa;
	pbuf1[1431]=0xaa;
//--------------------2
    pbuf2[38] =0xdd; //190418
    pbuf2[39] =0xdd;

	pbuf2[1426]=0xaa;//Î²²¿Ìî³ä
	pbuf2[1427]=0xaa;
	pbuf2[1428]=0xaa;
	pbuf2[1429]=0xaa;
	pbuf2[1430]=0xaa;
	pbuf2[1431]=0xaa;
//-------------------3   
    for(i=0;i<75;i++)//±¨ÎÄ3µÄÌî³ä
	{
    	memcpy(&(pbuf3[55+i*17]),pack_reserve,6);
    }
	pbuf3[1429]=0x55;//Î²²¿Ìî³ä
	pbuf3[1430]=0x55;
	pbuf3[1431]=0x55;
     
	fdOpenSession( TaskSelf() );
  	bzero( &sin1, sizeof(struct sockaddr_in) );
    sin1.sin_family = AF_INET;
    sin1.sin_len    = sizeof( sin1 );
    sin1.sin_port   = htons(55000);//htons(7);;//6006;//
	sin1.sin_addr.s_addr=inet_addr(LocalIpAddr); //

	bzero( &sin, sizeof(struct sockaddr_in) );
    sin.sin_family = AF_INET;
	sin.sin_port = htons(55002);        //default
    sin.sin_len    = sizeof( sin );
	sin.sin_addr.s_addr =0x0100a8c0;   //  default

 	sudp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if( sudp ==INVALID_SOCKET )
        ErrorPro( sudp);
	if (setsockopt(sudp, SOL_SOCKET, SO_REUSEPORT, (char *)&reuse, sizeof(reuse)) < 0)
    {
        printf ("Error: Unable to set the reuse port socket option\n", fdError());
		return;
    }  
	if (setsockopt (sudp, SOL_SOCKET, SO_RCVBUF, (void *)&recvsize, sizeof(recvsize)) < 0)
	{
		printf ("Error: Unable to confgiure sock size; Error: %d\n", fdError());
		return;
	}
    if ( bind( sudp, (PSA) &sin1, sizeof(sin1) ) < 0 )  //Ô´
        ErrorPro( sudp);

   while(1)
   {
		//zhangfulong add Ê±¼ä¸üĞÂ
		UTC_time_real[0] = UTC_time_all[13];
		UTC_time_real[1] = UTC_time_all[12];
		UTC_time_real[2] = UTC_time_all[15];
		UTC_time_real[3] = UTC_time_all[14];
		UTC_time_real[4] = UTC_time_all[17];
		UTC_time_real[5] = UTC_time_all[16];
		//zhangfulong add Ê±¼ä¸üĞÂ

 	  if(fpga_SetFlag==1)
	  {    
	      sin.sin_addr.s_addr =fpga_SetIP;   // 0x0100a8c0
		  sin.sin_port = htons(fpga_SetPort);//55002	
        //  sin.sin_port = fpga_SetPort;
	      pbuf1[5]= fpga_SetCH ;
	      pbuf2[5]= fpga_SetCH ;
		  pbuf3[5]= fpga_SetCH ;
	      fpga_SetFlag=0;
	  }
      if(test_flag==1)  //·¢ËÍ²âÊÔÊı¾İ
	  {  
//----------²âÊÔÊı¾İ--------------------------	    
        pbuf1[1]=0x0d;//ÈÚºÏÊı¾İ
		if(total_cnt<0xffffff)
		{
			total_cnt++;
		}
		else
		{
 		    total_cnt=0;
		}
       // memcpy(&(pbuf1[2]), &total_cnt, 3);//×Ü¼ÆÊı
	    pbuf1[2] =(total_cnt>>16);
        pbuf1[3] =(total_cnt & 0xFFFF)>>8;
	    pbuf1[4] = (Uint8)(total_cnt & 0xFF);
//[41][42]ÈÎÎñºÅ
        //pbuf1[40]=s_FPGA_YC.reserve[0];  //40-41
        //pbuf1[41]=s_FPGA_YC.reserve[1];  //
        pbuf1[40]=TestArray[9];  //40-41
        pbuf1[41]=TestArray[8];

      //  memcpy(&(pbuf1[42]), &test_cnt, 2);//²âÊÔ¼ÆÊı
    //    pbuf1[42]=test_cnt<<8;// 190927
        pbuf1[42]=test_cnt>>8; //190927
        pbuf1[43]=(unsigned char)test_cnt&0xff;
		test_cnt++;
      //  memcpy(&(pbuf1[44]), &s_FPGA_YC.UTCtime, 4); //utcÊ±¼ä
	   // memcpy(&(pbuf1[44]), &test_time, 4);
 		hhtmp=htonl(test_time);
		memcpy(&(pbuf1[44]), &hhtmp, 4);
	
        pbuf1[49]=TestArray[14]; //¸öÊı
        pbuf1[48]=TestArray[15];
        
        for(i=50;i<178;i++)  //190610
		{
		  pbuf1[i]=0;
		}
        for(i=0;i< TestArray[14];i++)
		{
      //   memcpy(&(pbuf1[50+i*16]),&TestArray[16+i*12], 4);
             pbuf1[54+i*16]=TestArray[17+i*12];  //50  //190610
             pbuf1[55+i*16]=TestArray[16+i*12];  //51  //190610
    		 pbuf1[56+i*16]=TestArray[19+i*12];  //52  //190610
   			 pbuf1[57+i*16]=TestArray[18+i*12];  //53  //190610
	 //   memcpy(&(pbuf1[54+i*16]),&test_time,4);
	     	 memcpy(&(pbuf1[50+i*16]), &hhtmp, 4);  //190610 //54
	  //    memcpy(&(pbuf1[58+i*16]),&TestArray[20+i*12], 4);
             pbuf1[58+i*16]=TestArray[21+i*12];
             pbuf1[59+i*16]=TestArray[20+i*12];
    		 pbuf1[60+i*16]=TestArray[23+i*12];
   			 pbuf1[61+i*16]=TestArray[22+i*12];
	     //  memcpy(&(pbuf1[62+i*16]),&TestArray[24+i*12], 4);
             pbuf1[62+i*16]=TestArray[25+i*12];
             pbuf1[63+i*16]=TestArray[24+i*12];
    		 pbuf1[64+i*16]=TestArray[27+i*12];
   			 pbuf1[65+i*16]=TestArray[26+i*12];
		}
		test_time++;
// [49][50]--¸öÊı    
//[51]¿ªÊ¼¹²¼ÇÂ¼86×é¼ä¸ô16
     //   memcpy(&(pbuf1[49]),TestArray,200);//   ÔõÃ´Ìî³ä£¬´ı¶¨
	    if(TestArray[14]>0)  //190610
		{
		  	if(sendto( sudp,pbuf1, 1432, 0, (struct sockaddr *)&sin, sizeof(sin) )<0)
			{
				;
				//udp_snd_lost++;//	  cmd_buf_test[4]=hh_udp_snd_cnt>>8;
			}
			//	hh_udp_snd_cnt++;
		}
	  }
	  else  //·¢ËÍÁ½ÖÖÍøÂç±¨ÎÄ
	  {
//------  ÈÚºÏÊı¾İ ----------------------
#if 1
      for(j=0;j<2;j++)   //2
	  {
		if(position_cnt>0)  //±ØĞëÒªÓĞÓĞĞ§µÄÎ»ÖÃĞÅÏ¢
		{
		    pbuf2[1]=0x0d;//ÈÚºÏÊı¾İ 
	        //memcpy(&(pbuf2[2]), &total_cnt, 3);//×Ü¼ÆÊı
           	if(total_cnt<0xffffff)
			{
				total_cnt++;
			}
			else
			{
	 		    total_cnt=0;
			}
            pbuf2[2] =(total_cnt>>16);
            pbuf2[3] =(total_cnt & 0xFFFF)>>8;
			pbuf2[4] = (Uint8)(total_cnt & 0xFF);
	//[41][42]ÈÎÎñºÅ
			pbuf2[40]=s_FPGA_YC.reserve[0];  //
			pbuf2[41]=s_FPGA_YC.reserve[1];
	   //     memcpy(&(pbuf2[42]), &join_cnt, 2);//ÈÚºÏ¼ÆÊı
	  //		pbuf2[42] =join_cnt<<8;//190927
        	pbuf2[42] =join_cnt>>8;//190927
			pbuf2[43] =(unsigned char)join_cnt&0xFF;//190418
  	        join_cnt++;
	        memcpy(&(pbuf2[44]), &s_FPGA_YC.UTCtime, 4); //utcÊ±¼ä 
	// [49][50]--¸öÊı  
	//[51]¿ªÊ¼¹²¼ÇÂ¼86×é¼ä¸ô16
	#if 1
			if(position_cnt>=86)
			{
				rh_snd_t =86;
 				pbuf2[48]=0x0;
		        pbuf2[49]= 86 ;//48-49
		        for(i=0;i<86;i++)
			    {
			      memcpy(&(pbuf2[50+i*16]), &position[position_rd], 16); //51
			      if(position_rd <position_BUFLEN -2) //
				  {
					position_rd ++;
			      }
				  else
				  {
					position_rd =0;
				  }
				  if(position_cnt>0)
				  {
				   	position_cnt--;
				  }	 
			    }
			}
			else
			{
				rh_snd_t=position_cnt;//190523
	        	pbuf2[48]=0x0;
		        pbuf2[49]=position_cnt;
			 //   pbuf2[53]=position_cnt;
				for(i=0;i<rh_snd_t;i++)
			    {
			      memcpy(&(pbuf2[50+i*16]), &position[position_rd ], 16); //51
			      if(position_rd <position_BUFLEN -2) //
				  {
					position_rd ++;
			      }
				  else
				  {
					position_rd =0;
				  }
				  if(position_cnt>0)
				  {
				   	position_cnt--;
				  }	 
			    }
				for(i=rh_snd_t;i<86;i++)//pack_ads_nodata  //Ìî³ä0x0   //190523
				{
				  memcpy(&(pbuf2[50+i*16]), pack_ads_nodata, 16); //51
				}
			}  
	#endif
		   	if(sendto(sudp,pbuf2, 1432, 0, (struct sockaddr *)&sin, sizeof(sin) )<0) //  »¹ĞèÈ·¶¨·¢ËÍÌõ¼ş
			{
			;
			//	udp_snd_lost_1++;//	  cmd_buf_test[4]=hh_udp_snd_cnt>>8;
			}
			//	hh_udp_snd_cnt++;
         }  //ÈÚºÏÊı¾İ  //	if(position_cnt>0)  //±ØĞëÒªÓĞÓĞĞ§µÄÎ»ÖÃĞÅÏ¢
	  }//for(j)
#endif
//------- adsÊı¾İ    -------------------
#if 1
         for(j=0;j<14;j++)   //2
		 {
	         if(pulse_cnt_IP>0)  // ±ØĞëÒªÓĞÔ­Ê¼Êı¾İ
			 {
		        pbuf3[1]=0x0c;//adsÊı¾İ 
				if(total_cnt_2<0xffffff)
				{
					total_cnt_2++;
				}
				else
				{
		 		    total_cnt_2=0;
				}
			 //    memcpy(&(pbuf3[2]), &total_cnt, 3);//×Ü¼ÆÊı
	            pbuf3[2] =(total_cnt_2>>16);
	            pbuf3[3] =(total_cnt_2 & 0xFFFF)>>8;
				pbuf3[4] = (Uint8)(total_cnt_2 & 0xFF);
				pbuf3[38]=ads_cnt;//°üÄÚ¼ÆÊı
				ads_cnt++;
			    memcpy(&(pbuf3[39]), &s_FPGA_YC, 14); //Ò£²â  //39-38 13-14
			
				if(pulse_cnt_IP>=75)
				{
					ads_snd_t =75;
			//[53] [54] --±¾´ÎµÄ¸öÊı
			        pbuf3[53]=0; 
			        pbuf3[54]= 75 ;  //ÕâÀïĞèÒªÅĞpulse_cnt_IPÊÇ·ñ´óÓÚ75

			//[60]¿ªÊ¼11¸öads±¨ÎÄ£¬¼ä¸ôÊÇ17
			        for(i=0;i<75;i++)
				    {
				      memcpy(&(pbuf3[61+i*17]), &pulse_IP[pulse_rd_IP], 11); //60
				      if(pulse_rd_IP<pulse_BUFLEN_IP-2) //
					  {
						pulse_rd_IP++;
				      }
					  else
					  {
						pulse_rd_IP=0;
					  }
					  if(pulse_cnt_IP>0)
					  {
					   	pulse_cnt_IP--;
					  }	 
				    }
				}
				else
				{
					ads_snd_t=pulse_cnt_IP;
			//[53] [54] --±¾´ÎµÄ¸öÊı
					pbuf3[53]=0;
				    pbuf3[54]=pulse_cnt_IP;
			//[60]¿ªÊ¼11¸öads±¨ÎÄ£¬¼ä¸ôÊÇ17
					for(i=0;i<ads_snd_t;i++)
				    {
				      memcpy(&(pbuf3[61+i*17]), &pulse_IP[pulse_rd_IP], 11); //60
				      if(pulse_rd_IP<pulse_BUFLEN_IP-2) //
					  {
						pulse_rd_IP++;
				      }
					  else
					  {
						pulse_rd_IP=0;
					  }
					  if(pulse_cnt_IP>0)
					  {
					   	pulse_cnt_IP--;
					  }	 
				    }
#if 1
					if(ads_snd_t<70)  //
					{
						for(i=ads_snd_t;i<73;i++)//pack_ads_nodata  //Ìî³ä0x0
						{
						  memcpy(&(pbuf3[61+i*17]), pack_ads_nodata, 11); //60
						}
						//73 74
	                    memcpy(&(pbuf3[61+73*17]), int_cnt, 11);  //Íø¿ÚÍÆÄÚ²¿Ò£²â
	                    memcpy(&(pbuf3[61+74*17]), &(int_cnt[11]), 11);
					}
					else
					{
						for(i=ads_snd_t;i<75;i++)//pack_ads_nodata  //Ìî³ä0x0
						{
						  memcpy(&(pbuf3[61+i*17]), pack_ads_nodata, 11); //60
						}
					}
#endif                     
				}  
		#if 1
		//[1328]¿ªÊ¼ 100¸ö×Ö½ÚÌî 5¸öICAO £¨5*20£©  //---1330
		//s_FPGA_YC.UTCtime
#if 1        
		        ads_snd_t=0;
				while((ads_snd_t<5)&&(velocity_cnt>0))
				{
				   hhtmp=(unsigned int )(velocity[velocity_rd][4]<<24)+(unsigned int )(velocity[velocity_rd][5]<<16)
				        +(unsigned int )(velocity[velocity_rd][6]<<8)+(unsigned int )(velocity[velocity_rd][7]);
			//	   hhtmp=htonl(s_FPGA_YC.UTCtime);
				   if((htonl(s_FPGA_YC.UTCtime)-hhtmp)<=4)  //4s
				   {
				      memcpy(&(pbuf3[1330+ads_snd_t*20]), velocity[velocity_rd], 20);   //1328
				   	  ads_snd_t++;
				   }      
		           if(velocity_rd <velocity_BUFLEN -2) //
				   {
						velocity_rd ++;
				   }
				   else
				   {
						velocity_rd =0;
				   }
				   if(velocity_cnt>0)
				   {
					   	velocity_cnt--;
				   }	 
		    	}  
				if(ads_snd_t<5)
				{
			        for(i=ads_snd_t;i<5;i++)//pack_ads_nodata  //Ìî³ä0x0
					{
					  memcpy(&(pbuf3[1330+i*20]), pack_ads_nodata, 20);   //1328
					}
				}
#endif

		#endif
			    if(sendto( sudp,pbuf3, 1432, 0, (struct sockaddr *)&sin, sizeof(sin) )<0)//  »¹ĞèÈ·¶¨·¢ËÍÌõ¼ş
				{
					;
					//udp_snd_lost_2++;//	  cmd_buf_test[4]=hh_udp_snd_cnt>>8;
				}
	        }//adsÊı¾İ   if(pulse_cnt_IP>0)  // ±ØĞëÒªÓĞÔ­Ê¼Êı¾İ
        }//for(j) 
#endif
	}
	  TaskSleep(1000);
	//   TaskSleep(500);
	}
}
#endif
/**********************************************/

/**********************************************/
//²ÉÓÃEMIF´úÌæÔ­À´µÄÍøÂçÄ£Ê½
Uint8 pbuf3[5040 + 2];//adsÊı¾İ zhangfulong change
Uint8 pbuf2[5040 + 2];//ÈÚºÏÊı¾İ

void EMIF_sndPacket()  //ÈÎÎñ4 ·¢ËÍÁ½ÖÖ±¨ÎÄºÍ²âÊÔÊı¾İ 
{
//	SOCKET   							sudp = INVALID_SOCKET;
//  	struct   sockaddr_in 				sin1,sin;
//	int 	 							reuse = 1;
	int    								i=0,j=0;
//    unsigned int 						recvsize = 1024*8;
	//Uint8 pbuf1[1432];//²âÊÔÊı¾İ
	//Uint8 pbuf2[1432 + 2];//ÈÚºÏÊı¾İ
//	Uint8 pbuf3[1432];//adsÊı¾İ
//	Uint8 pbuf3[5040];//adsÊı¾İ zhangfulong change
	Uint32 adsb_cnt = 0;//adsbÔ­Ê¼Êı¾İ°ü¼ÆÊı

    Uint32 total_cnt=0;  //×Ü¼ÆÊı ÔÚ
	Uint32 total_cnt_2=0;  //×Ü¼ÆÊı ÔÚ
    Uint16 test_cnt=0;   // °üÄÚ¼ÆÊı
	Uint16 join_cnt=0;  //°üÄÚ¼ÆÊı
    Uint8 ads_cnt=0 ;   //°üÄÚ¼ÆÊı
    Uint8 ads_snd_t=0;    //ÅĞÊÇ·ñĞèÒªÌî³ä
    Uint8 rh_snd_t=0;    //ÅĞÊÇ·ñĞèÒªÌî³ä
	Uint8 rh_snd_t_2=0;    //ÅĞÊÇ·ñĞèÒªÌî³ä
	Uint32 hhtmp;
#if 0
	Uint32 udp_snd_lost=0;
	Uint32 udp_snd_lost_1=0;
	Uint32 udp_snd_lost_2=0;
#endif
	const Uint8 pack32[32]={
		0x0,0x0,0x0,0x0,0x0,0x55,//
		0x55,0x55,0x55,0x55,0x55,0x55,	//190610
		0x55,0x55,0x55,0x55,0x55,0x55,
		0x55,0x55,0x55,0x55,0x55,0x55,
		0x55,0x55,0x55,0x55,0x55,0x55,
	    0x55,0x55
	};//32×Ö½Ú--ÈÎÎñºÅ(1)--Êı¾İÀàĞÍ(1)--Ö¡¼ÆÊı£¨3£©--Êı¾İÍ¨µÀ£¨1£©--Ô¤Áô£¨26£©

	const Uint8 pack_samp1[6]={
	0x9e,0xda,0xa7,0x8b,0xF4,0xc5
    } ;///²âÊÔÊı¾İ
    const Uint8 pack_samp2[6]={
	0x9e,0xda,0xa7,0x8b,0xF4,0xc5
    } ;//ÈÚºÏÊı¾İ
    const Uint8 pack_samp3[6]={
	0x61,0x25,0x58,0x74,0x0b,0x3a
    } ;//adsÊı¾İ

    const Uint8 pack_reserve[6]={
    0x5a,0x5a,0x5a,0xff,0xff,0xff
    } ;
//zhangfulong change pack_ads_nodata from 20 to 34 because the most need back data is this
    const Uint8 pack_ads_nodata[34]={0
    } ;
  //---32
//--------------------1

     


	while(1)
   	{
	
		//zhangfulong add Ê±¼ä¸üĞÂ
		UTC_time_real[0] = UTC_time_all[13];
		UTC_time_real[1] = UTC_time_all[12];
		UTC_time_real[2] = UTC_time_all[15];
		UTC_time_real[3] = UTC_time_all[14];
		UTC_time_real[4] = UTC_time_all[17];
		UTC_time_real[5] = UTC_time_all[16];
		//zhangfulong add ?

		if(fpga_SetFlag==1)
	  	{    
			fpga_SetFlag=0;
		}

//------  ÈÚºÏÊı¾İ ---------------------- //  ½âËãÊı¾İ
		//zhangfulong È¥µô Ê¹ÓÃºóÃæµÄÉú³ÉÈÚºÏ½âËãÊı¾İ
	
			//zhangfulong add startÈÚºÏÊı¾İ°ü
		for (i = 0; i < 1432 + 2; i++)//Uint8 pbuf2[1432 + 2];//ÈÚºÏÊı¾İ
		{
			pbuf2[i] = 0;
		}

		//position_cnt = 150;
		//if(position_cnt>0)  //±ØĞëÒªÓĞÓĞĞ§µÄÎ»ÖÃĞÅÏ¢
		if (ronghe_cnt_p >=1500)
		{
			printf("ronghe_cnt_p is out of range is %d \n",ronghe_cnt_p);
		}
		if (ronghe_cnt_v >=1500)
		{
			printf("ronghe_cnt_v is out of range is %d\n",ronghe_cnt_v);
		}
		if ((ronghe_cnt_p + ronghe_cnt_v) >0)
		{
				//°üÍ· 7788AC8BF6E4
			pbuf2[0] = 0x77;
			pbuf2[1] = 0x88;
			pbuf2[2] = 0xAC;
			pbuf2[3] = 0x8B;
			pbuf2[4] = 0xF6;
			pbuf2[5] = 0xE4;
				//ÀàĞÍDDDD
			pbuf2[6] = 0xDD;
			pbuf2[7] = 0xDD;
				//UTCÊ±¼ä
			//memcpy(&(pbuf2[8]), &(UTC_time_real[0]), 6);
			//zhangfulong add Ê±¼ä¸üĞÂ
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add Ê±¼ä¸üĞÂ
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//½«Ê±¼ä¸´ÖÆ¹ıÈ¥ 4->6
				pbuf2[8] = UTC_time_real[0];
				pbuf2[9] = UTC_time_real[1];
				pbuf2[10] = UTC_time_real[2];
				pbuf2[11] = UTC_time_real[3];
				pbuf2[12] = UTC_time_real[4];
				pbuf2[13] = UTC_time_real[5];

				//ÀÛ¼Æ·¢°ü¼ÆÊı
			pbuf2[14] = (Uint8)((total_cnt & 0xFF00) >> 8);
			pbuf2[15] = (Uint8)((total_cnt & 0x00FF));
			total_cnt = total_cnt + 1;
				//Ä¿±êUTCÊ±¼ä
/*			memcpy(&(pbuf2[16]), &airplane_location_tmp.time , 4);//Õâ¸öÓ¦¸ÃÊÇÒıµ¼Ä¿±êµÄÊ±¼ä°É¡£
			memcpy(&(pbuf2[22]), &airplane_location_tmp.ICAO_address , 4);//ICAO
			memcpy(&(pbuf2[26]), &airplane_location_tmp.coordinate[1] , 4);//¾­¶È
			memcpy(&(pbuf2[30]), &airplane_location_tmp.coordinate[0] , 4);//Î³¶È
			memcpy(&(pbuf2[34]), &airplane_speed_tmp.N_S_velocity , 4);//±±ËÙ¶È
			memcpy(&(pbuf2[38]), &airplane_speed_tmp.E_W_velocity , 4);//¶«ËÙ¶È
			memcpy(&(pbuf2[42]), &airplane_speed_tmp.E_W_velocity , 4);//ÌìËÙ¶È
			memcpy(&(pbuf2[46]), &airplane_speed_tmp.E_W_velocity , 4);//¸ß¶È
				//110*********
*/				
			pbuf2[16] = 0xEE; 	
			pbuf2[17] = 0xEE;
			/*if(position_cnt>=144)
			{
				rh_snd_t = 144;
			}
			else
			{
				rh_snd_t = position_cnt;
			}*/
			if (ronghe_cnt_p> 144)//ÏÈÎ»ÖÃ
			{
				rh_snd_t = 144;
			}
			else
			{
				rh_snd_t = ronghe_cnt_p;
			}
			
			pbuf2[18] = (Uint8)((rh_snd_t & 0xFF00) >> 8); 	//°üÄÚ¼ÆÊı
			pbuf2[19] = (Uint8)((rh_snd_t & 0x00FF));
				
			/*for(i=0;i<rh_snd_t;i++)
		    {
				//memcpy(&(pbuf2[18 +2 + i*34 + 0]), &airplane_location_static[position_rd].time , 4);
				pbuf2[18 +2 + i*34 + 0] = (airplane_location_static[position_rd].time & 0xFF0000000000)>>38;
				pbuf2[18 +2 + i*34 + 0 + 1] = (airplane_location_static[position_rd].time & 0x00FF00000000)>>32;
				pbuf2[18 +2 + i*34 + 0 + 2] = (airplane_location_static[position_rd].time & 0x0000FF000000)>>24;
				pbuf2[18 +2 + i*34 + 0 + 3] = (airplane_location_static[position_rd].time & 0x000000FF0000)>>16;
				pbuf2[18 +2 + i*34 + 0 + 4] = (airplane_location_static[position_rd].time & 0x00000000FF00)>>8;
				pbuf2[18 +2 + i*34 + 0 + 5] = (airplane_location_static[position_rd].time & 0x0000000000FF);
				//memcpy(&(pbuf2[18 +2 + i*34 + 6]), &airplane_location_static[position_rd].ICAO_address , 4);//ICAO				
				pbuf2[18 +2 + i*34 + 6] = (airplane_location_static[position_rd].ICAO_address & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 6 + 1] = (airplane_location_static[position_rd].ICAO_address & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 6 + 2] = (airplane_location_static[position_rd].ICAO_address & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 6 + 3] = (airplane_location_static[position_rd].ICAO_address & 0x000000FF);

				//memcpy(&(pbuf2[18 +2 + i*34 + 10]), &airplane_location_static[position_rd].coordinate[1] , 4);//¾­¶È
				pbuf2[18 +2 + i*34 + 10] = (airplane_location_static[position_rd].coordinate[1] & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 10 + 1] = (airplane_location_static[position_rd].coordinate[1] & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 10 + 2] = (airplane_location_static[position_rd].coordinate[1] & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 10 + 3] = (airplane_location_static[position_rd].coordinate[1] & 0x000000FF);
				//memcpy(&(pbuf2[18 +2 + i*34 + 14]), &airplane_location_static[position_rd].coordinate[0] , 4);//Î³¶È
				pbuf2[18 +2 + i*34 + 14] = (airplane_location_static[position_rd].coordinate[0] & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 14 + 1] = (airplane_location_static[position_rd].coordinate[0] & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 14 + 2] = (airplane_location_static[position_rd].coordinate[0] & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 14 + 3] = (airplane_location_static[position_rd].coordinate[0] & 0x000000FF);
				//memcpy(&(pbuf2[18 +2 + i*34 + 18]), &airplane_velocity_three_static[position_rd].N_S_velocity , 4);//±±ËÙ¶È
				pbuf2[18 +2 + i*34 + 18] = (airplane_velocity_three_static[position_rd].N_S_velocity & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 18 + 1] = (airplane_velocity_three_static[position_rd].N_S_velocity & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 18 + 2] = (airplane_velocity_three_static[position_rd].N_S_velocity & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 18 + 3] = (airplane_velocity_three_static[position_rd].N_S_velocity & 0x000000FF);
				
				//memcpy(&(pbuf2[18 +2 + i*34 + 22]), &airplane_velocity_three_static[position_rd].E_W_velocity , 4);//¶«ËÙ¶È
				pbuf2[18 +2 + i*34 + 22] = (airplane_velocity_three_static[position_rd].E_W_velocity & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 22 + 1] = (airplane_velocity_three_static[position_rd].E_W_velocity & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 22 + 2] = (airplane_velocity_three_static[position_rd].E_W_velocity & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 22 + 3] = (airplane_velocity_three_static[position_rd].E_W_velocity & 0x000000FF);
				//memcpy(&(pbuf2[18 +2 + i*34 + 24]), &airplane_velocity_three_static[position_rd].VERT_velocity , 4);//ÌìËÙ¶È
				pbuf2[18 +2 + i*34 + 24] = (airplane_velocity_three_static[position_rd].VERT_velocity & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 24 + 1] = (airplane_velocity_three_static[position_rd].VERT_velocity & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 24 + 2] = (airplane_velocity_three_static[position_rd].VERT_velocity & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 24 + 3] = (airplane_velocity_three_static[position_rd].VERT_velocity & 0x000000FF);
				//memcpy(&(pbuf2[18 +2 + i*34 + 28]), &airplane_location_static[position_rd].altitude , 4);//¸ß¶È
				pbuf2[18 +2 + i*34 + 28] = (airplane_location_static[position_rd].altitude & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 28 + 1] = (airplane_location_static[position_rd].altitude & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 28 + 2] = (airplane_location_static[position_rd].altitude & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 28 + 3] = (airplane_location_static[position_rd].altitude & 0x000000FF);

				if(position_rd <position_BUFLEN -2) //
			  	{
					position_rd ++;
		      	}
			 	 else
			 	 {
					position_rd =0;
				 }
				 if(position_cnt>0)
				 {
				   	position_cnt--;
				 }
			}*/
			for(i=0;i<rh_snd_t;i++)
		    {
				//memcpy(&(pbuf2[18 +2 + i*34 + 0]), &airplane_location_static_ronghe[ronghe_cnt_p].time , 4);
				pbuf2[18 +2 + i*34 + 0] = (airplane_location_static_ronghe[ronghe_cnt_p].time & 0xFF0000000000)>>38;
				pbuf2[18 +2 + i*34 + 0 + 1] = (airplane_location_static_ronghe[ronghe_cnt_p].time & 0x00FF00000000)>>32;
				pbuf2[18 +2 + i*34 + 0 + 2] = (airplane_location_static_ronghe[ronghe_cnt_p].time & 0x0000FF000000)>>24;
				pbuf2[18 +2 + i*34 + 0 + 3] = (airplane_location_static_ronghe[ronghe_cnt_p].time & 0x000000FF0000)>>16;
				pbuf2[18 +2 + i*34 + 0 + 4] = (airplane_location_static_ronghe[ronghe_cnt_p].time & 0x00000000FF00)>>8;
				pbuf2[18 +2 + i*34 + 0 + 5] = (airplane_location_static_ronghe[ronghe_cnt_p].time & 0x0000000000FF);
				//memcpy(&(pbuf2[18 + 2 + i*34 + 6]), &airplane_location_static_ronghe[ronghe_cnt_p].ICAO_address , 4);//ICAO				
				pbuf2[18 +2 + i*34 + 6] = (airplane_location_static_ronghe[ronghe_cnt_p].ICAO_address & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 6 + 1] = (airplane_location_static_ronghe[ronghe_cnt_p].ICAO_address & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 6 + 2] = (airplane_location_static_ronghe[ronghe_cnt_p].ICAO_address & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 6 + 3] = (airplane_location_static_ronghe[ronghe_cnt_p].ICAO_address & 0x000000FF);
				//memcpy(&(pbuf2[18 +2 + i*34 + 10]), &airplane_location_static_ronghe[ronghe_cnt_p].coordinate[1] , 4);//¾­¶È
				pbuf2[18 +2 + i*34 + 10] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[1] & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 10 + 1] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[1] & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 10 + 2] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[1] & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 10 + 3] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[1] & 0x000000FF);
				//memcpy(&(pbuf2[18 + i*34 + 14]), &airplane_location_static_ronghe[ronghe_cnt_p].coordinate[0] , 4);//Î³¶È
				pbuf2[18 +2 + i*34 + 14] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[0] & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 14 + 1] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[0] & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 14 + 2] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[0] & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 14 + 3] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[0] & 0x000000FF);
			//	memcpy(&(pbuf2[18 + i*34 + 18]), &airplane_velocity_three_static[position_rd].N_S_velocity , 4);//±±ËÙ¶È
			//	memcpy(&(pbuf2[18 + i*34 + 22]), &airplane_velocity_three_static[position_rd].E_W_velocity , 4);//¶«ËÙ¶È
			//	memcpy(&(pbuf2[18 + i*34 + 26]), &airplane_velocity_three_static[position_rd].E_W_velocity , 4);//ÌìËÙ¶È
				//memcpy(&(pbuf2[18 +2 + i*34 + 30]), &airplane_location_static_ronghe[ronghe_cnt_p].altitude , 4);//¸ß¶È
				pbuf2[18 +2 + i*34 + 30] = (airplane_location_static_ronghe[ronghe_cnt_p].altitude & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 30 + 1] = (airplane_location_static_ronghe[ronghe_cnt_p].altitude & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 30 + 2] = (airplane_location_static_ronghe[ronghe_cnt_p].altitude & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 30 + 3] = (airplane_location_static_ronghe[ronghe_cnt_p].altitude & 0x000000FF);
				if(airplane_location_static_ronghe[ronghe_cnt_p].ICAO_address == 0xBEBEBEBE)
				{
					printf("get in a 0xBEBEBEBE \n");
				}
				if (ronghe_cnt_p > 0)
				{
					ronghe_cnt_p--;
				}
				else
				{
					printf("ronghe_cnt_p \n");
				}
			}

			//sËÙ¶È
			if (rh_snd_t== 144)//Î»ÖÃÕ¼Âú£¬Ã»ÓĞËÙ¶È
			{
				rh_snd_t_2 = 0;
			}
			else if ( rh_snd_t + ronghe_cnt_v < 144)//ËÙ¶È¿ÉÒÔÈ«²¿Ğ´½øÈ¥
			{
				rh_snd_t_2 = ronghe_cnt_v;
			}else//ËÙ¶ÈĞ´½øÈ¥²¿·Ö
			{
				rh_snd_t_2 = 144 - rh_snd_t;
			}
			for(i=rh_snd_t;i<rh_snd_t + rh_snd_t_2;i++)
		    {
				//memcpy(&(pbuf2[18+2 + i*34 + 0]), &airplane_velocity_three_static_ronghe[ronghe_cnt_p].time , 4);
				pbuf2[18 +2 + i*34 + 0] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].time & 0xFF0000000000)>>38;
				pbuf2[18 +2 + i*34 + 0 + 1] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].time & 0x00FF00000000)>>32;
				pbuf2[18 +2 + i*34 + 0 + 2] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].time & 0x0000FF000000)>>24;
				pbuf2[18 +2 + i*34 + 0 + 3] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].time & 0x000000FF0000)>>16;
				pbuf2[18 +2 + i*34 + 0 + 4] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].time & 0x00000000FF00)>>8;
				pbuf2[18 +2 + i*34 + 0 + 5] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].time & 0x0000000000FF);
				//memcpy(&(pbuf2[18+2 + i*34 + 6]), &airplane_velocity_three_static_ronghe[ronghe_cnt_p].ICAO_address , 4);//ICAO				
				pbuf2[18 +2 + i*34 + 6] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].ICAO_address & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 6 + 1] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].ICAO_address & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 6 + 2] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].ICAO_address & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 6 + 3] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].ICAO_address & 0x000000FF);
			//	memcpy(&(pbuf2[18 + i*34 + 10]), &airplane_location_static[ronghe_cnt_p].coordinate[1] , 4);//¾­¶È
			//	memcpy(&(pbuf2[18 + i*34 + 14]), &airplane_location_static[ronghe_cnt_p].coordinate[0] , 4);//Î³¶È
				//memcpy(&(pbuf2[18+2 + i*34 + 18]), &airplane_velocity_three_static_ronghe[position_rd].N_S_velocity , 4);//±±ËÙ¶È
				pbuf2[18 +2 + i*34 + 18] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].N_S_velocity & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 18 + 1] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].N_S_velocity & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 18 + 2] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].N_S_velocity & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 18 + 3] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].N_S_velocity & 0x000000FF);
				//memcpy(&(pbuf2[18+2 + i*34 + 22]), &airplane_velocity_three_static_ronghe[position_rd].E_W_velocity , 4);//¶«ËÙ¶È
				pbuf2[18 +2 + i*34 + 22] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].E_W_velocity & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 22 + 1] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].E_W_velocity & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 22 + 2] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].E_W_velocity & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 22 + 3] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].E_W_velocity & 0x000000FF);
				//memcpy(&(pbuf2[18+2 + i*34 + 26]), &airplane_velocity_three_static_ronghe[position_rd].E_W_velocity , 4);//ÌìËÙ¶È
				pbuf2[18 +2 + i*34 + 26] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].VERT_velocity & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 26 + 1] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].VERT_velocity & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 26 + 2] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].VERT_velocity & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 26 + 3] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].VERT_velocity & 0x000000FF);
			//	memcpy(&(pbuf2[18 + i*34 + 30]), &airplane_location_static[ronghe_cnt_p].altitude , 4);//¸ß¶È
				if (ronghe_cnt_v > 0)
				{
					ronghe_cnt_v--;
				}
				else
				{
					printf("ronghe_cnt_v may be -1\n ");
				}
			}

			rh_snd_t = rh_snd_t + rh_snd_t_2;
			pbuf2[18] = (Uint8)((rh_snd_t & 0xFF00) >> 8); 	//°üÄÚ¼ÆÊı
			pbuf2[19] = (Uint8)((rh_snd_t & 0x00FF));


			for(i = rh_snd_t; i < 144; i++)
			{
				memcpy(&(pbuf2[18 +2 + i*34]), pack_ads_nodata , 34);
					;//pack_ads_nodata
			}

			for (i = 4913 +2; i < 5040; i++)//Ìî³ä4914-5040µÄAA
			{
				pbuf2[i] = 0xAA;
			}
			


				//zhangfulong add end
				//ÈÚºÏÊı¾İĞèÒª×ª³ÉEMIF¿ÚÊä³ö-zhangfulong
	
			for (i = 0; i< 10; i++)//504 = 0x1F8
			{
				TaskSleep(1);
				EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
				pEDMA3CC_PaRAM->OPT = 0x00130104;//16 
				pEDMA3CC_PaRAM->SRC =  (unsigned char *)(&pbuf2[i * 0x1F8]);  //ce4
				//	pEDMA3CC_PaRAM->SRC =  (unsigned char *)XSH_Array2;  //ce4	
				//pEDMA3CC_PaRAM->BCNT_ACNT =0x13B00001;//0x13B0 = 5040  //1280*4	//0x003d0001D¡ä¦Ì?FPGA??¦Ì??¡¤A0F	//51
				//pEDMA3CC_PaRAM->BCNT_ACNT =0x02000001;//Ò»´Î·¢512¸ö£¬·Ö10´Î·¢£¬9´Î512
				pEDMA3CC_PaRAM->BCNT_ACNT =0x01F80001;
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
				pEDMA3CC_PaRAM->DST = (unsigned char *)(DSP_TO_FPGA_RONGHE_PBUF2 + 0x1F8 * i);//Ä¿±êµØÖ·ĞèÒªĞŞ¸Ä£¬FPGA·½ÃæÌá¹©
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
			}
			
			
		}//end  if(position_cnt>0)
		else
		{
			pbuf2[0] = 0x77;
			pbuf2[1] = 0x88;
			pbuf2[2] = 0xAC;
			pbuf2[3] = 0x8B;
			pbuf2[4] = 0xF6;
			pbuf2[5] = 0xE4;
				//ÀàĞÍDDDD
			pbuf2[6] = 0xDD;
			pbuf2[7] = 0xDD;
				//UTCÊ±¼ä
			//memcpy(&(pbuf2[8]), &(UTC_time_real[0]), 6);
			//zhangfulong add Ê±¼ä¸üĞÂ
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add Ê±¼ä¸üĞÂ
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//½«Ê±¼ä¸´ÖÆ¹ıÈ¥ 4->6
				pbuf2[8] = UTC_time_real[0];
				pbuf2[9] = UTC_time_real[1];
				pbuf2[10] = UTC_time_real[2];
				pbuf2[11] = UTC_time_real[3];
				pbuf2[12] = UTC_time_real[4];
				pbuf2[13] = UTC_time_real[5];
				//ÀÛ¼Æ·¢°ü¼ÆÊı
			pbuf2[14] = (Uint8)((total_cnt & 0xFF00) >> 8);
			pbuf2[15] = (Uint8)((total_cnt & 0x00FF));
			total_cnt = total_cnt + 1;
				//Ä¿±êUTCÊ±¼ä
			
				//110*********
				
			//pbuf2[16] = (Uint8)((0 & 0xFF00) >> 8); 	
			//pbuf2[17] = (Uint8)((0 & 0x00FF));
			pbuf2[16] = 0xEE;
			pbuf2[17] = 0xEE;

		//		rh_snd_t = 0;

			for(i = 0; i < 144; i++)
			{
				memcpy(&(pbuf2[18 + i*34]), pack_ads_nodata , 34);
					;//pack_ads_nodata
			}
			for (i = 4913 + 2; i < 5040; i++)//Ìî³ä4914-5040µÄAA
			{
				pbuf2[i] = 0xAA;
			}
			

			for (i = 0; i< 10; i++)//504 = 0x1F8
			{
				TaskSleep(1);
				EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
				pEDMA3CC_PaRAM->OPT = 0x00130104;//16 
				pEDMA3CC_PaRAM->SRC =  (unsigned char *)(&pbuf2[i * 0x1F8]);  //ce4
				//	pEDMA3CC_PaRAM->SRC =  (unsigned char *)XSH_Array2;  //ce4	
				//pEDMA3CC_PaRAM->BCNT_ACNT =0x13B00001;//0x13B0 = 5040  //1280*4	//0x003d0001D¡ä¦Ì?FPGA??¦Ì??¡¤A0F	//51
				//pEDMA3CC_PaRAM->BCNT_ACNT =0x02000001;//Ò»´Î·¢512¸ö£¬·Ö10´Î·¢£¬9´Î512
				pEDMA3CC_PaRAM->BCNT_ACNT =0x01F80001;
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
				pEDMA3CC_PaRAM->DST = (unsigned char *)(DSP_TO_FPGA_RONGHE_PBUF2 + 0x1F8 * i);//Ä¿±êµØÖ·ĞèÒªĞŞ¸Ä£¬FPGA·½ÃæÌá¹©
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
			}

		}
      

//------- adsÊı¾İ    ------------------- //   Ô­Ê¼±¨ÎÄ

				//zhangfulong add start Ô­Ê¼±¨ÎÄÉú³É Ğ´µ½pbuf3ÀïÃæ
		//if(pulse_cnt_IP>0)
		//{
		//Uint8 pbuf3[5040 + 2];³õÊ¼»¯
//		for (i = 0; i < 5040 + 2; i++)
//		{
//			pbuf3[i] = 0;
//		}
			//°üÍ·612558740b3a
		pbuf3[0] = 0x61;
		pbuf3[1] = 0x25;
		pbuf3[2] = 0x58;
		pbuf3[3] = 0x74;
		pbuf3[4] = 0x0B;
		pbuf3[5] = 0x3A;
		if(pulse_cnt_IP>0)  // ±ØĞëÒªÓĞÔ­Ê¼Êı¾İ
	 	{
				//°ü¼ÆÊı
//			pbuf3[6] = (Uint8) ((adsb_cnt & 0xFF00)>>8);
//			pbuf3[7] = (Uint8) (adsb_cnt & 0x00FF);
			pbuf3[6] = (Uint8) (adsb_cnt & 0x00FF);
			adsb_cnt = adsb_cnt + 1;//×Ô¼Ó1 ¼ÆÊı
			//memcpy(&(pbuf3[7]),&(UTC_time_real[0]),6);//Ê±¼ä 8 9 10 11
			//zhangfulong add Ê±¼ä¸üĞÂ
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add Ê±¼ä¸üĞÂ
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//½«Ê±¼ä¸´ÖÆ¹ıÈ¥ 4->6
				pbuf3[7] = UTC_time_real[0];
				pbuf3[8] = UTC_time_real[1];
				pbuf3[9] = UTC_time_real[2];
				pbuf3[10] = UTC_time_real[3];
				pbuf3[11] = UTC_time_real[4];
				pbuf3[12] = UTC_time_real[5];
				
				
		        	//pbuf3[1]=0x0c;//adsÊı¾İ 
			if(total_cnt_2<0xffffff)
			{
				total_cnt_2++;
			}
			else
			{
 		   		total_cnt_2=0;
			}
			 
			ads_cnt++;
			    //memcpy(&(pbuf3[39]), &s_FPGA_YC, 14); //Ò£²â  //39-38 13-14


			if(pulse_cnt_IP>=295)
			{
				ads_snd_t =295;
			}
			else
			{
				ads_snd_t =pulse_cnt_IP;
			}
			//[53] [54] --±¾´ÎµÄ¸öÊı
	        pbuf3[14 - 1]=(Uint8)((ads_snd_t & 0x00FF) >> 8); 
	        pbuf3[15 - 1]= (Uint8) (ads_snd_t & 0x00FF) ;  //ÕâÀïĞèÒªÅĞpulse_cnt_IPÊÇ·ñ´óÓÚ75

			//[60]¿ªÊ¼11¸öads±¨ÎÄ£¬¼ä¸ôÊÇ17
	        for(i=0;i<ads_snd_t;i++)
		    {
				pbuf3[15 + i * 17 + 0] = 0x5A;
				pbuf3[15 + i * 17 + 1] = 0x5A;
				pbuf3[15 + i * 17 + 2] = 0x5A;
				pbuf3[15 + i * 17 + 3] = 0xFF;
				pbuf3[15 + i * 17 + 4] = 0xFF;
				pbuf3[15 + i * 17 + 5] = 0xFF;
		    	memcpy(&(pbuf3[15 + i * 17 + 6]), &pulse_IP[pulse_rd_IP], 11); //60
		      	if(pulse_rd_IP<pulse_BUFLEN_IP-2) //
			  	{
					pulse_rd_IP++;
		   		}
				else
				{
					pulse_rd_IP=0;
			  	}
			  	if(pulse_cnt_IP>0)
			  	{
			   		pulse_cnt_IP--;
			  	}	 
		   	}
			for (i = ads_snd_t; i < 295; i++)
			{
				pbuf3[15 + i * 17 + 0] = 0x5A;
				pbuf3[15 + i * 17 + 1] = 0x5A;
				pbuf3[15 + i * 17 + 2] = 0x5A;
				pbuf3[15 + i * 17 + 3] = 0xFF;
				pbuf3[15 + i * 17 + 4] = 0xFF;
				pbuf3[15 + i * 17 + 5] = 0xFF;
		   		memcpy(&(pbuf3[15 + i * 17 + 6]), &pack_ads_nodata[0], 11);
							;//pack_ads_nodata
			}
			for (i = 0; i < 27; i++)//Ìî³ä
			{
				if (15 + 295 * 17 + i < 5040)
				{
					pbuf3[15 + 295 * 17 + i] = 0x55;
				}
			}
					
				

				//zhangfulong add end Ô­Ê¼±¨ÎÄÉú³É Ğ´µ½pbuf3ÀïÃæ
				//ADSBÊı¾İĞèÒª×ª³ÉEMIF¿ÚÊä³ö-zhangfulong
			
			for (i = 0; i< 10; i++)
			{
				TaskSleep(1);
				EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
				pEDMA3CC_PaRAM->OPT = 0x00130104;//16 
				pEDMA3CC_PaRAM->SRC =  (unsigned char *)(&pbuf3[i * 0x1F8]);  //ce4
				//	pEDMA3CC_PaRAM->SRC =  (unsigned char *)XSH_Array2;  //ce4	
				//pEDMA3CC_PaRAM->BCNT_ACNT =0x13B00001;//0x13B0 = 5040  //1280*4	//0x003d0001D¡ä¦Ì?FPGA??¦Ì??¡¤A0F	//51
				pEDMA3CC_PaRAM->BCNT_ACNT =0x01F80001;//Ò»´Î·¢512¸ö£¬·Ö10´Î·¢£¬9´Î512
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
				pEDMA3CC_PaRAM->DST = (unsigned char *)(DSP_TO_FPGA_ADSB_PBUF3 + 0x1F8 * i);//Ä¿±êµØÖ·ĞèÒªĞŞ¸Ä£¬FPGA·½ÃæÌá¹©
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
			}

		}//end  if(pulse_cnt_IP>0)
		else
		{
			//pbuf3[6] = (Uint8) ((adsb_cnt & 0xFF00)>>8);
			pbuf3[6] = (Uint8) (adsb_cnt & 0x00FF);
			//pbuf3[7] = (Uint8) (adsb_cnt & 0x00FF);
			adsb_cnt = adsb_cnt + 1;//×Ô¼Ó1 ¼ÆÊı
			//memcpy(&(pbuf3[8-1]),&(UTC_time_real[0]),6);//Ê±¼ä 8 9 10 11
			//zhangfulong add Ê±¼ä¸üĞÂ
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add Ê±¼ä¸üĞÂ
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//½«Ê±¼ä¸´ÖÆ¹ıÈ¥ 4->6
				pbuf3[7] = UTC_time_real[0];
				pbuf3[8] = UTC_time_real[1];
				pbuf3[9] = UTC_time_real[2];
				pbuf3[10] = UTC_time_real[3];
				pbuf3[11] = UTC_time_real[4];
				pbuf3[12] = UTC_time_real[5];
				

			ads_snd_t = 0;
			//[53] [54] --±¾´ÎµÄ¸öÊı
	        pbuf3[14-1]=(Uint8)((ads_snd_t & 0x00FF) >> 8); 
	        pbuf3[15-1]= (Uint8) (ads_snd_t & 0x00FF) ;  //ÕâÀïĞèÒªÅĞpulse_cnt_IPÊÇ·ñ´óÓÚ75

			//[60]¿ªÊ¼11¸öads±¨ÎÄ£¬¼ä¸ôÊÇ17
			for (i = ads_snd_t; i < 295; i++)
			{
				pbuf3[15 + i * 17 + 0] = 0x5A;
				pbuf3[15 + i * 17 + 1] = 0x5A;
				pbuf3[15 + i * 17 + 2] = 0x5A;
				pbuf3[15 + i * 17 + 3] = 0xFF;
				pbuf3[15 + i * 17 + 4] = 0xFF;
				pbuf3[15 + i * 17 + 5] = 0xFF;
		   		memcpy(&(pbuf3[15 + i * 17 + 6]), &pack_ads_nodata[0], 11);
							;//pack_ads_nodata
			}
			for (i = 0; i < 27; i++)
			{
				if (15 + 295 * 17 + i < 5040)
				{
					pbuf3[15 + 295 * 17 + i] = 0x55;
				}
			}

			
			for (i = 0; i< 10; i++)
			{
				TaskSleep(1);
				EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
				pEDMA3CC_PaRAM->OPT = 0x00130104;//16 
				pEDMA3CC_PaRAM->SRC =  (unsigned char *)(&pbuf3[i * 0x1F8]);  //ce4
				//	pEDMA3CC_PaRAM->SRC =  (unsigned char *)XSH_Array2;  //ce4	
				//pEDMA3CC_PaRAM->BCNT_ACNT =0x13B00001;//0x13B0 = 5040  //1280*4	//0x003d0001D¡ä¦Ì?FPGA??¦Ì??¡¤A0F	//51
				pEDMA3CC_PaRAM->BCNT_ACNT =0x01F80001;//Ò»´Î·¢512¸ö£¬·Ö10´Î·¢£¬9´Î512
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
				pEDMA3CC_PaRAM->DST = (unsigned char *)(DSP_TO_FPGA_ADSB_PBUF3 + 0x1F8 * i);//Ä¿±êµØÖ·ÒªĞŞ¸Ä£¬FPGA·½ÃæÌá¹©
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
			}
			
			
		}

			    
	   //     }//adsÊı¾İ   if(pulse_cnt_IP>0)  // ±ØĞëÒªÓĞÔ­Ê¼Êı¾İ
    //    }//for(j) 
//#endif
	//  }

		//TaskSleep(1000);
		TaskSleep(1000 ); //1000-20
	
	//   TaskSleep(500);
	}//while(1)
}


/**********************************************/


/**********************************************/
void  get_pulse_TOByte(Uint8 * dst,unsigned short * src)
{
  Uint8 i=0;
  for(i=0;i<11;i++)
  {
	(*(dst+i))=(Uint8)(((*(src+8*i))<<7)+((*(src+8*i+1))<<6)+((*(src+8*i+2))<<5)+((*(src+8*i+3))<<4)
	           +((*(src+8*i+4))<<3)+((*(src+8*i+5))<<2)+((*(src+8*i+6))<<1)+((*(src+8*i+7))));   
	    
  }
}
/**********************************************/
void get_Pulse()  //ÈÎÎñ1£º 1280->112
{
  unsigned short int i,j;
  unsigned short int data_head=0,max_position=0;	        
  unsigned short int data_begin=0;
  unsigned int temp[8]={0},max_value=0;
  unsigned short   * data_fifo;
  unsigned  IER_tmp=0;
  unsigned short hh_message;
  long time_tmp_01;
  while(1)
  {
//¿¼ÂÇÒ»ÏÂ´¦ÀíµÄ¶ÔÏóÊÇÊ²Ã´£¿--IP_DATA[g_u32IpSendIndex];
//Ó¦¸ÃÊÇ¸öÊı×é£¬Ö»Òª·Ç¿Õ¾Í¿ÉÒÔ´¦Àí--SND_Catch_Program_Full>0
//»¹ĞèÒªÎ¬»¤Ò»ÏÂÖ¸Õë --g_u32IpSendIndex
//Êä³öÊÇÊ²Ã´£¿
//
    if(SND_Catch_Program_Full>0)  //
	{
    	while(1)  //
	    {
	   	//	if(SEND_ADDR_CACHE[g_u32IpSendIndex].pwrite==1)  //
			{
				  if(SND_Catch_Program_Full==1)//1
				  {
				     TaskSleep(1);
				  }
                  t1=CLK_gethtime();
				  //gpio7_set_1();
				  data_fifo= IP_DATA[g_u32IpSendIndex];	      //data_fifo= first_fifo_1;//´«µİÒ»ÏÂ
	              data_head=0;
			#if 1
				  for(i=0;i<112;i++) //³õÊ¼»¯Ò»ÏÂ±äÁ¿
				  {
				  	  pulse_amp[i]=0;
					  confi[i]=0;
				  }
            #endif
			#if 1
				  for(i=0;i<24;i++)//?????ÎªÊ²Ã´£¿£¿£¿£¿
				  {	
				  	  correct[i]=0; //  correct[i]=1;   //190426
				  } 
           #endif
			      for(i=0;i<10;i++)//2  ÑÍ?
			      {
			         if((data_fifo[i]==0x5a5a)&&(data_fifo[i+1]==0x5a5a)&&
					   	(data_fifo[i+2]==0xffff)&&(data_fifo[i+3]==0xffff))
				     {
				          data_head=1;
						  data_begin=i+4;
						  int_cnt[8]++;//
						  break;
				     }
					
			      }
				  if(data_head==0)
				  {
						TaskSleep(0);
				  }
			      if(data_head==1) //ÓĞÍ·
			      {
			        memset(temp,0,sizeof(temp));//lh,±ØĞëÇå0
			        for(i=0;i<6;i++)
			        {
			             for(j=0;j<5;j++)
						 {
			               temp[i]=data_fifo[i+j+data_begin]
				               +data_fifo[i+j+10+data_begin]
				               +data_fifo[i+j+35+data_begin]
				               +data_fifo[i+j+45+data_begin]
				               +data_fifo[i+j+80+data_begin]
				               +temp[i];
						 }
			        }
			        max_position=0;
				    max_value=0;
			        for(i=0;i<6;i++)
			        {
			           if(temp[i]>max_value)  
			           {
			              max_position=i;  
			              max_value=temp[i];
					   }
			        }
			        data_begin=data_begin+max_position;
			        sumoba_pt=data_fifo+data_begin;//ADS-B±¨Í·ÆğÊ¼Î»ÖÃÖ¸Õë 
			        ampoba_pt=data_fifo+data_begin+80;//ADS-BÊı¾İÆğÊ¼Î»ÖÃÖ¸Õë
    				//C62_enableIER(1<<9|1<<8);//9--fpgaÖĞ¶Ï 8--edmaÖĞ¶Ï
    				refer_amp=com_refer(sumoba_pt);//¼ÆËã·ù¶ÈµÄ²Î¿¼Öµ
    		        //´Ë¹¦ÂÊ¼ì²âÊÊÓÃÓÚ¶ÔÊı¼ì²¨ºó£¬ÏßĞÔ½âµ÷ÖÆºó¿ÉÒÔò»¯ÎªÓÃÖĞ¼?¸öÑùµãÖµÖ±½ÓÆ½¾ùµÃµ½£¨ĞèÒªÑéÖ¤£©
    				code_confi(refer_amp,ampoba_pt,pulse_amp,confi);//¼ÆËã´úÂëÎ»ºÍÖÃĞÅ¶È
		
    				crc_check_flag=crc_check(pulse_amp,correct);//CRCĞ£Ñé
    				if(crc_check_flag==0)  	  
					{
					   	plane_ok=0;//´Ë±äÁ¿Îª0±íÊ¾·É»úADS-B¹ã²¥ĞÅÏ¢½ÓÊÕÕıÈ·
					}
					else
					{
				   		plane_ok=err_corr_8bit(confi,pulse_amp,correct,syndrome);//¾À´í
					}
	            	t5=CLK_gethtime();
	            	//gpio7_set_0(); 
			        if(plane_ok==0)//ËµÃ÷½âËã³É¹¦
					{ 
				#if 1	
						for(i=0;i<88;i++)  //¶ÔÄ£ÄâÔ´½øĞĞ¼ì²â  
				        {
							if(pulse_amp[i]!=pulse_amp_bak[i])
							{
								int_cnt[7]++;
								break;				
							}
						}
				#endif
#if 1
                     //ÓÃÓÚÌŞ³ıÈ«0µÄÊı¾İ
					     hh_message=0;
					     for(i=0;i<5;i++)  //Ç°5Î»?£¬»ù±¾¿ÉÄÜÊÇÈ«0
					     {   
					        hh_message=hh_message|(pulse_amp[i]<<(4-i));//È¡DFÎ»
						 }
						 if(hh_message!=0)//
						 {
						  //   hh_message=1;
						  //   continue;
						// }
#endif						
							int_cnt[3]++;
							int_cnt_3++;
	//-------------¼ÌĞø½âÎö
							mmCopy(pulse[pulse_wr],pulse_amp,112<<1);//
							//zhangfulong add Ê±¼ä¸üĞÂ½øÀ´µÄ¹¦ÄÜ
							//zhangfulong add Ê±¼ä¸üĞÂ  Ã»ÓĞÓÃ ÒªÔÚÆäËûµØ·½Ê¹ÓÃ¸üĞÂ£¬±ÜÃâÃ»´«Íê¾ÍÓÃ
							UTC_time_real[0] = UTC_time_all[13];
							UTC_time_real[1] = UTC_time_all[12];
							UTC_time_real[2] = UTC_time_all[15];
							UTC_time_real[3] = UTC_time_all[14];
							UTC_time_real[4] = UTC_time_all[17];
							UTC_time_real[5] = UTC_time_all[16];
							//s_FPGA_YC.time = 0;
							time_tmp_01 = (long)(UTC_time_real[0]<<40) + (long)(UTC_time_real[1]<<32) + (long)(UTC_time_real[2]<<24) + (long)(UTC_time_real[3]<<16) + (long)(UTC_time_real[4]<<8) + (long)(UTC_time_real[5]);
							time_tmp_01 = ((long)(time_tmp_01/10000)) & 0x00000000FFFFFFFF;
							//zhangfulong add Ê±¼ä¸üĞÂ
							s_FPGA_YC.UTCtime = (int)(time_tmp_01);
							//
							//pulse_time[pulse_wr]=s_FPGA_YC.UTCtime;//190521
							//zhangfulong add time change start
							//zhangfulong add Ê±¼ä¸üĞÂ
							UTC_time_real[0] = UTC_time_all[13];
							UTC_time_real[1] = UTC_time_all[12];
							UTC_time_real[2] = UTC_time_all[15];
							UTC_time_real[3] = UTC_time_all[14];
							UTC_time_real[4] = UTC_time_all[17];
							UTC_time_real[5] = UTC_time_all[16];
							//zhangfulong add Ê±¼ä¸üĞÂ

							pulse_time[pulse_wr]= (UTC_time_real[2]) + (UTC_time_real[3] << 8) +(UTC_time_real[4] << 16) + (UTC_time_real[5]<< 24);
							//pulse_time[pulse_wr]= (int) (((UTC_time_real[0]) + (UTC_time_real[1] << 8) +(UTC_time_real[2] << 16) + (UTC_time_real[3]<< 24) + (UTC_time_real[4]<< 32) + (UTC_time_real[5]<< 40))/10000);

							//zhangfulong add time change end

							if(pulse_cnt<pulse_BUFLEN-2)
							{
								pulse_cnt++;
							}
							else  //»º³åÂú £¨²»Ó¦¸ÃÂú£©
							{
								pulse_cnt =pulse_BUFLEN-1;
							}
							if(pulse_wr<pulse_BUFLEN-2)  //190613 -1
							{
								pulse_wr++;
							}
							else
							{
								pulse_wr=0;
							}
	//-------------¸øÍøÂç×¼±¸µÄ
	                 	//	mmCopy(pulse_IP[pulse_wr_IP],pulse_amp,112);//
					    	get_pulse_TOByte(pulse_IP[pulse_wr_IP],pulse_amp);
						    if(pulse_cnt_IP<pulse_BUFLEN_IP-2)
							{
								pulse_cnt_IP++;
						    }
							else  //»º³åÂú £¨¸Ğ¾õ¿Ï¶¨»áÂú?//ÊÇ·ñÉÒÔÊÊµ±¿ª´ópulse_BUFLEN_IP
							{
						    //	pulse_cnt_IP =pulse_BUFLEN_IP-1;//190410
	                        //--
	                            pulse_cnt_IP=0;
								pulse_wr_IP=0;
								pulse_rd_IP=0;
							}
							if(pulse_wr_IP<pulse_BUFLEN_IP-2)  //190613 -1
							{
								pulse_wr_IP++;
							}
							else
							{
								pulse_wr_IP=0;
							}
							//zhangfulong
							/*if (position_cnt < position_BUFLEN-2)
							{
								position_cnt++;
							}
							else
							{
								position_cnt =0;
								position_rd = 0;
							}*/
							//zhangfulong
	                     }// if(hh_message!=0)
			        }// if(plane_ok==0)//ËµÃ÷½âËã³É?
				  }     // if(data_head==1)
                  IER_tmp = IER&0x200;
				  IER     &=0xfffffdff;
		          SEND_ADDR_CACHE[g_u32IpSendIndex].pwrite=0;
				  if(g_u32IpSendIndex<SEND_IP_DATA_BUFLEN-2)
				  {
						g_u32IpSendIndex++;
				  }
				  else
				  {
						g_u32IpSendIndex=0;
				  }
				  if(SND_Catch_Program_Full>0)
				  {
				    	SND_Catch_Program_Full--;
				  }
				  //else
				  IER |=IER_tmp; ///2013
				  if(SND_Catch_Program_Full==0)
				  {
						break;
				  }
				}//if(SEND_ADDR_CACHE[g_u32IpSendIndex].pwrite==1) 
#if 0
				else
				{
					//C62_disableIER(1<<9);
					IER_tmp = IER&0x200;
					IER     &=0xfffffdff;
					if(g_u32IpSendIndex<SEND_IP_DATA_BUFLEN-2)
					{
						g_u32IpSendIndex++;
					}
					else
					{
						g_u32IpSendIndex=0;
					}

					IER |=IER_tmp;
					if(SND_Catch_Program_Full==0)
					{
						break;
					}
				//	C62_enableIER( 1<<9 ); //test
				    //IER |=IER_tmp;
				 }//if(SEND_ADDR_CACHE[g_u32IpSendIndex].pwrite==1) else 
#endif
	        }//while(1) 
	 }//if(SND_Catch_Program_Full>0)
  	 TaskSleep(1); //1s
  } //while(1)
}
//int jiebaolv = 0;//zhangfulong add
/**********************************************/
void get_PosV()  //ÈÎÎñ2£º 112->½âËãÎ»ÖÃ
{
  unsigned short int dp;
  unsigned short int position_ok,velocity_ok;
  unsigned short   * data_fifo;
  unsigned int hhtmp;
  unsigned  IER_tmp=0;
  while(1)
  {
//¿¼ÂÇÒ»ÏÂdata_pro£¨£©´¦ÀíµÄ¶ÔÏóÊÇÊ²Ã´£¿---pulse[]
//Ó¦¸ÃÊÇ¸öÊı×é£¬Ö»Òª·Ç¿Õ¾Í¿ÉÒÔ´¦Àí  --pulse_cnt·Ç¿Õ
//»¹ĞèÒªÎ¬»¤Ò»ÏÂÖ¸Õë  --pulse_rd
//´¦ÀíÍêµÄÊı¾İÔõÃ´´æ·Å£¿
    
     if(pulse_cnt>0)
     {
        while(1)
		{
		    if(pulse_cnt==1)//1
		    {
			   TaskSleep(1);
		    }
			//×ª»»Ò»ÏÂ
            data_fifo = pulse[pulse_rd]; //
		    t4=CLK_gethtime();
		//	gpio7_set_1();
			dp= data_pro(data_fifo);  //demodulator   
		 //   gpio7_set_0();
	        if((dp==0)||(dp==1))
			{
			  ;//  return(dp);
			}
		    if(dp==2)
			{
	//		  gpio7_set_1();
			  position_ok=decode_position();
	//		  gpio7_set_0();
			  if(position_ok==2)	//zhangfulong add 20201012 Î´ÖªÎÊÌâ¸ü¸Ä£ºÎ»ÖÃ½âËã²»Ğ´½ø½á¹û
			  //if (0)
			  {
			   	pos_2_cnt++ ;
			  }
			  else
			  {
			    pos_3_cnt++; 
			    t2=CLK_gethtime();
			//	gpio7_set_0();
/*            //»ñÈ¡airplane_location
              airplane_location.time;
              airplane_location.ICAO_adress_with_mark;
			  airplane_location.position.coordinate[0];
			  airplane_location.position.coordinate[1];*/
              //--ÈÚºÏ 
             //   memcpy(position[position_wr], &airplane_location, 16);//
             //   hhtmp=htonl(airplane_location.ICAO_address);  //190610
 				hhtmp=htonl(airplane_location.time); //190610
                memcpy(&position[position_wr][0],(void *)(&hhtmp), 4);//
             //   hhtmp=htonl(airplane_location.time);  //190610
			    hhtmp=htonl(airplane_location.ICAO_address);  //190610
				memcpy(&position[position_wr][4],(void *)(&hhtmp),4);//190419
              //  memcpy(&position[position_wr][4],(void *)&(airplane_location.time), 4);//
                hhtmp=htonl(airplane_location.coordinate[0]);
				memcpy(&position[position_wr][8],(void *)(&hhtmp), 4);//
                hhtmp=htonl(airplane_location.coordinate[1]);
				memcpy(&position[position_wr][12],(void *)(&hhtmp), 4);//

				if(position_cnt<position_BUFLEN-2)
				{
					position_cnt++;
				}
				else  //»º³åÂú £¨Ó¦¸ÃÂú£©
				{
				//	position_cnt = position_BUFLEN-1;  //190410
				//--
					position_cnt=0;  //190410
					position_wr=0;
					position_rd=0;
				}
				if (ronghe_cnt_p < position_BUFLEN-2)
				{
					ronghe_cnt_p++;
				}
				else
				{
					ronghe_cnt_p = 0;
				}
				if(position_wr < position_BUFLEN-2)  //190523 -1
 				{
					position_wr++;
					//ronghe_cnt_p++;
				}
				else
				{
					position_wr=0;
					//ronghe_cnt_p =0;
				}
              //--Ï¡Êè»¯
		        memcpy(position_xsh[position_wr_xsh], &airplane_location, 20);//190418
                memcpy(&position_xsh_ICAO[position_wr_xsh],&airplane_location,4);  //icao 190410
				//Ï¡Êè»¯¸øĞÂµÄ¸³Öµ
				airplane_location_XSH_static[position_wr_xsh].altitude = airplane_location.altitude;
				airplane_location_XSH_static[position_wr_xsh].coordinate[0] = airplane_location.coordinate[0];
				airplane_location_XSH_static[position_wr_xsh].coordinate[1] = airplane_location.coordinate[1];
				airplane_location_XSH_static[position_wr_xsh].ICAO_address = airplane_location.ICAO_address & 0x00FFFFFF;
				airplane_location_XSH_static[position_wr_xsh].time = airplane_location.time;

				
				//zhangfulong add start½«Êı¾İ´æÈëÈ«¾ÖµÄÀïÃæ airplane_location_staticĞÂ¶¨Òå1500È«¾Ö
				//memcpy(&(airplane_location_static[position_wr]), &airplane_location, 20);
				airplane_location_static[position_wr].altitude = airplane_location.altitude;
				airplane_location_static[position_wr].coordinate[0] = airplane_location.coordinate[0];
				airplane_location_static[position_wr].coordinate[1] = airplane_location.coordinate[1];
				airplane_location_static[position_wr].ICAO_address = airplane_location.ICAO_address & 0x00FFFFFF;
				airplane_location_static[position_wr].time = airplane_location.time;
				//airplane_location_static[position_wr].ishandle = 0;

				airplane_location_static_ronghe[ronghe_position_p ].altitude = airplane_location.altitude;
				airplane_location_static_ronghe[ronghe_position_p ].coordinate[0] = airplane_location.coordinate[0];
				airplane_location_static_ronghe[ronghe_position_p ].coordinate[1] = airplane_location.coordinate[1];
				airplane_location_static_ronghe[ronghe_position_p ].ICAO_address = airplane_location.ICAO_address & 0x00FFFFFF;
				airplane_location_static_ronghe[ronghe_position_p ].time = airplane_location.time;
				if (airplane_location_static_ronghe[ronghe_position_p ].ICAO_address  == 0x00666666)
				{
					printf("jiechu ICAO_address  == 0x00666666 ; time = %x ;\n",airplane_location.time);
				}

				if(ronghe_position_p < position_BUFLEN-2)
				{
					ronghe_position_p++;
				}
				else
				{
					ronghe_position_p = 0;
				}
				//printf("get data %x !!\n",airplane_location.time);
				//jiebaolv++;
				//printf("jiebaolv : %d \n", jiebaolv);
				
				
				
				if(position_cnt_xsh<position_BUFLEN_xsh-2)
				{
					position_cnt_xsh++;
				}
				else  //»º³åÂú ¦¸ÃÂú£?
				{
				//	position_cnt_xsh = position_BUFLEN_xsh-1;//  190410
                //--
				    position_cnt_xsh=0; //190410
					position_wr_xsh=0;
					position_rd_xsh=0;

				}
				if(position_wr_xsh < position_BUFLEN_xsh-2)   //190613 -1
				{
					position_wr_xsh++;
				}
				else
				{
					position_wr_xsh=0;
				}
			  }
	      	  //   return(decode_position());  
	      	  //·µ»ØÖµ2£¬Ö¸Ê¾¸ÃÖ¡´¦ÀíÊÇÎ»ÖÃ±¨ÎÄ£¬µ«ÊÇ½âËã²»³É¹¦£¬½ö¸üĞÂÁËADSB_messageÊı×é£¬ÒÔ¼°Êı¾İ¿â£¬Î´½âËã³öÓĞĞ§µÄÎ»ÖÃĞÅÏ¢
	          //·µ»ØÖµ3£¬Ö¸Ê¾¸ÃÖ¡´¦ÀíÊÇÎ»ÖÃ¨ÎÄ£¬½âËã³É¹¦£¬¸üĞÂÁËADSB_messageÊı×é£¬¸üĞÂÁËÊı¾İ¿â£¬¸üĞÂÁËÉÏ±¨Î»ÖÃÓÃµÄairplane_location½á¹¹Ìå±ä	                                       
			} 
			if(dp==4)   
			{
			//	gpio7_set_1();
	     	   velocity_ok=decode_velocity();  //4,5
	
               if(velocity_ok==4)   
               {
				 vel_4_cnt++;//
               }
               else
               {
                 vel_5_cnt++;
				 t3=CLK_gethtime();
			//	 gpio7_set_0();
			//	 gpio7_set_0();
/*   //airplane_velocity
		airplane_velocity.time;
		airplane_velocity.ICAO_address;
		airplane_velocity.E_W_velocity;
		airplane_velocity.N_S_velocity;
		airplane_velocity.VERT_velocity;*/
           //    memcpy(velocity[velocity_wr], (&airplane_velocity_three), 20);//
			   hhtmp=htonl(airplane_velocity_three.ICAO_address);
               memcpy(&velocity[velocity_wr][0],(void *)(&hhtmp), 4);//
               hhtmp=htonl(airplane_velocity_three.time);
               memcpy(&velocity[velocity_wr][4],(void *)(&hhtmp), 4);//
               hhtmp=htonl(airplane_velocity_three.N_S_velocity);
               memcpy(&velocity[velocity_wr][8],(void *)(&hhtmp), 4);//
	           hhtmp=htonl(airplane_velocity_three.E_W_velocity);
               memcpy(&velocity[velocity_wr][12],(void *)(&hhtmp), 4);//
		       hhtmp=htonl(airplane_velocity_three.VERT_velocity);
               memcpy(&velocity[velocity_wr][16],(void *)(&hhtmp), 4);//
 			//	memcpy(&velocity[velocity_wr][8],&(airplane_velocity_three.E_W_velocity), 12);//  ÁÙÊ±µÄ£¬ÒòÎªËÙ¶ÈµÄ±äÁ¿ÀàÍ²»¶?0190422
				

				//ÕÅÂ¡ add XSH ËÙ¶È
				airplane_velocity_three_XSH_static[position_wr_xsh_v].ICAO_address = (airplane_velocity_three.ICAO_address) & 0x00FFFFFF;
				airplane_velocity_three_XSH_static[position_wr_xsh_v].E_W_velocity = airplane_velocity_three.E_W_velocity;
				airplane_velocity_three_XSH_static[position_wr_xsh_v].N_S_velocity = airplane_velocity_three.N_S_velocity;
				airplane_velocity_three_XSH_static[position_wr_xsh_v].time = airplane_velocity_three.time;
				airplane_velocity_three_XSH_static[position_wr_xsh_v].VERT_velocity = airplane_velocity_three.VERT_velocity;
				if(position_wr_xsh_v<position_BUFLEN_xsh-2)
				{
					position_wr_xsh_v++;
				}
				else  //»º³åÂú ¦¸ÃÂú£?
				{
				    position_wr_xsh_v=0; //190410

				}

				//zhangfulong add start
				//memcpy(&(airplane_velocity_three_static[velocity_wr]), (&airplane_velocity_three), sizeof(airplane_velocity_three));
				airplane_velocity_three_static[velocity_wr].E_W_velocity = airplane_velocity_three.E_W_velocity;
				airplane_velocity_three_static[velocity_wr].ICAO_address = airplane_velocity_three.ICAO_address & 0x00FFFFFF;
				airplane_velocity_three_static[velocity_wr].N_S_velocity = airplane_velocity_three.N_S_velocity;
				airplane_velocity_three_static[velocity_wr].time = airplane_velocity_three.time;
				airplane_velocity_three_static[velocity_wr].VERT_velocity = airplane_velocity_three.VERT_velocity;
				
				//zhangfulong add end
				airplane_velocity_three_static_ronghe[ronghe_position_v ].E_W_velocity = airplane_velocity_three.E_W_velocity;
				airplane_velocity_three_static_ronghe[ronghe_position_v ].ICAO_address = airplane_velocity_three.ICAO_address & 0x00FFFFFF;
				airplane_velocity_three_static_ronghe[ronghe_position_v ].N_S_velocity = airplane_velocity_three.N_S_velocity;
				airplane_velocity_three_static_ronghe[ronghe_position_v ].time = airplane_velocity_three.time;
				airplane_velocity_three_static_ronghe[ronghe_position_v ].VERT_velocity = airplane_velocity_three.VERT_velocity;
				if(ronghe_position_v  < position_BUFLEN-2)
				{
					ronghe_position_v++;
				}
				else
				{
					ronghe_position_v  = 0;
				}

 			//	memcpy(&velocity[velocity_wr][8],&(airplane_velocity_three.E_W_velocity), 12);//  ÁÙÊ±µÄ£¬ÒòÎªËÙ¶ÈµÄ±äÁ¿ÀàÍ²»¶?0190422
				
				
				if(velocity_cnt<velocity_BUFLEN-2)
				{
			     	velocity_cnt++;
				}
				else  //»º³åÂú £¨Ó¦¸ÃÂú£©
				{
			//		velocity_cnt = velocity_BUFLEN-1;  //190410
			//--
					velocity_cnt=0;//190410
					velocity_wr=0;
					velocity_rd=0;
				}
				if (ronghe_cnt_v < velocity_BUFLEN-2)
				{
					ronghe_cnt_v++;
				}
				else
				{
					ronghe_cnt_v = 0;
				}
				if(velocity_wr < velocity_BUFLEN-2)  //190613 -1
				{
					velocity_wr++;
					//ronghe_cnt_v++;
				}
				else
				{
					velocity_wr=0;
					//ronghe_cnt_v =0;
				}
               } 
		       //  return(decode_velocity());
		       //·µ»ØÖµ4£¬Ö¸Ê¾¸ÃÖ¡´¦ÀíÊÇËÙ¶È±¨ÎÄ£¬µ«ÊÇ½âËã²»³É¹¦£¬½ö¸üĞÂÁËADSB_messageÊı×é£¬ÓÉÓÚÎ´ÖªµÄÔ­ÒòÎ´½âËã³öÓĞĞ§µÄËÙ¶ÈĞÅÏ¢
			   //·µ»ØÖµ5£¬Ö¸Ê¾¸ÃÖ¡´¦ÀíÊÇËÙ¶È±¨ÎÄ£¬½âËã³É¹¦£¬¸üĞÂÁËADSB_messageÊı×é£¬¸üĞÂÁËÉÏ±¨Î»ÖÃÓÃµÄXXXÊı×é£¬ÉÏ±¨Î»ÖÃĞÅÏ¢	                                       
			}
		    IER_tmp = IER&0x200;
		    IER     &=0xfffffdff;
            if(pulse_rd<pulse_BUFLEN-2) //
			{
				pulse_rd++;
			}
			else
			{
				pulse_rd=0;
			}
			if(pulse_cnt>0)
			{
			   	pulse_cnt--;
			}
            IER |=IER_tmp; ///2013
			if(pulse_cnt==0)
			{
			   break;
			}
	   }// while(1)
    }//if(pulse_cnt>0)
	TaskSleep(1); //1s
  } //while
}
/*****************************************/
Uint8 find_ICAO_2(Uint8 num,unsigned int icao)
{
   Uint8 i=0; //50´ú±íÃ»ÕÒµ½
   Uint8 ret=FIND_LOST;
   if(end_pos[num]>start_pos[num])  //1¶Î
   {
     for(i=start_pos[num];i<=end_pos[num];i++)
     {
        if(icao==trans_Record[i].ICAO_adress_with_mark)
		{
		    ret=i;
		    break;//ÕÒµ½ÄĞºÅ 
		}
     }
   }
   else  //2¶Î
   {
      for(i=0;i<=end_pos[num];i++)
      {
         if(icao==trans_Record[i].ICAO_adress_with_mark)
	     { 
		     ret=i;
         	 break;//ÕÒµ½µÄĞòºÅ
		 }
      }
      for(i=start_pos[num];i<FIND_RANGE;i++)
      {
        if(icao==trans_Record[i].ICAO_adress_with_mark)
		{
		    ret=i;
		    break;//ÕÒµ½µÄĞòºÅ 
		}
      }
   } 
   return ret;
}
/*****************************************/
Uint32 XSH_last_ICAOnumber[4] = {0};
int find_in_lastICAOnumber(int ICAO)
{
	//±¾º¯ÊıÔÚXSH_last_ICAOnumberÀïÃæËÑË÷Ö¸¶¨µÄICAOºÅÒªÊÇÓĞ·µ»Ø1£¬·ñÔòÃ»ÓĞµÄ»°·µ»Ø0
	int i,j,k;
	if (ICAO == 0)
	{
		return 0;
	}
	for (i = 0; i < 4; i++)
	{
		if (ICAO == XSH_last_ICAOnumber[i])
		{
			return 1;
		}
	}
	return 0;
}

void xsh_2_task()  //20190211  ÈÎÎñ3  Ï¡Êè»¯
{
    Uint16 i=0;
	Uint16 j,k;
	Uint32 time1=0,time2=0;
    Uint16 time_delay=0;
//	Uint8 pro_stat[60]={0};
//    Uint8 pro_cnt=0;
	Uint16 h_cnt=0; 
	Uint8 tr_num=0;
	Uint8 fi=0;
	Uint8 bakU20=0;
	Uint8 bakU40=0;
	Uint8 bakU41=0;
	Uint32 cnt_tmp;
	Uint32 timefirst=0;
	Uint32 timefive=0;
	Uint32 timelast5=0;
	Uint8 insert_idx=0;
	Uint8 tR_add_cnt=0;
	Uint8 XSH_bao_cnt = 0;
	//
    const Uint8 xsh_samp[8]={
	0x5a,0xff,0x5a,0xff,0x0,0x0,0x0,0x0
    } ;
    const Uint8 pack_xsh_nodata[20]={0};  //Ìî³ä0
    memcpy(XSH_Array, xsh_samp, 8);//
//³õÊ¼»¯----
    for(i=0;i<FIND_RANGE;i++)
	{
		trans_Record[i].ICAO_adress_with_mark=0xffffffff;
		start_pos[i]=(i+5+FIND_RANGE-1)%FIND_RANGE;
    	end_pos[i]=(i+FIND_RANGE-1)%FIND_RANGE;
	}

//×ö¸öÖ¡Í·
 	while( 1 )
	{
		//zhangfulong add Ê±¼ä¸üĞÂ
		UTC_time_real[0] = UTC_time_all[13];
		UTC_time_real[1] = UTC_time_all[12];
		UTC_time_real[2] = UTC_time_all[15];
		UTC_time_real[3] = UTC_time_all[14];
		UTC_time_real[4] = UTC_time_all[17];
		UTC_time_real[5] = UTC_time_all[16];
		//zhangfulong add Ê±¼ä¸üĞÂ
		for (k = 0; k < 114; k++)
		{
			XSH_Array[k] = 0;
		}

		flag=5; //
		XSH_Array[4]++; //¼ÆÊı
	//	XSH_Array2[23]++;
	//	XSH_Array2[23+16]++;
	//  XSH_Array2[23+16+16]++;
	//	XSH_Array2[23+16+16+16]++;
		while((position_cnt_xsh<20)&&(time_delay<800))  //ÀÛ¼ÆÒ»¶¨µÄ½âÎöÊı¾İ
		{
		  TaskSleep(1);   //ÀÛ¼ÆÎó²îĞèÒª¿Û³ı
		  time_delay++;
		}
#if 1
	    //1 Î¬»¤Ò»ÏÂtrans_Record
		tR_add_cnt=0;
        while(position_cnt_xsh>0)  // ÓĞÊı¿ÉÒÔ½â
		//if(position_cnt_xsh>0)
		{
		 	fi=find_ICAO_2(sum_cnt,(position_xsh_ICAO[position_rd_xsh]&0x00ffffff));
#if 0			//ÕÅ¸»Â¡add È¥µôÔ­À´µÄÌî³ä£¬ÖØĞÂĞ´
            if(FIND_LOST==fi) //50  Ã»ÕÒµ½
	        {
                 //²åÔÚÕâÀï
				 trans_Record[sum_cnt].ICAO_adress_with_mark=position_xsh_ICAO[position_rd_xsh] & 0x00ffffff;

			     trans_Record[sum_cnt].time=(Uint32)(position_xsh[position_rd_xsh][4]<<24)
			            +(Uint32)(position_xsh[position_rd_xsh][5]<<16)
			            +(Uint32)(position_xsh[position_rd_xsh][6]<<8)
			            +position_xsh[position_rd_xsh][7];
		
				 trans_Record[sum_cnt].flag=1; 
			     memcpy(&(XSH_Array[20+tR_add_cnt*20]), position_xsh[position_rd_xsh], 20); //20

	              //Î¬»¤Ö¸Õë 
				 if(position_rd_xsh<position_BUFLEN_xsh-2) //
				 {
						position_rd_xsh++;
				 }
				 else
				 {
						position_rd_xsh=0;
			     }
				 if(position_cnt_xsh>0)
				 {
					   	position_cnt_xsh--;
				 } //190611 
				 tR_add_cnt++;//
				 sum_cnt++;
			     if(tR_add_cnt>=5)
			     {
			           break;
			     }  

			}
            else  //ÕÒµ½  --Õâü²»·¢Á?
		    {
	           ; //ÕâÀï»áÉáÆúÒ»°ü//ºóÃæÔõÃ´Íê?
		       trans_Record[fi].time=(Uint32)(position_xsh[position_rd_xsh][4]<<24)
		            +(Uint32)(position_xsh[position_rd_xsh][5]<<16)
		            +(Uint32)(position_xsh[position_rd_xsh][6]<<8)
		            +position_xsh[position_rd_xsh][7];  //¸üĞÂÊ±¼ä  --ÆäÊµÃ»ÓĞÈÎºÎÒâÒå
			  //Î¬»¤Ö¸Õë
			   if(position_rd_xsh<position_BUFLEN_xsh-2) //
			   {
					position_rd_xsh++;
			   }
			   else
			   {
					position_rd_xsh=0;
			   }
			   if(position_cnt_xsh>0)
			   {
				   	position_cnt_xsh--;
			   }
		   	}//
#else
			if(FIND_LOST==fi) //50  Ã»ÕÒµ½
	        {
                 //²åÔÚÕâÀï
				 trans_Record[sum_cnt].ICAO_adress_with_mark=position_xsh_ICAO[position_rd_xsh] & 0x00ffffff;

			     trans_Record[sum_cnt].time=(Uint32)(position_xsh[position_rd_xsh][4]<<24)
			            +(Uint32)(position_xsh[position_rd_xsh][5]<<16)
			            +(Uint32)(position_xsh[position_rd_xsh][6]<<8)
			            +position_xsh[position_rd_xsh][7];
		
				 trans_Record[sum_cnt].flag=1; 
			     //memcpy(&(XSH_Array[20+tR_add_cnt*20]), position_xsh[position_rd_xsh], 20); //20
				//Ìî³äÊı¾İ
				//È·±£²»ÖØ¸´
				if (find_in_lastICAOnumber(airplane_location_XSH_static[position_rd_xsh].ICAO_address) != 1)
				{
					XSH_Array[tR_add_cnt * 12 + 8]=(airplane_location_XSH_static[position_rd_xsh].ICAO_address&0xff000000)>>24;
					XSH_Array[tR_add_cnt * 12 + 9]=(airplane_location_XSH_static[position_rd_xsh].ICAO_address&0x00ff0000)>>16;
					XSH_Array[tR_add_cnt * 12 + 10]=(airplane_location_XSH_static[position_rd_xsh].ICAO_address&0x0000ff00)>>8;
					XSH_Array[tR_add_cnt * 12 + 11]=(airplane_location_XSH_static[position_rd_xsh].ICAO_address&0x000000ff);

					XSH_Array[tR_add_cnt * 12 + 12]=(airplane_location_XSH_static[position_rd_xsh].coordinate[1]&0xff000000)>>24;
					XSH_Array[tR_add_cnt * 12 + 13]=(airplane_location_XSH_static[position_rd_xsh].coordinate[1]&0x00ff0000)>>16;
					XSH_Array[tR_add_cnt * 12 + 14]=(airplane_location_XSH_static[position_rd_xsh].coordinate[1]&0x0000ff00)>>8;
					XSH_Array[tR_add_cnt * 12 + 15]=(airplane_location_XSH_static[position_rd_xsh].coordinate[1]&0x000000ff);

					XSH_Array[tR_add_cnt * 12 + 16]=(airplane_location_XSH_static[position_rd_xsh].coordinate[0]&0xff000000)>>24;
					XSH_Array[tR_add_cnt * 12 + 17]=(airplane_location_XSH_static[position_rd_xsh].coordinate[0]&0x00ff0000)>>16;
					XSH_Array[tR_add_cnt * 12 + 18]=(airplane_location_XSH_static[position_rd_xsh].coordinate[0]&0x0000ff00)>>8;
					XSH_Array[tR_add_cnt * 12 + 19]=(airplane_location_XSH_static[position_rd_xsh].coordinate[0]&0x000000ff);
				/*
					memcpy(&(XSH_Array[tR_add_cnt * 12 + 0]), &(airplane_location_XSH_static[position_rd_xsh].ICAO_address), 4);
					memcpy(&(XSH_Array[tR_add_cnt * 12 + 4]), &(airplane_location_XSH_static[position_rd_xsh].coordinate[1]), 4);
					memcpy(&(XSH_Array[tR_add_cnt * 12 + 8]), &(airplane_location_XSH_static[position_rd_xsh].coordinate[0]), 4);
				*/	
					XSH_last_ICAOnumber[tR_add_cnt] = airplane_location_XSH_static[position_rd_xsh].ICAO_address;
				}
				else//ÖØ¸´ÁËĞ´0
				{
					memcpy(&(XSH_Array[tR_add_cnt * 12 + 0]), pack_xsh_nodata, 4);
					memcpy(&(XSH_Array[tR_add_cnt * 12 + 4]), pack_xsh_nodata, 4);
					memcpy(&(XSH_Array[tR_add_cnt * 12 + 8]), pack_xsh_nodata, 4);
					XSH_last_ICAOnumber[tR_add_cnt] = 0;
					
				}
			
				//¼ÇÂ¼±¾´ÎICAOºÅ
				//XSH_Array[] = ;
				
	              //Î¬»¤Ö¸Õë 
				 if(position_rd_xsh<position_BUFLEN_xsh-2) //
				 {
						position_rd_xsh++;
				 }
				 else
				 {
						position_rd_xsh=0;
			     }
				 if(position_cnt_xsh>0)
				 {
					   	position_cnt_xsh--;
				 } //190611 
				 tR_add_cnt++;//
				 sum_cnt++;
			     if(tR_add_cnt>=3)
			     {
			           break;
			     }  

			}
            else  //ÕÒµ½  --Õâ°ü²»·¢ÁË
		    {
	           ; //ÕâÀï»áÉáÆúÒ»°ü//ºóÃæÔõÃ´Íê?
		       trans_Record[fi].time=(Uint32)(position_xsh[position_rd_xsh][4]<<24)
		            +(Uint32)(position_xsh[position_rd_xsh][5]<<16)
		            +(Uint32)(position_xsh[position_rd_xsh][6]<<8)
		            +position_xsh[position_rd_xsh][7];  //¸üĞÂÊ±¼ä  --ÆäÊµÃ»ÓĞÈÎºÎÒâÒå
			  //Î¬»¤Ö¸Õë
			   if(position_rd_xsh<position_BUFLEN_xsh-2) //
			   {
					position_rd_xsh++;
			   }
			   else
			   {
					position_rd_xsh=0;
			   }
			   if(position_cnt_xsh>0)
			   {
				   	position_cnt_xsh--;
			   }
		   	}
#endif
		} //while(position_cnt_xsh>0)
//Ìî³ä	
		if(tR_add_cnt<3)//ÊÇ·ñÒªÌî³ä
		{
		    for(i=tR_add_cnt;i<3;i++)//pack_ads_nodata  //Ìî³ä0x0
			{
			    //memcpy(&(XSH_Array[20+i*20]), pack_xsh_nodata, 20); //20
				memcpy(&(XSH_Array[i * 12 + 8]), pack_xsh_nodata, 12);
				sum_cnt++;
			}
		}
		//Ìî¸öËÙ¶È
		if (XSH_last_ICAOnumber[3] != (airplane_velocity_three_XSH_static[position_wr_xsh_v].ICAO_address))
		{
			memcpy(&(XSH_Array[44]), &(airplane_velocity_three_XSH_static[position_wr_xsh_v].ICAO_address), 4);
			memcpy(&(XSH_Array[48]), &(airplane_velocity_three_XSH_static[position_wr_xsh_v].N_S_velocity), 4);
			memcpy(&(XSH_Array[52]), &(airplane_velocity_three_XSH_static[position_wr_xsh_v].E_W_velocity), 4);
			memcpy(&(XSH_Array[56]), &(airplane_velocity_three_XSH_static[position_wr_xsh_v].VERT_velocity), 4);
			XSH_last_ICAOnumber[3] = airplane_velocity_three_XSH_static[position_wr_xsh_v].ICAO_address;
		}
		else
		{
			memcpy(&(XSH_Array[44]), pack_xsh_nodata, 4);
			memcpy(&(XSH_Array[48]), pack_xsh_nodata, 4);
			memcpy(&(XSH_Array[52]), pack_xsh_nodata, 4);
			memcpy(&(XSH_Array[56]), pack_xsh_nodata, 4);
			XSH_last_ICAOnumber[3] = 0;
		}
		//
#if 0
	    if(tR_add_cnt==0)
		{
			zero_cnt++;
		//	pro_stat[pro_cnt]=0;
			if(zero_cnt>6)
			   zero_cnt=0;
		}
		else
		{
		//	pro_stat[pro_cnt]=1;
			zero_cnt=0;
		}
/*		pro_cnt++;
		if(pro_cnt>=60)
		{
		  pro_cnt=0;
		}
	*/
#endif
#endif
//-------  

		//for(i = 0; i < 52; i++)
		//{
		//	XSH_Array[i] = i;
		//}
		//Ï¡Êè»¯Êı¾İ¸ü¸Ä£¬Ç°Ãæ¼Ó0£¬Ê±¼ä£¬°ü¼ÆÊı£¬ºóÃæ¼ÓÒıµ¼Êı¾İ
		/*for (i = 51; i > 12; i--)//Êı¾İ°á¹ıÈ¥
		{
			XSH_Array[i + 8] = XSH_Array[i];
		}*/
		/*
		*/

/*
		for(i = 1;i<53;i++)
		{
			if(i%4==0)
			{
					XSH_Array[8+i-4] = XSH_Array[i-1];
					XSH_Array[8+i-3] = XSH_Array[i-2];
					XSH_Array[8+i-2] = XSH_Array[i-3];
					XSH_Array[8+i-1] = XSH_Array[i-4];

			}
		}*/
		
//		XSH_Array[0] = 0;
		//memcpy(&(XSH_Array[0]),&(UTC_time_real[0]),4+ 2);//½«Ê±¼ä¸´ÖÆ¹ıÈ¥

				//zhangfulong add Ê±¼ä¸üĞÂ
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add Ê±¼ä¸üĞÂ
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//½«Ê±¼ä¸´ÖÆ¹ıÈ¥ 4->6
				XSH_Array[0] = UTC_time_real[0];
				XSH_Array[1] = UTC_time_real[1];
				XSH_Array[2] = UTC_time_real[2];
				XSH_Array[3] = UTC_time_real[3];
				XSH_Array[4] = UTC_time_real[4];
				XSH_Array[5] = UTC_time_real[5];
				//XSH Ê±¼ä
		XSH_Array[6] = XSH_bao_cnt;//¼ÆÊı
		XSH_bao_cnt = (XSH_bao_cnt + 1) & 0xFF;//
		//XSH_Array[7] = (up_commend_count) & 0xFF;
		XSH_Array[7] = (yaokongcmd_all_data.chelue_new) & 0xFF;
		//XSH_Array[59] = 0;

		//60¿ªÊ¼¼ÓÒıµ¼Êı¾İ
		for (i = 0; i < 36; i++)
		{
			XSH_Array[60 + i] = pbuf_yindao[i];
		}

		//pbuf_yindao
		//

     	EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130104;//16
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)XSH_Array;  //ce4
	//	pEDMA3CC_PaRAM->SRC =  (unsigned char *)XSH_Array2;  //ce4	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x005E0001; //1280*4	//0x003d0001D¡ä¦Ì?FPGA??¦Ì??¡¤A0F	//51
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)DSP_TO_FPGA_XSH;
	    int_cnt[10]++;//
		EDMA3CC_ESRH = 0x00010000;  //???¡¥
   	 	if(sum_cnt>=FIND_RANGE) //Õâ¸ö±ß½çÎ£ÏÕ
			sum_cnt=0;
        if(time_delay<2000)
   	   	{
   	   		 TaskSleep(2000-time_delay); //1s
			 time_delay=0;
		}
		else
		{
		    time_delay=0; //Õâ¸ö·ÖÖ§
		}
		//printf("XSH count is %d \n",XSH_bao_cnt);
	}//while(1)
}

int V_get(int ICAO)// ÊäÈëICAOºÅ£¬Êä³ö·µ»ØÒ»¸öĞòºÅ£¬×îĞÂµÄËÙ¶ÈµÄËùÔÚÎ»ÖÃÔÚairplane_velocity_three_staticÀïµÄ
{
	int result = -1;
	int i,j,k;
	if (ICAO <= 0)
	{
		return (-1);
	}
	for (i = 0; i < position_BUFLEN; i++)
	{
		if(airplane_velocity_three_static[i].ICAO_address == ICAO)
		{
			if(result < 0)//µÚÒ»´Î½ø
			{
				result = i;
			}
			else//¸üĞÂ
			{
				if(airplane_velocity_three_static[i].time > airplane_velocity_three_static[result].time)
				{
					result = i;
				}
			}
		}
	}
	return result;
}
int P_get(int ICAO)//ÊäÈëICAOºÅ£¬Êä³ö·µ»ØÒ»¸öĞòºÅ£¬×îĞÂµÄÎ»ÖÃµÄËùÔÚÎ»ÖÃÔÚairplane_location_staticÀïµÄ
{
	int result = -1;
	int i,j,k;
	if (ICAO <= 0)
	{
		return (-1);
	}
	for (i = 0; i < position_BUFLEN; i++)
	{
		if(airplane_location_static[i].ICAO_address == ICAO)
		{
			if(result < 0)//µÚÒ»´Î½ø
			{
				result = i;
			}
			else//¸üĞÂ
			{
				if(airplane_location_static[i].time > airplane_location_static[result].time)
				{
					result = i;
				}
			}
		}
	}
	return result;
}

//ÏÂÃæÁ½¸öÔÚ×î¿ªÊ¼¶¨Òå
//	struct location_struct airplane_location_tmp = {0};//ÓÃÀ´¼ÇÂ¼ÁÙÊ±µÄÃüÖĞµÄICAOºÅµÈĞÅÏ¢ Î»ÖÃ
//	struct speed_struct_three airplane_speed_tmp = {0};//ÓÃÀ´¼ÇÂ¼ÁÙÊ±µÄÃüÖĞµÄICAOºÅµÈĞÅÏ¢ ËÙ¶È
	struct location_struct_withnumber
	{
	  unsigned int ICAO_address;
	  unsigned int time;
	  int coordinate[2];//ÕıÊ½ÉÏ±¨Ê±ºòÊ¹ÓÃintĞÍÕıÊ½ÉÏ±¨Ê±ºòÊ¹ÓÃint(1E-7Îªµ¥Î»)¡£
	  int altitude;
	  int number;
	};
	struct location_struct_withnumber airplane_location_array_tmp[position_BUFLEN] = {0};//ÔÚ×ÔÓÉËÑË÷ÏÂ´æ´¢ÔÚÇøÓòÄÚµÄ

	int location_arrary[1500];//¸Ã±äÁ¿ÓÃÀ´¼ÇÂ¼¸÷¸ö²ßÂÔµÄÊä³ö½á¹ûµÄÎ»ÖÃ£¬Ã¿´Î¿ªÊ¼Ç°¶¼Ó¦¸Ã¸³Öµ³É-1×´Ì¬
//ĞÂ¼Ó²ßÂÔº¯Êı£¬ºóÃæ°´ÕÕÒªÇóµ÷ÓÃ¹ØÏµ£¬µ÷ÓÃº¯ÊıµÄË³Ğò   ½ö×ÔÓÉËÑË÷Ê¹ÓÃ
// ·µ»Ø ÔÚairplane_location_array_tmpÊı×éÖĞµÄÒ»¸öÎ»ÖÃÖµ ¼ÇÂ¼location_arrary[1500]ÖĞ
//ÊäÈë ÔÚairplane_location_array_tmpÖĞµÄcountÖµ£¬Ç°ÃæÓĞÓĞ¶àÉÙÓĞÓÃµÄ
void chelue_FF(const int count)//¸üĞÂÆµÂÊ×î¿ìµÄ
{
	int i,j,k;
	int location_arrary_tmp[1500] = {-1};//ÁÙÊ±¼ÇÂ¼
	int max_location;
	int count_tmp = 0;
	for (i = 0; i < count; i++)
	{
		for (j = i; j < count ; j++)
		{
			if((airplane_location_array_tmp[i].ICAO_address == airplane_location_array_tmp[j].ICAO_address) && (airplane_location_array_tmp[i].ICAO_address != 0) )
			{
				//µÚÒ»´Î¿Ï¶¨»á½øÀ´
				airplane_location_array_tmp[i].number = airplane_location_array_tmp[i].number + 1;
			}
		}
	}
	//ÉÏÃæÑ­»·½«Ã¿Ò»¸öICAOºÅ³öÏÖµÄ´ÎÊıĞ´Èëairplane_location_array_tmp[count].numberÀïÃæ
	//ÆµÂÊµÄ¿ÉÒÔÏÈÖ´ĞĞ£¬·´ÕıÒ²ÊÇĞ´µ½
	if (location_arrary[0] == -1)//µÚÒ»´Î½ø
	{
		//¿ªÊ¼Ñ°ÕÒ³öÏÖÆµÂÊ×î¶àµÄ·É»ú
		max_location = 0;
		for (i = 1; i < count; i++)
		{
			if ((airplane_location_array_tmp[i].number > airplane_location_array_tmp[max_location].number)&& (airplane_location_array_tmp[i].ICAO_address != 0))
			{
				//ÕâÒ»ÌõµÄ³öÏÖ´ÎÊı×î¶à£¬¸üĞÂmax_locationÎ»ÖÃ
				max_location = i;
				//Çå¿ÕÖ®Ç°¼ÇÂ¼µÄÎ»ÖÃÖµ
				for (j = 0; j < 1500; j++)
				{
					location_arrary_tmp[j] = -1;
				}
				count_tmp = 0;
				location_arrary[count_tmp] = max_location;
				count_tmp = count_tmp + 1;
			}
			else if (airplane_location_array_tmp[i].number == airplane_location_array_tmp[max_location].number)
			{
				//Á½¸öÏàµÈ£¬ĞèÒªcount_tmpÉÏ¼ÓÒ»¸ö¡£
				location_arrary[count_tmp] = i;
				count_tmp = count_tmp + 1;	
			}
			else//Õâ¸ö³öÏÖµÄ´ÎÊıÃ»ÓĞÄ¿Ç°×î´óÖµ¶à£¬²»¸üĞÂ
			{
				;
			}
		}
		//¾­¹ıÉÏÃæÑ­»·£¬½«³öÏÖÆµÂÊ×î´óµÄĞòºÅĞ´½ølocation_arrary[] Êı×éÀïÃæ
		
	}
	else if ((location_arrary[0] != -1) && (location_arrary[1] == -1))
	{
		//ÄÜ½øÀ´ËµÃ÷Êı×éÀïÃæÖ»ÓĞÒ»¸öÖµ£¬²»Ö´ĞĞ²ßÂÔ£¬Ö±½Ó·µ»Ø
		return;
	}
	else 
	{
		count_tmp = 0;
		i = 0;
		while (location_arrary[i] >= 0)
		{
			i++;
			count_tmp++;
		}
		max_location = 0;
		for (i = 0; i < count; i++)//±éÀú±¨ÎÄÊı¾İ¿â£¬
		{
			for (j = 1; j < count_tmp; j++)//±éÀúÉÏÒ»¸ö²ßÂÔµÄ½á¹û
			{
				if (airplane_location_array_tmp[i].ICAO_address == airplane_location_array_tmp[location_arrary[j]].ICAO_address)
				//Õâ¸ö·É»úICAOºÅ·¶Î§ÔÚÉÏÒ»¸ö²ßÂÔÄÚ
				{
					if (airplane_location_array_tmp[i].number > airplane_location_array_tmp[location_arrary[max_location]].number)
					{
						//³öÏÖ´ÎÊı¸ü¶à
						max_location = i;
					}
				}
			}
		}
		location_arrary[0] = max_location;
		for (k = 1; k < 1500; k++)
		{
			location_arrary[k] = -1;
		}
		//ÄÜ½øÀ´ËµÃ÷ÆäËû²ßÂÔ¸ø³öÁË²»Ö¹Ò»¸öICAOºÅµÄ½á¹û£¬ÏÂÃæ±È½Ï³öÏÖÆµÂÊ
		printf("chelue_FF get in !!!!!\n");
		//ÆäËû²ßÂÔÔÚÇ°Ãæ£¬Ö»¿ÉÄÜÓĞÒ»¸ö½á¹û¸ø½øÀ´
		return;
	}
	;
}

void chelue_AA(const int count)//¸ß¶È×î¸ß
{
	int i, j, k;
	int max_location = 0;
	int count_tmp = 0;
	if (location_arrary[0] == -1)//µÚÒ»´Î½ø
	{
		max_location = 0;
		for (i = 1; i < count; i++)
		{
			if ((airplane_location_array_tmp[max_location].altitude < airplane_location_array_tmp[i].altitude) && (airplane_location_array_tmp[i].ICAO_address != 0) )
			{
				//¸üĞÂµÄ·É»ú¸ß¶È¸ü¸ß
				max_location = i;
				location_arrary[0] = max_location;
				for (j = 0; j < 1500; j++)
				{
					location_arrary[j] = -1;
				}
				count_tmp = 0;
				location_arrary[count_tmp] = max_location;
				count_tmp = count_tmp + 1;
			}
			else if (airplane_location_array_tmp[max_location].altitude == airplane_location_array_tmp[i].altitude)
			{
				//Á½¸öÒ»Ñù¸ß
				location_arrary[count_tmp] = i;
				count_tmp = count_tmp + 1;

				//printf("chelue_AA error two altitude is same !!! \n");
				;
			}else
			{
				//ĞÂµÄ·É»úÃ»ÓĞÖ®Ç°µÄ¸ß£¬²»¸üĞÂ
				;
			}
			;
		}
		//Ñ­»·Ö®ºó£¬½«½á¹ûĞ´µ½location_arrary[]Êı×éÀïÃæ£¬È»ºó¿ÉÒÔ·µ»ØÁË
		return;
	}
	else if ((location_arrary[0] != -1) && (location_arrary[1] == -1))
	{
		//½øÀ´Ê±ºò½á¹ûÖ»ÓĞÒ»¸ö£¬Ö±½Ó·µ»Ø
		return;
	}
	else
	{
		//ÄÜ½øÀ´ËµÃ÷ÓĞÁ½¸öÒÔÉÏ·É»ú£¬½øÀ´
		count_tmp = 0;
		i = 0;
		while (location_arrary[i] >= 0)
		{
			i++;
			count_tmp++;
		}
		//ÉÏÃæÍ³¼Æ½øÀ´ÁË¼¸¸öÊı¾İ
		//ÏÂÃæÑ­»·½«½øÀ´µÄÎ»ÖÃµ÷Õû³ÉÃ¿¸ö×î´óµÄ¸ß¶ÈµÄ·É»ú
		for (i = 0; i < count_tmp; i++)
		{
			//location_arrary[i]
			for(j = 0; j < 1500; j++)
			{
				if ((airplane_location_array_tmp[location_arrary[i]].ICAO_address ==  airplane_location_array_tmp[j].ICAO_address) && (airplane_location_array_tmp[j].ICAO_address != 0))
				{
					//Á½¸öICAOºÅÒ»Ñù£¬;
					if (airplane_location_array_tmp[location_arrary[i]].altitude <  airplane_location_array_tmp[j].altitude)
					{
						//¸ß¶È¸ü¸ß£¬¸üĞÂ;
						location_arrary[i] = j;
					}
					else
					{
						;
					}
				}
			}
		}
		//¾­¹ıÉÏÃæÑ­»·ºó¾ÍÊÇÃ¿Ò»¸ö×î¸ßµÄÎ»ÖÃĞòºÅÁË,ÃæÑ°ÕÒ×î¸ßµÄ
		max_location = location_arrary[0];
		for (i = 0; i < count_tmp; i++)
		{
			if (airplane_location_array_tmp[max_location].altitude < airplane_location_array_tmp[location_arrary[i]].altitude)
			{
				max_location = location_arrary[i];
			}
		}
		location_arrary[0] = max_location;
		for (k = 1; k < 1500; k++)
		{
			location_arrary[k] = -1;
		}
		return;
	}
	;
}

void chelue_DD(const int count)//Ê±¼ä×îĞÂ
{
	int i, j, k;
	Uint32 time_tmp = 0;
	int max_location = 0;
	int count_tmp = 0;
	if (location_arrary[0] == -1)//µÚÒ»´Î½ø
	{
		for (i = 0; i < count; i++)
		{
			if ((airplane_location_array_tmp[max_location].time < airplane_location_array_tmp[i].time) && (airplane_location_array_tmp[i].ICAO_address != 0))
			{
				max_location = i;
				location_arrary[0] = max_location;
				for (j = 0; j < 1500; j++)
				{
					location_arrary[j] = -1;
				}
				count_tmp = 0;
				location_arrary[count_tmp] = max_location;
				count_tmp = count_tmp + 1;
			}
			else if((airplane_location_array_tmp[max_location].time == airplane_location_array_tmp[i].time) && (airplane_location_array_tmp[i].ICAO_address != 0))
			{//Á½¸öÒ»ÑùÊ±¼ä
				location_arrary[count_tmp] = i;
				count_tmp = count_tmp + 1;
			}
			else
			//ĞÂµÄÃ»ÓĞÖ®Ç°µÄÊ±¼äĞÂ£¬²»¸üĞÂ
			{
				;
			}
		}
		location_arrary[0] = max_location;
		return;
	}
	else if ((location_arrary[0] != -1) && (location_arrary[1] == -1))
	{
		//Ö±½ÓÌø¹ı£¬
		return;
	}
	else
	{
		//ÄÜ½øÀ´²»Ö¹Ò»¸öÇ°ÃæµÄÔ¼ÊøÌõ¼ş;
		i = 0;
		count_tmp = 0;
		while (location_arrary[i] >= 0)
		{
			i = i + 1;
			count_tmp = count_tmp + 1;
		}
		for (i = 0; i < count_tmp; i++)
		{
			for (j = 0; j < 1500; j++)
			{
				if ((airplane_location_array_tmp[location_arrary[i]].ICAO_address ==  airplane_location_array_tmp[j].ICAO_address) && (airplane_location_array_tmp[location_arrary[i]].ICAO_address !=  0))
				{
					//Á½¸öICAOºÅÒ»Ñù£¬;
					if (airplane_location_array_tmp[location_arrary[i]].time <  airplane_location_array_tmp[j].time)
					{
						//¸ß¶È¸ü¸ß£¬¸üĞÂ;
						location_arrary[i] = j;
					}
					else
					{
						;
					}
				}
			}
		}
		//ÉÏÃæ¸üĞÂÁËËùÓĞµÄµÄÎ»ÖÃ¶¼ÊÇ×îĞÂµÄ
		max_location = location_arrary[0];
		for (i = 0; i < count_tmp; i++)
		{
			if (airplane_location_array_tmp[max_location].time < airplane_location_array_tmp[location_arrary[i]].time)
			{
				max_location = location_arrary[i];
			}
		}
		location_arrary[0] = max_location;
		for (k = 1; k < 1500; k++)
		{
			location_arrary[k] = -1;
		}
		return;
	}
}

void lead_tesk()	//´´½¨Òıµ¼µÄÈÎÎñ
{
	
	int i, j, k;
	int count = 0;//¼ÇÂ¼ÓĞÓÃµÄÊı¾İµÄ¸öÊı
	int isfirst = 0;//0±íÊ¾µÚÒ»´Î£¬´óÓÚ0±íÊ¾²»ÊÇµÚÒ»´Î
	int V_time_last = 0;//¼ÇÂ¼ÉÏÒ»´ÎµÄÊ±¼ä£¬ºÍÕâ´Î±ÈÊÇ·ñÓĞ¸üĞÂ
	int P_time_last = 0;//¼ÇÂ¼ÉÏÒ»´ÎµÄÊ±¼ä£¬ºÍÕâ´Î±ÈÊÇ·ñÓĞ¸üĞÂ
	int ICAO_last = 0;//¼ÇÂ¼ÉÏÒ»´ÎµÄICAOºÅ£¬¿´ÊÇ²»ÊÇÏàÍ¬£¬Ã¿´Î¸üĞÂ
	int v_count = -1;//¼ÇÂ¼µÃµ½µÄËÙ¶ÈµÄĞòºÅ
	int max_location = 0;//¼ÇÂ¼³öÏÖ×î¶àµÄICAOºÅµÄÎ»ÖÃ£¬ÊÇ¸öĞòºÅĞÅÏ¢

	//Uint8 pbuf_yindao[36] = {0};//Òıµ¼Êı¾İ¸ñÊ½
	//Òıµ¼Êı¾İ¶¨Òå³ÉÈ«¾ÖµÄ£¬XSHÒ²ÒªÓÃ¡£
	float tmp1;
	float tmp2;
	float tmp3;
/*
	float a;
	int b;
	float c;
*/
	int second_30_count = 0;//¼ÆÊı£¬Ã¿ÃëÒ»´Î£¬Ğ¡ÓÚ30·¢¿Õ°ü£¬´óÓÚ30Ö´ĞĞºóÃæµÄ²ßÂÔ
	int UTC_time_tmp;
	Uint8 tmp_change[4];
//	memcpy(&(UTC_time_tmp),&s_FPGA_YC,4);//½«UTCÊ±¼ä¸øµ½UTC_time_tmp±äÁ¿ÀïÃæ

/*	
	a = 0.09;
	memcpy(&(b),&a,4);
	printf("b = %d\n",b);
	memcpy(&(c),&b,4);
	//c = (float *)(&b);
	printf("c = %f\n",c);
	
*/
//	TaskSleep(30 * 1000);//ÏÈµÈ30s
	int count_tmp_zhang_for_test = 0;
	up_commend_count = 0;
	while (1)
	{
		count_tmp_zhang_for_test++;
		//zhangfulong add Ê±¼ä¸üĞÂ
/*		UTC_time_real[0] = UTC_time_all[13];
		UTC_time_real[1] = UTC_time_all[12];
		UTC_time_real[2] = UTC_time_all[15];
		UTC_time_real[3] = UTC_time_all[14];
		UTC_time_real[4] = UTC_time_all[17];
		UTC_time_real[5] = UTC_time_all[16];*/
		//zhangfulong add Ê±¼ä¸üĞÂ
		//µ÷ÕûË³Ğò£¬Êı¾İ´¦Àí²¿·Ö·ÅÔÚleadÈÎÎñforÑ­»·¿ªÍ·×´Ì¬
		for (i = 0; i < 200; i = i + 2)
		{
			ICAOArray_real[i] = ICAOArray[i + 1 + 2];
			ICAOArray_real[i + 1] = ICAOArray[i + 2];
		}
		
		if ((ICAOArray_real[12] == 0xbb) && (ICAOArray_real[13] == 0x77) && (ICAOArray_real[14] == 0x3D) )
		{
			//°üÍ·ÕıÈ· £¬ÏÂÃæ½øĞĞĞ£ÑéºÍ¼ÆËã ´Ó 12 µÄ00 04 ¿ªÊ¼  Uint16 check_sum_1,check_sum_2;
			check_sum_1 = 0;
			check_sum_2 = 0;
			for (i = 12; i < 151 - 2; i = i +2 )
			{
				check_sum_1 = (check_sum_1 ^ ( (Uint16)(ICAOArray_real[i] << 8)+(Uint16)(ICAOArray_real[i]) ) );
			}
			check_sum_2 = ( (Uint16)(ICAOArray_real[152 - 2] << 8)+(Uint16)(ICAOArray_real[153 - 2]) );
//			if (check_sum_1 == check_sum_2)		//Ğ£ÑéÏÈÈ¥µô£¬ÒÔºó¼ÓÉÏ
			{
				//ÄÜ½øÀ´ËµÃ÷Ğ£ÑéºÍÕıÈ·£¬ÏÂÃæ¿ªÊ¼½«Öµ¸üĞÂµ½ÎÒÃÇµÄ½á¹¹ÌåÀïÃæ
				if (ICAOArray_real[15] == 0x33)//×ÔÓÉËÑË÷
				{
					yaokongcmd_all_data.mode = 1;
				}
				else if (ICAOArray_real[15] == 0xCC)//¸øICAOºÅµÄÄ£Ê½
				{
					yaokongcmd_all_data.mode = 2;
				}
				else if (ICAOArray_real[15] == 0xFF)//0xFF Çå¿ÕÎ»ÖÃ·É»úĞÅÏ¢
				{
					for (i = 0; i < 1500; i++)
					{
						airplane_location_static[i].altitude = 0;;
						airplane_location_static[i].coordinate[0] = 0;
						airplane_location_static[i].coordinate[1] = 0;
						airplane_location_static[i].ICAO_address = 0;
						airplane_location_static[i].time = 0;
					}
					yaokongcmd_all_data.mode = 0;
					yaokongcmd_all_data.jingdu = 0;
					yaokongcmd_all_data.weidu = 0;
					yaokongcmd_all_data.gaodu = 0;
					yaokongcmd_all_data.chelue_1 = 0;
					yaokongcmd_all_data.chelue_2 = 0;
					yaokongcmd_all_data.chelue_3 = 0;
					for (i = 0; i < 200; i++)
					{
						ICAOArray_real[i] = 0;
						ICAOArray[i] = 0;
					}
					goto outclear;
				}
				else
				{
					yaokongcmd_all_data.mode = 0;
				}
				tmp_change[3] = ICAOArray_real[16];
				tmp_change[2] = ICAOArray_real[17];
				tmp_change[1] = ICAOArray_real[18];
				tmp_change[0] = ICAOArray_real[19];
				memcpy(&(yaokongcmd_all_data.jingdu),&(tmp_change[0]),4);//¾­¶È
				//yaokongcmd_all_data.jingdu = 90.0;//test ÁÙÊ±
				yaokongcmd_all_data.jingdu = yaokongcmd_all_data.jingdu / 3.1415926 *180;//test ÁÙÊ±
				yaokongcmd_all_data.jingdu = 90.0;
				//memcpy(&(yaokongcmd_all_data.jingdu),&(ICAOArray_real[16]),4);//¾­¶È
				tmp_change[3] = ICAOArray_real[20];
				tmp_change[2] = ICAOArray_real[21];
				tmp_change[1] = ICAOArray_real[22];
				tmp_change[0] = ICAOArray_real[23];
				memcpy(&(yaokongcmd_all_data.weidu),&(tmp_change[0]),4);//Î³¶È
				//yaokongcmd_all_data.weidu = 90.0;//test ÁÙÊ±
				yaokongcmd_all_data.weidu = yaokongcmd_all_data.weidu /3.1415926 *180;							//20201112
				yaokongcmd_all_data.weidu = 90.0;
				//memcpy(&(yaokongcmd_all_data.weidu),&(ICAOArray_real[20]),4);//Î³¶È
				tmp_change[3] = ICAOArray_real[24];
				tmp_change[2] = ICAOArray_real[25];
				tmp_change[1] = ICAOArray_real[26];
				tmp_change[0] = ICAOArray_real[27];
				memcpy(&(yaokongcmd_all_data.gaodu),&(tmp_change[0]),4);//¸ß¶È
				//memcpy(&(yaokongcmd_all_data.gaodu),&(ICAOArray_real[24]),4);//¸ß¶È
				yaokongcmd_all_data.banzhuijiao = ICAOArray_real[28];
				//yaokongcmd_all_data.banzhuijiao = 90;
				//yaokongcmd_all_data.chelue_1 = ICAOArray_real[29];
				//yaokongcmd_all_data.chelue_2 = ICAOArray_real[30];
				//yaokongcmd_all_data.chelue_3 = ICAOArray_real[31];
				if (ICAOArray_real[29] == 0x00)
				{
					//¸ß¶È ÆµÂÊ
					yaokongcmd_all_data.chelue_1 = 0xAA;
					yaokongcmd_all_data.chelue_2 = 0xFF;
					yaokongcmd_all_data.chelue_3 = 0x00;
				}
				else if (ICAOArray_real[29] == 0x02)
				{
					//ÆµÂÊ ¸ß¶È 
					yaokongcmd_all_data.chelue_1 = 0xFF;
					yaokongcmd_all_data.chelue_2 = 0xAA;
					yaokongcmd_all_data.chelue_3 = 0x00;
				}
				else
				{
					yaokongcmd_all_data.chelue_1 = 0x00;
					yaokongcmd_all_data.chelue_2 = 0x00;
					yaokongcmd_all_data.chelue_3 = 0x00;
				}
				yaokongcmd_all_data.chelue_new = ICAOArray_real[29];

				for (k = 0; k < 30; k++)
				{
					//memcpy(&(yaokongcmd_all_data.ICAO[k]),&(ICAOArray[36 + k*4]),4);;
					tmp_change[3] = ICAOArray_real[32 + k*4 - 2];
					tmp_change[2] = ICAOArray_real[32 + k*4 + 1 - 2];
					tmp_change[1] = ICAOArray_real[32 + k*4 + 2 - 2];
					tmp_change[0] = ICAOArray_real[32 + k*4 + 3 - 2];
					memcpy(&(yaokongcmd_all_data.ICAO[k]),&(tmp_change[0]),4);;
				}
				
				isfirst = 0;//ĞÂµÄ²ßÂÔ£¬´ÓµÚÒ»´Î¿ªÊ¼
				for (i = 0; i < 200; i++)
				{
					ICAOArray_real[i] = 0;
					ICAOArray[i] = 0;
				}
				up_commend_count = up_commend_count + 1;
			}			
		}
outclear:
#if 0		//È¥µôÇ°30sµÄµÈ´ı×´Ì¬£¬±ä³ÉÓĞÒ£¿ØÊı¾İºó¿ªÊ¼Ö´ĞĞ²ßÂÔ
		//Ç°30sµÄ×´Ì¬
		if(second_30_count < 30)
		{
			for (k = 0; k < 34; k++)//³õÊ¼»¯
			{
				pbuf_yindao[k] = 0x00;
			}
			pbuf_yindao[0] = 0x6A;//°üÍ·
			//UTCÊ±¼ä
			memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//½«Ê±¼ä¸´ÖÆ¹ıÈ¥ 4->6

			pbuf_yindao[8 + 2] = 0xF0;//µÚÒ»°üÊÇF0  0-30×´Ì¬
			//Ğ£ÑéºÍ
			for( k = 0; k < 36; k++)
			{
				pbuf_yindao[33+ 2] = (pbuf_yindao[33+ 2] + pbuf_yindao[k]) & 0xFF;
			}
			
			TaskSleep(1);
			EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
			pEDMA3CC_PaRAM->OPT = 0x00130104;//16
			pEDMA3CC_PaRAM->SRC =  (unsigned char *)(&pbuf_yindao[0]);  //ce4
			pEDMA3CC_PaRAM->BCNT_ACNT =0x00240001;	//0x22 = 34
			pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
		    pEDMA3CC_PaRAM->DST = (unsigned char *)DSP_TO_FPGA_YINDAO;
			EDMA3CC_ESRH = 0x00010000;;
			second_30_count = second_30_count + 1;
			TaskSleep(1);
			continue;
		}
		//Ç°30sµÄ×´Ì¬
#endif
		//memcpy(&(UTC_time_tmp),&s_FPGA_YC,4);
		
		for(i = 0; i < position_BUFLEN;i++)
		{
			airplane_location_array_tmp[i].altitude = 0;
			airplane_location_array_tmp[i].coordinate[1] = 0;
			airplane_location_array_tmp[i].coordinate[0] = 0;
			airplane_location_array_tmp[i].ICAO_address = 0;
			airplane_location_array_tmp[i].time = 0;
		}

		if (yaokongcmd_all_data.mode == 1)//×ÔÓÉÄ¿±êËÑË÷
		{
			count = 0;//¼ÇÂ¼ÊÇ·ñÔÚËÑË÷ÇøÓòÄÚµÄÊıÁ¿
			if(isfirst == 0)
			{
				count = 0;//¼ÇÂ¼ÊÇ·ñÔÚËÑË÷ÇøÓòÄÚµÄÊıÁ¿
				for (i = 0; i < position_BUFLEN; i++)
				{
					//tmp1 = fabs(airplane_location_static[i].coordinate[1] - yaokongcmd_all_data.jingdu);//Î³¶È²î
					//tmp2 = fabs(airplane_location_static[i].coordinate[0] - yaokongcmd_all_data.weidu);//¾­¶È²î
					//memcpy(&(tmp1),&(airplane_location_static[i].coordinate[1]),4);
					//memcpy(&(tmp2),&(airplane_location_static[i].coordinate[0]),4);
					//tmp1 = fabs(tmp1 - yaokongcmd_all_data.jingdu);//Î³¶È²î
					//tmp2 = fabs(tmp2 - yaokongcmd_all_data.weidu);//¾­¶È²î

					tmp1 = fabs(airplane_location_static[i].coordinate[0]/10000000 - yaokongcmd_all_data.jingdu);//Î³¶È²î
					tmp2 = fabs(airplane_location_static[i].coordinate[1]/10000000 - yaokongcmd_all_data.weidu);//¾­¶È²î
					if (airplane_location_static[i].ICAO_address != 0)
					{
						//printf("location ICAO is %d,i = %d ~ \n",airplane_location_static[i].ICAO_address, i);
					}		
					
					if ( (tmp1 <= yaokongcmd_all_data.banzhuijiao) && (tmp2 <= yaokongcmd_all_data.banzhuijiao) && (airplane_location_static[i].ICAO_address != 0))
					{
						//Âú×ãÔÚÊÓ³¡ÄÚ,ÏàÓ¦µÄÖµĞ´½øÈ¥
						airplane_location_array_tmp[count].altitude = airplane_location_static[i].altitude;
						airplane_location_array_tmp[count].coordinate[1] = airplane_location_static[i].coordinate[1];
						airplane_location_array_tmp[count].coordinate[0] = airplane_location_static[i].coordinate[0];
						airplane_location_array_tmp[count].ICAO_address = airplane_location_static[i].ICAO_address;
						airplane_location_array_tmp[count].time = airplane_location_static[i].time;
						airplane_location_array_tmp[count].number = 0;//½«¼ÆÊıÇå¿ÕÎª0
						count = count + 1;
						;
					}
					if ((airplane_location_static[i].ICAO_address == 0x123456) || (airplane_location_static[i].ICAO_address == 123456)
						|| (airplane_location_static[i].ICAO_address == 0x111111) || (airplane_location_static[i].ICAO_address == 111111)
						|| (airplane_location_static[i].ICAO_address == 0x222222)  || (airplane_location_static[i].ICAO_address == 222222) )
					{
						//printf("find 0x123456 at %d in airplane_location_static; \n",i);
					}		
				}
#if 0	//¸ü¸Ä¿ÉÒÔ±ä»¯Ë³ĞòµÄ²ßÂÔ×é£¬ËùÒÔ×ßºóÃæµÄelse·ÖÖ§
				//ÉÏÃæÑ­»·ÒÑ¾­½«ËùÓĞµÄÔÚÊÓ³¡ÀïÃæĞÅÏ¢Ğ´Èë
				//ÏÂÂú½«Í³¼Æ³öÏÖµÄ¸öÊıĞÅÏ¢
				//for (i = 0; i < count; i++)//Çå¿Õ¼ÆÊı
				//{
				//	airplane_location_array_tmp[i].number = 0;
				//}
				for (i = 0; i < count; i++)
				{
					for (j = i; j < count ; j++)
					{
						if(airplane_location_array_tmp[i].ICAO_address == airplane_location_array_tmp[j].ICAO_address)
						{
							//µÚÒ»´Î¿Ï¶¨»á½øÀ´
							airplane_location_array_tmp[i].number = airplane_location_array_tmp[i].number + 1;
						}
					}
				}
				//ÉÏÃæÑ­»·½«Ã¿Ò»¸öICAOºÅ³öÏÖµÄ´ÎÊıĞ´Èëairplane_location_array_tmp[count].numberÀïÃæ
				//ÏÂÃæÕÒ³ö³öÏÖÆµÂÊ×î¸ßµÄ
				max_location = 0;
				for (i = 1; i < count; i++)
				{
					if ((airplane_location_array_tmp[i].number > airplane_location_array_tmp[max_location].number)&& (airplane_location_array_tmp[i].ICAO_address != 0))
					{
						//ÕâÒ»ÌõµÄ³öÏÖ´ÎÊı×î¶à£¬¸üĞÂmax_locationÎ»ÖÃ
						max_location = i;
					}
					else if (airplane_location_array_tmp[i].number == airplane_location_array_tmp[max_location].number)
					{
						//Á½¸öÏàµÈ£¬½øĞĞ¸ß¶ÈÅĞ¶Ï
						if (airplane_location_array_tmp[i].altitude >= airplane_location_array_tmp[max_location].altitude)
						{
							//ĞÂµÄ¸ß¶È¸ü¸ß£¬½øĞĞ¸üĞÂ
							max_location = i;
						}
						else
						{
							;
						}
						;
					}
					else//Õâ¸ö³öÏÖµÄ´ÎÊıÃ»ÓĞÄ¿Ç°×î´óÖµ¶à£¬²»¸üĞÂ
					{
						;
					}
				}
#else
				//½âËãË³ĞòÊÇÎ¨Ò»µÄ²ßÂÔ
				//
				for(i= 0; i < 1500; i++)
				{
					location_arrary[i] = -1;//½øĞĞ³õÊ¼»¯-1ÅäÖÃ
				}
if (1){
				switch (yaokongcmd_all_data.chelue_1)
				{
					case (0xFF):
						chelue_FF(count);
						break;
					case (0xAA):
						chelue_AA(count);
						break;
					case (0xDD):
						chelue_DD(count);
						break;
					default:
						printf("chelue 1 wrong \n");
				}
				switch (yaokongcmd_all_data.chelue_2)
				{
					case (0xFF):
						chelue_FF(count);
						break;
					case (0xAA):
						chelue_AA(count);
						break;
					case (0xDD):
						chelue_DD(count);
						break;
					default:
						printf("chelue 2 wrong \n");
				}
				switch (yaokongcmd_all_data.chelue_3)
				{
					case (0xFF):
						chelue_FF(count);
						break;
					case (0xAA):
						chelue_AA(count);
						break;
					case (0xDD):
						chelue_DD(count);
						break;
					default:
						printf("chelue 3 wrong \n");
				}
}
				
				if (location_arrary[0] >= 0)
				{
					max_location = location_arrary[0];
				}
				else
				{
					max_location = -1;
					count = 0;
				}
				;
#endif			
				//printf("first max location is %din airplane_location_array_tmp[],ICAO:%d\n",max_location,airplane_location_array_tmp[max_location].ICAO_address);
				//ÉÏÃæÑ­»·½«»ñµÃÒ»¸ö³öÏÖ´ÎÊı×î¶àµÄ£¬´æÔÚmax_locationÀïÃæ
				//½ÓÏÂÀ´Ñ°ÕÒÖ¸¶¨ICAOºÅµÄ×îĞÂÎ»ÖÃĞÅÏ¢
if (count > 0)//Ã»ÕÒµ½£¬²»·¢ÁË
{
				if (count > 0)//Èç¹û count ´óÓÚ0 £¬ÔòÖÁÉÙÓĞÒ»¸ö·É»ú
				{
					max_location = P_get(airplane_location_array_tmp[max_location].ICAO_address);//´ËÊ±max_locationÊÇÔÚairplane_location_staticÊı×éÀïÃæ
					airplane_location_tmp.ICAO_address = airplane_location_static[max_location].ICAO_address;
					airplane_location_tmp.altitude = airplane_location_static[max_location].altitude;
					airplane_location_tmp.coordinate[0] = airplane_location_static[max_location].coordinate[0];
					airplane_location_tmp.coordinate[1] = airplane_location_static[max_location].coordinate[1];
					airplane_location_tmp.time = airplane_location_static[max_location].time;
				}
				else//ËµÃ÷Ã»ÓĞÃüÖĞ
				{
					airplane_location_tmp.ICAO_address = 0;
					airplane_location_tmp.altitude = 0;
					airplane_location_tmp.coordinate[0] = 0;
					airplane_location_tmp.coordinate[1] = 0;
					airplane_location_tmp.time = 0;
				}
				//Éú³É°ü
				for (k = 0; k < 36; k++)//³õÊ¼»¯
				{
					pbuf_yindao[k] = 0x00;
				}
				pbuf_yindao[0] = 0x6A;//°üÍ·
				pbuf_yindao[1] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x00FF0000)>>16);//ICAOºÅ
				pbuf_yindao[2] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x0000FF00)>>8);
				pbuf_yindao[3] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x000000FF));
				//UTCÊ±¼ä
				//zhangfulong add Ê±¼ä¸üĞÂ
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add Ê±¼ä¸üĞÂ
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//½«Ê±¼ä¸´ÖÆ¹ıÈ¥ 4->6
				pbuf_yindao[4] = UTC_time_real[0];
				pbuf_yindao[5] = UTC_time_real[1];
				pbuf_yindao[6] = UTC_time_real[2];
				pbuf_yindao[7] = UTC_time_real[3];
				pbuf_yindao[8] = UTC_time_real[4];
				pbuf_yindao[9] = UTC_time_real[5];
				//pbuf_yindao[9] = count_tmp_zhang_for_test & 0x000000FF;//ÁÙÊ±ÒªÈ¥µô
				//Ê¹ÓÃ½â°üÊ±¼ä
				pbuf_yindao[6] = (airplane_location_tmp.time & 0xFF000000)>>24;
				pbuf_yindao[7] = (airplane_location_tmp.time & 0x00FF0000)>>16;
				pbuf_yindao[8] = (airplane_location_tmp.time & 0x0000FF00)>>8;
				pbuf_yindao[9] = (airplane_location_tmp.time & 0x000000FF);
				//Ê¹ÓÃ½â°üÊ±¼ä
				//Ê±¼ä×îºóÅªÉÏ
				v_count = V_get(airplane_location_tmp.ICAO_address);
				//¸üĞÂ×´Ì¬

				pbuf_yindao[8 + 2] = 0xFF;//µÚÒ»°üÊÇFF
				//memcpy(&(pbuf_yindao[9+ 2]),&(airplane_location_tmp.coordinate[1]),4);//¾­¶È
				//memcpy(&(pbuf_yindao[13+ 2]),&(airplane_location_tmp.coordinate[0]),4);//Î³¶È
				pbuf_yindao[9+ 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0xFF000000) >> 24);
				pbuf_yindao[9+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x00FF0000) >> 16);
				pbuf_yindao[9+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x0000FF00) >> 8);
				pbuf_yindao[9+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x000000FF));

				pbuf_yindao[13+ 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0xFF000000) >> 24);
				pbuf_yindao[13+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x00FF0000) >> 16);
				pbuf_yindao[13+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x0000FF00) >> 8);
				pbuf_yindao[13+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x000000FF));

				if(v_count < 0)//Ã»ÓĞÆ¥ÅäËÙ¶È
				{
					pbuf_yindao[17+ 2] = 0x00;
					pbuf_yindao[28+ 2] = 0x00;//ÀïÃæÒÑ¾­ÊÇ0ÁË
					V_time_last = 0;
					airplane_speed_tmp.E_W_velocity = 0;
					airplane_speed_tmp.ICAO_address = 0;
					airplane_speed_tmp.N_S_velocity = 0;
					airplane_speed_tmp.time = 0;
					airplane_speed_tmp.VERT_velocity = 0;
				}
				else//ÓĞÆ¥ÅäËÙ¶È
				{
					//memcpy(&(pbuf_yindao[17+ 2]),&(airplane_velocity_three_static[v_count].E_W_velocity),4);
				//	memcpy(&(pbuf_yindao[21+ 2]),&(airplane_velocity_three_static[v_count].N_S_velocity),4);
					//memcpy(&(pbuf_yindao[25+ 2]),&(airplane_velocity_three_static[v_count].VERT_velocity),4);
					
					pbuf_yindao[17+ 2] = (Uint8)((airplane_velocity_three_static[v_count].E_W_velocity & 0xFF000000)>>24);
					pbuf_yindao[17+ 2 + 1] = (Uint8)((airplane_velocity_three_static[v_count].E_W_velocity & 0x00FF0000)>>16);
					pbuf_yindao[17+ 2 + 2] = (Uint8)((airplane_velocity_three_static[v_count].E_W_velocity & 0x0000FF00)>>8);
					pbuf_yindao[17+ 2 + 3] = (Uint8)((airplane_velocity_three_static[v_count].E_W_velocity & 0x000000FF));

					pbuf_yindao[21+ 2] = (Uint8)((airplane_velocity_three_static[v_count].N_S_velocity & 0xFF000000)>>24);
					pbuf_yindao[21+ 2 + 1] = (Uint8)((airplane_velocity_three_static[v_count].N_S_velocity & 0x00FF0000)>>16);
					pbuf_yindao[21+ 2 + 2] = (Uint8)((airplane_velocity_three_static[v_count].N_S_velocity & 0x0000FF00)>>8);
					pbuf_yindao[21+ 2 + 3] = (Uint8)((airplane_velocity_three_static[v_count].N_S_velocity & 0x000000FF));

					pbuf_yindao[25+ 2] = (Uint8)((airplane_velocity_three_static[v_count].VERT_velocity & 0xFF000000)>>24);
					pbuf_yindao[25+ 2 + 1] = (Uint8)((airplane_velocity_three_static[v_count].VERT_velocity & 0x00FF0000)>>16);
					pbuf_yindao[25+ 2 + 2] = (Uint8)((airplane_velocity_three_static[v_count].VERT_velocity & 0x0000FF00)>>8);
					pbuf_yindao[25+ 2 + 3] = (Uint8)((airplane_velocity_three_static[v_count].VERT_velocity & 0x000000FF));


					memcpy(&V_time_last,&(airplane_velocity_three_static[v_count].time), 4);

					airplane_speed_tmp.E_W_velocity = airplane_velocity_three_static[v_count].E_W_velocity;
					airplane_speed_tmp.ICAO_address = airplane_velocity_three_static[v_count].ICAO_address;
					airplane_speed_tmp.N_S_velocity = airplane_velocity_three_static[v_count].N_S_velocity;
					airplane_speed_tmp.time = airplane_velocity_three_static[v_count].time;
					airplane_speed_tmp.VERT_velocity = airplane_velocity_three_static[v_count].VERT_velocity;
				}
				//¸ß¶È
				//memcpy(&(pbuf_yindao[29+ 2]),&(airplane_location_tmp.altitude),4);
				pbuf_yindao[29+ 2] = (Uint8)((airplane_location_tmp.altitude & 0xFF000000)>>24);
				pbuf_yindao[29+ 2 + 1] = (Uint8)((airplane_location_tmp.altitude & 0x00FF0000)>>16);
				pbuf_yindao[29+ 2 + 2] = (Uint8)((airplane_location_tmp.altitude & 0x0000FF00)>>8);
				pbuf_yindao[29+ 2 + 3] = (Uint8)((airplane_location_tmp.altitude & 0x000000FF));
				//
				//¼ÇÂ¼ÉÏÒ»´ÎµÄÊ±¼ä´ÁICAO
				memcpy(&P_time_last,&(airplane_location_tmp.time), 4);

				//Ğ£ÑéºÍ
				pbuf_yindao[33+ 2] = 0;
				for( k = 0; k < 35; k++)
				{
					pbuf_yindao[33+ 2] = (pbuf_yindao[33+ 2] + pbuf_yindao[k]) & 0xFF;
				}
				//½«ICAOºÅ±£´æºÍÏÂÒ»´ÎÏà±È
				ICAO_last = airplane_location_tmp.ICAO_address;
				;
				if (airplane_location_tmp.ICAO_address != 0)
				{
					isfirst = isfirst + 1;//±êÖ¾µÚÒ»´Î½áÊø£¬½ÓÏÂÀ´½øÈëºóÃæµÄ
				}
				else
				{
					;
				}
				if (pbuf_yindao[0] == 0)
				{
					printf("yindao bao error!!!\n");
				}
				//for (i = 0; i < 30; i++)
				//{
				//	pbuf_yindao[i] = 0xFF;
				//}
				TaskSleep(1);
				EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
				pEDMA3CC_PaRAM->OPT = 0x00130104;//16
				pEDMA3CC_PaRAM->SRC =  (unsigned char *)(&pbuf_yindao[0]);  //ce4
				//	pEDMA3CC_PaRAM->SRC =  (unsigned char *)XSH_Array2;  //ce4	
				//	pEDMA3CC_PaRAM->BCNT_ACNT =0x00660001; //1280*4	//0x003d0001D¡ä¦Ì?FPGA??¦Ì??¡¤A0F	//51
				pEDMA3CC_PaRAM->BCNT_ACNT =0x00240001;	//0x22 = 34
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
			    pEDMA3CC_PaRAM->DST = (unsigned char *)DSP_TO_FPGA_YINDAO;
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
}//Ã»ÕÒµ½²»·¢À¨ºÅ
				//µÈ´ı15s
				//task_sleep(15);
			}
			else//ÉÏÃæµÄÊÇÚÒ»´Î½øÈë×ÔÓÉËÑË÷²¢·¢³ö°ü£¬½ÓÏÂÀ´»áÌí¼?5sÒÔÄÚµÄÏŞÖÆÌõ¼ş²¢¼ÌĞø
			{
				count = 0;//¼ÇÂ¼ÊÇ·ñÔÚËÑË÷ÇøÓòÄÚµÄÊıÁ¿
//if (ICAO_last == 0)
{
				for (i = 0; i < position_BUFLEN; i++)
				{
					//tmp1 = fabs(airplane_location_static[i].coordinate[1] - yaokongcmd_all_data.jingdu);//Î³¶È²î
					//tmp2 = fabs(airplane_location_static[i].coordinate[0] - yaokongcmd_all_data.weidu);//¾­¶È²î
					tmp3 = UTC_time_tmp - airplane_location_static[i].time;
					
					//memcpy(&(tmp1),&(airplane_location_static[i].coordinate[1]),4);
					//memcpy(&(tmp2),&(airplane_location_static[i].coordinate[0]),4);
					//tmp1 = fabs(tmp1 - yaokongcmd_all_data.jingdu);//Î³¶È²î
					//tmp2 = fabs(tmp2 - yaokongcmd_all_data.weidu);//¾­¶È²î

					tmp1 = fabs(airplane_location_static[i].coordinate[0]/10000000 - yaokongcmd_all_data.jingdu);//¾­¶È²î
					tmp2 = fabs(airplane_location_static[i].coordinate[1]/10000000 - yaokongcmd_all_data.weidu);//Î³¶È²î
					
					
				
					//if ( (tmp1 <= yaokongcmd_all_data.banzhuijiao) & (tmp2 <= yaokongcmd_all_data.banzhuijiao) & (tmp3 <= 15) )
					//ÔİÊ±È¥µôÊ±¼äÏŞÖÆ£¬²âÊÔÊ±ºò£¬ÒÔºóÔÚ¼ÓÉÏ
					if ( (tmp1 <= yaokongcmd_all_data.banzhuijiao) & (tmp2 <= yaokongcmd_all_data.banzhuijiao)&& (airplane_location_static[i].ICAO_address != 0))
					{
						//Âú×ãÔÚÊÓ³¡ÄÚ,ÏàÓ¦µÄÖµĞ´½øÈ¥
						airplane_location_array_tmp[count].altitude = airplane_location_static[i].altitude;
						airplane_location_array_tmp[count].coordinate[1] = airplane_location_static[i].coordinate[1];
						airplane_location_array_tmp[count].coordinate[0] = airplane_location_static[i].coordinate[0];
						airplane_location_array_tmp[count].ICAO_address = airplane_location_static[i].ICAO_address;
						airplane_location_array_tmp[count].time = airplane_location_static[i].time;
						airplane_location_array_tmp[count].number = 0;//½«¼ÆÊıÇå¿ÕÎª0
						count = count + 1;
						;
					}
				//	if ((airplane_location_static[i].ICAO_address & 0x00FFFFFF) == 0x123456)
				//	{
				//		printf("No.%d.altitude = %d;\n",i,airplane_location_static[i].altitude);
				//	}		
				}
				//ÉÏÃæÑ­»·ÒÑ¾­½«ËùÓĞµÄÔÚÊÓ³¡ÀïÃæĞÅÏ¢Ğ´Èë
				//ÏÂÂú½«Í³¼Æ³öÏÖµÄ¸öÊıĞÅÏ¢
				//for (i = 0; i < count; i++)//Çå¿Õ¼ÆÊı
				//{
				//	airplane_location_array_tmp[i].number = 0;
				//}
#if 0		//È¥µôÕâ¸ö²ßÂÔ£¬¸ú×ÙÒ»¸ö·É»ú
				for (i = 0; i < count; i++)
				{
					for (j = i; j < count ; j++)
					{
						if(airplane_location_array_tmp[i].ICAO_address == airplane_location_array_tmp[j].ICAO_address)
						{
							//µÚÒ»´Î¿Ï¶¨»á½øÀ´
							airplane_location_array_tmp[i].number = airplane_location_array_tmp[i].number + 1;
							/*if (airplane_location_array_tmp[j].altitude != 0)
							{
								printf("airplane_location_array_tmp[%d].altitude = %d\n",j,airplane_location_array_tmp[j].altitude);
							}*/
							
						}
					}
				}
				//ÉÏÃæÑ­»·½«Ã¿Ò»¸öICAOºÅ³öÏÖµÄ´ÎÊıĞ´Èëairplane_location_array_tmp[count].numberÀïÃæ
				//ÏÂÃæÕÒ³ö³öÏÖÆµÂÊ×î¸ßµÄ
				max_location = 0;
				for (i = 1; i < count; i++)
				{
					if ((airplane_location_array_tmp[i].number > airplane_location_array_tmp[max_location].number) && (airplane_location_array_tmp[i].ICAO_address != 0) )
					{
						//ÕâÒ»ÌõµÄ³öÏÖ´ÎÊı×î¶à£¬¸üĞÂmax_locationÎ»ÖÃ
						max_location = i;
					}
					else if (airplane_location_array_tmp[i].number == airplane_location_array_tmp[max_location].number)
					{
						//Á½¸öÏàµÈ£¬½øĞĞ¸ß¶ÈÅĞ¶Ï
						if (airplane_location_array_tmp[i].altitude >= airplane_location_array_tmp[max_location].altitude)
						{
							//ĞÂµÄ¸ß¶È¸ü¸ß£¬½øĞĞ¸üĞÂ
							max_location = i;
						}
						else
						{
							;
						}
						;
					}
					else//Õâ¸ö³öÏÖµÄ´ÎÊıÃ»ÓĞÄ¿Ç°×î´óÖµ¶à£¬²»¸üĞÂ
					{
						;
					}
				}
				//ÉÏÃæÑ­»·½«»ñµÃÒ»¸ö³öÏÖ´ÎÊı×î¶àµÄ£¬´æÔÚmax_locationÀïÃæ				
				//½ÓÏÂÀ´Ñ°ÕÒÖ¸¶¨ICAOºÅµÄ×îĞÂÎ»ÖÃĞÅÏ¢
				if (count > 0)//Èç¹û count ´óÓÚ0 £¬ÔòÖÁÉÙÓĞÒ»¸ö·É»ú
				{
					max_location = P_get(airplane_location_array_tmp[max_location].ICAO_address);//´ËÊ±max_locationÊÇÔÚairplane_location_staticÊı×éÀïÃæ
					airplane_location_tmp.ICAO_address = airplane_location_static[max_location].ICAO_address;
					airplane_location_tmp.altitude = airplane_location_static[max_location].altitude;
					airplane_location_tmp.coordinate[0] = airplane_location_static[max_location].coordinate[0];
					airplane_location_tmp.coordinate[1] = airplane_location_static[max_location].coordinate[1];
					airplane_location_tmp.time = airplane_location_static[max_location].time;
				}
				else//ËµÃ÷Ã»ÓĞÃüÖĞ
				{
					airplane_location_tmp.ICAO_address = 0;
					airplane_location_tmp.altitude = 0;
					airplane_location_tmp.coordinate[0] = 0;
					airplane_location_tmp.coordinate[1] = 0;
					airplane_location_tmp.time = 0;
				}
#endif			//È¥µôÕâ¸ö²ßÂÔ£¬¸ú×ÙÒ»¸ö·É»ú
}
//else//ÉÏÃæ·ÖÖ§ÊÇµÚÒ»´ÎÃ»ÕÒµ½£¬ÔÚ15sÀïÃæ¼ÌĞøÕÒÏÂÃæÊÇÕÒµ½ÁËICAO£¬ËùÒÔ°´ÕÕICAO²éÕÒ£¬
{
				for (i = 0; i < position_BUFLEN; i++)//±éÀúÊÕµ½Ä1500¸öÊı×é
				{
					if(airplane_location_static[i].ICAO_address == airplane_location_tmp.ICAO_address)
					{
						//ICAOºÅÃüÖĞ,ÏÂÃæ¿´ÊÇ·ñÊÇÊ±¼ä×îĞÂµÄ£¬
						if(airplane_location_static[i].time > airplane_location_tmp.time)
						{
							memcpy(&airplane_location_tmp,&(airplane_location_static[i]), 20);
							count = count + 1;//count±íÊ¾ËÙ¶ÈÓĞ¸üĞÂ
						}
						
					}
				//	if (airplane_location_static[i].ICAO_address == 0x00111111)
				//	{
				//		printf("No.%d.altitude = %d;\n",i,airplane_location_static[i].altitude);
				//	}
				}
}//´óµÄif elseÑ­»·½áÊø
				//Éú³É°ü
				for (k = 0; k < 36; k++)//³õÊ¼»¯
				{
					pbuf_yindao[k] = 0x00;
				}
				pbuf_yindao[0] = 0x6A;//°üÍ·
				pbuf_yindao[1] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x00FF0000)>>16);//ICAOºÅ
				pbuf_yindao[2] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x0000FF00)>>8);
				pbuf_yindao[3] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x000000FF));
				//UTCÊ±¼ä
				//zhangfulong add Ê±¼ä¸üĞÂ
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add Ê±¼ä¸üĞÂ
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),4+ 2);//½«Ê±¼ä¸´ÖÆ¹ıÈ¥
				pbuf_yindao[4] = UTC_time_real[0];
				pbuf_yindao[5] = UTC_time_real[1];
				pbuf_yindao[6] = UTC_time_real[2];
				pbuf_yindao[7] = UTC_time_real[3];
				pbuf_yindao[8] = UTC_time_real[4];
				pbuf_yindao[9] = UTC_time_real[5];
				//pbuf_yindao[9] = count_tmp_zhang_for_test & 0x000000FF;//ÁÙÊ±ÒªÈ¥µô
				//Ê¹ÓÃ½â°üÊ±¼ä
				pbuf_yindao[6] = (airplane_location_tmp.time & 0xFF000000)>>24;
				pbuf_yindao[7] = (airplane_location_tmp.time & 0x00FF0000)>>16;
				pbuf_yindao[8] = (airplane_location_tmp.time & 0x0000FF00)>>8;
				pbuf_yindao[9] = (airplane_location_tmp.time & 0x000000FF);
				//Ê¹ÓÃ½â°üÊ±¼ä
				//v_count = V_get(airplane_location_tmp.ICAO_address);
				//¸üĞÂ×´Ì¬ ÒÑ¾­²»ÊÇµÚÒ»°üÁË£¬ËùÒÔÏÂÃæµÄÅĞ¶ÏÓĞĞ§ĞÔ
#if 1
				if(airplane_location_tmp.ICAO_address == ICAO_last)//ICAOÏàÍ¬
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] | 0xF0;
				}
				else//ICAO²»Í¬
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0x0F;//üĞÂ³?x0X ÓëÉÏÒ»°ü²»Í¬
				}
				
				if ((v_count == -1) && (P_time_last==airplane_location_tmp.time))
				//ËÙ¶ÈÃ»ÕÒµ½£¬Î»ÖÃÊ±¼äÏàÍ¬
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
				}
				else//ÕÒµ½ÁËËÙ¶È,
				{
					if((V_time_last == airplane_velocity_three_static[v_count].time) && (P_time_last==airplane_location_tmp.time))
					{
						//ÕÒµ½ËÙ¶È ËÙ¶ÈÎŞ¸üĞÂ,Î»ÖÃÒ²Ã»¸üĞÂ
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
					}
					else
					{
						//ËÙ¶ÈÓĞ¸üĞÂ£¬»òÕßÎ»ÖÃÓĞ¸üĞÂ£¬¶¼Ëã¸üĞÂ
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] | 0x0F;
						printf("PPPPPPPVVVVVVVV\n");
					}
				}
				/*if(v_count == -1)//Ã»ÕÒµ½
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
				}
				else
				{
					if(V_time_last == airplane_velocity_three_static[v_count].time)
					{
						//ÕÒµ½ËÙ¶ÈÎŞ¸üĞÂ
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
					}
					else
					{
						//ËÙ¶ÈÓĞ¸üĞÂ
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] | 0x0F;
					}
					
				}*/
#endif
				//pbuf[8] = 0xFF;//µÚÒ»°üÊÇFF
				//memcpy(&(pbuf_yindao[9+ 2]),&(airplane_location_tmp.coordinate[1]),4);//¾­¶È
				//memcpy(&(pbuf_yindao[13+ 2]),&(airplane_location_tmp.coordinate[0]),4);//Î³¶È
				pbuf_yindao[9+ 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0xFF000000) >> 24);
				pbuf_yindao[9+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x00FF0000) >> 16);
				pbuf_yindao[9+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x0000FF00) >> 8);
				pbuf_yindao[9+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x000000FF));

				pbuf_yindao[13+ 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0xFF000000) >> 24);
				pbuf_yindao[13+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x00FF0000) >> 16);
				pbuf_yindao[13+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x0000FF00) >> 8);
				pbuf_yindao[13+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x000000FF));
				if(v_count < 0)//Ã»ÓĞÆ¥ÅäËÙ¶È
				{
					pbuf_yindao[17+ 2] = 0x00;
					pbuf_yindao[28+ 2] = 0x00;//ÀïÃæÒÑ¾­ÊÇ0ÁË
					V_time_last = 0;

					airplane_speed_tmp.E_W_velocity = 0;
					airplane_speed_tmp.ICAO_address = 0;
					airplane_speed_tmp.N_S_velocity = 0;
					airplane_speed_tmp.time = 0;
					airplane_speed_tmp.VERT_velocity = 0;
				}
				else//ÓĞÆ¥ÅäËÙ¶È
				{
					//memcpy(&(pbuf_yindao[17+ 2]),&(airplane_velocity_three_static[v_count].E_W_velocity),4);
					//memcpy(&(pbuf_yindao[21+ 2]),&(airplane_velocity_three_static[v_count].N_S_velocity),4);
					//memcpy(&(pbuf_yindao[25+ 2]),&(airplane_velocity_three_static[v_count].VERT_velocity),4);
					pbuf_yindao[17+ 2] = (Uint8)((airplane_velocity_three_static[v_count].E_W_velocity & 0xFF000000)>>24);
					pbuf_yindao[17+ 2 + 1] = (Uint8)((airplane_velocity_three_static[v_count].E_W_velocity & 0x00FF0000)>>16);
					pbuf_yindao[17+ 2 + 2] = (Uint8)((airplane_velocity_three_static[v_count].E_W_velocity & 0x0000FF00)>>8);
					pbuf_yindao[17+ 2 + 3] = (Uint8)((airplane_velocity_three_static[v_count].E_W_velocity & 0x000000FF));

					pbuf_yindao[21+ 2] = (Uint8)((airplane_velocity_three_static[v_count].N_S_velocity & 0xFF000000)>>24);
					pbuf_yindao[21+ 2 + 1] = (Uint8)((airplane_velocity_three_static[v_count].N_S_velocity & 0x00FF0000)>>16);
					pbuf_yindao[21+ 2 + 2] = (Uint8)((airplane_velocity_three_static[v_count].N_S_velocity & 0x0000FF00)>>8);
					pbuf_yindao[21+ 2 + 3] = (Uint8)((airplane_velocity_three_static[v_count].N_S_velocity & 0x000000FF));

					pbuf_yindao[25+ 2] = (Uint8)((airplane_velocity_three_static[v_count].VERT_velocity & 0xFF000000)>>24);
					pbuf_yindao[25+ 2 + 1] = (Uint8)((airplane_velocity_three_static[v_count].VERT_velocity & 0x00FF0000)>>16);
					pbuf_yindao[25+ 2 + 2] = (Uint8)((airplane_velocity_three_static[v_count].VERT_velocity & 0x0000FF00)>>8);
					pbuf_yindao[25+ 2 + 3] = (Uint8)((airplane_velocity_three_static[v_count].VERT_velocity & 0x000000FF));

					//¼ÇÂ¼ÉÏÒ»´ÎµÄÊ±¼ä´ÁICAO ËÙ¶È
					memcpy(&V_time_last,&(airplane_velocity_three_static[v_count].time), 4);
					
					airplane_speed_tmp.E_W_velocity = airplane_velocity_three_static[v_count].E_W_velocity;
					airplane_speed_tmp.ICAO_address = airplane_velocity_three_static[v_count].ICAO_address;
					airplane_speed_tmp.N_S_velocity = airplane_velocity_three_static[v_count].N_S_velocity;
					airplane_speed_tmp.time = airplane_velocity_three_static[v_count].time;
					airplane_speed_tmp.VERT_velocity = airplane_velocity_three_static[v_count].VERT_velocity;

				}
				//¸ß¶È
				//memcpy(&(pbuf_yindao[29+ 2]),&(airplane_location_tmp.altitude),4);
				pbuf_yindao[29+ 2] = (Uint8)((airplane_location_tmp.altitude & 0xFF000000)>>24);
				pbuf_yindao[29+ 2 + 1] = (Uint8)((airplane_location_tmp.altitude & 0x00FF0000)>>16);
				pbuf_yindao[29+ 2 + 2] = (Uint8)((airplane_location_tmp.altitude & 0x0000FF00)>>8);
				pbuf_yindao[29+ 2 + 3] = (Uint8)((airplane_location_tmp.altitude & 0x000000FF));
				//
				//¼ÇÂ¼ÉÏÒ»´ÎµÄ±¼´ÁICAO Î»ÖÃ
				memcpy(&P_time_last,&(airplane_location_tmp.time), 4);

				//Ğ£ÑéºÍ
				pbuf_yindao[33+ 2] = 0;
				for( k = 0; k < 35; k++)
				{
					pbuf_yindao[33+ 2] = (pbuf_yindao[33+ 2] + pbuf_yindao[k]) & 0xFF;
				}

				if (pbuf_yindao[0] == 0)
				{
					printf("yindao bao error!!!\n");
				}
				
				TaskSleep(1);
				EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
				pEDMA3CC_PaRAM->OPT = 0x00130104;//16
				pEDMA3CC_PaRAM->SRC =  (unsigned char *)(&pbuf_yindao[0]);  //ce4
				//	pEDMA3CC_PaRAM->SRC =  (unsigned char *)XSH_Array2;  //ce4	
				//	pEDMA3CC_PaRAM->BCNT_ACNT =0x00660001; //1280*4	//0x003d0001D¡ä¦Ì?FPGA??¦Ì??¡¤A0F	//51
				pEDMA3CC_PaRAM->BCNT_ACNT =0x00240001;	//0x22 = 34
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
			    //pEDMA3CC_PaRAM->DST = (unsigned char *)DSP_TO_FPGA_YINDAO;

				pEDMA3CC_PaRAM->DST = (unsigned char *)DSP_TO_FPGA_YINDAO;



				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
				
				ICAO_last = airplane_location_tmp.ICAO_address;

				printf("%x   ",airplane_location_tmp.ICAO_address);
				printf("%f   ",(float)airplane_location_tmp.coordinate[0]/10000000);
				printf("%f   ",(float)airplane_location_tmp.coordinate[1]/10000000);
				printf("%d   \n",airplane_location_tmp.time);
				
				;
			}
		}
		else if (yaokongcmd_all_data.mode == 2)//Ä¿±ê¸ú×ÙÄ£Ê½
		{
			count = 0;
			if(isfirst == 0)
			{
				//ÏÂÃæÁ½¸öÑ­»·¿ÉÄÜÒª»»Ò»ÏÂ£¬Ó¦¸ÃÏÈ°´ÕÕÓÅÏÈ¼¶´Ó¸ßµ½µÍµÚÒ»¸ö³öÏÖµÄ
				//for (i = 0; i < position_BUFLEN; i++)//±éÀúÊÕµ½µÄ1500¸öÊı×é
				for (j = 0; j < 30; j++)//±éÀúÉÏ´«µÄICAOºÅ
				{
					//for (j = 0; j < 30; j++)//±éÀúÉÏ´«µÄICAOºÅ
					for (i = 0; i < position_BUFLEN; i++)//±éÀúÊÕµ½µÄ1500¸öÊı×é
					{
						if((airplane_location_static[i].ICAO_address == yaokongcmd_all_data.ICAO[j]) && (yaokongcmd_all_data.ICAO[j] !=0))//ICAO Âú×ã
						{
							//tmp1 = fabs(airplane_location_static[i].coordinate[1] - yaokongcmd_all_data.jingdu);//Î³¶È²î
							//tmp2 = fabs(airplane_location_static[i].coordinate[0] - yaokongcmd_all_data.weidu);//¾­¶È²î
							
							//memcpy(&(tmp1),&(airplane_location_static[i].coordinate[1]),4);
							//memcpy(&(tmp2),&(airplane_location_static[i].coordinate[0]),4);
							tmp1 = fabs(airplane_location_static[i].coordinate[0]/10000000 - yaokongcmd_all_data.jingdu);//¾­¶È²î
							tmp2 = fabs(airplane_location_static[i].coordinate[1]/10000000 - yaokongcmd_all_data.weidu);//Î³¶È²î
							
							
							if ( (tmp1 <= yaokongcmd_all_data.banzhuijiao) & (tmp2 <= yaokongcmd_all_data.banzhuijiao) )
							{
								//ÔÚ½Ç¶È·¶Î§ÄÚ£¬½«ÃüÖĞµÄ·É»ú¸øµ½airplane_location_tmp±äÁ¿ÀïÃæ
								memcpy(&airplane_location_tmp,&(airplane_location_static[i]), 20);
								goto L1;//Ìø³öÁ½¸öÑ­»·
							}
						}
					}
				}
L1:
				
				//Êä³ö·É»úµÄÃüÖĞĞÅÏ¢£¬Ã»ÓĞÃüÖĞÒ²¿ÉÒÔÊä³ö£¬·´Õı¾ÍÊÇ0
				for (k = 0; k < 34; k++)//³õÊ¼»¯
				{
					pbuf_yindao[k] = 0x00;
				}
				pbuf_yindao[0] = 0x6A;//°üÍ·
				pbuf_yindao[1] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x00FF0000)>>16);//ICAOºÅ
				pbuf_yindao[2] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x0000FF00)>>8);
				pbuf_yindao[3] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x000000FF));
				//UTCÊ±¼ä
				//zhangfulong add Ê±¼ä¸üĞÂ
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add Ê±¼ä¸üĞÂ
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),4+ 2);//½«Ê±¼ä¸´ÖÆ¹ıÈ¥
				pbuf_yindao[4] = UTC_time_real[0];
				pbuf_yindao[5] = UTC_time_real[1];
				pbuf_yindao[6] = UTC_time_real[2];
				pbuf_yindao[7] = UTC_time_real[3];
				pbuf_yindao[8] = UTC_time_real[4];
				pbuf_yindao[9] = UTC_time_real[5];
				//pbuf_yindao[9] = count_tmp_zhang_for_test & 0x000000FF;//ÁÙÊ±ÒªÈ¥µô
				//Ê¹ÓÃ½â°üÊ±¼ä
				pbuf_yindao[6] = (airplane_location_tmp.time & 0xFF000000)>>24;
				pbuf_yindao[7] = (airplane_location_tmp.time & 0x00FF0000)>>16;
				pbuf_yindao[8] = (airplane_location_tmp.time & 0x0000FF00)>>8;
				pbuf_yindao[9] = (airplane_location_tmp.time & 0x000000FF);
				//Ê¹ÓÃ½â°üÊ±¼ä

				v_count = V_get(airplane_location_tmp.ICAO_address);
				//¸üĞÂ×´Ì¬
#if 0
				if(airplane_location_tmp.ICAO_address == 0)//Ã»ÕÒµ½ICAO
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0x0F;
				}
				else//ÕÒµ½ÁËICAO
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0x0F;//¸üĞÂ³É0x0X ÓëÉÏÒ»°ü²»Í¬
				}

				if(v_count == -1)//Ã»ÕÒµ½
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
				}
				else
				{
					pbuf_yindao[8+ 2] = (pbuf_yindao[8+ 2] | 0x0F ) ;//¸üĞÂ³É0x0X ÓëÉÏÒ»°ü²»Í¬
				}
#endif
				pbuf_yindao[8+ 2] = 0xFF;
				//memcpy(&(pbuf_yindao[9+ 2]),&(airplane_location_tmp.coordinate[1]),4);//¾­¶È
				//memcpy(&(pbuf_yindao[13+ 2]),&(airplane_location_tmp.coordinate[0]),4);//Î³¶È
				pbuf_yindao[9+ 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0xFF000000) >> 24);
				pbuf_yindao[9+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x00FF0000) >> 16);
				pbuf_yindao[9+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x0000FF00) >> 8);
				pbuf_yindao[9+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x000000FF));

				pbuf_yindao[13+ 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0xFF000000) >> 24);
				pbuf_yindao[13+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x00FF0000) >> 16);
				pbuf_yindao[13+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x0000FF00) >> 8);
				pbuf_yindao[13+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x000000FF));

				if(v_count < 0)//Ã»ÓĞÆ¥ÅäËÙ¶È
				{
					pbuf_yindao[17+ 2] = 0x00;
					pbuf_yindao[28+ 2] = 0x00;//ÀïÃæÒÑ¾­ÊÇ0ÁË
					V_time_last = 0;

					airplane_speed_tmp.E_W_velocity = 0;
					airplane_speed_tmp.ICAO_address = 0;
					airplane_speed_tmp.N_S_velocity = 0;
					airplane_speed_tmp.time = 0;
					airplane_speed_tmp.VERT_velocity = 0;

				}
				else//ÓĞÆ¥ÅäËÙ¶È
				{
					//memcpy(&(pbuf_yindao[17+ 2]),&(airplane_velocity_three_static[v_count].E_W_velocity),4);
					//memcpy(&(pbuf_yindao[21+ 2]),&(airplane_velocity_three_static[v_count].N_S_velocity),4);
					//memcpy(&(pbuf_yindao[25+ 2]),&(airplane_velocity_three_static[v_count].VERT_velocity),4);

					pbuf_yindao[17+ 2] = (Uint8)((airplane_velocity_three_static[v_count].E_W_velocity & 0xFF000000)>>24);
					pbuf_yindao[17+ 2 + 1] = (Uint8)((airplane_velocity_three_static[v_count].E_W_velocity & 0x00FF0000)>>16);
					pbuf_yindao[17+ 2 + 2] = (Uint8)((airplane_velocity_three_static[v_count].E_W_velocity & 0x0000FF00)>>8);
					pbuf_yindao[17+ 2 + 3] = (Uint8)((airplane_velocity_three_static[v_count].E_W_velocity & 0x000000FF));

					pbuf_yindao[21+ 2] = (Uint8)((airplane_velocity_three_static[v_count].N_S_velocity & 0xFF000000)>>24);
					pbuf_yindao[21+ 2 + 1] = (Uint8)((airplane_velocity_three_static[v_count].N_S_velocity & 0x00FF0000)>>16);
					pbuf_yindao[21+ 2 + 2] = (Uint8)((airplane_velocity_three_static[v_count].N_S_velocity & 0x0000FF00)>>8);
					pbuf_yindao[21+ 2 + 3] = (Uint8)((airplane_velocity_three_static[v_count].N_S_velocity & 0x000000FF));

					pbuf_yindao[25+ 2] = (Uint8)((airplane_velocity_three_static[v_count].VERT_velocity & 0xFF000000)>>24);
					pbuf_yindao[25+ 2 + 1] = (Uint8)((airplane_velocity_three_static[v_count].VERT_velocity & 0x00FF0000)>>16);
					pbuf_yindao[25+ 2 + 2] = (Uint8)((airplane_velocity_three_static[v_count].VERT_velocity & 0x0000FF00)>>8);
					pbuf_yindao[25+ 2 + 3] = (Uint8)((airplane_velocity_three_static[v_count].VERT_velocity & 0x000000FF));

					memcpy(&V_time_last,&(airplane_velocity_three_static[v_count].time), 4);
				
					airplane_speed_tmp.E_W_velocity = airplane_velocity_three_static[v_count].E_W_velocity;
					airplane_speed_tmp.ICAO_address = airplane_velocity_three_static[v_count].ICAO_address;
					airplane_speed_tmp.N_S_velocity = airplane_velocity_three_static[v_count].N_S_velocity;
					airplane_speed_tmp.time = airplane_velocity_three_static[v_count].time;
					airplane_speed_tmp.VERT_velocity = airplane_velocity_three_static[v_count].VERT_velocity;
				
				}
				//¸ß¶È
				//memcpy(&(pbuf_yindao[29+ 2]),&(airplane_location_tmp.altitude),4);
				pbuf_yindao[29+ 2] = (Uint8)((airplane_location_tmp.altitude & 0xFF000000)>>24);
				pbuf_yindao[29+ 2 + 1] = (Uint8)((airplane_location_tmp.altitude & 0x00FF0000)>>16);
				pbuf_yindao[29+ 2 + 2] = (Uint8)((airplane_location_tmp.altitude & 0x0000FF00)>>8);
				pbuf_yindao[29+ 2 + 3] = (Uint8)((airplane_location_tmp.altitude & 0x000000FF));

				//
				//ÇÂ¼ÉÏÒ»´ÎµÊ±¼ä´ÁICAO
				memcpy(&P_time_last,&(airplane_location_tmp.time), 4);

				//Ğ£ÑéºÍ
				for( k = 0; k < 36; k++)
				{
					pbuf_yindao[33+ 2] = (pbuf_yindao[33+ 2] + pbuf_yindao[k]) & 0xFF;
				}
				//½ÓÏÂÀ´·¢³öÈ¥pbuf£¬»¹Ã»×ö

				//
				ICAO_last = airplane_location_tmp.ICAO_address;
				if(airplane_location_tmp.ICAO_address != 0)//²»µÈÓÚ0£¬ÕÒµ½ÁËÒ»¸öºó£¬¸ú×Ù£¬ÒÔºó²»ÕÒ?
				{
					isfirst = isfirst + 1;
				}

				if (pbuf_yindao[0] == 0)
				{
					printf("yindao bao error!!!\n");
				}
				
				TaskSleep(1);
				EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
				pEDMA3CC_PaRAM->OPT = 0x00130104;//16
				pEDMA3CC_PaRAM->SRC =  (unsigned char *)(&pbuf_yindao[0]);  //ce4
				//	pEDMA3CC_PaRAM->SRC =  (unsigned char *)XSH_Array2;  //ce4	
				//	pEDMA3CC_PaRAM->BCNT_ACNT =0x00660001; //1280*4	//0x003d0001D¡ä¦Ì?FPGA??¦Ì??¡¤A0F	//51
				pEDMA3CC_PaRAM->BCNT_ACNT =0x00240001;	//0x22 = 34
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
			    pEDMA3CC_PaRAM->DST = (unsigned char *)DSP_TO_FPGA_YINDAO;
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
				
			}
			else
			{
				for (i = 0; i < position_BUFLEN; i++)//±éÀúÊÕµ½µÄ1500¸öÊı×é
				{
					if(airplane_location_static[i].ICAO_address == airplane_location_tmp.ICAO_address)
					{
						//ICAOºÅÃüÖĞ,ÏÂÃæ¿´ÊÇ·ñÊÇÊ±¼ä×îĞÂµÄ£¬
						if(airplane_location_static[i].time > airplane_location_tmp.time)
						{
							memcpy(&airplane_location_tmp,&(airplane_location_static[i]), 20);
						}
						
					}
				}

				for (k = 0; k < 34; k++)//³õÊ¼»¯
				{
					pbuf_yindao[k] = 0x00;
				}
				pbuf_yindao[0] = 0x6A;//°üÍ·
				pbuf_yindao[1] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x00FF0000)>>16);//ICAOºÅ
				pbuf_yindao[2] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x0000FF00)>>8);
				pbuf_yindao[3] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x000000FF));
				//UTCÊ±¼ä
				//zhangfulong add Ê±¼ä¸üĞÂ
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add Ê±¼ä¸üĞÂ
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),4+ 2);//½«Ê±¼ä¸´ÖÆ¹ıÈ¥
				pbuf_yindao[4] = UTC_time_real[0];
				pbuf_yindao[5] = UTC_time_real[1];
				pbuf_yindao[6] = UTC_time_real[2];
				pbuf_yindao[7] = UTC_time_real[3];
				pbuf_yindao[8] = UTC_time_real[4];
				pbuf_yindao[9] = UTC_time_real[5];
				//pbuf_yindao[9] = count_tmp_zhang_for_test & 0x000000FF;//ÁÙÊ±ÒªÈ¥µô
				//Ê¹ÓÃ½â°üÊ±¼ä
				pbuf_yindao[6] = (airplane_location_tmp.time & 0xFF000000)>>24;
				pbuf_yindao[7] = (airplane_location_tmp.time & 0x00FF0000)>>16;
				pbuf_yindao[8] = (airplane_location_tmp.time & 0x0000FF00)>>8;
				pbuf_yindao[9] = (airplane_location_tmp.time & 0x000000FF);
				//Ê¹ÓÃ½â°üÊ±¼ä
				//v_count = V_get(airplane_location_tmp.ICAO_address);
				//¸üĞÂ×´Ì¬
#if 1
				if(airplane_location_tmp.ICAO_address == ICAO_last)//ICAOÏàÍ¬
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] | 0xF0;
				}
				else//ICAO²»Í¬
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0x0F;//¸üĞÂ³É0x0X ÓëÉÏÒ»°ü²»Í¬
				}

				if ((v_count == -1) && (P_time_last==airplane_location_tmp.time))
				//ËÙ¶ÈÃ»ÕÒµ½£¬Î»ÖÃÊ±¼äÏàÍ¬
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
					
				}
				else//ÕÒµ½ÁËËÙ¶È,
				{
					if((V_time_last == airplane_velocity_three_static[v_count].time) && (P_time_last==airplane_location_tmp.time))
					{
						//ÕÒµ½ËÙ¶È ËÙ¶ÈÎŞ¸üĞÂ,Î»ÖÃÒ²Ã»¸üĞÂ
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
					}
					else
					{
						//ËÙ¶ÈÓĞ¸üĞÂ¬»òÕßÎ»ÖÃÓĞ¸üĞÂ£¬¶¼Ëã¸üĞ?
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] | 0x0F;
						printf("new FFFFFF !!!");
					}
				}

				/*if(v_count == -1)//Ã»ÕÒµ½
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
				}
				else
				{
					if(V_time_last == airplane_velocity_three_static[v_count].time)
					{
						//ÕÒµ½ËÙ¶ÈÎŞ¸üĞÂ
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
					}
					else
					{
						//ËÙ¶ÈÓĞ¸üĞÂ
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] | 0x0F;
					}
					
				}*/
#endif
				//pbuf[8] = 0xFF;
				//memcpy(&(pbuf_yindao[9+ 2]),&(airplane_location_tmp.coordinate[1]),4);//¾­¶È
				//memcpy(&(pbuf_yindao[13+ 2]),&(airplane_location_tmp.coordinate[0]),4);//Î³¶È
				pbuf_yindao[9+ 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0xFF000000) >> 24);
				pbuf_yindao[9+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x00FF0000) >> 16);
				pbuf_yindao[9+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x0000FF00) >> 8);
				pbuf_yindao[9+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x000000FF));

				pbuf_yindao[13+ 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0xFF000000) >> 24);
				pbuf_yindao[13+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x00FF0000) >> 16);
				pbuf_yindao[13+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x0000FF00) >> 8);
				pbuf_yindao[13+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x000000FF));
				if(v_count < 0)//Ã»ÓĞÆ¥ÅäËÙ¶È
				{
					pbuf_yindao[17+ 2] = 0x00;
					pbuf_yindao[28+ 2] = 0x00;//ÀïÃæÒÑ¾­ÊÇ0ÁË
					V_time_last = 0;

					airplane_speed_tmp.E_W_velocity = 0;
					airplane_speed_tmp.ICAO_address = 0;
					airplane_speed_tmp.N_S_velocity = 0;
					airplane_speed_tmp.time = 0;
					airplane_speed_tmp.VERT_velocity = 0;
				}
				else//ÓĞÆ¥ÅäËÙ¶È
				{
					//memcpy(&(pbuf_yindao[17+ 2]),&(airplane_velocity_three_static[v_count].E_W_velocity),4);
					//memcpy(&(pbuf_yindao[21+ 2]),&(airplane_velocity_three_static[v_count].N_S_velocity),4);
					//memcpy(&(pbuf_yindao[25+ 2]),&(airplane_velocity_three_static[v_count].VERT_velocity),4);
					pbuf_yindao[17+ 2] = (Uint8)((airplane_velocity_three_static[v_count].E_W_velocity & 0xFF000000)>>24);
					pbuf_yindao[17+ 2 + 1] = (Uint8)((airplane_velocity_three_static[v_count].E_W_velocity & 0x00FF0000)>>16);
					pbuf_yindao[17+ 2 + 2] = (Uint8)((airplane_velocity_three_static[v_count].E_W_velocity & 0x0000FF00)>>8);
					pbuf_yindao[17+ 2 + 3] = (Uint8)((airplane_velocity_three_static[v_count].E_W_velocity & 0x000000FF));

					pbuf_yindao[21+ 2] = (Uint8)((airplane_velocity_three_static[v_count].N_S_velocity & 0xFF000000)>>24);
					pbuf_yindao[21+ 2 + 1] = (Uint8)((airplane_velocity_three_static[v_count].N_S_velocity & 0x00FF0000)>>16);
					pbuf_yindao[21+ 2 + 2] = (Uint8)((airplane_velocity_three_static[v_count].N_S_velocity & 0x0000FF00)>>8);
					pbuf_yindao[21+ 2 + 3] = (Uint8)((airplane_velocity_three_static[v_count].N_S_velocity & 0x000000FF));

					pbuf_yindao[25+ 2] = (Uint8)((airplane_velocity_three_static[v_count].VERT_velocity & 0xFF000000)>>24);
					pbuf_yindao[25+ 2 + 1] = (Uint8)((airplane_velocity_three_static[v_count].VERT_velocity & 0x00FF0000)>>16);
					pbuf_yindao[25+ 2 + 2] = (Uint8)((airplane_velocity_three_static[v_count].VERT_velocity & 0x0000FF00)>>8);
					pbuf_yindao[25+ 2 + 3] = (Uint8)((airplane_velocity_three_static[v_count].VERT_velocity & 0x000000FF));

					memcpy(&V_time_last,&(airplane_velocity_three_static[v_count].time), 4);
				
					airplane_speed_tmp.E_W_velocity = airplane_velocity_three_static[v_count].E_W_velocity;
					airplane_speed_tmp.ICAO_address = airplane_velocity_three_static[v_count].ICAO_address;
					airplane_speed_tmp.N_S_velocity = airplane_velocity_three_static[v_count].N_S_velocity;
					airplane_speed_tmp.time = airplane_velocity_three_static[v_count].time;
					airplane_speed_tmp.VERT_velocity = airplane_velocity_three_static[v_count].VERT_velocity;
				
				}
				//¸ß¶È
				//memcpy(&(pbuf_yindao[29+ 2]),&(airplane_location_tmp.altitude),4);
				pbuf_yindao[29+ 2] = (Uint8)((airplane_location_tmp.altitude & 0xFF000000)>>24);
				pbuf_yindao[29+ 2 + 1] = (Uint8)((airplane_location_tmp.altitude & 0x00FF0000)>>16);
				pbuf_yindao[29+ 2 + 2] = (Uint8)((airplane_location_tmp.altitude & 0x0000FF00)>>8);
				pbuf_yindao[29+ 2 + 3] = (Uint8)((airplane_location_tmp.altitude & 0x000000FF));

				//
				//¼ÇÂ¼ÉÏÒ»´ÎµÄÊ±¼ä´ÁICAO
				memcpy(&P_time_last,&(airplane_location_tmp.time), 4);

				//Ğ£ÑéºÍ
				for( k = 0; k < 36; k++)
				{
					pbuf_yindao[33+ 2] = (pbuf_yindao[33+ 2] + pbuf_yindao[k]) & 0xFF;
				}
				//½ÓÏÂÀ´·¢³öÈ¥pbuf£¬»¹Ã»×ö
				
				//Êä³ö·É»úµÄÃüÖĞĞÅÏ¢£¬Ã»ÓĞÃüÖĞÒ²¿ÉÒÔÊä³ö£¬·´Õı¾Í?
				ICAO_last = airplane_location_tmp.ICAO_address;
				//
				//TaskSleep(15);
				if (pbuf_yindao[0] == 0)
				{
					printf("yindao bao error!!!\n");
				}
				TaskSleep(1);
				EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
				pEDMA3CC_PaRAM->OPT = 0x00130104;//16
				pEDMA3CC_PaRAM->SRC =  (unsigned char *)(&pbuf_yindao[0]);  //ce4
				//	pEDMA3CC_PaRAM->SRC =  (unsigned char *)XSH_Array2;  //ce4	
				//	pEDMA3CC_PaRAM->BCNT_ACNT =0x00660001; //1280*4	//0x003d0001D¡ä¦Ì?FPGA??¦Ì??¡¤A0F	//51
				pEDMA3CC_PaRAM->BCNT_ACNT =0x00240001;	//0x22 = 34
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
			    pEDMA3CC_PaRAM->DST = (unsigned char *)DSP_TO_FPGA_YINDAO;
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
			}
			
		}
		else
		{
			V_time_last = 0;//¼ÇÂ¼ÉÏÒ»´ÎµÄÊ±¼ä£¬ºÍÕâ´Î±ÈÊÇ·ñÓĞ¸üĞÂ
			P_time_last = 0;//¼ÇÂ¼ÉÏÒ»´ÎµÄÊ±¼ä£¬ºÍÕâ´Î±ÈÊÇ·ñĞ¸üĞ?
		}
		TaskSleep(1 * 1000);
	}
	;
	;
}
void lead_tesk_2()
{
 
     Uint8 pbuf_yindao_hh[150] = {0};
	 Uint8 i=0;
	for (i = 0; i < 30; i++)
	    {
		   pbuf_yindao_hh[i] = 0xFF;
		}
	while(1)
	{
	
			//		TaskSleep(1);
			EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
			pEDMA3CC_PaRAM->OPT = 0x00130104;//16
			pEDMA3CC_PaRAM->SRC =  (unsigned char *)(&pbuf_yindao_hh[0]);  //ce4
					//	pEDMA3CC_PaRAM->SRC =  (unsigned char *)XSH_Array2;  //ce4	
					//	pEDMA3CC_PaRAM->BCNT_ACNT =0x00660001; //1280*4	//0x003d0001D¡ä¦Ì?FPGA??¦Ì??¡¤A0F	//51
			pEDMA3CC_PaRAM->BCNT_ACNT =0x00240001;	//0x22 = 34
			pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
		    pEDMA3CC_PaRAM->DST = (unsigned char *)DSP_TO_FPGA_YINDAO;
			EDMA3CC_ESRH = 0x00010000;
				
			TaskSleep(1000);
	}
}
int ICAO_task_tmp[1000] = {0};
void ICAO_tesk()
{
	int i, j, k;
	int cnt = 0;//¼ÇÂ¼ICAOÎ»ÖÃ
	for (i = 0; i < 1000; i++)
	{
		ICAO_task_tmp[i] = 0;
	}
	while (1)
	{
		for (i = 0; i < 1500; i++)//±éÀúËùÓĞ¿Õ¼ä
		{
			//µ±Ç°ICAOºÅ airplane_location_static[i].ICAO_address
			for (j = 0; j < 1000; j++)//±éÀúICAOºÅÊı×é
			{
				if ((airplane_location_static[i].ICAO_address == ICAO_task_tmp[j]) && (airplane_location_static[i].ICAO_address != 0))
				{
					//ÃüÖĞÕâ¸öICAOºÅÒÑ¾­ÓĞÁË
					goto ICAO_L;
				}
				;
			}
			//ÉÏÃæÅÜ¹ıÁË£¬ËµÃ÷ÊÇ¸öĞÂICAO
			if(airplane_location_static[i].ICAO_address != 0)
			{
				ICAO_task_tmp[cnt] = airplane_location_static[i].ICAO_address;
				cnt++;
			}

ICAO_L:	
		}

		printf("get ICAO %d\n",cnt);
		TaskSleep(800);
	}
}


 /**********************************************/
void videosrv()
{
  //  Uint16 pData;
  //  Uint16 i=0;
  //  Uint32 *p;
  int i, j, k;
    fdOpenSession( TaskSelf() );   
    TaskCreate( watchdog,   "watchdog", OS_TASKPRILOW, 0x500, 0, 0, 0 );	
//  TaskSleep(1000);//20150710
//   sem0 = SEM_create(0,0); 
    DSP_TO_FPGA_HANDLE = 0x55aa; //ÕÊ?
    MEM_initial();

	C62_disableIER(1<<8);  // edmaÖĞ¶Ï 
	asm("nop");	
	C62_clearIFR(1<<8); // ÇåEDMAÖĞ	
	EDMA_init();
	
	C62_clearIFR(1<<9|1<<8);
	C62_enableIER(1<<9|1<<8);//9--fpgaÖĞ¶Ï 8--edmaÖĞ¶Ï	

//	gpio7_set_0();
#if 0
    C62_enableIER(1<<8);
	while(1)
	{
		   testEDMA();
		   DSK6455_wait(10);
    }
#endif

	//zhangfulong add õÊ¼»?
	yaokongcmd_all_data.banzhuijiao = 180;
	yaokongcmd_all_data.weidu = 0.0;
	yaokongcmd_all_data.jingdu = 90.0;
	yaokongcmd_all_data.gaodu = 0.0;
	yaokongcmd_all_data.mode = 0;
	yaokongcmd_all_data.chelue_new = 0xFF;
	for (i = 0; i < position_BUFLEN; i++)
	{
		airplane_location_static[i].altitude = 0;
		airplane_location_static[i].coordinate[1] = 0;
		airplane_location_static[i].coordinate[0] = 0;
		airplane_location_static[i].ICAO_address = 0;
		airplane_location_static[i].time = 0;

		airplane_velocity_three_static[i].E_W_velocity = 0;
		airplane_velocity_three_static[i].ICAO_address = 0;
		airplane_velocity_three_static[i].N_S_velocity = 0;
		airplane_velocity_three_static[i].time = 0;
		airplane_velocity_three_static[i].VERT_velocity = 0;
		
	}		
	for (i = 0; i < position_BUFLEN_xsh; i++)
	{
		airplane_velocity_three_XSH_static[i].ICAO_address=0;
		airplane_velocity_three_XSH_static[i].N_S_velocity=0;
		airplane_velocity_three_XSH_static[i].E_W_velocity=0;
		airplane_velocity_three_XSH_static[i].VERT_velocity=0;	
	}		
	//zhangfulong add
	printf("start now !!!\n");


//	TaskCreate( lead_tesk_2 ,"lead_tesk_2",OS_TASKPRILOW, 0x3000, 0, 0, 0); 
#if 1

   TaskCreate( get_Pulse ,"get_Pulse",OS_TASKPRINORM, 0x3000, 0, 0, 0); // 1280´¦Àíº¯Êı //OS_TASKPRILOW
#if 1
   TaskCreate( get_PosV ," get_PosV",OS_TASKPRINORM, 0x3000, 0, 0, 0);  //OS_TASKPRILOW
#endif
#if 1
   //TaskCreate( xsh_task ," xsh_task",OS_TASKPRINORM, 0x3000, 0, 0, 0); //20190211  //OS_TASKPRILOW
   //Ï¡Êè»¯
   TaskCreate( xsh_2_task ," xsh_2_task",OS_TASKPRINORM, 0x3000, 0, 0, 0); //20190211  //OS_TASKPRILOW
#endif

	//´´½¨EMIF¿ÚÍ¨ĞÅÈÎÎñ£¬´úÌæÔ­À´µÄUDP´«Êä
	//Ô­Ê¼ºÍÈÚºÏ
	TaskCreate( EMIF_sndPacket ,"EMIF_sndPacket",OS_TASKPRILOW, 0x5000, 0, 0, 0);

	//´´½¨Ò»¸öÒıµ¼ÈÎÎñ lead_tesk
	//Òıµ¼
	TaskCreate( lead_tesk ,"lead_tesk",OS_TASKPRILOW, 0x3000, 0, 0, 0);
	//check ICAO tesk
	//TaskCreate( ICAO_tesk ,"ICAO_tesk",OS_TASKPRILOW, 0x3000, 0, 0, 0);

#endif
#if 0
   TaskCreate( udp_sndPacket ,"udp_sndPacket",OS_TASKPRILOW, 0x3000, 0, 0, 0); //
#endif
   TaskBlock( TaskSelf() );
}
