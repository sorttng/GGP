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
Uint8 test_flag =0; //有测试数据置成1，什么时候结束呢？
Uint8 sum_cnt=0;
Uint8 zero_cnt=0;
Uint8 XSH_Array[288]={0};//88 
Uint8 TestArray[160]={0};


Uint8 ICAOArray[800]={0};	//用来接收FPGA传过来的ICAO号，30*4B
Uint8 ICAOArray_real[200]={0};	//调整顺序后的存储位置
Uint16 check_sum_1,check_sum_2;

//zhangfulong add start
Uint8	yaokongcmd[140] = {0};//遥控指令信号数据格式
Uint32 ICAO_number[30] = {0};//用来将收到的ICAO号写到32位的数组里面
int taokong_workmode = 0;// 0表示没有发，1：自由目标搜索 ；2：目标跟踪
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

//Uint8 pbuf_yindao[36] = {0};//引导数据格式
Uint8 pbuf_yindao[150] = {0};
Uint8 up_commend_count = 0;//上注包计数变量
//zhangfulong add end

//Uint8 XSH_Array2[188]={0};//?
//EDMA_Handle hEdma,hEdma_samp;     /* Handle for the EDMA channel  */
#if 0
	unsigned int int_cnt_1=0;//中断个数
	unsigned int int_cnt_2=0;//CRC全对的
	unsigned int int_cnt_3=0;//发送包个数
	unsigned int int_cnt_4=0;//可纠错的
	unsigned int int_cnt_5=0;
	unsigned int int_cnt_6=0;
	unsigned int int_cnt_7=0;  //和码源不一致的
	unsigned int int_cnt_8=0;   //搜头对的
	unsigned int int_cnt_9=0;  //yc
	unsigned int int_cnt_10=0;  //xsh
	unsigned int int_cnt_11=0;  //ykip
	unsigned int int_cnt_12=0;  //test
#endif
unsigned int int_cnt_1=0;//中断个数
unsigned int int_cnt_3=0;//发送包个数
Uint8  int_cnt[22]={0x0};   //
unsigned int pos_2_cnt=0; //解算位置2计数
unsigned int pos_3_cnt=0; //解算位置3计数(有效)
unsigned int vel_4_cnt=0; //解算速度4计数
unsigned int vel_5_cnt=0; //解算速度5计数（有效）
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
volatile Uint32 IP_Pkg_index=0;//1;  //写指针 //0//170904 //?óê?????
#pragma DATA_SECTION ( SEND_ADDR_CACHE, ".DATA_IP" );
//#pragma DATA_ALIGN(SEND_ADDR_CACHE, 128);
IP_ADDR_CACHE  SEND_ADDR_CACHE[SEND_IP_DATA_BUFLEN];
Uint32 g_u32IpSendIndex=1;   //读指针
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
//感觉给网络准备的缓冲区很容易满，是否有必要开大
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

struct location_struct airplane_location_XSH_static[position_BUFLEN_xsh];  //张富隆 add XSH稀疏化的位置数组
struct speed_struct_three airplane_velocity_three_XSH_static[position_BUFLEN_xsh];  //张富隆 add XSH稀疏化的速度数组

Uint32 position_xsh_ICAO[position_BUFLEN_xsh];
volatile Uint32 position_cnt_xsh=0;//
volatile Uint32 position_wr_xsh=0;
volatile Uint32 position_wr_xsh_v=0;//zhangfulong add 稀疏化数据，
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
FPGA_YC s_FPGA_YC;  //16字节遥测
Uint8 UTC_time_all[20] = 0;//接收的数据
Uint8 UTC_time_real[6] = 0;//记录UTC时间，用来给其他的值
#pragma DATA_ALIGN(s_FPGA_YC, 8);
//用于报文信息的上报的全局结构体
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
  unsigned char data_demodulator[11];  //ADSB的原始88bit消息报文，11字节							
}ADSB_message; 
//******************************************************************************************************************************
//结构体变量的声明必须包含结构体类型定义本身,用于位置信息的计算
//此结构体数组和结构体变量全局使用，在decode_position函数更新该结构体data_save[]数组，new_data结构体变量在demodulator函数中更新
//该结构体必须全局保存数据，下面是各个元素的定义。
//time是报文的时间标签，精确到秒；
//ICAO_adress_with_mark用于存放ICAO地址，用低24位，第25位是标志位，为0指示共用体data中存的是经纬度信息，为1指示指示共用体data中存的是前一次收到的CPR编码信息
//position共用体保存经纬度信息coordinate[0]是纬度，coordinate[1]是经度，为double类型各占8字节；
//position或者保存CPR编码信息CPR_code[0]为纬度编码，CPR_code[1]为经度编码，CPR_code[2]为奇偶编码指示，用最低位，CPR_code[3]未使用，unsigned int类型，各占4个字节
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
	  int coordinate[2];//正式上报时候使用int型，正式上报时候使用int(1E-7为单位)。
	  int altitude;
};
struct location_struct airplane_location;//此结构体数组和结构体变量全局使用,airplane_location;

//zhangfulong add start 定义一结构是location_struct ICAO 时间 经纬度 高度 1500个
struct location_struct airplane_location_static[position_BUFLEN];
struct location_struct airplane_location_static_ronghe[position_BUFLEN];//位置给融合用
//zhangfulong add end

#pragma DATA_ALIGN(data_save,32);//对齐
//****************************************************************************************************************************
//结构体变量的声明必须包含结构体类型定义本身,用于速度信息的计算
//time是速度信息的时间标签，精确到秒；
//ICAO_adress是ICAO地址
//E_W_velocity是东西方向速度，东为正，西为负，单位km/h；N_S_velocity是南北方向速度，北为正，南为负，单位km/h；
//VERT_velocity是垂直方向速度，上升为正，下降为负，单位m/sdisu；

/*airplane_velocity.ICAO_address的低24bit用于存放ICAO地址，第25-27位用于指示速度状态（从0位编号，第24-26位）
三位的组合定义为001,010,011,100，四种状态，其它状态无效；三位顺序按照为27，26，25从高到低的方式。

当27-25为001时：指示当前给出的是"地速"，非超音速模式（跟飞行器实际是否超音速无关）
此时：东西或南北方向的速度绝对数值如等于1891.818，表示在东西或南北方向速度值超过1891.818公里/小时，具体数值未可知。（绝对值小于1891.818时，数值对应各个方向的速度）
      在垂直方向的速度绝对值如等于9938.9184，表示垂直方向的速度大于9938.9184米/分钟，具体数值未可知。（绝对值小9938.9184时，数值对应垂直方向的速度?

当27-25为010时：指示当前给龅氖?地速"，超音速模式（跟飞行器实际是否超音速无关）
此保憾骰蚰媳狈较虻乃俣染允等绲扔?567.272，表示在东西或南北方向速度值超?567.272公里/小时，具体数值未可知。（绝对值小于7567.272时，数值对应各个方向的速度）
      在垂直方向的速度绝对值如等于9938.9184，表示垂直方向的速度大于9938.9184米/分钟，具体数值未可知。（绝对值小9938.9184时，数值对应垂直方向的速度）

当27-25为011时：指示当前给出的是"空速"，非超音速模式（跟飞行器实际是否超音速无关
此时：平面方向速度以极坐标方式给出，为统一表示方法，将其分解为东西和南北方向给出速度。
      如骱湍媳狈较蚋龅乃俣惹?向量和"后，绝对数值等于1891.818，指示在平面上的速度大于1891.818公里/小时，具体数值未可知。（"向量和"小于1891.818时，数值对应平面上的速度）      
      在垂直方向的速度绝对值如等于9938.9184，表示垂直方向的速度大于9938.9184米/分钟，具体数值未可。（绝对值小9938.9184时，数值对应垂直方向的速度）

当27-25为100时：指示当前给出的是"空速"，超音速模式（跟飞行器实际是否超音速无关）
此时：平面方向速度以极坐标方式给出，为统一表示方法，将其分解为东西和南北方向给出速度。
      如果东西和南北方向给出的速度求"向量和"后，绝对数值等于7567.272，指示在平面上的速度大于7567.272公里/小时，具体数值未可知。（"向量和"小于7567.272时，数值对应平面上的速度）        
      在垂直方向的速度绝对值如等于9938.9184，表示垂直方向的速度大于9938.9184米/分钟，具体数值未可知。（绝对值小9938.9184时，数值对应垂直方虻乃偃）*/
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
struct speed_struct_three airplane_velocity_three_static_ronghe[position_BUFLEN];  //速度给融合用1500

struct location_struct airplane_location_tmp = {0};//用来记录临时的命中的ICAO号等信息 位置
struct speed_struct_three airplane_speed_tmp = {0};//用来记录临时的命中的ICAO号等信息 速度
//zhangfulong add end

unsigned char velocity_subtype=0;//速度解算全局变量，用于传递信息使用，用户不用关心
struct speed_code
{
	  unsigned char bit46;
	  unsigned short bit47_56;
	  unsigned char bit57;
	  unsigned short bit58_67;
	  unsigned char bit69;
	  unsigned short bit70_78;
}velocity_code;              //速度解算全局结构体变量，用于传递信息使用，用户不用关心
struct trans_Record
{
  unsigned int ICAO_adress_with_mark;  //190315
  unsigned int time;
  unsigned int flag;
}trans_Record[FIND_RANGE];
//}trans_Record[20];
//****************************************************************************************************************************
/**旧模拟源**/
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
							   
/**新模拟源**/
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
extern UINT8 bMacAddr[8]; //mac地址
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
void MEM_initial() //内存及全局变量初始化
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
  	adsb_message_counter=0;//ADSB消息的计数器，需要清零
	Yaoce_counter=0;//遥测剖
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
//对更新位置报文信息用的new_data清0，对全局保存位置信息的结构体数组data_save[]初始化处理，注意：为了满足更新策略的要求
//data_save[i].ICAO_adress_with_mark的第25位需要设置为0！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
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
//初始化估计不全 （还有很多变量需要初始化）
//**************************************************************************
   memset(pulse,0,sizeof(pulse)) ;
   memset(&new_data, 0, sizeof(new_data));     //190305     
   memset(data_save, 0, sizeof(data_save));
   memset(&velocity_code, 0, sizeof(velocity_code));              
   memset(&airplane_location, 0, sizeof(airplane_location));      //上报经纬位置用结构体
   memset(&airplane_velocity_three, 0, sizeof(airplane_velocity_three));      //上报速度用结构体
   memset(&ADSB_message, 0, sizeof(ADSB_message));                //上报原始88bit报男畔⒂媒峁体
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
  //  Uint8 int_cnt=0;//中断计数
  	//	C62_disableIER(1<<9);  //1?±??D??
#if 1   
    CHAN_LENTH_FLAG = DSP_INIT_ADDR_2;   //要写2次
    CHAN_LENTH_FLAG = DSP_INIT_ADDR;//
    FPGA_CHAN_FLAG = CHAN_LENTH_FLAG & 0xff;//oxf
    if(FPGA_CHAN_FLAG==0x02) //2  //5  //1280 ADSB AD data****************
#endif
	{
	    flag=1;  //用于晔臼鞘裁蠢嘈偷闹卸?
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
		{    //缓冲未满
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
		 {   //如果缓冲满了   （不应该满）
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
	    //pEDMA3CC_PaRAM->DST = (unsigned char *)(&s_FPGA_YC); //16字节遥测
		pEDMA3CC_PaRAM->DST = (unsigned char *)(UTC_time_all);	
		EDMA3CC_ESRH = 0x00010000;  //???ˉ

#if 0		
		//zhangfulong add 时间更新  没有用 要在其他地方使用更新，避免没传完就用
		UTC_time_real[0] = UTC_time_all[13];
		UTC_time_real[1] = UTC_time_all[12];
		UTC_time_real[2] = UTC_time_all[15];
		UTC_time_real[3] = UTC_time_all[14];
		UTC_time_real[4] = UTC_time_all[17];
		UTC_time_real[5] = UTC_time_all[16];
		s_FPGA_YC.time = 0;
		//zhangfulong add 时间更新
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
		EDMA3CC_ESRH = 0x00010000;  //???ˉ
		return;
	}
#endif
    else if(FPGA_CHAN_FLAG==0x08)  //4  // test
	{
	    flag=4;
		test_flag =1; //进入测试模式
	    int_cnt[12]++;//
    	EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32			
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_TEST;  //ce4	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x006a0004; //104*4	 	
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)TestArray;
		EDMA3CC_ESRH = 0x00010000;  //???ˉ
		return;
	} 
    else if(FPGA_CHAN_FLAG==0x0a)  //5  // test  
	{
	    flag=5;
		test_flag =0 ; //退出测试模式
	    int_cnt[12]++;//
    	EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32			
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_TEST;  //ce4	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x006a0004; //104*4	 	
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)TestArray;
		EDMA3CC_ESRH = 0x00010000;  //???ˉ
		return;
	} 
	else if(FPGA_CHAN_FLAG==0xff)
	{
	   FPGA_CHAN_FLAG=0;
	}
#endif

	else if(FPGA_CHAN_FLAG==0x0b)  //20200811 张富隆添加，独立部分接收ICAO号
	{
	    flag=4;
		test_flag =1; //进入测试模式
	    int_cnt[12]++;//
    	EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32			
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_ICAO;  //ce4 和IP共用一个地址空间	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x006a0004; //104*4	 	
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)ICAOArray;	//长度30*4B = 120 个UINT型
		EDMA3CC_ESRH = 0x00010000;  //???ˉ
		return;
	} 
	else if(FPGA_CHAN_FLAG==0x55)//0x55自由目标收索模式
	{
		//
		EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32	
				
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_ICAO;  //ce4 和IP共用一个地址空间	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x00860004; //	 0x86 = 134	
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)(&yaokongcmd_all_data.mode);;	//长度30*4B = 120 个UINT型
		EDMA3CC_ESRH = 0x00010000;
		//
		/*
		Uint32 ICAO_number[30] = {0};//用来将收到的ICAO号吹?2位的数组里面
		int taokong_workmode = 0;// 0表示没有发，1：自由目标搜索 ；2：目标跟踪
		int yaokong_jingdu = 0.0;
		int yaokong_weidu = 0.0;
		int yaokong_gaodu = 0.0;
		int yaokong_banzhuijiao = 0;//0-90
		*/
		taokong_workmode = 1;
	}
	else if(FPGA_CHAN_FLAG==0xAA)//0x55目标跟踪模式
	{
		//
		EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32	
				
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_ICAO;  //ce4 和IP共用一个地址空间	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x00860004; //	 0x86 = 134	
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)(&yaokongcmd_all_data.mode);;	//长度30*4B = 120 个UINT型
		EDMA3CC_ESRH = 0x00010000;
		//
		/*
		Uint32 ICAO_number[30] = {0};//用来将收到的ICAO号写到32位的数组里面
		int taokong_workmode = 0;// 0表示没有发，1：自由目标搜索 ；2：目标跟踪
		int yaokong_jingdu = 0.0;
		int yaokong_weidu = 0.0;
		int yaokong_gaodu = 0.0;
		int yaokong_banzhuijiao = 0;//0-90
		*/
		taokong_workmode = 2;
	}
	else if(FPGA_CHAN_FLAG==0x03)//0x03 遥控模式
	{
		//flag=4;
		//test_flag =1; //进入测试模式
	    //int_cnt[12]++;//
		//for (i = 0; i < 200; i++)
		//{
		//	ICAOArray[i] = 0;
		//}
    	EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32			
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_ICAO;  //ce4 和IP共用一个地址空间	
		//pEDMA3CC_PaRAM->BCNT_ACNT =0x006a0004; //104*4
		pEDMA3CC_PaRAM->BCNT_ACNT =0x00C80004;	 //0xC8 = 200	
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)ICAOArray;	//长度30*4B = 120 个UINT型
		EDMA3CC_ESRH = 0x00010000;  //???ˉ
#if 0		
		//调整顺序，数据处理部分放在lead任务for循环开头状态，这段没用，应该移动走
		for (i = 0; i < 200; i = i + 2)
		{
			ICAOArray_real[i] = ICAOArray[i + 1];
			ICAOArray_real[i + 1] = ICAOArray[i];
		}
		
		if ((ICAOArray_real[12] = 0xbb) && (ICAOArray_real[13] = 0x77) && (ICAOArray_real[14] = 0x3D) )
		{
			//包头正确 ，下面进行校验和计算 从 12 的00 04 开始  Uint16 check_sum_1,check_sum_2;
			check_sum_1 = 0;
			check_sum_2 = 0;
			for (i = 12; i < 151; i = i +2 )
			{
				check_sum_1 = (check_sum_1 ^ ( (Uint16)(ICAOArray_real[i] << 8)+(Uint16)(ICAOArray_real[i]) ) );
			}
			check_sum_2 = ( (Uint16)(ICAOArray_real[152] << 8)+(Uint16)(ICAOArray_real[153]) );
//			if (check_sum_1 != check_sum_2)
			{
				//能进来说明校验和正确，下面开始将值更新到我们的结构体里面
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
				memcpy(&(yaokongcmd_all_data.jingdu),&(ICAOArray_real[16]),4);//经度
				memcpy(&(yaokongcmd_all_data.weidu),&(ICAOArray_real[20]),4);//纬度
				memcpy(&(yaokongcmd_all_data.gaodu),&(ICAOArray_real[24]),4);//高度
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
		}//到此，拿走
#endif
/*
		if (ICAOArray[1] == 0x55)//自由目标搜索
		{
			yaokongcmd_all_data.mode = 1;
			memcpy(&(yaokongcmd_all_data.jingdu),&(ICAOArray[2]),4);//经度
			memcpy(&(yaokongcmd_all_data.weidu),&(ICAOArray[6]),4);//纬度
			memcpy(&(yaokongcmd_all_data.gaodu),&(ICAOArray[10]),4);//高度
			yaokongcmd_all_data.banzhuijiao = ICAOArray[14];
			yaokongcmd_all_data.chelue_1 = ICAOArray[15];
			yaokongcmd_all_data.chelue_2 = ICAOArray[16];
			yaokongcmd_all_data.chelue_3 = ICAOArray[17];
			for (k = 0; k < 30; k++)
			{
				memcpy(&(yaokongcmd_all_data.ICAO[k]),&(ICAOArray[18 + k*4]),4);;
			}
		}
		else if (ICAOArray[1] == 0xAA)//跟踪模式
		{
			yaokongcmd_all_data.mode = 2;
			memcpy(&(yaokongcmd_all_data.jingdu),&(ICAOArray[2]),4);//经度
			memcpy(&(yaokongcmd_all_data.weidu),&(ICAOArray[6]),4);//纬度
			memcpy(&(yaokongcmd_all_data.gaodu),&(ICAOArray[10]),4);//高度
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
void EDMA_int()  //EDMA中断
{
	int_cnt[5]++;
 	EDMA3CC_ICRH=0x00010000;   //清edma中断   //???170718
  //  C62_clearIFR(1<<8); 
 //     *(CIPRL)=0x10;//清除DMA4中断
 //     *(ECRL)=0x10;//清除DMA4中断
//	   *(GPVAL)=*(GPVAL)&0xfffffdff;  //GPIO
//----------------------------------------------------
// 目前启动EDMA的有4类数据 （是否都需要中断）
//EDMA 中断需要根据标志处理？
   if(flag==1){
			SEND_ADDR_CACHE[IP_Pkg_index].pwrite = 1; //
			SEND_ADDR_CACHE[IP_Pkg_index].IP_HDADDR=(Uint32)&IP_DATA[IP_Pkg_index][0];
			SND_Catch_Program_Full++; //190201
	        edma_finished = 1;
   }
   else if(flag==2)
   {
		//zhangfulong add 时间更新
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
		                  + (Uint32)(DSTIP[2]<<8)+ (Uint32)(DSTIP[3]) ; //IP地址
		  fpga_SetPort =  (Uint16)(DSTIP[6]<<8)+ DSTIP[9]; //数据通达
		  fpga_SetCH   =  DSTIP[7];//端口号
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
//返回值0，指示该帧采样数据不正常/或是解调制后数据不正确/或不是DF17的报文, ADSB_message变量不会被更新；
  //返回值1，指示该帧采样数据处理正常，但该帧不是位置报文或速度报文，仅更新ADSB_message变量用于上传原始报文；
  //        （续）用户可在ADSB_message数组中提取需要上报的原始88bit报文，按照字节存放

  //返回值2，指示该帧采样数据处理正常，并且该帧是位置报文，但是解算不成功，仅更新了ADSB_message变量，
  //        （续）用户可在ADSB_message数组中提取需要上报的原始88bit报文，按照字节存放
  //返回值3，指示该帧采样数据处理正常，并且该帧是位置报文，解算成功，更新了ADSB_message变量，更新了数据库，更新了上报位置用的airplane_location结构体变量
  //	    （续）用户可在ADSB_message数组中提取需要上报的原始88bit报文，按照字节存放
   //	    （续）用户可在airplane_location结构体变量中提取ICAO和位置信息上报宥ㄒ寮直淞慷ㄒ澹    
  //返回值4，指示该帧采样处理是速度报文，由于未知的原蛭唇馑愠鲇行У乃俣刃畔ⅲ龈铝薃DSB_message变量，（缺）
  //返回值5，指示该帧处理是速度ㄎ模馑愠晒Γ铝薃DSB_message变量更新了上报位置用的XXX数组，上报速度信息（缺）
  
//message_type返回值0，指示该帧采样数据不正常,或是解调制后数据不正确,或不是DF17的报文, ADSB_message变量不会被更新;此时无任何信息需要上传或保存，等待进入下一次采样数据的处怼?

//message_type返回值1，指示该帧采样数据处理正常，但该帧不是位置报文或速度报文，仅更新ADSB_message结构体变量用于上传原始报文；
//                   （可在ADSB_message结构体中提取需要上报的原始88bit报文，ADSB_message的定义方式见结固灞淞慷ㄒ濉?

//message_type返回值2，指示该帧采样数据处理正常，并且该帧是位置报文，但是经纬位置解算不成功，仅更新ADSB_message结构体变量用于上传原始报文；
//                   （续）用户可在ADSB_message结构体中提取需要上报的原始88bit报文，ADSB_message的定义方式见结构体变量定义。
//message_type返回值3，指示该帧采样数据处理正常，并且该帧是位置报文，经纬位置解算ADSB_message结构体变量，更新松媳ň澄恢糜玫腶irplane_location结构体变量；
//	                 （续）用户可在ADSB_message结构体中提取需要上报的原始88bit报文，ADSB_message的定义方式见结构体变量定义。
//	                 （续）用户可在airplane_location结构体变量中提取ICAO、时间和经纬位置信息上报, airplane_location的定义方式见结构体变量定义。
   
//message_type返回值4，指示该帧采样数据处理正常，并且该帧是速度ㄎ模捎诟髦衷蛭唇馑愠鲇行У乃俣刃畔ⅲ龈翧DSB_message结构体变量用于上传原始报文；
//	                 （续）用户可在ADSB_message结构体中提取需要上报的原始88bit报文，ADSB_message的定义方式见结构体变量定义。
//message_type返回值5，指示该帧采样数据处理正常，并腋弥∈撬俣缺ㄎ模俣冉馑愠晒Γ铝薃DSB_message变量，更新了上报速度信息用的airplane_velocity结构体变量；
//	                 （续）用户可在ADSB_message结构体中提取需要报的原始88bit报文，ADSB_message的定义方式见结构体变量定义。
//	                 （续）用户可在airplane_velocity结构体变量中提取ICAO、时间和速度位置信息上报, airplane_velocity的定义方式见结构体变量定义。
                       

/*******************************/
#if 1
//报文1.2 1400
//头(6)+类型(2)+任务(2)+包内计数(2)+时间(4)+有效目标(2)+ICAO（16*86）+填充(6)

//报文3  1400
//头(6)+计数(1)+遥测(13)+有效数(2)+ads(75*17)+icao(5*20)+填充(3)
void udp_sndPacket()  //任务4 发送两种报文和测试数据 
{
	SOCKET   							sudp = INVALID_SOCKET;
  	struct   sockaddr_in 				sin1,sin;
	int 	 							reuse = 1;
	int    								i=0,j=0;
    unsigned int 						recvsize = 1024*8;
	Uint8 pbuf1[1432];//测试数据
	Uint8 pbuf2[1432];//融合数据
	Uint8 pbuf3[1432];//ads数据
    Uint32 total_cnt=0;  //总计数 在
	Uint32 total_cnt_2=0;  //总计数 在
    Uint16 test_cnt=0;   // 包内计数
	Uint16 join_cnt=0;  //包内计数
    Uint8 ads_cnt=0 ;   //包内计数
    Uint8 ads_snd_t=0;    //判是否需要填充
    Uint8 rh_snd_t=0;    //判是否需要填充
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
	};//32字节--任务号(1)--数据类型(1)--帧计数（3）--数据通道（1）--预留（26）

	const Uint8 pack_samp1[6]={
	0x9e,0xda,0xa7,0x8b,0xF4,0xc5
    } ;///测试数据
    const Uint8 pack_samp2[6]={
	0x9e,0xda,0xa7,0x8b,0xF4,0xc5
    } ;//融合数据
    const Uint8 pack_samp3[6]={
	0x61,0x25,0x58,0x74,0x0b,0x3a
    } ;//ads数据

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
	pbuf1[1426]=0xaa;//尾部填充
	pbuf1[1427]=0xaa;
	pbuf1[1428]=0xaa;
	pbuf1[1429]=0xaa;
	pbuf1[1430]=0xaa;
	pbuf1[1431]=0xaa;
//--------------------2
    pbuf2[38] =0xdd; //190418
    pbuf2[39] =0xdd;

	pbuf2[1426]=0xaa;//尾部填充
	pbuf2[1427]=0xaa;
	pbuf2[1428]=0xaa;
	pbuf2[1429]=0xaa;
	pbuf2[1430]=0xaa;
	pbuf2[1431]=0xaa;
//-------------------3   
    for(i=0;i<75;i++)//报文3的填充
	{
    	memcpy(&(pbuf3[55+i*17]),pack_reserve,6);
    }
	pbuf3[1429]=0x55;//尾部填充
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
    if ( bind( sudp, (PSA) &sin1, sizeof(sin1) ) < 0 )  //源
        ErrorPro( sudp);

   while(1)
   {
		//zhangfulong add 时间更新
		UTC_time_real[0] = UTC_time_all[13];
		UTC_time_real[1] = UTC_time_all[12];
		UTC_time_real[2] = UTC_time_all[15];
		UTC_time_real[3] = UTC_time_all[14];
		UTC_time_real[4] = UTC_time_all[17];
		UTC_time_real[5] = UTC_time_all[16];
		//zhangfulong add 时间更新

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
      if(test_flag==1)  //发送测试数据
	  {  
//----------测试数据--------------------------	    
        pbuf1[1]=0x0d;//融合数据
		if(total_cnt<0xffffff)
		{
			total_cnt++;
		}
		else
		{
 		    total_cnt=0;
		}
       // memcpy(&(pbuf1[2]), &total_cnt, 3);//总计数
	    pbuf1[2] =(total_cnt>>16);
        pbuf1[3] =(total_cnt & 0xFFFF)>>8;
	    pbuf1[4] = (Uint8)(total_cnt & 0xFF);
//[41][42]任务号
        //pbuf1[40]=s_FPGA_YC.reserve[0];  //40-41
        //pbuf1[41]=s_FPGA_YC.reserve[1];  //
        pbuf1[40]=TestArray[9];  //40-41
        pbuf1[41]=TestArray[8];

      //  memcpy(&(pbuf1[42]), &test_cnt, 2);//测试计数
    //    pbuf1[42]=test_cnt<<8;// 190927
        pbuf1[42]=test_cnt>>8; //190927
        pbuf1[43]=(unsigned char)test_cnt&0xff;
		test_cnt++;
      //  memcpy(&(pbuf1[44]), &s_FPGA_YC.UTCtime, 4); //utc时间
	   // memcpy(&(pbuf1[44]), &test_time, 4);
 		hhtmp=htonl(test_time);
		memcpy(&(pbuf1[44]), &hhtmp, 4);
	
        pbuf1[49]=TestArray[14]; //个数
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
// [49][50]--个数    
//[51]开始共记录86组间隔16
     //   memcpy(&(pbuf1[49]),TestArray,200);//   怎么填充，待定
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
	  else  //发送两种网络报文
	  {
//------  融合数据 ----------------------
#if 1
      for(j=0;j<2;j++)   //2
	  {
		if(position_cnt>0)  //必须要有有效的位置信息
		{
		    pbuf2[1]=0x0d;//融合数据 
	        //memcpy(&(pbuf2[2]), &total_cnt, 3);//总计数
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
	//[41][42]任务号
			pbuf2[40]=s_FPGA_YC.reserve[0];  //
			pbuf2[41]=s_FPGA_YC.reserve[1];
	   //     memcpy(&(pbuf2[42]), &join_cnt, 2);//融合计数
	  //		pbuf2[42] =join_cnt<<8;//190927
        	pbuf2[42] =join_cnt>>8;//190927
			pbuf2[43] =(unsigned char)join_cnt&0xFF;//190418
  	        join_cnt++;
	        memcpy(&(pbuf2[44]), &s_FPGA_YC.UTCtime, 4); //utc时间 
	// [49][50]--个数  
	//[51]开始共记录86组间隔16
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
				for(i=rh_snd_t;i<86;i++)//pack_ads_nodata  //填充0x0   //190523
				{
				  memcpy(&(pbuf2[50+i*16]), pack_ads_nodata, 16); //51
				}
			}  
	#endif
		   	if(sendto(sudp,pbuf2, 1432, 0, (struct sockaddr *)&sin, sizeof(sin) )<0) //  还需确定发送条件
			{
			;
			//	udp_snd_lost_1++;//	  cmd_buf_test[4]=hh_udp_snd_cnt>>8;
			}
			//	hh_udp_snd_cnt++;
         }  //融合数据  //	if(position_cnt>0)  //必须要有有效的位置信息
	  }//for(j)
#endif
//------- ads数据    -------------------
#if 1
         for(j=0;j<14;j++)   //2
		 {
	         if(pulse_cnt_IP>0)  // 必须要有原始数据
			 {
		        pbuf3[1]=0x0c;//ads数据 
				if(total_cnt_2<0xffffff)
				{
					total_cnt_2++;
				}
				else
				{
		 		    total_cnt_2=0;
				}
			 //    memcpy(&(pbuf3[2]), &total_cnt, 3);//总计数
	            pbuf3[2] =(total_cnt_2>>16);
	            pbuf3[3] =(total_cnt_2 & 0xFFFF)>>8;
				pbuf3[4] = (Uint8)(total_cnt_2 & 0xFF);
				pbuf3[38]=ads_cnt;//包内计数
				ads_cnt++;
			    memcpy(&(pbuf3[39]), &s_FPGA_YC, 14); //遥测  //39-38 13-14
			
				if(pulse_cnt_IP>=75)
				{
					ads_snd_t =75;
			//[53] [54] --本次的个数
			        pbuf3[53]=0; 
			        pbuf3[54]= 75 ;  //这里需要判pulse_cnt_IP是否大于75

			//[60]开始11个ads报文，间隔是17
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
			//[53] [54] --本次的个数
					pbuf3[53]=0;
				    pbuf3[54]=pulse_cnt_IP;
			//[60]开始11个ads报文，间隔是17
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
						for(i=ads_snd_t;i<73;i++)//pack_ads_nodata  //填充0x0
						{
						  memcpy(&(pbuf3[61+i*17]), pack_ads_nodata, 11); //60
						}
						//73 74
	                    memcpy(&(pbuf3[61+73*17]), int_cnt, 11);  //网口推内部遥测
	                    memcpy(&(pbuf3[61+74*17]), &(int_cnt[11]), 11);
					}
					else
					{
						for(i=ads_snd_t;i<75;i++)//pack_ads_nodata  //填充0x0
						{
						  memcpy(&(pbuf3[61+i*17]), pack_ads_nodata, 11); //60
						}
					}
#endif                     
				}  
		#if 1
		//[1328]开始 100个字节填 5个ICAO （5*20）  //---1330
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
			        for(i=ads_snd_t;i<5;i++)//pack_ads_nodata  //填充0x0
					{
					  memcpy(&(pbuf3[1330+i*20]), pack_ads_nodata, 20);   //1328
					}
				}
#endif

		#endif
			    if(sendto( sudp,pbuf3, 1432, 0, (struct sockaddr *)&sin, sizeof(sin) )<0)//  还需确定发送条件
				{
					;
					//udp_snd_lost_2++;//	  cmd_buf_test[4]=hh_udp_snd_cnt>>8;
				}
	        }//ads数据   if(pulse_cnt_IP>0)  // 必须要有原始数据
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
//采用EMIF代替原来的网络模式
Uint8 pbuf3[5040 + 2];//ads数据 zhangfulong change
Uint8 pbuf2[5040 + 2];//融合数据

void EMIF_sndPacket()  //任务4 发送两种报文和测试数据 
{
//	SOCKET   							sudp = INVALID_SOCKET;
//  	struct   sockaddr_in 				sin1,sin;
//	int 	 							reuse = 1;
	int    								i=0,j=0;
//    unsigned int 						recvsize = 1024*8;
	//Uint8 pbuf1[1432];//测试数据
	//Uint8 pbuf2[1432 + 2];//融合数据
//	Uint8 pbuf3[1432];//ads数据
//	Uint8 pbuf3[5040];//ads数据 zhangfulong change
	Uint32 adsb_cnt = 0;//adsb原始数据包计数

    Uint32 total_cnt=0;  //总计数 在
	Uint32 total_cnt_2=0;  //总计数 在
    Uint16 test_cnt=0;   // 包内计数
	Uint16 join_cnt=0;  //包内计数
    Uint8 ads_cnt=0 ;   //包内计数
    Uint8 ads_snd_t=0;    //判是否需要填充
    Uint8 rh_snd_t=0;    //判是否需要填充
	Uint8 rh_snd_t_2=0;    //判是否需要填充
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
	};//32字节--任务号(1)--数据类型(1)--帧计数（3）--数据通道（1）--预留（26）

	const Uint8 pack_samp1[6]={
	0x9e,0xda,0xa7,0x8b,0xF4,0xc5
    } ;///测试数据
    const Uint8 pack_samp2[6]={
	0x9e,0xda,0xa7,0x8b,0xF4,0xc5
    } ;//融合数据
    const Uint8 pack_samp3[6]={
	0x61,0x25,0x58,0x74,0x0b,0x3a
    } ;//ads数据

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
	
		//zhangfulong add 时间更新
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

//------  融合数据 ---------------------- //  解算数据
		//zhangfulong 去掉 使用后面的生成融合解算数据
	
			//zhangfulong add start融合数据包
		for (i = 0; i < 1432 + 2; i++)//Uint8 pbuf2[1432 + 2];//融合数据
		{
			pbuf2[i] = 0;
		}

		//position_cnt = 150;
		//if(position_cnt>0)  //必须要有有效的位置信息
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
				//包头 7788AC8BF6E4
			pbuf2[0] = 0x77;
			pbuf2[1] = 0x88;
			pbuf2[2] = 0xAC;
			pbuf2[3] = 0x8B;
			pbuf2[4] = 0xF6;
			pbuf2[5] = 0xE4;
				//类型DDDD
			pbuf2[6] = 0xDD;
			pbuf2[7] = 0xDD;
				//UTC时间
			//memcpy(&(pbuf2[8]), &(UTC_time_real[0]), 6);
			//zhangfulong add 时间更新
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add 时间更新
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//将时间复制过去 4->6
				pbuf2[8] = UTC_time_real[0];
				pbuf2[9] = UTC_time_real[1];
				pbuf2[10] = UTC_time_real[2];
				pbuf2[11] = UTC_time_real[3];
				pbuf2[12] = UTC_time_real[4];
				pbuf2[13] = UTC_time_real[5];

				//累计发包计数
			pbuf2[14] = (Uint8)((total_cnt & 0xFF00) >> 8);
			pbuf2[15] = (Uint8)((total_cnt & 0x00FF));
			total_cnt = total_cnt + 1;
				//目标UTC时间
/*			memcpy(&(pbuf2[16]), &airplane_location_tmp.time , 4);//这个应该是引导目标的时间吧。
			memcpy(&(pbuf2[22]), &airplane_location_tmp.ICAO_address , 4);//ICAO
			memcpy(&(pbuf2[26]), &airplane_location_tmp.coordinate[1] , 4);//经度
			memcpy(&(pbuf2[30]), &airplane_location_tmp.coordinate[0] , 4);//纬度
			memcpy(&(pbuf2[34]), &airplane_speed_tmp.N_S_velocity , 4);//北速度
			memcpy(&(pbuf2[38]), &airplane_speed_tmp.E_W_velocity , 4);//东速度
			memcpy(&(pbuf2[42]), &airplane_speed_tmp.E_W_velocity , 4);//天速度
			memcpy(&(pbuf2[46]), &airplane_speed_tmp.E_W_velocity , 4);//高度
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
			if (ronghe_cnt_p> 144)//先位置
			{
				rh_snd_t = 144;
			}
			else
			{
				rh_snd_t = ronghe_cnt_p;
			}
			
			pbuf2[18] = (Uint8)((rh_snd_t & 0xFF00) >> 8); 	//包内计数
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

				//memcpy(&(pbuf2[18 +2 + i*34 + 10]), &airplane_location_static[position_rd].coordinate[1] , 4);//经度
				pbuf2[18 +2 + i*34 + 10] = (airplane_location_static[position_rd].coordinate[1] & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 10 + 1] = (airplane_location_static[position_rd].coordinate[1] & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 10 + 2] = (airplane_location_static[position_rd].coordinate[1] & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 10 + 3] = (airplane_location_static[position_rd].coordinate[1] & 0x000000FF);
				//memcpy(&(pbuf2[18 +2 + i*34 + 14]), &airplane_location_static[position_rd].coordinate[0] , 4);//纬度
				pbuf2[18 +2 + i*34 + 14] = (airplane_location_static[position_rd].coordinate[0] & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 14 + 1] = (airplane_location_static[position_rd].coordinate[0] & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 14 + 2] = (airplane_location_static[position_rd].coordinate[0] & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 14 + 3] = (airplane_location_static[position_rd].coordinate[0] & 0x000000FF);
				//memcpy(&(pbuf2[18 +2 + i*34 + 18]), &airplane_velocity_three_static[position_rd].N_S_velocity , 4);//北速度
				pbuf2[18 +2 + i*34 + 18] = (airplane_velocity_three_static[position_rd].N_S_velocity & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 18 + 1] = (airplane_velocity_three_static[position_rd].N_S_velocity & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 18 + 2] = (airplane_velocity_three_static[position_rd].N_S_velocity & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 18 + 3] = (airplane_velocity_three_static[position_rd].N_S_velocity & 0x000000FF);
				
				//memcpy(&(pbuf2[18 +2 + i*34 + 22]), &airplane_velocity_three_static[position_rd].E_W_velocity , 4);//东速度
				pbuf2[18 +2 + i*34 + 22] = (airplane_velocity_three_static[position_rd].E_W_velocity & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 22 + 1] = (airplane_velocity_three_static[position_rd].E_W_velocity & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 22 + 2] = (airplane_velocity_three_static[position_rd].E_W_velocity & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 22 + 3] = (airplane_velocity_three_static[position_rd].E_W_velocity & 0x000000FF);
				//memcpy(&(pbuf2[18 +2 + i*34 + 24]), &airplane_velocity_three_static[position_rd].VERT_velocity , 4);//天速度
				pbuf2[18 +2 + i*34 + 24] = (airplane_velocity_three_static[position_rd].VERT_velocity & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 24 + 1] = (airplane_velocity_three_static[position_rd].VERT_velocity & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 24 + 2] = (airplane_velocity_three_static[position_rd].VERT_velocity & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 24 + 3] = (airplane_velocity_three_static[position_rd].VERT_velocity & 0x000000FF);
				//memcpy(&(pbuf2[18 +2 + i*34 + 28]), &airplane_location_static[position_rd].altitude , 4);//高度
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
				//memcpy(&(pbuf2[18 +2 + i*34 + 10]), &airplane_location_static_ronghe[ronghe_cnt_p].coordinate[1] , 4);//经度
				pbuf2[18 +2 + i*34 + 10] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[1] & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 10 + 1] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[1] & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 10 + 2] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[1] & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 10 + 3] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[1] & 0x000000FF);
				//memcpy(&(pbuf2[18 + i*34 + 14]), &airplane_location_static_ronghe[ronghe_cnt_p].coordinate[0] , 4);//纬度
				pbuf2[18 +2 + i*34 + 14] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[0] & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 14 + 1] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[0] & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 14 + 2] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[0] & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 14 + 3] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[0] & 0x000000FF);
			//	memcpy(&(pbuf2[18 + i*34 + 18]), &airplane_velocity_three_static[position_rd].N_S_velocity , 4);//北速度
			//	memcpy(&(pbuf2[18 + i*34 + 22]), &airplane_velocity_three_static[position_rd].E_W_velocity , 4);//东速度
			//	memcpy(&(pbuf2[18 + i*34 + 26]), &airplane_velocity_three_static[position_rd].E_W_velocity , 4);//天速度
				//memcpy(&(pbuf2[18 +2 + i*34 + 30]), &airplane_location_static_ronghe[ronghe_cnt_p].altitude , 4);//高度
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

			//s速度
			if (rh_snd_t== 144)//位置占满，没有速度
			{
				rh_snd_t_2 = 0;
			}
			else if ( rh_snd_t + ronghe_cnt_v < 144)//速度可以全部写进去
			{
				rh_snd_t_2 = ronghe_cnt_v;
			}else//速度写进去部分
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
			//	memcpy(&(pbuf2[18 + i*34 + 10]), &airplane_location_static[ronghe_cnt_p].coordinate[1] , 4);//经度
			//	memcpy(&(pbuf2[18 + i*34 + 14]), &airplane_location_static[ronghe_cnt_p].coordinate[0] , 4);//纬度
				//memcpy(&(pbuf2[18+2 + i*34 + 18]), &airplane_velocity_three_static_ronghe[position_rd].N_S_velocity , 4);//北速度
				pbuf2[18 +2 + i*34 + 18] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].N_S_velocity & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 18 + 1] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].N_S_velocity & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 18 + 2] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].N_S_velocity & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 18 + 3] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].N_S_velocity & 0x000000FF);
				//memcpy(&(pbuf2[18+2 + i*34 + 22]), &airplane_velocity_three_static_ronghe[position_rd].E_W_velocity , 4);//东速度
				pbuf2[18 +2 + i*34 + 22] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].E_W_velocity & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 22 + 1] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].E_W_velocity & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 22 + 2] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].E_W_velocity & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 22 + 3] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].E_W_velocity & 0x000000FF);
				//memcpy(&(pbuf2[18+2 + i*34 + 26]), &airplane_velocity_three_static_ronghe[position_rd].E_W_velocity , 4);//天速度
				pbuf2[18 +2 + i*34 + 26] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].VERT_velocity & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 26 + 1] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].VERT_velocity & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 26 + 2] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].VERT_velocity & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 26 + 3] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].VERT_velocity & 0x000000FF);
			//	memcpy(&(pbuf2[18 + i*34 + 30]), &airplane_location_static[ronghe_cnt_p].altitude , 4);//高度
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
			pbuf2[18] = (Uint8)((rh_snd_t & 0xFF00) >> 8); 	//包内计数
			pbuf2[19] = (Uint8)((rh_snd_t & 0x00FF));


			for(i = rh_snd_t; i < 144; i++)
			{
				memcpy(&(pbuf2[18 +2 + i*34]), pack_ads_nodata , 34);
					;//pack_ads_nodata
			}

			for (i = 4913 +2; i < 5040; i++)//填充4914-5040的AA
			{
				pbuf2[i] = 0xAA;
			}
			


				//zhangfulong add end
				//融合数据需要转成EMIF口输出-zhangfulong
	
			for (i = 0; i< 10; i++)//504 = 0x1F8
			{
				TaskSleep(1);
				EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
				pEDMA3CC_PaRAM->OPT = 0x00130104;//16 
				pEDMA3CC_PaRAM->SRC =  (unsigned char *)(&pbuf2[i * 0x1F8]);  //ce4
				//	pEDMA3CC_PaRAM->SRC =  (unsigned char *)XSH_Array2;  //ce4	
				//pEDMA3CC_PaRAM->BCNT_ACNT =0x13B00001;//0x13B0 = 5040  //1280*4	//0x003d0001D′μ?FPGA??μ??·A0F	//51
				//pEDMA3CC_PaRAM->BCNT_ACNT =0x02000001;//一次发512个，分10次发，9次512
				pEDMA3CC_PaRAM->BCNT_ACNT =0x01F80001;
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
				pEDMA3CC_PaRAM->DST = (unsigned char *)(DSP_TO_FPGA_RONGHE_PBUF2 + 0x1F8 * i);//目标地址需要修改，FPGA方面提供
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
				//类型DDDD
			pbuf2[6] = 0xDD;
			pbuf2[7] = 0xDD;
				//UTC时间
			//memcpy(&(pbuf2[8]), &(UTC_time_real[0]), 6);
			//zhangfulong add 时间更新
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add 时间更新
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//将时间复制过去 4->6
				pbuf2[8] = UTC_time_real[0];
				pbuf2[9] = UTC_time_real[1];
				pbuf2[10] = UTC_time_real[2];
				pbuf2[11] = UTC_time_real[3];
				pbuf2[12] = UTC_time_real[4];
				pbuf2[13] = UTC_time_real[5];
				//累计发包计数
			pbuf2[14] = (Uint8)((total_cnt & 0xFF00) >> 8);
			pbuf2[15] = (Uint8)((total_cnt & 0x00FF));
			total_cnt = total_cnt + 1;
				//目标UTC时间
			
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
			for (i = 4913 + 2; i < 5040; i++)//填充4914-5040的AA
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
				//pEDMA3CC_PaRAM->BCNT_ACNT =0x13B00001;//0x13B0 = 5040  //1280*4	//0x003d0001D′μ?FPGA??μ??·A0F	//51
				//pEDMA3CC_PaRAM->BCNT_ACNT =0x02000001;//一次发512个，分10次发，9次512
				pEDMA3CC_PaRAM->BCNT_ACNT =0x01F80001;
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
				pEDMA3CC_PaRAM->DST = (unsigned char *)(DSP_TO_FPGA_RONGHE_PBUF2 + 0x1F8 * i);//目标地址需要修改，FPGA方面提供
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
			}

		}
      

//------- ads数据    ------------------- //   原始报文

				//zhangfulong add start 原始报文生成 写到pbuf3里面
		//if(pulse_cnt_IP>0)
		//{
		//Uint8 pbuf3[5040 + 2];初始化
//		for (i = 0; i < 5040 + 2; i++)
//		{
//			pbuf3[i] = 0;
//		}
			//包头612558740b3a
		pbuf3[0] = 0x61;
		pbuf3[1] = 0x25;
		pbuf3[2] = 0x58;
		pbuf3[3] = 0x74;
		pbuf3[4] = 0x0B;
		pbuf3[5] = 0x3A;
		if(pulse_cnt_IP>0)  // 必须要有原始数据
	 	{
				//包计数
//			pbuf3[6] = (Uint8) ((adsb_cnt & 0xFF00)>>8);
//			pbuf3[7] = (Uint8) (adsb_cnt & 0x00FF);
			pbuf3[6] = (Uint8) (adsb_cnt & 0x00FF);
			adsb_cnt = adsb_cnt + 1;//自加1 计数
			//memcpy(&(pbuf3[7]),&(UTC_time_real[0]),6);//时间 8 9 10 11
			//zhangfulong add 时间更新
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add 时间更新
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//将时间复制过去 4->6
				pbuf3[7] = UTC_time_real[0];
				pbuf3[8] = UTC_time_real[1];
				pbuf3[9] = UTC_time_real[2];
				pbuf3[10] = UTC_time_real[3];
				pbuf3[11] = UTC_time_real[4];
				pbuf3[12] = UTC_time_real[5];
				
				
		        	//pbuf3[1]=0x0c;//ads数据 
			if(total_cnt_2<0xffffff)
			{
				total_cnt_2++;
			}
			else
			{
 		   		total_cnt_2=0;
			}
			 
			ads_cnt++;
			    //memcpy(&(pbuf3[39]), &s_FPGA_YC, 14); //遥测  //39-38 13-14


			if(pulse_cnt_IP>=295)
			{
				ads_snd_t =295;
			}
			else
			{
				ads_snd_t =pulse_cnt_IP;
			}
			//[53] [54] --本次的个数
	        pbuf3[14 - 1]=(Uint8)((ads_snd_t & 0x00FF) >> 8); 
	        pbuf3[15 - 1]= (Uint8) (ads_snd_t & 0x00FF) ;  //这里需要判pulse_cnt_IP是否大于75

			//[60]开始11个ads报文，间隔是17
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
			for (i = 0; i < 27; i++)//填充
			{
				if (15 + 295 * 17 + i < 5040)
				{
					pbuf3[15 + 295 * 17 + i] = 0x55;
				}
			}
					
				

				//zhangfulong add end 原始报文生成 写到pbuf3里面
				//ADSB数据需要转成EMIF口输出-zhangfulong
			
			for (i = 0; i< 10; i++)
			{
				TaskSleep(1);
				EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
				pEDMA3CC_PaRAM->OPT = 0x00130104;//16 
				pEDMA3CC_PaRAM->SRC =  (unsigned char *)(&pbuf3[i * 0x1F8]);  //ce4
				//	pEDMA3CC_PaRAM->SRC =  (unsigned char *)XSH_Array2;  //ce4	
				//pEDMA3CC_PaRAM->BCNT_ACNT =0x13B00001;//0x13B0 = 5040  //1280*4	//0x003d0001D′μ?FPGA??μ??·A0F	//51
				pEDMA3CC_PaRAM->BCNT_ACNT =0x01F80001;//一次发512个，分10次发，9次512
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
				pEDMA3CC_PaRAM->DST = (unsigned char *)(DSP_TO_FPGA_ADSB_PBUF3 + 0x1F8 * i);//目标地址需要修改，FPGA方面提供
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
			}

		}//end  if(pulse_cnt_IP>0)
		else
		{
			//pbuf3[6] = (Uint8) ((adsb_cnt & 0xFF00)>>8);
			pbuf3[6] = (Uint8) (adsb_cnt & 0x00FF);
			//pbuf3[7] = (Uint8) (adsb_cnt & 0x00FF);
			adsb_cnt = adsb_cnt + 1;//自加1 计数
			//memcpy(&(pbuf3[8-1]),&(UTC_time_real[0]),6);//时间 8 9 10 11
			//zhangfulong add 时间更新
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add 时间更新
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//将时间复制过去 4->6
				pbuf3[7] = UTC_time_real[0];
				pbuf3[8] = UTC_time_real[1];
				pbuf3[9] = UTC_time_real[2];
				pbuf3[10] = UTC_time_real[3];
				pbuf3[11] = UTC_time_real[4];
				pbuf3[12] = UTC_time_real[5];
				

			ads_snd_t = 0;
			//[53] [54] --本次的个数
	        pbuf3[14-1]=(Uint8)((ads_snd_t & 0x00FF) >> 8); 
	        pbuf3[15-1]= (Uint8) (ads_snd_t & 0x00FF) ;  //这里需要判pulse_cnt_IP是否大于75

			//[60]开始11个ads报文，间隔是17
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
				//pEDMA3CC_PaRAM->BCNT_ACNT =0x13B00001;//0x13B0 = 5040  //1280*4	//0x003d0001D′μ?FPGA??μ??·A0F	//51
				pEDMA3CC_PaRAM->BCNT_ACNT =0x01F80001;//一次发512个，分10次发，9次512
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
				pEDMA3CC_PaRAM->DST = (unsigned char *)(DSP_TO_FPGA_ADSB_PBUF3 + 0x1F8 * i);//目标地址要修改，FPGA方面提供
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
			}
			
			
		}

			    
	   //     }//ads数据   if(pulse_cnt_IP>0)  // 必须要有原始数据
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
void get_Pulse()  //任务1： 1280->112
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
//考虑一下处理的对象是什么？--IP_DATA[g_u32IpSendIndex];
//应该是个数组，只要非空就可以处理--SND_Catch_Program_Full>0
//还需要维护一下指针 --g_u32IpSendIndex
//输出是什么？
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
				  data_fifo= IP_DATA[g_u32IpSendIndex];	      //data_fifo= first_fifo_1;//传递一下
	              data_head=0;
			#if 1
				  for(i=0;i<112;i++) //初始化一下变量
				  {
				  	  pulse_amp[i]=0;
					  confi[i]=0;
				  }
            #endif
			#if 1
				  for(i=0;i<24;i++)//?????为什么？？？？
				  {	
				  	  correct[i]=0; //  correct[i]=1;   //190426
				  } 
           #endif
			      for(i=0;i<10;i++)//2  淹?
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
			      if(data_head==1) //有头
			      {
			        memset(temp,0,sizeof(temp));//lh,必须清0
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
			        sumoba_pt=data_fifo+data_begin;//ADS-B报头起始位置指针 
			        ampoba_pt=data_fifo+data_begin+80;//ADS-B数据起始位置指针
    				//C62_enableIER(1<<9|1<<8);//9--fpga中断 8--edma中断
    				refer_amp=com_refer(sumoba_pt);//计算幅度的参考值
    		        //此功率检测适用于对数检波后，线性解调制后可以蚧弥屑?个样点值直接平均得到（需要验证）
    				code_confi(refer_amp,ampoba_pt,pulse_amp,confi);//计算代码位和置信度
		
    				crc_check_flag=crc_check(pulse_amp,correct);//CRC校验
    				if(crc_check_flag==0)  	  
					{
					   	plane_ok=0;//此变量为0表示飞机ADS-B广播信息接收正确
					}
					else
					{
				   		plane_ok=err_corr_8bit(confi,pulse_amp,correct,syndrome);//纠错
					}
	            	t5=CLK_gethtime();
	            	//gpio7_set_0(); 
			        if(plane_ok==0)//说明解算成功
					{ 
				#if 1	
						for(i=0;i<88;i++)  //对模拟源进行检测  
				        {
							if(pulse_amp[i]!=pulse_amp_bak[i])
							{
								int_cnt[7]++;
								break;				
							}
						}
				#endif
#if 1
                     //用于剔除全0的数据
					     hh_message=0;
					     for(i=0;i<5;i++)  //前5位?，基本可能是全0
					     {   
					        hh_message=hh_message|(pulse_amp[i]<<(4-i));//取DF位
						 }
						 if(hh_message!=0)//
						 {
						  //   hh_message=1;
						  //   continue;
						// }
#endif						
							int_cnt[3]++;
							int_cnt_3++;
	//-------------继续解析
							mmCopy(pulse[pulse_wr],pulse_amp,112<<1);//
							//zhangfulong add 时间更新进来的功能
							//zhangfulong add 时间更新  没有用 要在其他地方使用更新，避免没传完就用
							UTC_time_real[0] = UTC_time_all[13];
							UTC_time_real[1] = UTC_time_all[12];
							UTC_time_real[2] = UTC_time_all[15];
							UTC_time_real[3] = UTC_time_all[14];
							UTC_time_real[4] = UTC_time_all[17];
							UTC_time_real[5] = UTC_time_all[16];
							//s_FPGA_YC.time = 0;
							time_tmp_01 = (long)(UTC_time_real[0]<<40) + (long)(UTC_time_real[1]<<32) + (long)(UTC_time_real[2]<<24) + (long)(UTC_time_real[3]<<16) + (long)(UTC_time_real[4]<<8) + (long)(UTC_time_real[5]);
							time_tmp_01 = ((long)(time_tmp_01/10000)) & 0x00000000FFFFFFFF;
							//zhangfulong add 时间更新
							s_FPGA_YC.UTCtime = (int)(time_tmp_01);
							//
							//pulse_time[pulse_wr]=s_FPGA_YC.UTCtime;//190521
							//zhangfulong add time change start
							//zhangfulong add 时间更新
							UTC_time_real[0] = UTC_time_all[13];
							UTC_time_real[1] = UTC_time_all[12];
							UTC_time_real[2] = UTC_time_all[15];
							UTC_time_real[3] = UTC_time_all[14];
							UTC_time_real[4] = UTC_time_all[17];
							UTC_time_real[5] = UTC_time_all[16];
							//zhangfulong add 时间更新

							pulse_time[pulse_wr]= (UTC_time_real[2]) + (UTC_time_real[3] << 8) +(UTC_time_real[4] << 16) + (UTC_time_real[5]<< 24);
							//pulse_time[pulse_wr]= (int) (((UTC_time_real[0]) + (UTC_time_real[1] << 8) +(UTC_time_real[2] << 16) + (UTC_time_real[3]<< 24) + (UTC_time_real[4]<< 32) + (UTC_time_real[5]<< 40))/10000);

							//zhangfulong add time change end

							if(pulse_cnt<pulse_BUFLEN-2)
							{
								pulse_cnt++;
							}
							else  //缓冲满 （不应该满）
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
	//-------------给网络准备的
	                 	//	mmCopy(pulse_IP[pulse_wr_IP],pulse_amp,112);//
					    	get_pulse_TOByte(pulse_IP[pulse_wr_IP],pulse_amp);
						    if(pulse_cnt_IP<pulse_BUFLEN_IP-2)
							{
								pulse_cnt_IP++;
						    }
							else  //缓冲满 （感觉肯定会满?//是否梢允实笨髉ulse_BUFLEN_IP
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
			        }// if(plane_ok==0)//说明解算成?
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
void get_PosV()  //任务2： 112->解算位置
{
  unsigned short int dp;
  unsigned short int position_ok,velocity_ok;
  unsigned short   * data_fifo;
  unsigned int hhtmp;
  unsigned  IER_tmp=0;
  while(1)
  {
//考虑一下data_pro（）处理的对象是什么？---pulse[]
//应该是个数组，只要非空就可以处理  --pulse_cnt非空
//还需要维护一下指针  --pulse_rd
//处理完的数据怎么存放？
    
     if(pulse_cnt>0)
     {
        while(1)
		{
		    if(pulse_cnt==1)//1
		    {
			   TaskSleep(1);
		    }
			//转换一下
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
			  if(position_ok==2)	//zhangfulong add 20201012 未知问题更改：位置解算不写进结果
			  //if (0)
			  {
			   	pos_2_cnt++ ;
			  }
			  else
			  {
			    pos_3_cnt++; 
			    t2=CLK_gethtime();
			//	gpio7_set_0();
/*            //获取airplane_location
              airplane_location.time;
              airplane_location.ICAO_adress_with_mark;
			  airplane_location.position.coordinate[0];
			  airplane_location.position.coordinate[1];*/
              //--融合 
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
				else  //缓冲满 （应该满）
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
              //--稀疏化
		        memcpy(position_xsh[position_wr_xsh], &airplane_location, 20);//190418
                memcpy(&position_xsh_ICAO[position_wr_xsh],&airplane_location,4);  //icao 190410
				//稀疏化给新的赋值
				airplane_location_XSH_static[position_wr_xsh].altitude = airplane_location.altitude;
				airplane_location_XSH_static[position_wr_xsh].coordinate[0] = airplane_location.coordinate[0];
				airplane_location_XSH_static[position_wr_xsh].coordinate[1] = airplane_location.coordinate[1];
				airplane_location_XSH_static[position_wr_xsh].ICAO_address = airplane_location.ICAO_address & 0x00FFFFFF;
				airplane_location_XSH_static[position_wr_xsh].time = airplane_location.time;

				
				//zhangfulong add start将数据存入全局的里面 airplane_location_static新定义1500全局
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
				else  //缓冲满 Ω寐?
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
	      	  //返回值2，指示该帧处理是位置报文，但是解算不成功，仅更新了ADSB_message数组，以及数据库，未解算出有效的位置信息
	          //返回值3，指示该帧处理是位置ㄎ模馑愠晒Γ铝薃DSB_message数组，更新了数据库，更新了上报位置用的airplane_location结构体变	                                       
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
 			//	memcpy(&velocity[velocity_wr][8],&(airplane_velocity_three.E_W_velocity), 12);//  临时的，因为速度的变量类筒欢?0190422
				

				//张隆 add XSH 速度
				airplane_velocity_three_XSH_static[position_wr_xsh_v].ICAO_address = (airplane_velocity_three.ICAO_address) & 0x00FFFFFF;
				airplane_velocity_three_XSH_static[position_wr_xsh_v].E_W_velocity = airplane_velocity_three.E_W_velocity;
				airplane_velocity_three_XSH_static[position_wr_xsh_v].N_S_velocity = airplane_velocity_three.N_S_velocity;
				airplane_velocity_three_XSH_static[position_wr_xsh_v].time = airplane_velocity_three.time;
				airplane_velocity_three_XSH_static[position_wr_xsh_v].VERT_velocity = airplane_velocity_three.VERT_velocity;
				if(position_wr_xsh_v<position_BUFLEN_xsh-2)
				{
					position_wr_xsh_v++;
				}
				else  //缓冲满 Ω寐?
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

 			//	memcpy(&velocity[velocity_wr][8],&(airplane_velocity_three.E_W_velocity), 12);//  临时的，因为速度的变量类筒欢?0190422
				
				
				if(velocity_cnt<velocity_BUFLEN-2)
				{
			     	velocity_cnt++;
				}
				else  //缓冲满 （应该满）
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
		       //返回值4，指示该帧处理是速度报文，但是解算不成功，仅更新了ADSB_message数组，由于未知的原因未解算出有效的速度信息
			   //返回值5，指示该帧处理是速度报文，解算成功，更新了ADSB_message数组，更新了上报位置用的XXX数组，上报位置信息	                                       
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
   Uint8 i=0; //50代表没找到
   Uint8 ret=FIND_LOST;
   if(end_pos[num]>start_pos[num])  //1段
   {
     for(i=start_pos[num];i<=end_pos[num];i++)
     {
        if(icao==trans_Record[i].ICAO_adress_with_mark)
		{
		    ret=i;
		    break;//找到男号 
		}
     }
   }
   else  //2段
   {
      for(i=0;i<=end_pos[num];i++)
      {
         if(icao==trans_Record[i].ICAO_adress_with_mark)
	     { 
		     ret=i;
         	 break;//找到的序号
		 }
      }
      for(i=start_pos[num];i<FIND_RANGE;i++)
      {
        if(icao==trans_Record[i].ICAO_adress_with_mark)
		{
		    ret=i;
		    break;//找到的序号 
		}
      }
   } 
   return ret;
}
/*****************************************/
Uint32 XSH_last_ICAOnumber[4] = {0};
int find_in_lastICAOnumber(int ICAO)
{
	//本函数在XSH_last_ICAOnumber里面搜索指定的ICAO号要是有返回1，否则没有的话返回0
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

void xsh_2_task()  //20190211  任务3  稀疏化
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
    const Uint8 pack_xsh_nodata[20]={0};  //填充0
    memcpy(XSH_Array, xsh_samp, 8);//
//初始化----
    for(i=0;i<FIND_RANGE;i++)
	{
		trans_Record[i].ICAO_adress_with_mark=0xffffffff;
		start_pos[i]=(i+5+FIND_RANGE-1)%FIND_RANGE;
    	end_pos[i]=(i+FIND_RANGE-1)%FIND_RANGE;
	}

//做个帧头
 	while( 1 )
	{
		//zhangfulong add 时间更新
		UTC_time_real[0] = UTC_time_all[13];
		UTC_time_real[1] = UTC_time_all[12];
		UTC_time_real[2] = UTC_time_all[15];
		UTC_time_real[3] = UTC_time_all[14];
		UTC_time_real[4] = UTC_time_all[17];
		UTC_time_real[5] = UTC_time_all[16];
		//zhangfulong add 时间更新
		for (k = 0; k < 114; k++)
		{
			XSH_Array[k] = 0;
		}

		flag=5; //
		XSH_Array[4]++; //计数
	//	XSH_Array2[23]++;
	//	XSH_Array2[23+16]++;
	//  XSH_Array2[23+16+16]++;
	//	XSH_Array2[23+16+16+16]++;
		while((position_cnt_xsh<20)&&(time_delay<800))  //累计一定的解析数据
		{
		  TaskSleep(1);   //累计误差需要扣除
		  time_delay++;
		}
#if 1
	    //1 维护一下trans_Record
		tR_add_cnt=0;
        while(position_cnt_xsh>0)  // 有数可以解
		//if(position_cnt_xsh>0)
		{
		 	fi=find_ICAO_2(sum_cnt,(position_xsh_ICAO[position_rd_xsh]&0x00ffffff));
#if 0			//张富隆add 去掉原来的填充，重新写
            if(FIND_LOST==fi) //50  没找到
	        {
                 //插在这里
				 trans_Record[sum_cnt].ICAO_adress_with_mark=position_xsh_ICAO[position_rd_xsh] & 0x00ffffff;

			     trans_Record[sum_cnt].time=(Uint32)(position_xsh[position_rd_xsh][4]<<24)
			            +(Uint32)(position_xsh[position_rd_xsh][5]<<16)
			            +(Uint32)(position_xsh[position_rd_xsh][6]<<8)
			            +position_xsh[position_rd_xsh][7];
		
				 trans_Record[sum_cnt].flag=1; 
			     memcpy(&(XSH_Array[20+tR_add_cnt*20]), position_xsh[position_rd_xsh], 20); //20

	              //维护指针 
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
            else  //找到  --这环⒘?
		    {
	           ; //这里会舍弃一包//后面怎么完?
		       trans_Record[fi].time=(Uint32)(position_xsh[position_rd_xsh][4]<<24)
		            +(Uint32)(position_xsh[position_rd_xsh][5]<<16)
		            +(Uint32)(position_xsh[position_rd_xsh][6]<<8)
		            +position_xsh[position_rd_xsh][7];  //更新时间  --其实没有任何意义
			  //维护指针
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
			if(FIND_LOST==fi) //50  没找到
	        {
                 //插在这里
				 trans_Record[sum_cnt].ICAO_adress_with_mark=position_xsh_ICAO[position_rd_xsh] & 0x00ffffff;

			     trans_Record[sum_cnt].time=(Uint32)(position_xsh[position_rd_xsh][4]<<24)
			            +(Uint32)(position_xsh[position_rd_xsh][5]<<16)
			            +(Uint32)(position_xsh[position_rd_xsh][6]<<8)
			            +position_xsh[position_rd_xsh][7];
		
				 trans_Record[sum_cnt].flag=1; 
			     //memcpy(&(XSH_Array[20+tR_add_cnt*20]), position_xsh[position_rd_xsh], 20); //20
				//填充数据
				//确保不重复
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
				else//重复了写0
				{
					memcpy(&(XSH_Array[tR_add_cnt * 12 + 0]), pack_xsh_nodata, 4);
					memcpy(&(XSH_Array[tR_add_cnt * 12 + 4]), pack_xsh_nodata, 4);
					memcpy(&(XSH_Array[tR_add_cnt * 12 + 8]), pack_xsh_nodata, 4);
					XSH_last_ICAOnumber[tR_add_cnt] = 0;
					
				}
			
				//记录本次ICAO号
				//XSH_Array[] = ;
				
	              //维护指针 
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
            else  //找到  --这包不发了
		    {
	           ; //这里会舍弃一包//后面怎么完?
		       trans_Record[fi].time=(Uint32)(position_xsh[position_rd_xsh][4]<<24)
		            +(Uint32)(position_xsh[position_rd_xsh][5]<<16)
		            +(Uint32)(position_xsh[position_rd_xsh][6]<<8)
		            +position_xsh[position_rd_xsh][7];  //更新时间  --其实没有任何意义
			  //维护指针
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
//填充	
		if(tR_add_cnt<3)//是否要填充
		{
		    for(i=tR_add_cnt;i<3;i++)//pack_ads_nodata  //填充0x0
			{
			    //memcpy(&(XSH_Array[20+i*20]), pack_xsh_nodata, 20); //20
				memcpy(&(XSH_Array[i * 12 + 8]), pack_xsh_nodata, 12);
				sum_cnt++;
			}
		}
		//填个速度
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
		//稀疏化数据更改，前面加0，时间，包计数，后面加引导数据
		/*for (i = 51; i > 12; i--)//数据搬过去
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
		//memcpy(&(XSH_Array[0]),&(UTC_time_real[0]),4+ 2);//将时间复制过去

				//zhangfulong add 时间更新
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add 时间更新
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//将时间复制过去 4->6
				XSH_Array[0] = UTC_time_real[0];
				XSH_Array[1] = UTC_time_real[1];
				XSH_Array[2] = UTC_time_real[2];
				XSH_Array[3] = UTC_time_real[3];
				XSH_Array[4] = UTC_time_real[4];
				XSH_Array[5] = UTC_time_real[5];
				//XSH 时间
		XSH_Array[6] = XSH_bao_cnt;//计数
		XSH_bao_cnt = (XSH_bao_cnt + 1) & 0xFF;//
		//XSH_Array[7] = (up_commend_count) & 0xFF;
		XSH_Array[7] = (yaokongcmd_all_data.chelue_new) & 0xFF;
		//XSH_Array[59] = 0;

		//60开始加引导数据
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
		pEDMA3CC_PaRAM->BCNT_ACNT =0x005E0001; //1280*4	//0x003d0001D′μ?FPGA??μ??·A0F	//51
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)DSP_TO_FPGA_XSH;
	    int_cnt[10]++;//
		EDMA3CC_ESRH = 0x00010000;  //???ˉ
   	 	if(sum_cnt>=FIND_RANGE) //这个边界危险
			sum_cnt=0;
        if(time_delay<2000)
   	   	{
   	   		 TaskSleep(2000-time_delay); //1s
			 time_delay=0;
		}
		else
		{
		    time_delay=0; //这个分支
		}
		//printf("XSH count is %d \n",XSH_bao_cnt);
	}//while(1)
}

int V_get(int ICAO)// 输入ICAO号，输出返回一个序号，最新的速度的所在位置在airplane_velocity_three_static里的
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
			if(result < 0)//第一次进
			{
				result = i;
			}
			else//更新
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
int P_get(int ICAO)//输入ICAO号，输出返回一个序号，最新的位置的所在位置在airplane_location_static里的
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
			if(result < 0)//第一次进
			{
				result = i;
			}
			else//更新
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

//下面两个在最开始定义
//	struct location_struct airplane_location_tmp = {0};//用来记录临时的命中的ICAO号等信息 位置
//	struct speed_struct_three airplane_speed_tmp = {0};//用来记录临时的命中的ICAO号等信息 速度
	struct location_struct_withnumber
	{
	  unsigned int ICAO_address;
	  unsigned int time;
	  int coordinate[2];//正式上报时候使用int型正式上报时候使用int(1E-7为单位)。
	  int altitude;
	  int number;
	};
	struct location_struct_withnumber airplane_location_array_tmp[position_BUFLEN] = {0};//在自由搜索下存储在区域内的

	int location_arrary[1500];//该变量用来记录各个策略的输出结果的位置，每次开始前都应该赋值成-1状态
//新加策略函数，后面按照要求调用关系，调用函数的顺序   仅自由搜索使用
// 返回 在airplane_location_array_tmp数组中的一个位置值 记录location_arrary[1500]中
//输入 在airplane_location_array_tmp中的count值，前面有有多少有用的
void chelue_FF(const int count)//更新频率最快的
{
	int i,j,k;
	int location_arrary_tmp[1500] = {-1};//临时记录
	int max_location;
	int count_tmp = 0;
	for (i = 0; i < count; i++)
	{
		for (j = i; j < count ; j++)
		{
			if((airplane_location_array_tmp[i].ICAO_address == airplane_location_array_tmp[j].ICAO_address) && (airplane_location_array_tmp[i].ICAO_address != 0) )
			{
				//第一次肯定会进来
				airplane_location_array_tmp[i].number = airplane_location_array_tmp[i].number + 1;
			}
		}
	}
	//上面循环将每一个ICAO号出现的次数写入airplane_location_array_tmp[count].number里面
	//频率的可以先执行，反正也是写到
	if (location_arrary[0] == -1)//第一次进
	{
		//开始寻找出现频率最多的飞机
		max_location = 0;
		for (i = 1; i < count; i++)
		{
			if ((airplane_location_array_tmp[i].number > airplane_location_array_tmp[max_location].number)&& (airplane_location_array_tmp[i].ICAO_address != 0))
			{
				//这一条的出现次数最多，更新max_location位置
				max_location = i;
				//清空之前记录的位置值
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
				//两个相等，需要count_tmp上加一个。
				location_arrary[count_tmp] = i;
				count_tmp = count_tmp + 1;	
			}
			else//这个出现的次数没有目前最大值多，不更新
			{
				;
			}
		}
		//经过上面循环，将出现频率最大的序号写进location_arrary[] 数组里面
		
	}
	else if ((location_arrary[0] != -1) && (location_arrary[1] == -1))
	{
		//能进来说明数组里面只有一个值，不执行策略，直接返回
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
		for (i = 0; i < count; i++)//遍历报文数据库，
		{
			for (j = 1; j < count_tmp; j++)//遍历上一个策略的结果
			{
				if (airplane_location_array_tmp[i].ICAO_address == airplane_location_array_tmp[location_arrary[j]].ICAO_address)
				//这个飞机ICAO号范围在上一个策略内
				{
					if (airplane_location_array_tmp[i].number > airplane_location_array_tmp[location_arrary[max_location]].number)
					{
						//出现次数更多
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
		//能进来说明其他策略给出了不止一个ICAO号的结果，下面比较出现频率
		printf("chelue_FF get in !!!!!\n");
		//其他策略在前面，只可能有一个结果给进来
		return;
	}
	;
}

void chelue_AA(const int count)//高度最高
{
	int i, j, k;
	int max_location = 0;
	int count_tmp = 0;
	if (location_arrary[0] == -1)//第一次进
	{
		max_location = 0;
		for (i = 1; i < count; i++)
		{
			if ((airplane_location_array_tmp[max_location].altitude < airplane_location_array_tmp[i].altitude) && (airplane_location_array_tmp[i].ICAO_address != 0) )
			{
				//更新的飞机高度更高
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
				//两个一样高
				location_arrary[count_tmp] = i;
				count_tmp = count_tmp + 1;

				//printf("chelue_AA error two altitude is same !!! \n");
				;
			}else
			{
				//新的飞机没有之前的高，不更新
				;
			}
			;
		}
		//循环之后，将结果写到location_arrary[]数组里面，然后可以返回了
		return;
	}
	else if ((location_arrary[0] != -1) && (location_arrary[1] == -1))
	{
		//进来时候结果只有一个，直接返回
		return;
	}
	else
	{
		//能进来说明有两个以上飞机，进来
		count_tmp = 0;
		i = 0;
		while (location_arrary[i] >= 0)
		{
			i++;
			count_tmp++;
		}
		//上面统计进来了几个数据
		//下面循环将进来的位置调整成每个最大的高度的飞机
		for (i = 0; i < count_tmp; i++)
		{
			//location_arrary[i]
			for(j = 0; j < 1500; j++)
			{
				if ((airplane_location_array_tmp[location_arrary[i]].ICAO_address ==  airplane_location_array_tmp[j].ICAO_address) && (airplane_location_array_tmp[j].ICAO_address != 0))
				{
					//两个ICAO号一样，;
					if (airplane_location_array_tmp[location_arrary[i]].altitude <  airplane_location_array_tmp[j].altitude)
					{
						//高度更高，更新;
						location_arrary[i] = j;
					}
					else
					{
						;
					}
				}
			}
		}
		//经过上面循环后就是每一个最高的位置序号了,面寻找最高的
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

void chelue_DD(const int count)//时间最新
{
	int i, j, k;
	Uint32 time_tmp = 0;
	int max_location = 0;
	int count_tmp = 0;
	if (location_arrary[0] == -1)//第一次进
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
			{//两个一样时间
				location_arrary[count_tmp] = i;
				count_tmp = count_tmp + 1;
			}
			else
			//新的没有之前的时间新，不更新
			{
				;
			}
		}
		location_arrary[0] = max_location;
		return;
	}
	else if ((location_arrary[0] != -1) && (location_arrary[1] == -1))
	{
		//直接跳过，
		return;
	}
	else
	{
		//能进来不止一个前面的约束条件;
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
					//两个ICAO号一样，;
					if (airplane_location_array_tmp[location_arrary[i]].time <  airplane_location_array_tmp[j].time)
					{
						//高度更高，更新;
						location_arrary[i] = j;
					}
					else
					{
						;
					}
				}
			}
		}
		//上面更新了所有的的位置都是最新的
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

void lead_tesk()	//创建引导的任务
{
	
	int i, j, k;
	int count = 0;//记录有用的数据的个数
	int isfirst = 0;//0表示第一次，大于0表示不是第一次
	int V_time_last = 0;//记录上一次的时间，和这次比是否有更新
	int P_time_last = 0;//记录上一次的时间，和这次比是否有更新
	int ICAO_last = 0;//记录上一次的ICAO号，看是不是相同，每次更新
	int v_count = -1;//记录得到的速度的序号
	int max_location = 0;//记录出现最多的ICAO号的位置，是个序号信息

	//Uint8 pbuf_yindao[36] = {0};//引导数据格式
	//引导数据定义成全局的，XSH也要用。
	float tmp1;
	float tmp2;
	float tmp3;
/*
	float a;
	int b;
	float c;
*/
	int second_30_count = 0;//计数，每秒一次，小于30发空包，大于30执行后面的策略
	int UTC_time_tmp;
	Uint8 tmp_change[4];
//	memcpy(&(UTC_time_tmp),&s_FPGA_YC,4);//将UTC时间给到UTC_time_tmp变量里面

/*	
	a = 0.09;
	memcpy(&(b),&a,4);
	printf("b = %d\n",b);
	memcpy(&(c),&b,4);
	//c = (float *)(&b);
	printf("c = %f\n",c);
	
*/
//	TaskSleep(30 * 1000);//先等30s
	int count_tmp_zhang_for_test = 0;
	up_commend_count = 0;
	while (1)
	{
		count_tmp_zhang_for_test++;
		//zhangfulong add 时间更新
/*		UTC_time_real[0] = UTC_time_all[13];
		UTC_time_real[1] = UTC_time_all[12];
		UTC_time_real[2] = UTC_time_all[15];
		UTC_time_real[3] = UTC_time_all[14];
		UTC_time_real[4] = UTC_time_all[17];
		UTC_time_real[5] = UTC_time_all[16];*/
		//zhangfulong add 时间更新
		//调整顺序，数据处理部分放在lead任务for循环开头状态
		for (i = 0; i < 200; i = i + 2)
		{
			ICAOArray_real[i] = ICAOArray[i + 1 + 2];
			ICAOArray_real[i + 1] = ICAOArray[i + 2];
		}
		
		if ((ICAOArray_real[12] == 0xbb) && (ICAOArray_real[13] == 0x77) && (ICAOArray_real[14] == 0x3D) )
		{
			//包头正确 ，下面进行校验和计算 从 12 的00 04 开始  Uint16 check_sum_1,check_sum_2;
			check_sum_1 = 0;
			check_sum_2 = 0;
			for (i = 12; i < 151 - 2; i = i +2 )
			{
				check_sum_1 = (check_sum_1 ^ ( (Uint16)(ICAOArray_real[i] << 8)+(Uint16)(ICAOArray_real[i]) ) );
			}
			check_sum_2 = ( (Uint16)(ICAOArray_real[152 - 2] << 8)+(Uint16)(ICAOArray_real[153 - 2]) );
//			if (check_sum_1 == check_sum_2)		//校验先去掉，以后加上
			{
				//能进来说明校验和正确，下面开始将值更新到我们的结构体里面
				if (ICAOArray_real[15] == 0x33)//自由搜索
				{
					yaokongcmd_all_data.mode = 1;
				}
				else if (ICAOArray_real[15] == 0xCC)//给ICAO号的模式
				{
					yaokongcmd_all_data.mode = 2;
				}
				else if (ICAOArray_real[15] == 0xFF)//0xFF 清空位置飞机信息
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
				memcpy(&(yaokongcmd_all_data.jingdu),&(tmp_change[0]),4);//经度
				//yaokongcmd_all_data.jingdu = 90.0;//test 临时
				yaokongcmd_all_data.jingdu = yaokongcmd_all_data.jingdu / 3.1415926 *180;//test 临时
				yaokongcmd_all_data.jingdu = 90.0;
				//memcpy(&(yaokongcmd_all_data.jingdu),&(ICAOArray_real[16]),4);//经度
				tmp_change[3] = ICAOArray_real[20];
				tmp_change[2] = ICAOArray_real[21];
				tmp_change[1] = ICAOArray_real[22];
				tmp_change[0] = ICAOArray_real[23];
				memcpy(&(yaokongcmd_all_data.weidu),&(tmp_change[0]),4);//纬度
				//yaokongcmd_all_data.weidu = 90.0;//test 临时
				yaokongcmd_all_data.weidu = yaokongcmd_all_data.weidu /3.1415926 *180;							//20201112
				yaokongcmd_all_data.weidu = 90.0;
				//memcpy(&(yaokongcmd_all_data.weidu),&(ICAOArray_real[20]),4);//纬度
				tmp_change[3] = ICAOArray_real[24];
				tmp_change[2] = ICAOArray_real[25];
				tmp_change[1] = ICAOArray_real[26];
				tmp_change[0] = ICAOArray_real[27];
				memcpy(&(yaokongcmd_all_data.gaodu),&(tmp_change[0]),4);//高度
				//memcpy(&(yaokongcmd_all_data.gaodu),&(ICAOArray_real[24]),4);//高度
				yaokongcmd_all_data.banzhuijiao = ICAOArray_real[28];
				//yaokongcmd_all_data.banzhuijiao = 90;
				//yaokongcmd_all_data.chelue_1 = ICAOArray_real[29];
				//yaokongcmd_all_data.chelue_2 = ICAOArray_real[30];
				//yaokongcmd_all_data.chelue_3 = ICAOArray_real[31];
				if (ICAOArray_real[29] == 0x00)
				{
					//高度 频率
					yaokongcmd_all_data.chelue_1 = 0xAA;
					yaokongcmd_all_data.chelue_2 = 0xFF;
					yaokongcmd_all_data.chelue_3 = 0x00;
				}
				else if (ICAOArray_real[29] == 0x02)
				{
					//频率 高度 
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
				
				isfirst = 0;//新的策略，从第一次开始
				for (i = 0; i < 200; i++)
				{
					ICAOArray_real[i] = 0;
					ICAOArray[i] = 0;
				}
				up_commend_count = up_commend_count + 1;
			}			
		}
outclear:
#if 0		//去掉前30s的等待状态，变成有遥控数据后开始执行策略
		//前30s的状态
		if(second_30_count < 30)
		{
			for (k = 0; k < 34; k++)//初始化
			{
				pbuf_yindao[k] = 0x00;
			}
			pbuf_yindao[0] = 0x6A;//包头
			//UTC时间
			memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//将时间复制过去 4->6

			pbuf_yindao[8 + 2] = 0xF0;//第一包是F0  0-30状态
			//校验和
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
		//前30s的状态
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

		if (yaokongcmd_all_data.mode == 1)//自由目标搜索
		{
			count = 0;//记录是否在搜索区域内的数量
			if(isfirst == 0)
			{
				count = 0;//记录是否在搜索区域内的数量
				for (i = 0; i < position_BUFLEN; i++)
				{
					//tmp1 = fabs(airplane_location_static[i].coordinate[1] - yaokongcmd_all_data.jingdu);//纬度差
					//tmp2 = fabs(airplane_location_static[i].coordinate[0] - yaokongcmd_all_data.weidu);//经度差
					//memcpy(&(tmp1),&(airplane_location_static[i].coordinate[1]),4);
					//memcpy(&(tmp2),&(airplane_location_static[i].coordinate[0]),4);
					//tmp1 = fabs(tmp1 - yaokongcmd_all_data.jingdu);//纬度差
					//tmp2 = fabs(tmp2 - yaokongcmd_all_data.weidu);//经度差

					tmp1 = fabs(airplane_location_static[i].coordinate[0]/10000000 - yaokongcmd_all_data.jingdu);//纬度差
					tmp2 = fabs(airplane_location_static[i].coordinate[1]/10000000 - yaokongcmd_all_data.weidu);//经度差
					if (airplane_location_static[i].ICAO_address != 0)
					{
						//printf("location ICAO is %d,i = %d ~ \n",airplane_location_static[i].ICAO_address, i);
					}		
					
					if ( (tmp1 <= yaokongcmd_all_data.banzhuijiao) && (tmp2 <= yaokongcmd_all_data.banzhuijiao) && (airplane_location_static[i].ICAO_address != 0))
					{
						//满足在视场内,相应的值写进去
						airplane_location_array_tmp[count].altitude = airplane_location_static[i].altitude;
						airplane_location_array_tmp[count].coordinate[1] = airplane_location_static[i].coordinate[1];
						airplane_location_array_tmp[count].coordinate[0] = airplane_location_static[i].coordinate[0];
						airplane_location_array_tmp[count].ICAO_address = airplane_location_static[i].ICAO_address;
						airplane_location_array_tmp[count].time = airplane_location_static[i].time;
						airplane_location_array_tmp[count].number = 0;//将计数清空为0
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
#if 0	//更改可以变化顺序的策略组，所以走后面的else分支
				//上面循环已经将所有的在视场里面信息写入
				//下满将统计出现的个数信息
				//for (i = 0; i < count; i++)//清空计数
				//{
				//	airplane_location_array_tmp[i].number = 0;
				//}
				for (i = 0; i < count; i++)
				{
					for (j = i; j < count ; j++)
					{
						if(airplane_location_array_tmp[i].ICAO_address == airplane_location_array_tmp[j].ICAO_address)
						{
							//第一次肯定会进来
							airplane_location_array_tmp[i].number = airplane_location_array_tmp[i].number + 1;
						}
					}
				}
				//上面循环将每一个ICAO号出现的次数写入airplane_location_array_tmp[count].number里面
				//下面找出出现频率最高的
				max_location = 0;
				for (i = 1; i < count; i++)
				{
					if ((airplane_location_array_tmp[i].number > airplane_location_array_tmp[max_location].number)&& (airplane_location_array_tmp[i].ICAO_address != 0))
					{
						//这一条的出现次数最多，更新max_location位置
						max_location = i;
					}
					else if (airplane_location_array_tmp[i].number == airplane_location_array_tmp[max_location].number)
					{
						//两个相等，进行高度判断
						if (airplane_location_array_tmp[i].altitude >= airplane_location_array_tmp[max_location].altitude)
						{
							//新的高度更高，进行更新
							max_location = i;
						}
						else
						{
							;
						}
						;
					}
					else//这个出现的次数没有目前最大值多，不更新
					{
						;
					}
				}
#else
				//解算顺序是唯一的策略
				//
				for(i= 0; i < 1500; i++)
				{
					location_arrary[i] = -1;//进行初始化-1配置
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
				//上面循环将获得一个出现次数最多的，存在max_location里面
				//接下来寻找指定ICAO号的最新位置信息
if (count > 0)//没找到，不发了
{
				if (count > 0)//如果 count 大于0 ，则至少有一个飞机
				{
					max_location = P_get(airplane_location_array_tmp[max_location].ICAO_address);//此时max_location是在airplane_location_static数组里面
					airplane_location_tmp.ICAO_address = airplane_location_static[max_location].ICAO_address;
					airplane_location_tmp.altitude = airplane_location_static[max_location].altitude;
					airplane_location_tmp.coordinate[0] = airplane_location_static[max_location].coordinate[0];
					airplane_location_tmp.coordinate[1] = airplane_location_static[max_location].coordinate[1];
					airplane_location_tmp.time = airplane_location_static[max_location].time;
				}
				else//说明没有命中
				{
					airplane_location_tmp.ICAO_address = 0;
					airplane_location_tmp.altitude = 0;
					airplane_location_tmp.coordinate[0] = 0;
					airplane_location_tmp.coordinate[1] = 0;
					airplane_location_tmp.time = 0;
				}
				//生成包
				for (k = 0; k < 36; k++)//初始化
				{
					pbuf_yindao[k] = 0x00;
				}
				pbuf_yindao[0] = 0x6A;//包头
				pbuf_yindao[1] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x00FF0000)>>16);//ICAO号
				pbuf_yindao[2] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x0000FF00)>>8);
				pbuf_yindao[3] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x000000FF));
				//UTC时间
				//zhangfulong add 时间更新
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add 时间更新
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//将时间复制过去 4->6
				pbuf_yindao[4] = UTC_time_real[0];
				pbuf_yindao[5] = UTC_time_real[1];
				pbuf_yindao[6] = UTC_time_real[2];
				pbuf_yindao[7] = UTC_time_real[3];
				pbuf_yindao[8] = UTC_time_real[4];
				pbuf_yindao[9] = UTC_time_real[5];
				//pbuf_yindao[9] = count_tmp_zhang_for_test & 0x000000FF;//临时要去掉
				//使用解包时间
				pbuf_yindao[6] = (airplane_location_tmp.time & 0xFF000000)>>24;
				pbuf_yindao[7] = (airplane_location_tmp.time & 0x00FF0000)>>16;
				pbuf_yindao[8] = (airplane_location_tmp.time & 0x0000FF00)>>8;
				pbuf_yindao[9] = (airplane_location_tmp.time & 0x000000FF);
				//使用解包时间
				//时间最后弄上
				v_count = V_get(airplane_location_tmp.ICAO_address);
				//更新状态

				pbuf_yindao[8 + 2] = 0xFF;//第一包是FF
				//memcpy(&(pbuf_yindao[9+ 2]),&(airplane_location_tmp.coordinate[1]),4);//经度
				//memcpy(&(pbuf_yindao[13+ 2]),&(airplane_location_tmp.coordinate[0]),4);//纬度
				pbuf_yindao[9+ 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0xFF000000) >> 24);
				pbuf_yindao[9+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x00FF0000) >> 16);
				pbuf_yindao[9+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x0000FF00) >> 8);
				pbuf_yindao[9+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x000000FF));

				pbuf_yindao[13+ 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0xFF000000) >> 24);
				pbuf_yindao[13+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x00FF0000) >> 16);
				pbuf_yindao[13+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x0000FF00) >> 8);
				pbuf_yindao[13+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x000000FF));

				if(v_count < 0)//没有匹配速度
				{
					pbuf_yindao[17+ 2] = 0x00;
					pbuf_yindao[28+ 2] = 0x00;//里面已经是0了
					V_time_last = 0;
					airplane_speed_tmp.E_W_velocity = 0;
					airplane_speed_tmp.ICAO_address = 0;
					airplane_speed_tmp.N_S_velocity = 0;
					airplane_speed_tmp.time = 0;
					airplane_speed_tmp.VERT_velocity = 0;
				}
				else//有匹配速度
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
				//高度
				//memcpy(&(pbuf_yindao[29+ 2]),&(airplane_location_tmp.altitude),4);
				pbuf_yindao[29+ 2] = (Uint8)((airplane_location_tmp.altitude & 0xFF000000)>>24);
				pbuf_yindao[29+ 2 + 1] = (Uint8)((airplane_location_tmp.altitude & 0x00FF0000)>>16);
				pbuf_yindao[29+ 2 + 2] = (Uint8)((airplane_location_tmp.altitude & 0x0000FF00)>>8);
				pbuf_yindao[29+ 2 + 3] = (Uint8)((airplane_location_tmp.altitude & 0x000000FF));
				//
				//记录上一次的时间戳ICAO
				memcpy(&P_time_last,&(airplane_location_tmp.time), 4);

				//校验和
				pbuf_yindao[33+ 2] = 0;
				for( k = 0; k < 35; k++)
				{
					pbuf_yindao[33+ 2] = (pbuf_yindao[33+ 2] + pbuf_yindao[k]) & 0xFF;
				}
				//将ICAO号保存和下一次相比
				ICAO_last = airplane_location_tmp.ICAO_address;
				;
				if (airplane_location_tmp.ICAO_address != 0)
				{
					isfirst = isfirst + 1;//标志第一次结束，接下来进入后面的
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
				//	pEDMA3CC_PaRAM->BCNT_ACNT =0x00660001; //1280*4	//0x003d0001D′μ?FPGA??μ??·A0F	//51
				pEDMA3CC_PaRAM->BCNT_ACNT =0x00240001;	//0x22 = 34
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
			    pEDMA3CC_PaRAM->DST = (unsigned char *)DSP_TO_FPGA_YINDAO;
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
}//没找到不发括号
				//等待15s
				//task_sleep(15);
			}
			else//上面的是谝淮谓胱杂伤阉鞑⒎⒊霭酉吕椿崽砑?5s以内的限制条件并继续
			{
				count = 0;//记录是否在搜索区域内的数量
//if (ICAO_last == 0)
{
				for (i = 0; i < position_BUFLEN; i++)
				{
					//tmp1 = fabs(airplane_location_static[i].coordinate[1] - yaokongcmd_all_data.jingdu);//纬度差
					//tmp2 = fabs(airplane_location_static[i].coordinate[0] - yaokongcmd_all_data.weidu);//经度差
					tmp3 = UTC_time_tmp - airplane_location_static[i].time;
					
					//memcpy(&(tmp1),&(airplane_location_static[i].coordinate[1]),4);
					//memcpy(&(tmp2),&(airplane_location_static[i].coordinate[0]),4);
					//tmp1 = fabs(tmp1 - yaokongcmd_all_data.jingdu);//纬度差
					//tmp2 = fabs(tmp2 - yaokongcmd_all_data.weidu);//经度差

					tmp1 = fabs(airplane_location_static[i].coordinate[0]/10000000 - yaokongcmd_all_data.jingdu);//经度差
					tmp2 = fabs(airplane_location_static[i].coordinate[1]/10000000 - yaokongcmd_all_data.weidu);//纬度差
					
					
				
					//if ( (tmp1 <= yaokongcmd_all_data.banzhuijiao) & (tmp2 <= yaokongcmd_all_data.banzhuijiao) & (tmp3 <= 15) )
					//暂时去掉时间限制，测试时候，以后在加上
					if ( (tmp1 <= yaokongcmd_all_data.banzhuijiao) & (tmp2 <= yaokongcmd_all_data.banzhuijiao)&& (airplane_location_static[i].ICAO_address != 0))
					{
						//满足在视场内,相应的值写进去
						airplane_location_array_tmp[count].altitude = airplane_location_static[i].altitude;
						airplane_location_array_tmp[count].coordinate[1] = airplane_location_static[i].coordinate[1];
						airplane_location_array_tmp[count].coordinate[0] = airplane_location_static[i].coordinate[0];
						airplane_location_array_tmp[count].ICAO_address = airplane_location_static[i].ICAO_address;
						airplane_location_array_tmp[count].time = airplane_location_static[i].time;
						airplane_location_array_tmp[count].number = 0;//将计数清空为0
						count = count + 1;
						;
					}
				//	if ((airplane_location_static[i].ICAO_address & 0x00FFFFFF) == 0x123456)
				//	{
				//		printf("No.%d.altitude = %d;\n",i,airplane_location_static[i].altitude);
				//	}		
				}
				//上面循环已经将所有的在视场里面信息写入
				//下满将统计出现的个数信息
				//for (i = 0; i < count; i++)//清空计数
				//{
				//	airplane_location_array_tmp[i].number = 0;
				//}
#if 0		//去掉这个策略，跟踪一个飞机
				for (i = 0; i < count; i++)
				{
					for (j = i; j < count ; j++)
					{
						if(airplane_location_array_tmp[i].ICAO_address == airplane_location_array_tmp[j].ICAO_address)
						{
							//第一次肯定会进来
							airplane_location_array_tmp[i].number = airplane_location_array_tmp[i].number + 1;
							/*if (airplane_location_array_tmp[j].altitude != 0)
							{
								printf("airplane_location_array_tmp[%d].altitude = %d\n",j,airplane_location_array_tmp[j].altitude);
							}*/
							
						}
					}
				}
				//上面循环将每一个ICAO号出现的次数写入airplane_location_array_tmp[count].number里面
				//下面找出出现频率最高的
				max_location = 0;
				for (i = 1; i < count; i++)
				{
					if ((airplane_location_array_tmp[i].number > airplane_location_array_tmp[max_location].number) && (airplane_location_array_tmp[i].ICAO_address != 0) )
					{
						//这一条的出现次数最多，更新max_location位置
						max_location = i;
					}
					else if (airplane_location_array_tmp[i].number == airplane_location_array_tmp[max_location].number)
					{
						//两个相等，进行高度判断
						if (airplane_location_array_tmp[i].altitude >= airplane_location_array_tmp[max_location].altitude)
						{
							//新的高度更高，进行更新
							max_location = i;
						}
						else
						{
							;
						}
						;
					}
					else//这个出现的次数没有目前最大值多，不更新
					{
						;
					}
				}
				//上面循环将获得一个出现次数最多的，存在max_location里面				
				//接下来寻找指定ICAO号的最新位置信息
				if (count > 0)//如果 count 大于0 ，则至少有一个飞机
				{
					max_location = P_get(airplane_location_array_tmp[max_location].ICAO_address);//此时max_location是在airplane_location_static数组里面
					airplane_location_tmp.ICAO_address = airplane_location_static[max_location].ICAO_address;
					airplane_location_tmp.altitude = airplane_location_static[max_location].altitude;
					airplane_location_tmp.coordinate[0] = airplane_location_static[max_location].coordinate[0];
					airplane_location_tmp.coordinate[1] = airplane_location_static[max_location].coordinate[1];
					airplane_location_tmp.time = airplane_location_static[max_location].time;
				}
				else//说明没有命中
				{
					airplane_location_tmp.ICAO_address = 0;
					airplane_location_tmp.altitude = 0;
					airplane_location_tmp.coordinate[0] = 0;
					airplane_location_tmp.coordinate[1] = 0;
					airplane_location_tmp.time = 0;
				}
#endif			//去掉这个策略，跟踪一个飞机
}
//else//上面分支是第一次没找到，在15s里面继续找下面是找到了ICAO，所以按照ICAO查找，
{
				for (i = 0; i < position_BUFLEN; i++)//遍历收到�1500个数组
				{
					if(airplane_location_static[i].ICAO_address == airplane_location_tmp.ICAO_address)
					{
						//ICAO号命中,下面看是否是时间最新的，
						if(airplane_location_static[i].time > airplane_location_tmp.time)
						{
							memcpy(&airplane_location_tmp,&(airplane_location_static[i]), 20);
							count = count + 1;//count表示速度有更新
						}
						
					}
				//	if (airplane_location_static[i].ICAO_address == 0x00111111)
				//	{
				//		printf("No.%d.altitude = %d;\n",i,airplane_location_static[i].altitude);
				//	}
				}
}//大的if else循环结束
				//生成包
				for (k = 0; k < 36; k++)//初始化
				{
					pbuf_yindao[k] = 0x00;
				}
				pbuf_yindao[0] = 0x6A;//包头
				pbuf_yindao[1] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x00FF0000)>>16);//ICAO号
				pbuf_yindao[2] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x0000FF00)>>8);
				pbuf_yindao[3] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x000000FF));
				//UTC时间
				//zhangfulong add 时间更新
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add 时间更新
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),4+ 2);//将时间复制过去
				pbuf_yindao[4] = UTC_time_real[0];
				pbuf_yindao[5] = UTC_time_real[1];
				pbuf_yindao[6] = UTC_time_real[2];
				pbuf_yindao[7] = UTC_time_real[3];
				pbuf_yindao[8] = UTC_time_real[4];
				pbuf_yindao[9] = UTC_time_real[5];
				//pbuf_yindao[9] = count_tmp_zhang_for_test & 0x000000FF;//临时要去掉
				//使用解包时间
				pbuf_yindao[6] = (airplane_location_tmp.time & 0xFF000000)>>24;
				pbuf_yindao[7] = (airplane_location_tmp.time & 0x00FF0000)>>16;
				pbuf_yindao[8] = (airplane_location_tmp.time & 0x0000FF00)>>8;
				pbuf_yindao[9] = (airplane_location_tmp.time & 0x000000FF);
				//使用解包时间
				//v_count = V_get(airplane_location_tmp.ICAO_address);
				//更新状态 已经不是第一包了，所以下面的判断有效性
#if 1
				if(airplane_location_tmp.ICAO_address == ICAO_last)//ICAO相同
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] | 0xF0;
				}
				else//ICAO不同
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0x0F;//鲁?x0X 与上一包不同
				}
				
				if ((v_count == -1) && (P_time_last==airplane_location_tmp.time))
				//速度没找到，位置时间相同
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
				}
				else//找到了速度,
				{
					if((V_time_last == airplane_velocity_three_static[v_count].time) && (P_time_last==airplane_location_tmp.time))
					{
						//找到速度 速度无更新,位置也没更新
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
					}
					else
					{
						//速度有更新，或者位置有更新，都算更新
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] | 0x0F;
						printf("PPPPPPPVVVVVVVV\n");
					}
				}
				/*if(v_count == -1)//没找到
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
				}
				else
				{
					if(V_time_last == airplane_velocity_three_static[v_count].time)
					{
						//找到速度无更新
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
					}
					else
					{
						//速度有更新
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] | 0x0F;
					}
					
				}*/
#endif
				//pbuf[8] = 0xFF;//第一包是FF
				//memcpy(&(pbuf_yindao[9+ 2]),&(airplane_location_tmp.coordinate[1]),4);//经度
				//memcpy(&(pbuf_yindao[13+ 2]),&(airplane_location_tmp.coordinate[0]),4);//纬度
				pbuf_yindao[9+ 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0xFF000000) >> 24);
				pbuf_yindao[9+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x00FF0000) >> 16);
				pbuf_yindao[9+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x0000FF00) >> 8);
				pbuf_yindao[9+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x000000FF));

				pbuf_yindao[13+ 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0xFF000000) >> 24);
				pbuf_yindao[13+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x00FF0000) >> 16);
				pbuf_yindao[13+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x0000FF00) >> 8);
				pbuf_yindao[13+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x000000FF));
				if(v_count < 0)//没有匹配速度
				{
					pbuf_yindao[17+ 2] = 0x00;
					pbuf_yindao[28+ 2] = 0x00;//里面已经是0了
					V_time_last = 0;

					airplane_speed_tmp.E_W_velocity = 0;
					airplane_speed_tmp.ICAO_address = 0;
					airplane_speed_tmp.N_S_velocity = 0;
					airplane_speed_tmp.time = 0;
					airplane_speed_tmp.VERT_velocity = 0;
				}
				else//有匹配速度
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

					//记录上一次的时间戳ICAO 速度
					memcpy(&V_time_last,&(airplane_velocity_three_static[v_count].time), 4);
					
					airplane_speed_tmp.E_W_velocity = airplane_velocity_three_static[v_count].E_W_velocity;
					airplane_speed_tmp.ICAO_address = airplane_velocity_three_static[v_count].ICAO_address;
					airplane_speed_tmp.N_S_velocity = airplane_velocity_three_static[v_count].N_S_velocity;
					airplane_speed_tmp.time = airplane_velocity_three_static[v_count].time;
					airplane_speed_tmp.VERT_velocity = airplane_velocity_three_static[v_count].VERT_velocity;

				}
				//高度
				//memcpy(&(pbuf_yindao[29+ 2]),&(airplane_location_tmp.altitude),4);
				pbuf_yindao[29+ 2] = (Uint8)((airplane_location_tmp.altitude & 0xFF000000)>>24);
				pbuf_yindao[29+ 2 + 1] = (Uint8)((airplane_location_tmp.altitude & 0x00FF0000)>>16);
				pbuf_yindao[29+ 2 + 2] = (Uint8)((airplane_location_tmp.altitude & 0x0000FF00)>>8);
				pbuf_yindao[29+ 2 + 3] = (Uint8)((airplane_location_tmp.altitude & 0x000000FF));
				//
				//记录上一次的奔戳ICAO 位置
				memcpy(&P_time_last,&(airplane_location_tmp.time), 4);

				//校验和
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
				//	pEDMA3CC_PaRAM->BCNT_ACNT =0x00660001; //1280*4	//0x003d0001D′μ?FPGA??μ??·A0F	//51
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
		else if (yaokongcmd_all_data.mode == 2)//目标跟踪模式
		{
			count = 0;
			if(isfirst == 0)
			{
				//下面两个循环可能要换一下，应该先按照优先级从高到低第一个出现的
				//for (i = 0; i < position_BUFLEN; i++)//遍历收到的1500个数组
				for (j = 0; j < 30; j++)//遍历上传的ICAO号
				{
					//for (j = 0; j < 30; j++)//遍历上传的ICAO号
					for (i = 0; i < position_BUFLEN; i++)//遍历收到的1500个数组
					{
						if((airplane_location_static[i].ICAO_address == yaokongcmd_all_data.ICAO[j]) && (yaokongcmd_all_data.ICAO[j] !=0))//ICAO 满足
						{
							//tmp1 = fabs(airplane_location_static[i].coordinate[1] - yaokongcmd_all_data.jingdu);//纬度差
							//tmp2 = fabs(airplane_location_static[i].coordinate[0] - yaokongcmd_all_data.weidu);//经度差
							
							//memcpy(&(tmp1),&(airplane_location_static[i].coordinate[1]),4);
							//memcpy(&(tmp2),&(airplane_location_static[i].coordinate[0]),4);
							tmp1 = fabs(airplane_location_static[i].coordinate[0]/10000000 - yaokongcmd_all_data.jingdu);//经度差
							tmp2 = fabs(airplane_location_static[i].coordinate[1]/10000000 - yaokongcmd_all_data.weidu);//纬度差
							
							
							if ( (tmp1 <= yaokongcmd_all_data.banzhuijiao) & (tmp2 <= yaokongcmd_all_data.banzhuijiao) )
							{
								//在角度范围内，将命中的飞机给到airplane_location_tmp变量里面
								memcpy(&airplane_location_tmp,&(airplane_location_static[i]), 20);
								goto L1;//跳出两个循环
							}
						}
					}
				}
L1:
				
				//输出飞机的命中信息，没有命中也可以输出，反正就是0
				for (k = 0; k < 34; k++)//初始化
				{
					pbuf_yindao[k] = 0x00;
				}
				pbuf_yindao[0] = 0x6A;//包头
				pbuf_yindao[1] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x00FF0000)>>16);//ICAO号
				pbuf_yindao[2] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x0000FF00)>>8);
				pbuf_yindao[3] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x000000FF));
				//UTC时间
				//zhangfulong add 时间更新
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add 时间更新
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),4+ 2);//将时间复制过去
				pbuf_yindao[4] = UTC_time_real[0];
				pbuf_yindao[5] = UTC_time_real[1];
				pbuf_yindao[6] = UTC_time_real[2];
				pbuf_yindao[7] = UTC_time_real[3];
				pbuf_yindao[8] = UTC_time_real[4];
				pbuf_yindao[9] = UTC_time_real[5];
				//pbuf_yindao[9] = count_tmp_zhang_for_test & 0x000000FF;//临时要去掉
				//使用解包时间
				pbuf_yindao[6] = (airplane_location_tmp.time & 0xFF000000)>>24;
				pbuf_yindao[7] = (airplane_location_tmp.time & 0x00FF0000)>>16;
				pbuf_yindao[8] = (airplane_location_tmp.time & 0x0000FF00)>>8;
				pbuf_yindao[9] = (airplane_location_tmp.time & 0x000000FF);
				//使用解包时间

				v_count = V_get(airplane_location_tmp.ICAO_address);
				//更新状态
#if 0
				if(airplane_location_tmp.ICAO_address == 0)//没找到ICAO
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0x0F;
				}
				else//找到了ICAO
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0x0F;//更新成0x0X 与上一包不同
				}

				if(v_count == -1)//没找到
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
				}
				else
				{
					pbuf_yindao[8+ 2] = (pbuf_yindao[8+ 2] | 0x0F ) ;//更新成0x0X 与上一包不同
				}
#endif
				pbuf_yindao[8+ 2] = 0xFF;
				//memcpy(&(pbuf_yindao[9+ 2]),&(airplane_location_tmp.coordinate[1]),4);//经度
				//memcpy(&(pbuf_yindao[13+ 2]),&(airplane_location_tmp.coordinate[0]),4);//纬度
				pbuf_yindao[9+ 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0xFF000000) >> 24);
				pbuf_yindao[9+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x00FF0000) >> 16);
				pbuf_yindao[9+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x0000FF00) >> 8);
				pbuf_yindao[9+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x000000FF));

				pbuf_yindao[13+ 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0xFF000000) >> 24);
				pbuf_yindao[13+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x00FF0000) >> 16);
				pbuf_yindao[13+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x0000FF00) >> 8);
				pbuf_yindao[13+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x000000FF));

				if(v_count < 0)//没有匹配速度
				{
					pbuf_yindao[17+ 2] = 0x00;
					pbuf_yindao[28+ 2] = 0x00;//里面已经是0了
					V_time_last = 0;

					airplane_speed_tmp.E_W_velocity = 0;
					airplane_speed_tmp.ICAO_address = 0;
					airplane_speed_tmp.N_S_velocity = 0;
					airplane_speed_tmp.time = 0;
					airplane_speed_tmp.VERT_velocity = 0;

				}
				else//有匹配速度
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
				//高度
				//memcpy(&(pbuf_yindao[29+ 2]),&(airplane_location_tmp.altitude),4);
				pbuf_yindao[29+ 2] = (Uint8)((airplane_location_tmp.altitude & 0xFF000000)>>24);
				pbuf_yindao[29+ 2 + 1] = (Uint8)((airplane_location_tmp.altitude & 0x00FF0000)>>16);
				pbuf_yindao[29+ 2 + 2] = (Uint8)((airplane_location_tmp.altitude & 0x0000FF00)>>8);
				pbuf_yindao[29+ 2 + 3] = (Uint8)((airplane_location_tmp.altitude & 0x000000FF));

				//
				//锹忌弦淮蔚时间戳ICAO
				memcpy(&P_time_last,&(airplane_location_tmp.time), 4);

				//校验和
				for( k = 0; k < 36; k++)
				{
					pbuf_yindao[33+ 2] = (pbuf_yindao[33+ 2] + pbuf_yindao[k]) & 0xFF;
				}
				//接下来发出去pbuf，还没做

				//
				ICAO_last = airplane_location_tmp.ICAO_address;
				if(airplane_location_tmp.ICAO_address != 0)//不等于0，找到了一个后，跟踪，以后不找?
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
				//	pEDMA3CC_PaRAM->BCNT_ACNT =0x00660001; //1280*4	//0x003d0001D′μ?FPGA??μ??·A0F	//51
				pEDMA3CC_PaRAM->BCNT_ACNT =0x00240001;	//0x22 = 34
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
			    pEDMA3CC_PaRAM->DST = (unsigned char *)DSP_TO_FPGA_YINDAO;
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
				
			}
			else
			{
				for (i = 0; i < position_BUFLEN; i++)//遍历收到的1500个数组
				{
					if(airplane_location_static[i].ICAO_address == airplane_location_tmp.ICAO_address)
					{
						//ICAO号命中,下面看是否是时间最新的，
						if(airplane_location_static[i].time > airplane_location_tmp.time)
						{
							memcpy(&airplane_location_tmp,&(airplane_location_static[i]), 20);
						}
						
					}
				}

				for (k = 0; k < 34; k++)//初始化
				{
					pbuf_yindao[k] = 0x00;
				}
				pbuf_yindao[0] = 0x6A;//包头
				pbuf_yindao[1] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x00FF0000)>>16);//ICAO号
				pbuf_yindao[2] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x0000FF00)>>8);
				pbuf_yindao[3] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x000000FF));
				//UTC时间
				//zhangfulong add 时间更新
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add 时间更新
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),4+ 2);//将时间复制过去
				pbuf_yindao[4] = UTC_time_real[0];
				pbuf_yindao[5] = UTC_time_real[1];
				pbuf_yindao[6] = UTC_time_real[2];
				pbuf_yindao[7] = UTC_time_real[3];
				pbuf_yindao[8] = UTC_time_real[4];
				pbuf_yindao[9] = UTC_time_real[5];
				//pbuf_yindao[9] = count_tmp_zhang_for_test & 0x000000FF;//临时要去掉
				//使用解包时间
				pbuf_yindao[6] = (airplane_location_tmp.time & 0xFF000000)>>24;
				pbuf_yindao[7] = (airplane_location_tmp.time & 0x00FF0000)>>16;
				pbuf_yindao[8] = (airplane_location_tmp.time & 0x0000FF00)>>8;
				pbuf_yindao[9] = (airplane_location_tmp.time & 0x000000FF);
				//使用解包时间
				//v_count = V_get(airplane_location_tmp.ICAO_address);
				//更新状态
#if 1
				if(airplane_location_tmp.ICAO_address == ICAO_last)//ICAO相同
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] | 0xF0;
				}
				else//ICAO不同
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0x0F;//更新成0x0X 与上一包不同
				}

				if ((v_count == -1) && (P_time_last==airplane_location_tmp.time))
				//速度没找到，位置时间相同
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
					
				}
				else//找到了速度,
				{
					if((V_time_last == airplane_velocity_three_static[v_count].time) && (P_time_last==airplane_location_tmp.time))
					{
						//找到速度 速度无更新,位置也没更新
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
					}
					else
					{
						//速度有更新蛘呶恢糜懈拢妓愀?
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] | 0x0F;
						printf("new FFFFFF !!!");
					}
				}

				/*if(v_count == -1)//没找到
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
				}
				else
				{
					if(V_time_last == airplane_velocity_three_static[v_count].time)
					{
						//找到速度无更新
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
					}
					else
					{
						//速度有更新
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] | 0x0F;
					}
					
				}*/
#endif
				//pbuf[8] = 0xFF;
				//memcpy(&(pbuf_yindao[9+ 2]),&(airplane_location_tmp.coordinate[1]),4);//经度
				//memcpy(&(pbuf_yindao[13+ 2]),&(airplane_location_tmp.coordinate[0]),4);//纬度
				pbuf_yindao[9+ 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0xFF000000) >> 24);
				pbuf_yindao[9+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x00FF0000) >> 16);
				pbuf_yindao[9+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x0000FF00) >> 8);
				pbuf_yindao[9+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x000000FF));

				pbuf_yindao[13+ 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0xFF000000) >> 24);
				pbuf_yindao[13+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x00FF0000) >> 16);
				pbuf_yindao[13+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x0000FF00) >> 8);
				pbuf_yindao[13+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x000000FF));
				if(v_count < 0)//没有匹配速度
				{
					pbuf_yindao[17+ 2] = 0x00;
					pbuf_yindao[28+ 2] = 0x00;//里面已经是0了
					V_time_last = 0;

					airplane_speed_tmp.E_W_velocity = 0;
					airplane_speed_tmp.ICAO_address = 0;
					airplane_speed_tmp.N_S_velocity = 0;
					airplane_speed_tmp.time = 0;
					airplane_speed_tmp.VERT_velocity = 0;
				}
				else//有匹配速度
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
				//高度
				//memcpy(&(pbuf_yindao[29+ 2]),&(airplane_location_tmp.altitude),4);
				pbuf_yindao[29+ 2] = (Uint8)((airplane_location_tmp.altitude & 0xFF000000)>>24);
				pbuf_yindao[29+ 2 + 1] = (Uint8)((airplane_location_tmp.altitude & 0x00FF0000)>>16);
				pbuf_yindao[29+ 2 + 2] = (Uint8)((airplane_location_tmp.altitude & 0x0000FF00)>>8);
				pbuf_yindao[29+ 2 + 3] = (Uint8)((airplane_location_tmp.altitude & 0x000000FF));

				//
				//记录上一次的时间戳ICAO
				memcpy(&P_time_last,&(airplane_location_tmp.time), 4);

				//校验和
				for( k = 0; k < 36; k++)
				{
					pbuf_yindao[33+ 2] = (pbuf_yindao[33+ 2] + pbuf_yindao[k]) & 0xFF;
				}
				//接下来发出去pbuf，还没做
				
				//输出飞机的命中信息，没有命中也可以输出，反正就?
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
				//	pEDMA3CC_PaRAM->BCNT_ACNT =0x00660001; //1280*4	//0x003d0001D′μ?FPGA??μ??·A0F	//51
				pEDMA3CC_PaRAM->BCNT_ACNT =0x00240001;	//0x22 = 34
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
			    pEDMA3CC_PaRAM->DST = (unsigned char *)DSP_TO_FPGA_YINDAO;
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
			}
			
		}
		else
		{
			V_time_last = 0;//记录上一次的时间，和这次比是否有更新
			P_time_last = 0;//记录上一次的时间，和这次比是否懈?
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
					//	pEDMA3CC_PaRAM->BCNT_ACNT =0x00660001; //1280*4	//0x003d0001D′μ?FPGA??μ??·A0F	//51
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
	int cnt = 0;//记录ICAO位置
	for (i = 0; i < 1000; i++)
	{
		ICAO_task_tmp[i] = 0;
	}
	while (1)
	{
		for (i = 0; i < 1500; i++)//遍历所有空间
		{
			//当前ICAO号 airplane_location_static[i].ICAO_address
			for (j = 0; j < 1000; j++)//遍历ICAO号数组
			{
				if ((airplane_location_static[i].ICAO_address == ICAO_task_tmp[j]) && (airplane_location_static[i].ICAO_address != 0))
				{
					//命中这个ICAO号已经有了
					goto ICAO_L;
				}
				;
			}
			//上面跑过了，说明是个新ICAO
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
    DSP_TO_FPGA_HANDLE = 0x55aa; //帐?
    MEM_initial();

	C62_disableIER(1<<8);  // edma中断 
	asm("nop");	
	C62_clearIFR(1<<8); // 清EDMA中	
	EDMA_init();
	
	C62_clearIFR(1<<9|1<<8);
	C62_enableIER(1<<9|1<<8);//9--fpga中断 8--edma中断	

//	gpio7_set_0();
#if 0
    C62_enableIER(1<<8);
	while(1)
	{
		   testEDMA();
		   DSK6455_wait(10);
    }
#endif

	//zhangfulong add 跏蓟?
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

   TaskCreate( get_Pulse ,"get_Pulse",OS_TASKPRINORM, 0x3000, 0, 0, 0); // 1280处理函数 //OS_TASKPRILOW
#if 1
   TaskCreate( get_PosV ," get_PosV",OS_TASKPRINORM, 0x3000, 0, 0, 0);  //OS_TASKPRILOW
#endif
#if 1
   //TaskCreate( xsh_task ," xsh_task",OS_TASKPRINORM, 0x3000, 0, 0, 0); //20190211  //OS_TASKPRILOW
   //稀疏化
   TaskCreate( xsh_2_task ," xsh_2_task",OS_TASKPRINORM, 0x3000, 0, 0, 0); //20190211  //OS_TASKPRILOW
#endif

	//创建EMIF口通信任务，代替原来的UDP传输
	//原始和融合
	TaskCreate( EMIF_sndPacket ,"EMIF_sndPacket",OS_TASKPRILOW, 0x5000, 0, 0, 0);

	//创建一个引导任务 lead_tesk
	//引导
	TaskCreate( lead_tesk ,"lead_tesk",OS_TASKPRILOW, 0x3000, 0, 0, 0);
	//check ICAO tesk
	//TaskCreate( ICAO_tesk ,"ICAO_tesk",OS_TASKPRILOW, 0x3000, 0, 0, 0);

#endif
#if 0
   TaskCreate( udp_sndPacket ,"udp_sndPacket",OS_TASKPRILOW, 0x3000, 0, 0, 0); //
#endif
   TaskBlock( TaskSelf() );
}
