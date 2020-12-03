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
Uint8 test_flag =0; //�в��������ó�1��ʲôʱ������أ�
Uint8 sum_cnt=0;
Uint8 zero_cnt=0;
Uint8 XSH_Array[288]={0};//88 
Uint8 TestArray[160]={0};


Uint8 ICAOArray[800]={0};	//��������FPGA��������ICAO�ţ�30*4B
Uint8 ICAOArray_real[200]={0};	//����˳���Ĵ洢λ��
Uint16 check_sum_1,check_sum_2;

//zhangfulong add start
Uint8	yaokongcmd[140] = {0};//ң��ָ���ź����ݸ�ʽ
Uint32 ICAO_number[30] = {0};//�������յ���ICAO��д��32λ����������
int taokong_workmode = 0;// 0��ʾû�з���1������Ŀ������ ��2��Ŀ�����
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

//Uint8 pbuf_yindao[36] = {0};//�������ݸ�ʽ
Uint8 pbuf_yindao[150] = {0};
Uint8 up_commend_count = 0;//��ע����������
//zhangfulong add end

//Uint8 XSH_Array2[188]={0};//?
//EDMA_Handle hEdma,hEdma_samp;     /* Handle for the EDMA channel  */
#if 0
	unsigned int int_cnt_1=0;//�жϸ���
	unsigned int int_cnt_2=0;//CRCȫ�Ե�
	unsigned int int_cnt_3=0;//���Ͱ�����
	unsigned int int_cnt_4=0;//�ɾ����
	unsigned int int_cnt_5=0;
	unsigned int int_cnt_6=0;
	unsigned int int_cnt_7=0;  //����Դ��һ�µ�
	unsigned int int_cnt_8=0;   //��ͷ�Ե�
	unsigned int int_cnt_9=0;  //yc
	unsigned int int_cnt_10=0;  //xsh
	unsigned int int_cnt_11=0;  //ykip
	unsigned int int_cnt_12=0;  //test
#endif
unsigned int int_cnt_1=0;//�жϸ���
unsigned int int_cnt_3=0;//���Ͱ�����
Uint8  int_cnt[22]={0x0};   //
unsigned int pos_2_cnt=0; //����λ��2����
unsigned int pos_3_cnt=0; //����λ��3����(��Ч)
unsigned int vel_4_cnt=0; //�����ٶ�4����
unsigned int vel_5_cnt=0; //�����ٶ�5��������Ч��
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
volatile Uint32 IP_Pkg_index=0;//1;  //дָ�� //0//170904 //?����?????
#pragma DATA_SECTION ( SEND_ADDR_CACHE, ".DATA_IP" );
//#pragma DATA_ALIGN(SEND_ADDR_CACHE, 128);
IP_ADDR_CACHE  SEND_ADDR_CACHE[SEND_IP_DATA_BUFLEN];
Uint32 g_u32IpSendIndex=1;   //��ָ��
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
//�о�������׼���Ļ����������������Ƿ��б�Ҫ����
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

struct location_struct airplane_location_XSH_static[position_BUFLEN_xsh];  //�Ÿ�¡ add XSHϡ�軯��λ������
struct speed_struct_three airplane_velocity_three_XSH_static[position_BUFLEN_xsh];  //�Ÿ�¡ add XSHϡ�軯���ٶ�����

Uint32 position_xsh_ICAO[position_BUFLEN_xsh];
volatile Uint32 position_cnt_xsh=0;//
volatile Uint32 position_wr_xsh=0;
volatile Uint32 position_wr_xsh_v=0;//zhangfulong add ϡ�軯���ݣ�
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
FPGA_YC s_FPGA_YC;  //16�ֽ�ң��
Uint8 UTC_time_all[20] = 0;//���յ�����
Uint8 UTC_time_real[6] = 0;//��¼UTCʱ�䣬������������ֵ
#pragma DATA_ALIGN(s_FPGA_YC, 8);
//���ڱ�����Ϣ���ϱ���ȫ�ֽṹ��
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
  unsigned char data_demodulator[11];  //ADSB��ԭʼ88bit��Ϣ���ģ�11�ֽ�							
}ADSB_message; 
//******************************************************************************************************************************
//�ṹ�������������������ṹ�����Ͷ��屾��,����λ����Ϣ�ļ���
//�˽ṹ������ͽṹ�����ȫ��ʹ�ã���decode_position�������¸ýṹ��data_save[]���飬new_data�ṹ�������demodulator�����и���
//�ýṹ�����ȫ�ֱ������ݣ������Ǹ���Ԫ�صĶ��塣
//time�Ǳ��ĵ�ʱ���ǩ����ȷ���룻
//ICAO_adress_with_mark���ڴ��ICAO��ַ���õ�24λ����25λ�Ǳ�־λ��Ϊ0ָʾ������data�д���Ǿ�γ����Ϣ��Ϊ1ָʾָʾ������data�д����ǰһ���յ���CPR������Ϣ
//position�����屣�澭γ����Ϣcoordinate[0]��γ�ȣ�coordinate[1]�Ǿ��ȣ�Ϊdouble���͸�ռ8�ֽڣ�
//position���߱���CPR������ϢCPR_code[0]Ϊγ�ȱ��룬CPR_code[1]Ϊ���ȱ��룬CPR_code[2]Ϊ��ż����ָʾ�������λ��CPR_code[3]δʹ�ã�unsigned int���ͣ���ռ4���ֽ�
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
	  int coordinate[2];//��ʽ�ϱ�ʱ��ʹ��int�ͣ���ʽ�ϱ�ʱ��ʹ��int(1E-7Ϊ��λ)��
	  int altitude;
};
struct location_struct airplane_location;//�˽ṹ������ͽṹ�����ȫ��ʹ��,airplane_location;

//zhangfulong add start ����һ�ṹ��location_struct ICAO ʱ�� ��γ�� �߶� 1500��
struct location_struct airplane_location_static[position_BUFLEN];
struct location_struct airplane_location_static_ronghe[position_BUFLEN];//λ�ø��ں���
//zhangfulong add end

#pragma DATA_ALIGN(data_save,32);//����
//****************************************************************************************************************************
//�ṹ�������������������ṹ�����Ͷ��屾��,�����ٶ���Ϣ�ļ���
//time���ٶ���Ϣ��ʱ���ǩ����ȷ���룻
//ICAO_adress��ICAO��ַ
//E_W_velocity�Ƕ��������ٶȣ���Ϊ������Ϊ������λkm/h��N_S_velocity���ϱ������ٶȣ���Ϊ������Ϊ������λkm/h��
//VERT_velocity�Ǵ�ֱ�����ٶȣ�����Ϊ�����½�Ϊ������λm/sdisu��

/*airplane_velocity.ICAO_address�ĵ�24bit���ڴ��ICAO��ַ����25-27λ����ָʾ�ٶ�״̬����0λ��ţ���24-26λ��
��λ����϶���Ϊ001,010,011,100������״̬������״̬��Ч����λ˳����Ϊ27��26��25�Ӹߵ��͵ķ�ʽ��

��27-25Ϊ001ʱ��ָʾ��ǰ��������"����"���ǳ�����ģʽ����������ʵ���Ƿ������޹أ�
��ʱ���������ϱ�������ٶȾ�����ֵ�����1891.818����ʾ�ڶ������ϱ������ٶ�ֵ����1891.818����/Сʱ��������ֵδ��֪��������ֵС��1891.818ʱ����ֵ��Ӧ����������ٶȣ�
      �ڴ�ֱ������ٶȾ���ֵ�����9938.9184����ʾ��ֱ������ٶȴ���9938.9184��/���ӣ�������ֵδ��֪��������ֵС9938.9184ʱ����ֵ��Ӧ��ֱ������ٶ�?

��27-25Ϊ010ʱ��ָʾ��ǰ������?����"��������ģʽ����������ʵ���Ƿ������޹أ�
�˱���������ϱ�������ٶȾ�����ֵ����?567.272����ʾ�ڶ������ϱ������ٶ�ֵ��?567.272����/Сʱ��������ֵδ��֪��������ֵС��7567.272ʱ����ֵ��Ӧ����������ٶȣ�
      �ڴ�ֱ������ٶȾ���ֵ�����9938.9184����ʾ��ֱ������ٶȴ���9938.9184��/���ӣ�������ֵδ��֪��������ֵС9938.9184ʱ����ֵ��Ӧ��ֱ������ٶȣ�

��27-25Ϊ011ʱ��ָʾ��ǰ��������"����"���ǳ�����ģʽ����������ʵ���Ƿ������޹�
��ʱ��ƽ�淽���ٶ��Լ����귽ʽ������Ϊͳһ��ʾ����������ֽ�Ϊ�������ϱ���������ٶȡ�
      �������ϱ�����������ٶ��?������"�󣬾�����ֵ����1891.818��ָʾ��ƽ���ϵ��ٶȴ���1891.818����/Сʱ��������ֵδ��֪����"������"С��1891.818ʱ����ֵ��Ӧƽ���ϵ��ٶȣ�      
      �ڴ�ֱ������ٶȾ���ֵ�����9938.9184����ʾ��ֱ������ٶȴ���9938.9184��/���ӣ�������ֵδ�ɡ�������ֵС9938.9184ʱ����ֵ��Ӧ��ֱ������ٶȣ�

��27-25Ϊ100ʱ��ָʾ��ǰ��������"����"��������ģʽ����������ʵ���Ƿ������޹أ�
��ʱ��ƽ�淽���ٶ��Լ����귽ʽ������Ϊͳһ��ʾ����������ֽ�Ϊ�������ϱ���������ٶȡ�
      ����������ϱ�����������ٶ���"������"�󣬾�����ֵ����7567.272��ָʾ��ƽ���ϵ��ٶȴ���7567.272����/Сʱ��������ֵδ��֪����"������"С��7567.272ʱ����ֵ��Ӧƽ���ϵ��ٶȣ�        
      �ڴ�ֱ������ٶȾ���ֵ�����9938.9184����ʾ��ֱ������ٶȴ���9938.9184��/���ӣ�������ֵδ��֪��������ֵС9938.9184ʱ����ֵ��Ӧ��ֱ������ȣ�*/
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
struct speed_struct_three airplane_velocity_three_static_ronghe[position_BUFLEN];  //�ٶȸ��ں���1500

struct location_struct airplane_location_tmp = {0};//������¼��ʱ�����е�ICAO�ŵ���Ϣ λ��
struct speed_struct_three airplane_speed_tmp = {0};//������¼��ʱ�����е�ICAO�ŵ���Ϣ �ٶ�
//zhangfulong add end

unsigned char velocity_subtype=0;//�ٶȽ���ȫ�ֱ��������ڴ�����Ϣʹ�ã��û����ù���
struct speed_code
{
	  unsigned char bit46;
	  unsigned short bit47_56;
	  unsigned char bit57;
	  unsigned short bit58_67;
	  unsigned char bit69;
	  unsigned short bit70_78;
}velocity_code;              //�ٶȽ���ȫ�ֽṹ����������ڴ�����Ϣʹ�ã��û����ù���
struct trans_Record
{
  unsigned int ICAO_adress_with_mark;  //190315
  unsigned int time;
  unsigned int flag;
}trans_Record[FIND_RANGE];
//}trans_Record[20];
//****************************************************************************************************************************
/**��ģ��Դ**/
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
							   
/**��ģ��Դ**/
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
extern UINT8 bMacAddr[8]; //mac��ַ
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
void MEM_initial() //�ڴ漰ȫ�ֱ�����ʼ��
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
  	adsb_message_counter=0;//ADSB��Ϣ�ļ���������Ҫ����
	Yaoce_counter=0;//ң����
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
//�Ը���λ�ñ�����Ϣ�õ�new_data��0����ȫ�ֱ���λ����Ϣ�Ľṹ������data_save[]��ʼ������ע�⣺Ϊ��������²��Ե�Ҫ��
//data_save[i].ICAO_adress_with_mark�ĵ�25λ��Ҫ����Ϊ0������������������������������������������������������������
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
//��ʼ�����Ʋ�ȫ �����кܶ������Ҫ��ʼ����
//**************************************************************************
   memset(pulse,0,sizeof(pulse)) ;
   memset(&new_data, 0, sizeof(new_data));     //190305     
   memset(data_save, 0, sizeof(data_save));
   memset(&velocity_code, 0, sizeof(velocity_code));              
   memset(&airplane_location, 0, sizeof(airplane_location));      //�ϱ���γλ���ýṹ��
   memset(&airplane_velocity_three, 0, sizeof(airplane_velocity_three));      //�ϱ��ٶ��ýṹ��
   memset(&ADSB_message, 0, sizeof(ADSB_message));                //�ϱ�ԭʼ88bit�����Ϣ�ý���
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
  //  Uint8 int_cnt=0;//�жϼ���
  	//	C62_disableIER(1<<9);  //1?��??D??
#if 1   
    CHAN_LENTH_FLAG = DSP_INIT_ADDR_2;   //Ҫд2��
    CHAN_LENTH_FLAG = DSP_INIT_ADDR;//
    FPGA_CHAN_FLAG = CHAN_LENTH_FLAG & 0xff;//oxf
    if(FPGA_CHAN_FLAG==0x02) //2  //5  //1280 ADSB AD data****************
#endif
	{
	    flag=1;  //�����ʾ��ʲô���͵��ж?
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
		{    //����δ��
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
		 {   //�����������   ����Ӧ������
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
	    //pEDMA3CC_PaRAM->DST = (unsigned char *)(&s_FPGA_YC); //16�ֽ�ң��
		pEDMA3CC_PaRAM->DST = (unsigned char *)(UTC_time_all);	
		EDMA3CC_ESRH = 0x00010000;  //???��

#if 0		
		//zhangfulong add ʱ�����  û���� Ҫ�������ط�ʹ�ø��£�����û�������
		UTC_time_real[0] = UTC_time_all[13];
		UTC_time_real[1] = UTC_time_all[12];
		UTC_time_real[2] = UTC_time_all[15];
		UTC_time_real[3] = UTC_time_all[14];
		UTC_time_real[4] = UTC_time_all[17];
		UTC_time_real[5] = UTC_time_all[16];
		s_FPGA_YC.time = 0;
		//zhangfulong add ʱ�����
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
		EDMA3CC_ESRH = 0x00010000;  //???��
		return;
	}
#endif
    else if(FPGA_CHAN_FLAG==0x08)  //4  // test
	{
	    flag=4;
		test_flag =1; //�������ģʽ
	    int_cnt[12]++;//
    	EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32			
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_TEST;  //ce4	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x006a0004; //104*4	 	
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)TestArray;
		EDMA3CC_ESRH = 0x00010000;  //???��
		return;
	} 
    else if(FPGA_CHAN_FLAG==0x0a)  //5  // test  
	{
	    flag=5;
		test_flag =0 ; //�˳�����ģʽ
	    int_cnt[12]++;//
    	EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32			
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_TEST;  //ce4	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x006a0004; //104*4	 	
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)TestArray;
		EDMA3CC_ESRH = 0x00010000;  //???��
		return;
	} 
	else if(FPGA_CHAN_FLAG==0xff)
	{
	   FPGA_CHAN_FLAG=0;
	}
#endif

	else if(FPGA_CHAN_FLAG==0x0b)  //20200811 �Ÿ�¡��ӣ��������ֽ���ICAO��
	{
	    flag=4;
		test_flag =1; //�������ģʽ
	    int_cnt[12]++;//
    	EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32			
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_ICAO;  //ce4 ��IP����һ����ַ�ռ�	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x006a0004; //104*4	 	
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)ICAOArray;	//����30*4B = 120 ��UINT��
		EDMA3CC_ESRH = 0x00010000;  //???��
		return;
	} 
	else if(FPGA_CHAN_FLAG==0x55)//0x55����Ŀ������ģʽ
	{
		//
		EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32	
				
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_ICAO;  //ce4 ��IP����һ����ַ�ռ�	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x00860004; //	 0x86 = 134	
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)(&yaokongcmd_all_data.mode);;	//����30*4B = 120 ��UINT��
		EDMA3CC_ESRH = 0x00010000;
		//
		/*
		Uint32 ICAO_number[30] = {0};//�������յ���ICAO�Ŵ�?2λ����������
		int taokong_workmode = 0;// 0��ʾû�з���1������Ŀ������ ��2��Ŀ�����
		int yaokong_jingdu = 0.0;
		int yaokong_weidu = 0.0;
		int yaokong_gaodu = 0.0;
		int yaokong_banzhuijiao = 0;//0-90
		*/
		taokong_workmode = 1;
	}
	else if(FPGA_CHAN_FLAG==0xAA)//0x55Ŀ�����ģʽ
	{
		//
		EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32	
				
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_ICAO;  //ce4 ��IP����һ����ַ�ռ�	
		pEDMA3CC_PaRAM->BCNT_ACNT =0x00860004; //	 0x86 = 134	
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)(&yaokongcmd_all_data.mode);;	//����30*4B = 120 ��UINT��
		EDMA3CC_ESRH = 0x00010000;
		//
		/*
		Uint32 ICAO_number[30] = {0};//�������յ���ICAO��д��32λ����������
		int taokong_workmode = 0;// 0��ʾû�з���1������Ŀ������ ��2��Ŀ�����
		int yaokong_jingdu = 0.0;
		int yaokong_weidu = 0.0;
		int yaokong_gaodu = 0.0;
		int yaokong_banzhuijiao = 0;//0-90
		*/
		taokong_workmode = 2;
	}
	else if(FPGA_CHAN_FLAG==0x03)//0x03 ң��ģʽ
	{
		//flag=4;
		//test_flag =1; //�������ģʽ
	    //int_cnt[12]++;//
		//for (i = 0; i < 200; i++)
		//{
		//	ICAOArray[i] = 0;
		//}
    	EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
		pEDMA3CC_PaRAM->OPT = 0x00130204;//32			
		pEDMA3CC_PaRAM->SRC =  (unsigned char *)FPGA_TO_DSP_ICAO;  //ce4 ��IP����һ����ַ�ռ�	
		//pEDMA3CC_PaRAM->BCNT_ACNT =0x006a0004; //104*4
		pEDMA3CC_PaRAM->BCNT_ACNT =0x00C80004;	 //0xC8 = 200	
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00020004; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)ICAOArray;	//����30*4B = 120 ��UINT��
		EDMA3CC_ESRH = 0x00010000;  //???��
#if 0		
		//����˳�����ݴ����ַ���lead����forѭ����ͷ״̬�����û�ã�Ӧ���ƶ���
		for (i = 0; i < 200; i = i + 2)
		{
			ICAOArray_real[i] = ICAOArray[i + 1];
			ICAOArray_real[i + 1] = ICAOArray[i];
		}
		
		if ((ICAOArray_real[12] = 0xbb) && (ICAOArray_real[13] = 0x77) && (ICAOArray_real[14] = 0x3D) )
		{
			//��ͷ��ȷ ���������У��ͼ��� �� 12 ��00 04 ��ʼ  Uint16 check_sum_1,check_sum_2;
			check_sum_1 = 0;
			check_sum_2 = 0;
			for (i = 12; i < 151; i = i +2 )
			{
				check_sum_1 = (check_sum_1 ^ ( (Uint16)(ICAOArray_real[i] << 8)+(Uint16)(ICAOArray_real[i]) ) );
			}
			check_sum_2 = ( (Uint16)(ICAOArray_real[152] << 8)+(Uint16)(ICAOArray_real[153]) );
//			if (check_sum_1 != check_sum_2)
			{
				//�ܽ���˵��У�����ȷ�����濪ʼ��ֵ���µ����ǵĽṹ������
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
				memcpy(&(yaokongcmd_all_data.jingdu),&(ICAOArray_real[16]),4);//����
				memcpy(&(yaokongcmd_all_data.weidu),&(ICAOArray_real[20]),4);//γ��
				memcpy(&(yaokongcmd_all_data.gaodu),&(ICAOArray_real[24]),4);//�߶�
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
		}//���ˣ�����
#endif
/*
		if (ICAOArray[1] == 0x55)//����Ŀ������
		{
			yaokongcmd_all_data.mode = 1;
			memcpy(&(yaokongcmd_all_data.jingdu),&(ICAOArray[2]),4);//����
			memcpy(&(yaokongcmd_all_data.weidu),&(ICAOArray[6]),4);//γ��
			memcpy(&(yaokongcmd_all_data.gaodu),&(ICAOArray[10]),4);//�߶�
			yaokongcmd_all_data.banzhuijiao = ICAOArray[14];
			yaokongcmd_all_data.chelue_1 = ICAOArray[15];
			yaokongcmd_all_data.chelue_2 = ICAOArray[16];
			yaokongcmd_all_data.chelue_3 = ICAOArray[17];
			for (k = 0; k < 30; k++)
			{
				memcpy(&(yaokongcmd_all_data.ICAO[k]),&(ICAOArray[18 + k*4]),4);;
			}
		}
		else if (ICAOArray[1] == 0xAA)//����ģʽ
		{
			yaokongcmd_all_data.mode = 2;
			memcpy(&(yaokongcmd_all_data.jingdu),&(ICAOArray[2]),4);//����
			memcpy(&(yaokongcmd_all_data.weidu),&(ICAOArray[6]),4);//γ��
			memcpy(&(yaokongcmd_all_data.gaodu),&(ICAOArray[10]),4);//�߶�
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
void EDMA_int()  //EDMA�ж�
{
	int_cnt[5]++;
 	EDMA3CC_ICRH=0x00010000;   //��edma�ж�   //???170718
  //  C62_clearIFR(1<<8); 
 //     *(CIPRL)=0x10;//���DMA4�ж�
 //     *(ECRL)=0x10;//���DMA4�ж�
//	   *(GPVAL)=*(GPVAL)&0xfffffdff;  //GPIO
//----------------------------------------------------
// Ŀǰ����EDMA����4������ ���Ƿ���Ҫ�жϣ�
//EDMA �ж���Ҫ���ݱ�־����
   if(flag==1){
			SEND_ADDR_CACHE[IP_Pkg_index].pwrite = 1; //
			SEND_ADDR_CACHE[IP_Pkg_index].IP_HDADDR=(Uint32)&IP_DATA[IP_Pkg_index][0];
			SND_Catch_Program_Full++; //190201
	        edma_finished = 1;
   }
   else if(flag==2)
   {
		//zhangfulong add ʱ�����
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
		                  + (Uint32)(DSTIP[2]<<8)+ (Uint32)(DSTIP[3]) ; //IP��ַ
		  fpga_SetPort =  (Uint16)(DSTIP[6]<<8)+ DSTIP[9]; //����ͨ��
		  fpga_SetCH   =  DSTIP[7];//�˿ں�
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
//����ֵ0��ָʾ��֡�������ݲ�����/���ǽ���ƺ����ݲ���ȷ/����DF17�ı���, ADSB_message�������ᱻ���£�
  //����ֵ1��ָʾ��֡�������ݴ�������������֡����λ�ñ��Ļ��ٶȱ��ģ�������ADSB_message���������ϴ�ԭʼ���ģ�
  //        �������û�����ADSB_message��������ȡ��Ҫ�ϱ���ԭʼ88bit���ģ������ֽڴ��

  //����ֵ2��ָʾ��֡�������ݴ������������Ҹ�֡��λ�ñ��ģ����ǽ��㲻�ɹ�����������ADSB_message������
  //        �������û�����ADSB_message��������ȡ��Ҫ�ϱ���ԭʼ88bit���ģ������ֽڴ��
  //����ֵ3��ָʾ��֡�������ݴ������������Ҹ�֡��λ�ñ��ģ�����ɹ���������ADSB_message���������������ݿ⣬�������ϱ�λ���õ�airplane_location�ṹ�����
  //	    �������û�����ADSB_message��������ȡ��Ҫ�ϱ���ԭʼ88bit���ģ������ֽڴ��
   //	    �������û�����airplane_location�ṹ���������ȡICAO��λ����Ϣ�ϱ�嶨���ȫ�ֱ�������    
  //����ֵ4��ָʾ��֡�����������ٶȱ��ģ�����δ֪��ԭ�δ�������Ч���ٶ���Ϣ����������ADSB_message��������ȱ��
  //����ֵ5��ָʾ��֡�������ٶȨ�ģ�����ɹ���������ADSB_message�����������ϱ�λ���õ�XXX���飬�ϱ��ٶ���Ϣ��ȱ��
  
//message_type����ֵ0��ָʾ��֡�������ݲ�����,���ǽ���ƺ����ݲ���ȷ,����DF17�ı���, ADSB_message�������ᱻ����;��ʱ���κ���Ϣ��Ҫ�ϴ��򱣴棬�ȴ�������һ�β������ݵĴ��?

//message_type����ֵ1��ָʾ��֡�������ݴ�������������֡����λ�ñ��Ļ��ٶȱ��ģ�������ADSB_message�ṹ����������ϴ�ԭʼ���ģ�
//                   ��������ADSB_message�ṹ������ȡ��Ҫ�ϱ���ԭʼ88bit���ģ�ADSB_message�Ķ��巽ʽ�������������?

//message_type����ֵ2��ָʾ��֡�������ݴ������������Ҹ�֡��λ�ñ��ģ����Ǿ�γλ�ý��㲻�ɹ���������ADSB_message�ṹ����������ϴ�ԭʼ���ģ�
//                   �������û�����ADSB_message�ṹ������ȡ��Ҫ�ϱ���ԭʼ88bit���ģ�ADSB_message�Ķ��巽ʽ���ṹ��������塣
//message_type����ֵ3��ָʾ��֡�������ݴ������������Ҹ�֡��λ�ñ��ģ���γλ�ý���ADSB_message�ṹ�������������ϱ���γλ���õ�airplane_location�ṹ�������
//	                 �������û�����ADSB_message�ṹ������ȡ��Ҫ�ϱ���ԭʼ88bit���ģ�ADSB_message�Ķ��巽ʽ���ṹ��������塣
//	                 �������û�����airplane_location�ṹ���������ȡICAO��ʱ��;�γλ����Ϣ�ϱ�, airplane_location�Ķ��巽ʽ���ṹ��������塣
   
//message_type����ֵ4��ָʾ��֡�������ݴ������������Ҹ�֡���ٶȨ�ģ����ڸ���ԭ��δ�������Ч���ٶ���Ϣ��������ADSB_message�ṹ����������ϴ�ԭʼ���ģ�
//	                 �������û�����ADSB_message�ṹ������ȡ��Ҫ�ϱ���ԭʼ88bit���ģ�ADSB_message�Ķ��巽ʽ���ṹ��������塣
//message_type����ֵ5��ָʾ��֡�������ݴ�����������Ҹ�֡���ٶȱ��ģ��ٶȽ���ɹ���������ADSB_message�������������ϱ��ٶ���Ϣ�õ�airplane_velocity�ṹ�������
//	                 �������û�����ADSB_message�ṹ������ȡ��Ҫ����ԭʼ88bit���ģ�ADSB_message�Ķ��巽ʽ���ṹ��������塣
//	                 �������û�����airplane_velocity�ṹ���������ȡICAO��ʱ����ٶ�λ����Ϣ�ϱ�, airplane_velocity�Ķ��巽ʽ���ṹ��������塣
                       

/*******************************/
#if 1
//����1.2 1400
//ͷ(6)+����(2)+����(2)+���ڼ���(2)+ʱ��(4)+��ЧĿ��(2)+ICAO��16*86��+���(6)

//����3  1400
//ͷ(6)+����(1)+ң��(13)+��Ч��(2)+ads(75*17)+icao(5*20)+���(3)
void udp_sndPacket()  //����4 �������ֱ��ĺͲ������� 
{
	SOCKET   							sudp = INVALID_SOCKET;
  	struct   sockaddr_in 				sin1,sin;
	int 	 							reuse = 1;
	int    								i=0,j=0;
    unsigned int 						recvsize = 1024*8;
	Uint8 pbuf1[1432];//��������
	Uint8 pbuf2[1432];//�ں�����
	Uint8 pbuf3[1432];//ads����
    Uint32 total_cnt=0;  //�ܼ��� ��
	Uint32 total_cnt_2=0;  //�ܼ��� ��
    Uint16 test_cnt=0;   // ���ڼ���
	Uint16 join_cnt=0;  //���ڼ���
    Uint8 ads_cnt=0 ;   //���ڼ���
    Uint8 ads_snd_t=0;    //���Ƿ���Ҫ���
    Uint8 rh_snd_t=0;    //���Ƿ���Ҫ���
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
	};//32�ֽ�--�����(1)--��������(1)--֡������3��--����ͨ����1��--Ԥ����26��

	const Uint8 pack_samp1[6]={
	0x9e,0xda,0xa7,0x8b,0xF4,0xc5
    } ;///��������
    const Uint8 pack_samp2[6]={
	0x9e,0xda,0xa7,0x8b,0xF4,0xc5
    } ;//�ں�����
    const Uint8 pack_samp3[6]={
	0x61,0x25,0x58,0x74,0x0b,0x3a
    } ;//ads����

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
	pbuf1[1426]=0xaa;//β�����
	pbuf1[1427]=0xaa;
	pbuf1[1428]=0xaa;
	pbuf1[1429]=0xaa;
	pbuf1[1430]=0xaa;
	pbuf1[1431]=0xaa;
//--------------------2
    pbuf2[38] =0xdd; //190418
    pbuf2[39] =0xdd;

	pbuf2[1426]=0xaa;//β�����
	pbuf2[1427]=0xaa;
	pbuf2[1428]=0xaa;
	pbuf2[1429]=0xaa;
	pbuf2[1430]=0xaa;
	pbuf2[1431]=0xaa;
//-------------------3   
    for(i=0;i<75;i++)//����3�����
	{
    	memcpy(&(pbuf3[55+i*17]),pack_reserve,6);
    }
	pbuf3[1429]=0x55;//β�����
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
    if ( bind( sudp, (PSA) &sin1, sizeof(sin1) ) < 0 )  //Դ
        ErrorPro( sudp);

   while(1)
   {
		//zhangfulong add ʱ�����
		UTC_time_real[0] = UTC_time_all[13];
		UTC_time_real[1] = UTC_time_all[12];
		UTC_time_real[2] = UTC_time_all[15];
		UTC_time_real[3] = UTC_time_all[14];
		UTC_time_real[4] = UTC_time_all[17];
		UTC_time_real[5] = UTC_time_all[16];
		//zhangfulong add ʱ�����

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
      if(test_flag==1)  //���Ͳ�������
	  {  
//----------��������--------------------------	    
        pbuf1[1]=0x0d;//�ں�����
		if(total_cnt<0xffffff)
		{
			total_cnt++;
		}
		else
		{
 		    total_cnt=0;
		}
       // memcpy(&(pbuf1[2]), &total_cnt, 3);//�ܼ���
	    pbuf1[2] =(total_cnt>>16);
        pbuf1[3] =(total_cnt & 0xFFFF)>>8;
	    pbuf1[4] = (Uint8)(total_cnt & 0xFF);
//[41][42]�����
        //pbuf1[40]=s_FPGA_YC.reserve[0];  //40-41
        //pbuf1[41]=s_FPGA_YC.reserve[1];  //
        pbuf1[40]=TestArray[9];  //40-41
        pbuf1[41]=TestArray[8];

      //  memcpy(&(pbuf1[42]), &test_cnt, 2);//���Լ���
    //    pbuf1[42]=test_cnt<<8;// 190927
        pbuf1[42]=test_cnt>>8; //190927
        pbuf1[43]=(unsigned char)test_cnt&0xff;
		test_cnt++;
      //  memcpy(&(pbuf1[44]), &s_FPGA_YC.UTCtime, 4); //utcʱ��
	   // memcpy(&(pbuf1[44]), &test_time, 4);
 		hhtmp=htonl(test_time);
		memcpy(&(pbuf1[44]), &hhtmp, 4);
	
        pbuf1[49]=TestArray[14]; //����
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
// [49][50]--����    
//[51]��ʼ����¼86����16
     //   memcpy(&(pbuf1[49]),TestArray,200);//   ��ô��䣬����
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
	  else  //�����������籨��
	  {
//------  �ں����� ----------------------
#if 1
      for(j=0;j<2;j++)   //2
	  {
		if(position_cnt>0)  //����Ҫ����Ч��λ����Ϣ
		{
		    pbuf2[1]=0x0d;//�ں����� 
	        //memcpy(&(pbuf2[2]), &total_cnt, 3);//�ܼ���
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
	//[41][42]�����
			pbuf2[40]=s_FPGA_YC.reserve[0];  //
			pbuf2[41]=s_FPGA_YC.reserve[1];
	   //     memcpy(&(pbuf2[42]), &join_cnt, 2);//�ںϼ���
	  //		pbuf2[42] =join_cnt<<8;//190927
        	pbuf2[42] =join_cnt>>8;//190927
			pbuf2[43] =(unsigned char)join_cnt&0xFF;//190418
  	        join_cnt++;
	        memcpy(&(pbuf2[44]), &s_FPGA_YC.UTCtime, 4); //utcʱ�� 
	// [49][50]--����  
	//[51]��ʼ����¼86����16
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
				for(i=rh_snd_t;i<86;i++)//pack_ads_nodata  //���0x0   //190523
				{
				  memcpy(&(pbuf2[50+i*16]), pack_ads_nodata, 16); //51
				}
			}  
	#endif
		   	if(sendto(sudp,pbuf2, 1432, 0, (struct sockaddr *)&sin, sizeof(sin) )<0) //  ����ȷ����������
			{
			;
			//	udp_snd_lost_1++;//	  cmd_buf_test[4]=hh_udp_snd_cnt>>8;
			}
			//	hh_udp_snd_cnt++;
         }  //�ں�����  //	if(position_cnt>0)  //����Ҫ����Ч��λ����Ϣ
	  }//for(j)
#endif
//------- ads����    -------------------
#if 1
         for(j=0;j<14;j++)   //2
		 {
	         if(pulse_cnt_IP>0)  // ����Ҫ��ԭʼ����
			 {
		        pbuf3[1]=0x0c;//ads���� 
				if(total_cnt_2<0xffffff)
				{
					total_cnt_2++;
				}
				else
				{
		 		    total_cnt_2=0;
				}
			 //    memcpy(&(pbuf3[2]), &total_cnt, 3);//�ܼ���
	            pbuf3[2] =(total_cnt_2>>16);
	            pbuf3[3] =(total_cnt_2 & 0xFFFF)>>8;
				pbuf3[4] = (Uint8)(total_cnt_2 & 0xFF);
				pbuf3[38]=ads_cnt;//���ڼ���
				ads_cnt++;
			    memcpy(&(pbuf3[39]), &s_FPGA_YC, 14); //ң��  //39-38 13-14
			
				if(pulse_cnt_IP>=75)
				{
					ads_snd_t =75;
			//[53] [54] --���εĸ���
			        pbuf3[53]=0; 
			        pbuf3[54]= 75 ;  //������Ҫ��pulse_cnt_IP�Ƿ����75

			//[60]��ʼ11��ads���ģ������17
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
			//[53] [54] --���εĸ���
					pbuf3[53]=0;
				    pbuf3[54]=pulse_cnt_IP;
			//[60]��ʼ11��ads���ģ������17
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
						for(i=ads_snd_t;i<73;i++)//pack_ads_nodata  //���0x0
						{
						  memcpy(&(pbuf3[61+i*17]), pack_ads_nodata, 11); //60
						}
						//73 74
	                    memcpy(&(pbuf3[61+73*17]), int_cnt, 11);  //�������ڲ�ң��
	                    memcpy(&(pbuf3[61+74*17]), &(int_cnt[11]), 11);
					}
					else
					{
						for(i=ads_snd_t;i<75;i++)//pack_ads_nodata  //���0x0
						{
						  memcpy(&(pbuf3[61+i*17]), pack_ads_nodata, 11); //60
						}
					}
#endif                     
				}  
		#if 1
		//[1328]��ʼ 100���ֽ��� 5��ICAO ��5*20��  //---1330
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
			        for(i=ads_snd_t;i<5;i++)//pack_ads_nodata  //���0x0
					{
					  memcpy(&(pbuf3[1330+i*20]), pack_ads_nodata, 20);   //1328
					}
				}
#endif

		#endif
			    if(sendto( sudp,pbuf3, 1432, 0, (struct sockaddr *)&sin, sizeof(sin) )<0)//  ����ȷ����������
				{
					;
					//udp_snd_lost_2++;//	  cmd_buf_test[4]=hh_udp_snd_cnt>>8;
				}
	        }//ads����   if(pulse_cnt_IP>0)  // ����Ҫ��ԭʼ����
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
//����EMIF����ԭ��������ģʽ
Uint8 pbuf3[5040 + 2];//ads���� zhangfulong change
Uint8 pbuf2[5040 + 2];//�ں�����

void EMIF_sndPacket()  //����4 �������ֱ��ĺͲ������� 
{
//	SOCKET   							sudp = INVALID_SOCKET;
//  	struct   sockaddr_in 				sin1,sin;
//	int 	 							reuse = 1;
	int    								i=0,j=0;
//    unsigned int 						recvsize = 1024*8;
	//Uint8 pbuf1[1432];//��������
	//Uint8 pbuf2[1432 + 2];//�ں�����
//	Uint8 pbuf3[1432];//ads����
//	Uint8 pbuf3[5040];//ads���� zhangfulong change
	Uint32 adsb_cnt = 0;//adsbԭʼ���ݰ�����

    Uint32 total_cnt=0;  //�ܼ��� ��
	Uint32 total_cnt_2=0;  //�ܼ��� ��
    Uint16 test_cnt=0;   // ���ڼ���
	Uint16 join_cnt=0;  //���ڼ���
    Uint8 ads_cnt=0 ;   //���ڼ���
    Uint8 ads_snd_t=0;    //���Ƿ���Ҫ���
    Uint8 rh_snd_t=0;    //���Ƿ���Ҫ���
	Uint8 rh_snd_t_2=0;    //���Ƿ���Ҫ���
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
	};//32�ֽ�--�����(1)--��������(1)--֡������3��--����ͨ����1��--Ԥ����26��

	const Uint8 pack_samp1[6]={
	0x9e,0xda,0xa7,0x8b,0xF4,0xc5
    } ;///��������
    const Uint8 pack_samp2[6]={
	0x9e,0xda,0xa7,0x8b,0xF4,0xc5
    } ;//�ں�����
    const Uint8 pack_samp3[6]={
	0x61,0x25,0x58,0x74,0x0b,0x3a
    } ;//ads����

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
	
		//zhangfulong add ʱ�����
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

//------  �ں����� ---------------------- //  ��������
		//zhangfulong ȥ�� ʹ�ú���������ںϽ�������
	
			//zhangfulong add start�ں����ݰ�
		for (i = 0; i < 1432 + 2; i++)//Uint8 pbuf2[1432 + 2];//�ں�����
		{
			pbuf2[i] = 0;
		}

		//position_cnt = 150;
		//if(position_cnt>0)  //����Ҫ����Ч��λ����Ϣ
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
				//��ͷ 7788AC8BF6E4
			pbuf2[0] = 0x77;
			pbuf2[1] = 0x88;
			pbuf2[2] = 0xAC;
			pbuf2[3] = 0x8B;
			pbuf2[4] = 0xF6;
			pbuf2[5] = 0xE4;
				//����DDDD
			pbuf2[6] = 0xDD;
			pbuf2[7] = 0xDD;
				//UTCʱ��
			//memcpy(&(pbuf2[8]), &(UTC_time_real[0]), 6);
			//zhangfulong add ʱ�����
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add ʱ�����
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//��ʱ�临�ƹ�ȥ 4->6
				pbuf2[8] = UTC_time_real[0];
				pbuf2[9] = UTC_time_real[1];
				pbuf2[10] = UTC_time_real[2];
				pbuf2[11] = UTC_time_real[3];
				pbuf2[12] = UTC_time_real[4];
				pbuf2[13] = UTC_time_real[5];

				//�ۼƷ�������
			pbuf2[14] = (Uint8)((total_cnt & 0xFF00) >> 8);
			pbuf2[15] = (Uint8)((total_cnt & 0x00FF));
			total_cnt = total_cnt + 1;
				//Ŀ��UTCʱ��
/*			memcpy(&(pbuf2[16]), &airplane_location_tmp.time , 4);//���Ӧ��������Ŀ���ʱ��ɡ�
			memcpy(&(pbuf2[22]), &airplane_location_tmp.ICAO_address , 4);//ICAO
			memcpy(&(pbuf2[26]), &airplane_location_tmp.coordinate[1] , 4);//����
			memcpy(&(pbuf2[30]), &airplane_location_tmp.coordinate[0] , 4);//γ��
			memcpy(&(pbuf2[34]), &airplane_speed_tmp.N_S_velocity , 4);//���ٶ�
			memcpy(&(pbuf2[38]), &airplane_speed_tmp.E_W_velocity , 4);//���ٶ�
			memcpy(&(pbuf2[42]), &airplane_speed_tmp.E_W_velocity , 4);//���ٶ�
			memcpy(&(pbuf2[46]), &airplane_speed_tmp.E_W_velocity , 4);//�߶�
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
			if (ronghe_cnt_p> 144)//��λ��
			{
				rh_snd_t = 144;
			}
			else
			{
				rh_snd_t = ronghe_cnt_p;
			}
			
			pbuf2[18] = (Uint8)((rh_snd_t & 0xFF00) >> 8); 	//���ڼ���
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

				//memcpy(&(pbuf2[18 +2 + i*34 + 10]), &airplane_location_static[position_rd].coordinate[1] , 4);//����
				pbuf2[18 +2 + i*34 + 10] = (airplane_location_static[position_rd].coordinate[1] & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 10 + 1] = (airplane_location_static[position_rd].coordinate[1] & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 10 + 2] = (airplane_location_static[position_rd].coordinate[1] & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 10 + 3] = (airplane_location_static[position_rd].coordinate[1] & 0x000000FF);
				//memcpy(&(pbuf2[18 +2 + i*34 + 14]), &airplane_location_static[position_rd].coordinate[0] , 4);//γ��
				pbuf2[18 +2 + i*34 + 14] = (airplane_location_static[position_rd].coordinate[0] & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 14 + 1] = (airplane_location_static[position_rd].coordinate[0] & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 14 + 2] = (airplane_location_static[position_rd].coordinate[0] & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 14 + 3] = (airplane_location_static[position_rd].coordinate[0] & 0x000000FF);
				//memcpy(&(pbuf2[18 +2 + i*34 + 18]), &airplane_velocity_three_static[position_rd].N_S_velocity , 4);//���ٶ�
				pbuf2[18 +2 + i*34 + 18] = (airplane_velocity_three_static[position_rd].N_S_velocity & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 18 + 1] = (airplane_velocity_three_static[position_rd].N_S_velocity & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 18 + 2] = (airplane_velocity_three_static[position_rd].N_S_velocity & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 18 + 3] = (airplane_velocity_three_static[position_rd].N_S_velocity & 0x000000FF);
				
				//memcpy(&(pbuf2[18 +2 + i*34 + 22]), &airplane_velocity_three_static[position_rd].E_W_velocity , 4);//���ٶ�
				pbuf2[18 +2 + i*34 + 22] = (airplane_velocity_three_static[position_rd].E_W_velocity & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 22 + 1] = (airplane_velocity_three_static[position_rd].E_W_velocity & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 22 + 2] = (airplane_velocity_three_static[position_rd].E_W_velocity & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 22 + 3] = (airplane_velocity_three_static[position_rd].E_W_velocity & 0x000000FF);
				//memcpy(&(pbuf2[18 +2 + i*34 + 24]), &airplane_velocity_three_static[position_rd].VERT_velocity , 4);//���ٶ�
				pbuf2[18 +2 + i*34 + 24] = (airplane_velocity_three_static[position_rd].VERT_velocity & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 24 + 1] = (airplane_velocity_three_static[position_rd].VERT_velocity & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 24 + 2] = (airplane_velocity_three_static[position_rd].VERT_velocity & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 24 + 3] = (airplane_velocity_three_static[position_rd].VERT_velocity & 0x000000FF);
				//memcpy(&(pbuf2[18 +2 + i*34 + 28]), &airplane_location_static[position_rd].altitude , 4);//�߶�
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
				//memcpy(&(pbuf2[18 +2 + i*34 + 10]), &airplane_location_static_ronghe[ronghe_cnt_p].coordinate[1] , 4);//����
				pbuf2[18 +2 + i*34 + 10] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[1] & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 10 + 1] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[1] & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 10 + 2] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[1] & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 10 + 3] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[1] & 0x000000FF);
				//memcpy(&(pbuf2[18 + i*34 + 14]), &airplane_location_static_ronghe[ronghe_cnt_p].coordinate[0] , 4);//γ��
				pbuf2[18 +2 + i*34 + 14] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[0] & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 14 + 1] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[0] & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 14 + 2] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[0] & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 14 + 3] = (airplane_location_static_ronghe[ronghe_cnt_p].coordinate[0] & 0x000000FF);
			//	memcpy(&(pbuf2[18 + i*34 + 18]), &airplane_velocity_three_static[position_rd].N_S_velocity , 4);//���ٶ�
			//	memcpy(&(pbuf2[18 + i*34 + 22]), &airplane_velocity_three_static[position_rd].E_W_velocity , 4);//���ٶ�
			//	memcpy(&(pbuf2[18 + i*34 + 26]), &airplane_velocity_three_static[position_rd].E_W_velocity , 4);//���ٶ�
				//memcpy(&(pbuf2[18 +2 + i*34 + 30]), &airplane_location_static_ronghe[ronghe_cnt_p].altitude , 4);//�߶�
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

			//s�ٶ�
			if (rh_snd_t== 144)//λ��ռ����û���ٶ�
			{
				rh_snd_t_2 = 0;
			}
			else if ( rh_snd_t + ronghe_cnt_v < 144)//�ٶȿ���ȫ��д��ȥ
			{
				rh_snd_t_2 = ronghe_cnt_v;
			}else//�ٶ�д��ȥ����
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
			//	memcpy(&(pbuf2[18 + i*34 + 10]), &airplane_location_static[ronghe_cnt_p].coordinate[1] , 4);//����
			//	memcpy(&(pbuf2[18 + i*34 + 14]), &airplane_location_static[ronghe_cnt_p].coordinate[0] , 4);//γ��
				//memcpy(&(pbuf2[18+2 + i*34 + 18]), &airplane_velocity_three_static_ronghe[position_rd].N_S_velocity , 4);//���ٶ�
				pbuf2[18 +2 + i*34 + 18] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].N_S_velocity & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 18 + 1] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].N_S_velocity & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 18 + 2] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].N_S_velocity & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 18 + 3] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].N_S_velocity & 0x000000FF);
				//memcpy(&(pbuf2[18+2 + i*34 + 22]), &airplane_velocity_three_static_ronghe[position_rd].E_W_velocity , 4);//���ٶ�
				pbuf2[18 +2 + i*34 + 22] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].E_W_velocity & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 22 + 1] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].E_W_velocity & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 22 + 2] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].E_W_velocity & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 22 + 3] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].E_W_velocity & 0x000000FF);
				//memcpy(&(pbuf2[18+2 + i*34 + 26]), &airplane_velocity_three_static_ronghe[position_rd].E_W_velocity , 4);//���ٶ�
				pbuf2[18 +2 + i*34 + 26] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].VERT_velocity & 0xFF000000)>>24;
				pbuf2[18 +2 + i*34 + 26 + 1] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].VERT_velocity & 0x00FF0000)>>16;
				pbuf2[18 +2 + i*34 + 26 + 2] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].VERT_velocity & 0x0000FF00)>>8;
				pbuf2[18 +2 + i*34 + 26 + 3] = (airplane_velocity_three_static_ronghe[ronghe_cnt_v].VERT_velocity & 0x000000FF);
			//	memcpy(&(pbuf2[18 + i*34 + 30]), &airplane_location_static[ronghe_cnt_p].altitude , 4);//�߶�
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
			pbuf2[18] = (Uint8)((rh_snd_t & 0xFF00) >> 8); 	//���ڼ���
			pbuf2[19] = (Uint8)((rh_snd_t & 0x00FF));


			for(i = rh_snd_t; i < 144; i++)
			{
				memcpy(&(pbuf2[18 +2 + i*34]), pack_ads_nodata , 34);
					;//pack_ads_nodata
			}

			for (i = 4913 +2; i < 5040; i++)//���4914-5040��AA
			{
				pbuf2[i] = 0xAA;
			}
			


				//zhangfulong add end
				//�ں�������Ҫת��EMIF�����-zhangfulong
	
			for (i = 0; i< 10; i++)//504 = 0x1F8
			{
				TaskSleep(1);
				EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
				pEDMA3CC_PaRAM->OPT = 0x00130104;//16 
				pEDMA3CC_PaRAM->SRC =  (unsigned char *)(&pbuf2[i * 0x1F8]);  //ce4
				//	pEDMA3CC_PaRAM->SRC =  (unsigned char *)XSH_Array2;  //ce4	
				//pEDMA3CC_PaRAM->BCNT_ACNT =0x13B00001;//0x13B0 = 5040  //1280*4	//0x003d0001D���?FPGA??��??��A0F	//51
				//pEDMA3CC_PaRAM->BCNT_ACNT =0x02000001;//һ�η�512������10�η���9��512
				pEDMA3CC_PaRAM->BCNT_ACNT =0x01F80001;
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
				pEDMA3CC_PaRAM->DST = (unsigned char *)(DSP_TO_FPGA_RONGHE_PBUF2 + 0x1F8 * i);//Ŀ���ַ��Ҫ�޸ģ�FPGA�����ṩ
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
				//����DDDD
			pbuf2[6] = 0xDD;
			pbuf2[7] = 0xDD;
				//UTCʱ��
			//memcpy(&(pbuf2[8]), &(UTC_time_real[0]), 6);
			//zhangfulong add ʱ�����
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add ʱ�����
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//��ʱ�临�ƹ�ȥ 4->6
				pbuf2[8] = UTC_time_real[0];
				pbuf2[9] = UTC_time_real[1];
				pbuf2[10] = UTC_time_real[2];
				pbuf2[11] = UTC_time_real[3];
				pbuf2[12] = UTC_time_real[4];
				pbuf2[13] = UTC_time_real[5];
				//�ۼƷ�������
			pbuf2[14] = (Uint8)((total_cnt & 0xFF00) >> 8);
			pbuf2[15] = (Uint8)((total_cnt & 0x00FF));
			total_cnt = total_cnt + 1;
				//Ŀ��UTCʱ��
			
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
			for (i = 4913 + 2; i < 5040; i++)//���4914-5040��AA
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
				//pEDMA3CC_PaRAM->BCNT_ACNT =0x13B00001;//0x13B0 = 5040  //1280*4	//0x003d0001D���?FPGA??��??��A0F	//51
				//pEDMA3CC_PaRAM->BCNT_ACNT =0x02000001;//һ�η�512������10�η���9��512
				pEDMA3CC_PaRAM->BCNT_ACNT =0x01F80001;
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
				pEDMA3CC_PaRAM->DST = (unsigned char *)(DSP_TO_FPGA_RONGHE_PBUF2 + 0x1F8 * i);//Ŀ���ַ��Ҫ�޸ģ�FPGA�����ṩ
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
			}

		}
      

//------- ads����    ------------------- //   ԭʼ����

				//zhangfulong add start ԭʼ�������� д��pbuf3����
		//if(pulse_cnt_IP>0)
		//{
		//Uint8 pbuf3[5040 + 2];��ʼ��
//		for (i = 0; i < 5040 + 2; i++)
//		{
//			pbuf3[i] = 0;
//		}
			//��ͷ612558740b3a
		pbuf3[0] = 0x61;
		pbuf3[1] = 0x25;
		pbuf3[2] = 0x58;
		pbuf3[3] = 0x74;
		pbuf3[4] = 0x0B;
		pbuf3[5] = 0x3A;
		if(pulse_cnt_IP>0)  // ����Ҫ��ԭʼ����
	 	{
				//������
//			pbuf3[6] = (Uint8) ((adsb_cnt & 0xFF00)>>8);
//			pbuf3[7] = (Uint8) (adsb_cnt & 0x00FF);
			pbuf3[6] = (Uint8) (adsb_cnt & 0x00FF);
			adsb_cnt = adsb_cnt + 1;//�Լ�1 ����
			//memcpy(&(pbuf3[7]),&(UTC_time_real[0]),6);//ʱ�� 8 9 10 11
			//zhangfulong add ʱ�����
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add ʱ�����
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//��ʱ�临�ƹ�ȥ 4->6
				pbuf3[7] = UTC_time_real[0];
				pbuf3[8] = UTC_time_real[1];
				pbuf3[9] = UTC_time_real[2];
				pbuf3[10] = UTC_time_real[3];
				pbuf3[11] = UTC_time_real[4];
				pbuf3[12] = UTC_time_real[5];
				
				
		        	//pbuf3[1]=0x0c;//ads���� 
			if(total_cnt_2<0xffffff)
			{
				total_cnt_2++;
			}
			else
			{
 		   		total_cnt_2=0;
			}
			 
			ads_cnt++;
			    //memcpy(&(pbuf3[39]), &s_FPGA_YC, 14); //ң��  //39-38 13-14


			if(pulse_cnt_IP>=295)
			{
				ads_snd_t =295;
			}
			else
			{
				ads_snd_t =pulse_cnt_IP;
			}
			//[53] [54] --���εĸ���
	        pbuf3[14 - 1]=(Uint8)((ads_snd_t & 0x00FF) >> 8); 
	        pbuf3[15 - 1]= (Uint8) (ads_snd_t & 0x00FF) ;  //������Ҫ��pulse_cnt_IP�Ƿ����75

			//[60]��ʼ11��ads���ģ������17
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
			for (i = 0; i < 27; i++)//���
			{
				if (15 + 295 * 17 + i < 5040)
				{
					pbuf3[15 + 295 * 17 + i] = 0x55;
				}
			}
					
				

				//zhangfulong add end ԭʼ�������� д��pbuf3����
				//ADSB������Ҫת��EMIF�����-zhangfulong
			
			for (i = 0; i< 10; i++)
			{
				TaskSleep(1);
				EDMA3CC_IESRH = 0x00010000;   //neseceary  //170719		
				pEDMA3CC_PaRAM->OPT = 0x00130104;//16 
				pEDMA3CC_PaRAM->SRC =  (unsigned char *)(&pbuf3[i * 0x1F8]);  //ce4
				//	pEDMA3CC_PaRAM->SRC =  (unsigned char *)XSH_Array2;  //ce4	
				//pEDMA3CC_PaRAM->BCNT_ACNT =0x13B00001;//0x13B0 = 5040  //1280*4	//0x003d0001D���?FPGA??��??��A0F	//51
				pEDMA3CC_PaRAM->BCNT_ACNT =0x01F80001;//һ�η�512������10�η���9��512
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
				pEDMA3CC_PaRAM->DST = (unsigned char *)(DSP_TO_FPGA_ADSB_PBUF3 + 0x1F8 * i);//Ŀ���ַ��Ҫ�޸ģ�FPGA�����ṩ
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
			}

		}//end  if(pulse_cnt_IP>0)
		else
		{
			//pbuf3[6] = (Uint8) ((adsb_cnt & 0xFF00)>>8);
			pbuf3[6] = (Uint8) (adsb_cnt & 0x00FF);
			//pbuf3[7] = (Uint8) (adsb_cnt & 0x00FF);
			adsb_cnt = adsb_cnt + 1;//�Լ�1 ����
			//memcpy(&(pbuf3[8-1]),&(UTC_time_real[0]),6);//ʱ�� 8 9 10 11
			//zhangfulong add ʱ�����
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add ʱ�����
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//��ʱ�临�ƹ�ȥ 4->6
				pbuf3[7] = UTC_time_real[0];
				pbuf3[8] = UTC_time_real[1];
				pbuf3[9] = UTC_time_real[2];
				pbuf3[10] = UTC_time_real[3];
				pbuf3[11] = UTC_time_real[4];
				pbuf3[12] = UTC_time_real[5];
				

			ads_snd_t = 0;
			//[53] [54] --���εĸ���
	        pbuf3[14-1]=(Uint8)((ads_snd_t & 0x00FF) >> 8); 
	        pbuf3[15-1]= (Uint8) (ads_snd_t & 0x00FF) ;  //������Ҫ��pulse_cnt_IP�Ƿ����75

			//[60]��ʼ11��ads���ģ������17
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
				//pEDMA3CC_PaRAM->BCNT_ACNT =0x13B00001;//0x13B0 = 5040  //1280*4	//0x003d0001D���?FPGA??��??��A0F	//51
				pEDMA3CC_PaRAM->BCNT_ACNT =0x01F80001;//һ�η�512������10�η���9��512
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
				pEDMA3CC_PaRAM->DST = (unsigned char *)(DSP_TO_FPGA_ADSB_PBUF3 + 0x1F8 * i);//Ŀ���ַҪ�޸ģ�FPGA�����ṩ
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
			}
			
			
		}

			    
	   //     }//ads����   if(pulse_cnt_IP>0)  // ����Ҫ��ԭʼ����
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
void get_Pulse()  //����1�� 1280->112
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
//����һ�´���Ķ�����ʲô��--IP_DATA[g_u32IpSendIndex];
//Ӧ���Ǹ����飬ֻҪ�ǿվͿ��Դ���--SND_Catch_Program_Full>0
//����Ҫά��һ��ָ�� --g_u32IpSendIndex
//�����ʲô��
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
				  data_fifo= IP_DATA[g_u32IpSendIndex];	      //data_fifo= first_fifo_1;//����һ��
	              data_head=0;
			#if 1
				  for(i=0;i<112;i++) //��ʼ��һ�±���
				  {
				  	  pulse_amp[i]=0;
					  confi[i]=0;
				  }
            #endif
			#if 1
				  for(i=0;i<24;i++)//?????Ϊʲô��������
				  {	
				  	  correct[i]=0; //  correct[i]=1;   //190426
				  } 
           #endif
			      for(i=0;i<10;i++)//2  ��?
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
			      if(data_head==1) //��ͷ
			      {
			        memset(temp,0,sizeof(temp));//lh,������0
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
			        sumoba_pt=data_fifo+data_begin;//ADS-B��ͷ��ʼλ��ָ�� 
			        ampoba_pt=data_fifo+data_begin+80;//ADS-B������ʼλ��ָ��
    				//C62_enableIER(1<<9|1<<8);//9--fpga�ж� 8--edma�ж�
    				refer_amp=com_refer(sumoba_pt);//������ȵĲο�ֵ
    		        //�˹��ʼ�������ڶ����첨�����Խ���ƺ�����Ϊ���м?������ֱֵ��ƽ���õ�����Ҫ��֤��
    				code_confi(refer_amp,ampoba_pt,pulse_amp,confi);//�������λ�����Ŷ�
		
    				crc_check_flag=crc_check(pulse_amp,correct);//CRCУ��
    				if(crc_check_flag==0)  	  
					{
					   	plane_ok=0;//�˱���Ϊ0��ʾ�ɻ�ADS-B�㲥��Ϣ������ȷ
					}
					else
					{
				   		plane_ok=err_corr_8bit(confi,pulse_amp,correct,syndrome);//����
					}
	            	t5=CLK_gethtime();
	            	//gpio7_set_0(); 
			        if(plane_ok==0)//˵������ɹ�
					{ 
				#if 1	
						for(i=0;i<88;i++)  //��ģ��Դ���м��  
				        {
							if(pulse_amp[i]!=pulse_amp_bak[i])
							{
								int_cnt[7]++;
								break;				
							}
						}
				#endif
#if 1
                     //�����޳�ȫ0������
					     hh_message=0;
					     for(i=0;i<5;i++)  //ǰ5λ?������������ȫ0
					     {   
					        hh_message=hh_message|(pulse_amp[i]<<(4-i));//ȡDFλ
						 }
						 if(hh_message!=0)//
						 {
						  //   hh_message=1;
						  //   continue;
						// }
#endif						
							int_cnt[3]++;
							int_cnt_3++;
	//-------------��������
							mmCopy(pulse[pulse_wr],pulse_amp,112<<1);//
							//zhangfulong add ʱ����½����Ĺ���
							//zhangfulong add ʱ�����  û���� Ҫ�������ط�ʹ�ø��£�����û�������
							UTC_time_real[0] = UTC_time_all[13];
							UTC_time_real[1] = UTC_time_all[12];
							UTC_time_real[2] = UTC_time_all[15];
							UTC_time_real[3] = UTC_time_all[14];
							UTC_time_real[4] = UTC_time_all[17];
							UTC_time_real[5] = UTC_time_all[16];
							//s_FPGA_YC.time = 0;
							time_tmp_01 = (long)(UTC_time_real[0]<<40) + (long)(UTC_time_real[1]<<32) + (long)(UTC_time_real[2]<<24) + (long)(UTC_time_real[3]<<16) + (long)(UTC_time_real[4]<<8) + (long)(UTC_time_real[5]);
							time_tmp_01 = ((long)(time_tmp_01/10000)) & 0x00000000FFFFFFFF;
							//zhangfulong add ʱ�����
							s_FPGA_YC.UTCtime = (int)(time_tmp_01);
							//
							//pulse_time[pulse_wr]=s_FPGA_YC.UTCtime;//190521
							//zhangfulong add time change start
							//zhangfulong add ʱ�����
							UTC_time_real[0] = UTC_time_all[13];
							UTC_time_real[1] = UTC_time_all[12];
							UTC_time_real[2] = UTC_time_all[15];
							UTC_time_real[3] = UTC_time_all[14];
							UTC_time_real[4] = UTC_time_all[17];
							UTC_time_real[5] = UTC_time_all[16];
							//zhangfulong add ʱ�����

							pulse_time[pulse_wr]= (UTC_time_real[2]) + (UTC_time_real[3] << 8) +(UTC_time_real[4] << 16) + (UTC_time_real[5]<< 24);
							//pulse_time[pulse_wr]= (int) (((UTC_time_real[0]) + (UTC_time_real[1] << 8) +(UTC_time_real[2] << 16) + (UTC_time_real[3]<< 24) + (UTC_time_real[4]<< 32) + (UTC_time_real[5]<< 40))/10000);

							//zhangfulong add time change end

							if(pulse_cnt<pulse_BUFLEN-2)
							{
								pulse_cnt++;
							}
							else  //������ ����Ӧ������
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
	//-------------������׼����
	                 	//	mmCopy(pulse_IP[pulse_wr_IP],pulse_amp,112);//
					    	get_pulse_TOByte(pulse_IP[pulse_wr_IP],pulse_amp);
						    if(pulse_cnt_IP<pulse_BUFLEN_IP-2)
							{
								pulse_cnt_IP++;
						    }
							else  //������ ���о��϶�����?//�Ƿ�����ʵ�����pulse_BUFLEN_IP
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
			        }// if(plane_ok==0)//˵�������?
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
void get_PosV()  //����2�� 112->����λ��
{
  unsigned short int dp;
  unsigned short int position_ok,velocity_ok;
  unsigned short   * data_fifo;
  unsigned int hhtmp;
  unsigned  IER_tmp=0;
  while(1)
  {
//����һ��data_pro��������Ķ�����ʲô��---pulse[]
//Ӧ���Ǹ����飬ֻҪ�ǿվͿ��Դ���  --pulse_cnt�ǿ�
//����Ҫά��һ��ָ��  --pulse_rd
//�������������ô��ţ�
    
     if(pulse_cnt>0)
     {
        while(1)
		{
		    if(pulse_cnt==1)//1
		    {
			   TaskSleep(1);
		    }
			//ת��һ��
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
			  if(position_ok==2)	//zhangfulong add 20201012 δ֪������ģ�λ�ý��㲻д�����
			  //if (0)
			  {
			   	pos_2_cnt++ ;
			  }
			  else
			  {
			    pos_3_cnt++; 
			    t2=CLK_gethtime();
			//	gpio7_set_0();
/*            //��ȡairplane_location
              airplane_location.time;
              airplane_location.ICAO_adress_with_mark;
			  airplane_location.position.coordinate[0];
			  airplane_location.position.coordinate[1];*/
              //--�ں� 
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
				else  //������ ��Ӧ������
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
              //--ϡ�軯
		        memcpy(position_xsh[position_wr_xsh], &airplane_location, 20);//190418
                memcpy(&position_xsh_ICAO[position_wr_xsh],&airplane_location,4);  //icao 190410
				//ϡ�軯���µĸ�ֵ
				airplane_location_XSH_static[position_wr_xsh].altitude = airplane_location.altitude;
				airplane_location_XSH_static[position_wr_xsh].coordinate[0] = airplane_location.coordinate[0];
				airplane_location_XSH_static[position_wr_xsh].coordinate[1] = airplane_location.coordinate[1];
				airplane_location_XSH_static[position_wr_xsh].ICAO_address = airplane_location.ICAO_address & 0x00FFFFFF;
				airplane_location_XSH_static[position_wr_xsh].time = airplane_location.time;

				
				//zhangfulong add start�����ݴ���ȫ�ֵ����� airplane_location_static�¶���1500ȫ��
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
				else  //������ ������?
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
	      	  //����ֵ2��ָʾ��֡������λ�ñ��ģ����ǽ��㲻�ɹ�����������ADSB_message���飬�Լ����ݿ⣬δ�������Ч��λ����Ϣ
	          //����ֵ3��ָʾ��֡������λ�è�ģ�����ɹ���������ADSB_message���飬���������ݿ⣬�������ϱ�λ���õ�airplane_location�ṹ���	                                       
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
 			//	memcpy(&velocity[velocity_wr][8],&(airplane_velocity_three.E_W_velocity), 12);//  ��ʱ�ģ���Ϊ�ٶȵı�����Ͳ��?0190422
				

				//��¡ add XSH �ٶ�
				airplane_velocity_three_XSH_static[position_wr_xsh_v].ICAO_address = (airplane_velocity_three.ICAO_address) & 0x00FFFFFF;
				airplane_velocity_three_XSH_static[position_wr_xsh_v].E_W_velocity = airplane_velocity_three.E_W_velocity;
				airplane_velocity_three_XSH_static[position_wr_xsh_v].N_S_velocity = airplane_velocity_three.N_S_velocity;
				airplane_velocity_three_XSH_static[position_wr_xsh_v].time = airplane_velocity_three.time;
				airplane_velocity_three_XSH_static[position_wr_xsh_v].VERT_velocity = airplane_velocity_three.VERT_velocity;
				if(position_wr_xsh_v<position_BUFLEN_xsh-2)
				{
					position_wr_xsh_v++;
				}
				else  //������ ������?
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

 			//	memcpy(&velocity[velocity_wr][8],&(airplane_velocity_three.E_W_velocity), 12);//  ��ʱ�ģ���Ϊ�ٶȵı�����Ͳ��?0190422
				
				
				if(velocity_cnt<velocity_BUFLEN-2)
				{
			     	velocity_cnt++;
				}
				else  //������ ��Ӧ������
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
		       //����ֵ4��ָʾ��֡�������ٶȱ��ģ����ǽ��㲻�ɹ�����������ADSB_message���飬����δ֪��ԭ��δ�������Ч���ٶ���Ϣ
			   //����ֵ5��ָʾ��֡�������ٶȱ��ģ�����ɹ���������ADSB_message���飬�������ϱ�λ���õ�XXX���飬�ϱ�λ����Ϣ	                                       
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
   Uint8 i=0; //50����û�ҵ�
   Uint8 ret=FIND_LOST;
   if(end_pos[num]>start_pos[num])  //1��
   {
     for(i=start_pos[num];i<=end_pos[num];i++)
     {
        if(icao==trans_Record[i].ICAO_adress_with_mark)
		{
		    ret=i;
		    break;//�ҵ��к� 
		}
     }
   }
   else  //2��
   {
      for(i=0;i<=end_pos[num];i++)
      {
         if(icao==trans_Record[i].ICAO_adress_with_mark)
	     { 
		     ret=i;
         	 break;//�ҵ������
		 }
      }
      for(i=start_pos[num];i<FIND_RANGE;i++)
      {
        if(icao==trans_Record[i].ICAO_adress_with_mark)
		{
		    ret=i;
		    break;//�ҵ������ 
		}
      }
   } 
   return ret;
}
/*****************************************/
Uint32 XSH_last_ICAOnumber[4] = {0};
int find_in_lastICAOnumber(int ICAO)
{
	//��������XSH_last_ICAOnumber��������ָ����ICAO��Ҫ���з���1������û�еĻ�����0
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

void xsh_2_task()  //20190211  ����3  ϡ�軯
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
    const Uint8 pack_xsh_nodata[20]={0};  //���0
    memcpy(XSH_Array, xsh_samp, 8);//
//��ʼ��----
    for(i=0;i<FIND_RANGE;i++)
	{
		trans_Record[i].ICAO_adress_with_mark=0xffffffff;
		start_pos[i]=(i+5+FIND_RANGE-1)%FIND_RANGE;
    	end_pos[i]=(i+FIND_RANGE-1)%FIND_RANGE;
	}

//����֡ͷ
 	while( 1 )
	{
		//zhangfulong add ʱ�����
		UTC_time_real[0] = UTC_time_all[13];
		UTC_time_real[1] = UTC_time_all[12];
		UTC_time_real[2] = UTC_time_all[15];
		UTC_time_real[3] = UTC_time_all[14];
		UTC_time_real[4] = UTC_time_all[17];
		UTC_time_real[5] = UTC_time_all[16];
		//zhangfulong add ʱ�����
		for (k = 0; k < 114; k++)
		{
			XSH_Array[k] = 0;
		}

		flag=5; //
		XSH_Array[4]++; //����
	//	XSH_Array2[23]++;
	//	XSH_Array2[23+16]++;
	//  XSH_Array2[23+16+16]++;
	//	XSH_Array2[23+16+16+16]++;
		while((position_cnt_xsh<20)&&(time_delay<800))  //�ۼ�һ���Ľ�������
		{
		  TaskSleep(1);   //�ۼ������Ҫ�۳�
		  time_delay++;
		}
#if 1
	    //1 ά��һ��trans_Record
		tR_add_cnt=0;
        while(position_cnt_xsh>0)  // �������Խ�
		//if(position_cnt_xsh>0)
		{
		 	fi=find_ICAO_2(sum_cnt,(position_xsh_ICAO[position_rd_xsh]&0x00ffffff));
#if 0			//�Ÿ�¡add ȥ��ԭ������䣬����д
            if(FIND_LOST==fi) //50  û�ҵ�
	        {
                 //��������
				 trans_Record[sum_cnt].ICAO_adress_with_mark=position_xsh_ICAO[position_rd_xsh] & 0x00ffffff;

			     trans_Record[sum_cnt].time=(Uint32)(position_xsh[position_rd_xsh][4]<<24)
			            +(Uint32)(position_xsh[position_rd_xsh][5]<<16)
			            +(Uint32)(position_xsh[position_rd_xsh][6]<<8)
			            +position_xsh[position_rd_xsh][7];
		
				 trans_Record[sum_cnt].flag=1; 
			     memcpy(&(XSH_Array[20+tR_add_cnt*20]), position_xsh[position_rd_xsh], 20); //20

	              //ά��ָ�� 
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
            else  //�ҵ�  --��������?
		    {
	           ; //���������һ��//������ô��?
		       trans_Record[fi].time=(Uint32)(position_xsh[position_rd_xsh][4]<<24)
		            +(Uint32)(position_xsh[position_rd_xsh][5]<<16)
		            +(Uint32)(position_xsh[position_rd_xsh][6]<<8)
		            +position_xsh[position_rd_xsh][7];  //����ʱ��  --��ʵû���κ�����
			  //ά��ָ��
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
			if(FIND_LOST==fi) //50  û�ҵ�
	        {
                 //��������
				 trans_Record[sum_cnt].ICAO_adress_with_mark=position_xsh_ICAO[position_rd_xsh] & 0x00ffffff;

			     trans_Record[sum_cnt].time=(Uint32)(position_xsh[position_rd_xsh][4]<<24)
			            +(Uint32)(position_xsh[position_rd_xsh][5]<<16)
			            +(Uint32)(position_xsh[position_rd_xsh][6]<<8)
			            +position_xsh[position_rd_xsh][7];
		
				 trans_Record[sum_cnt].flag=1; 
			     //memcpy(&(XSH_Array[20+tR_add_cnt*20]), position_xsh[position_rd_xsh], 20); //20
				//�������
				//ȷ�����ظ�
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
				else//�ظ���д0
				{
					memcpy(&(XSH_Array[tR_add_cnt * 12 + 0]), pack_xsh_nodata, 4);
					memcpy(&(XSH_Array[tR_add_cnt * 12 + 4]), pack_xsh_nodata, 4);
					memcpy(&(XSH_Array[tR_add_cnt * 12 + 8]), pack_xsh_nodata, 4);
					XSH_last_ICAOnumber[tR_add_cnt] = 0;
					
				}
			
				//��¼����ICAO��
				//XSH_Array[] = ;
				
	              //ά��ָ�� 
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
            else  //�ҵ�  --���������
		    {
	           ; //���������һ��//������ô��?
		       trans_Record[fi].time=(Uint32)(position_xsh[position_rd_xsh][4]<<24)
		            +(Uint32)(position_xsh[position_rd_xsh][5]<<16)
		            +(Uint32)(position_xsh[position_rd_xsh][6]<<8)
		            +position_xsh[position_rd_xsh][7];  //����ʱ��  --��ʵû���κ�����
			  //ά��ָ��
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
//���	
		if(tR_add_cnt<3)//�Ƿ�Ҫ���
		{
		    for(i=tR_add_cnt;i<3;i++)//pack_ads_nodata  //���0x0
			{
			    //memcpy(&(XSH_Array[20+i*20]), pack_xsh_nodata, 20); //20
				memcpy(&(XSH_Array[i * 12 + 8]), pack_xsh_nodata, 12);
				sum_cnt++;
			}
		}
		//����ٶ�
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
		//ϡ�軯���ݸ��ģ�ǰ���0��ʱ�䣬���������������������
		/*for (i = 51; i > 12; i--)//���ݰ��ȥ
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
		//memcpy(&(XSH_Array[0]),&(UTC_time_real[0]),4+ 2);//��ʱ�临�ƹ�ȥ

				//zhangfulong add ʱ�����
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add ʱ�����
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//��ʱ�临�ƹ�ȥ 4->6
				XSH_Array[0] = UTC_time_real[0];
				XSH_Array[1] = UTC_time_real[1];
				XSH_Array[2] = UTC_time_real[2];
				XSH_Array[3] = UTC_time_real[3];
				XSH_Array[4] = UTC_time_real[4];
				XSH_Array[5] = UTC_time_real[5];
				//XSH ʱ��
		XSH_Array[6] = XSH_bao_cnt;//����
		XSH_bao_cnt = (XSH_bao_cnt + 1) & 0xFF;//
		//XSH_Array[7] = (up_commend_count) & 0xFF;
		XSH_Array[7] = (yaokongcmd_all_data.chelue_new) & 0xFF;
		//XSH_Array[59] = 0;

		//60��ʼ����������
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
		pEDMA3CC_PaRAM->BCNT_ACNT =0x005E0001; //1280*4	//0x003d0001D���?FPGA??��??��A0F	//51
		pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
	    pEDMA3CC_PaRAM->DST = (unsigned char *)DSP_TO_FPGA_XSH;
	    int_cnt[10]++;//
		EDMA3CC_ESRH = 0x00010000;  //???��
   	 	if(sum_cnt>=FIND_RANGE) //����߽�Σ��
			sum_cnt=0;
        if(time_delay<2000)
   	   	{
   	   		 TaskSleep(2000-time_delay); //1s
			 time_delay=0;
		}
		else
		{
		    time_delay=0; //�����֧
		}
		//printf("XSH count is %d \n",XSH_bao_cnt);
	}//while(1)
}

int V_get(int ICAO)// ����ICAO�ţ��������һ����ţ����µ��ٶȵ�����λ����airplane_velocity_three_static���
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
			if(result < 0)//��һ�ν�
			{
				result = i;
			}
			else//����
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
int P_get(int ICAO)//����ICAO�ţ��������һ����ţ����µ�λ�õ�����λ����airplane_location_static���
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
			if(result < 0)//��һ�ν�
			{
				result = i;
			}
			else//����
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

//�����������ʼ����
//	struct location_struct airplane_location_tmp = {0};//������¼��ʱ�����е�ICAO�ŵ���Ϣ λ��
//	struct speed_struct_three airplane_speed_tmp = {0};//������¼��ʱ�����е�ICAO�ŵ���Ϣ �ٶ�
	struct location_struct_withnumber
	{
	  unsigned int ICAO_address;
	  unsigned int time;
	  int coordinate[2];//��ʽ�ϱ�ʱ��ʹ��int����ʽ�ϱ�ʱ��ʹ��int(1E-7Ϊ��λ)��
	  int altitude;
	  int number;
	};
	struct location_struct_withnumber airplane_location_array_tmp[position_BUFLEN] = {0};//�����������´洢�������ڵ�

	int location_arrary[1500];//�ñ���������¼�������Ե���������λ�ã�ÿ�ο�ʼǰ��Ӧ�ø�ֵ��-1״̬
//�¼Ӳ��Ժ��������水��Ҫ����ù�ϵ�����ú�����˳��   ����������ʹ��
// ���� ��airplane_location_array_tmp�����е�һ��λ��ֵ ��¼location_arrary[1500]��
//���� ��airplane_location_array_tmp�е�countֵ��ǰ�����ж������õ�
void chelue_FF(const int count)//����Ƶ������
{
	int i,j,k;
	int location_arrary_tmp[1500] = {-1};//��ʱ��¼
	int max_location;
	int count_tmp = 0;
	for (i = 0; i < count; i++)
	{
		for (j = i; j < count ; j++)
		{
			if((airplane_location_array_tmp[i].ICAO_address == airplane_location_array_tmp[j].ICAO_address) && (airplane_location_array_tmp[i].ICAO_address != 0) )
			{
				//��һ�ο϶������
				airplane_location_array_tmp[i].number = airplane_location_array_tmp[i].number + 1;
			}
		}
	}
	//����ѭ����ÿһ��ICAO�ų��ֵĴ���д��airplane_location_array_tmp[count].number����
	//Ƶ�ʵĿ�����ִ�У�����Ҳ��д��
	if (location_arrary[0] == -1)//��һ�ν�
	{
		//��ʼѰ�ҳ���Ƶ�����ķɻ�
		max_location = 0;
		for (i = 1; i < count; i++)
		{
			if ((airplane_location_array_tmp[i].number > airplane_location_array_tmp[max_location].number)&& (airplane_location_array_tmp[i].ICAO_address != 0))
			{
				//��һ���ĳ��ִ�����࣬����max_locationλ��
				max_location = i;
				//���֮ǰ��¼��λ��ֵ
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
				//������ȣ���Ҫcount_tmp�ϼ�һ����
				location_arrary[count_tmp] = i;
				count_tmp = count_tmp + 1;	
			}
			else//������ֵĴ���û��Ŀǰ���ֵ�࣬������
			{
				;
			}
		}
		//��������ѭ����������Ƶ���������д��location_arrary[] ��������
		
	}
	else if ((location_arrary[0] != -1) && (location_arrary[1] == -1))
	{
		//�ܽ���˵����������ֻ��һ��ֵ����ִ�в��ԣ�ֱ�ӷ���
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
		for (i = 0; i < count; i++)//�����������ݿ⣬
		{
			for (j = 1; j < count_tmp; j++)//������һ�����ԵĽ��
			{
				if (airplane_location_array_tmp[i].ICAO_address == airplane_location_array_tmp[location_arrary[j]].ICAO_address)
				//����ɻ�ICAO�ŷ�Χ����һ��������
				{
					if (airplane_location_array_tmp[i].number > airplane_location_array_tmp[location_arrary[max_location]].number)
					{
						//���ִ�������
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
		//�ܽ���˵���������Ը����˲�ֹһ��ICAO�ŵĽ��������Ƚϳ���Ƶ��
		printf("chelue_FF get in !!!!!\n");
		//����������ǰ�棬ֻ������һ�����������
		return;
	}
	;
}

void chelue_AA(const int count)//�߶����
{
	int i, j, k;
	int max_location = 0;
	int count_tmp = 0;
	if (location_arrary[0] == -1)//��һ�ν�
	{
		max_location = 0;
		for (i = 1; i < count; i++)
		{
			if ((airplane_location_array_tmp[max_location].altitude < airplane_location_array_tmp[i].altitude) && (airplane_location_array_tmp[i].ICAO_address != 0) )
			{
				//���µķɻ��߶ȸ���
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
				//����һ����
				location_arrary[count_tmp] = i;
				count_tmp = count_tmp + 1;

				//printf("chelue_AA error two altitude is same !!! \n");
				;
			}else
			{
				//�µķɻ�û��֮ǰ�ĸߣ�������
				;
			}
			;
		}
		//ѭ��֮�󣬽����д��location_arrary[]�������棬Ȼ����Է�����
		return;
	}
	else if ((location_arrary[0] != -1) && (location_arrary[1] == -1))
	{
		//����ʱ����ֻ��һ����ֱ�ӷ���
		return;
	}
	else
	{
		//�ܽ���˵�����������Ϸɻ�������
		count_tmp = 0;
		i = 0;
		while (location_arrary[i] >= 0)
		{
			i++;
			count_tmp++;
		}
		//����ͳ�ƽ����˼�������
		//����ѭ����������λ�õ�����ÿ�����ĸ߶ȵķɻ�
		for (i = 0; i < count_tmp; i++)
		{
			//location_arrary[i]
			for(j = 0; j < 1500; j++)
			{
				if ((airplane_location_array_tmp[location_arrary[i]].ICAO_address ==  airplane_location_array_tmp[j].ICAO_address) && (airplane_location_array_tmp[j].ICAO_address != 0))
				{
					//����ICAO��һ����;
					if (airplane_location_array_tmp[location_arrary[i]].altitude <  airplane_location_array_tmp[j].altitude)
					{
						//�߶ȸ��ߣ�����;
						location_arrary[i] = j;
					}
					else
					{
						;
					}
				}
			}
		}
		//��������ѭ�������ÿһ����ߵ�λ�������,��Ѱ����ߵ�
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

void chelue_DD(const int count)//ʱ������
{
	int i, j, k;
	Uint32 time_tmp = 0;
	int max_location = 0;
	int count_tmp = 0;
	if (location_arrary[0] == -1)//��һ�ν�
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
			{//����һ��ʱ��
				location_arrary[count_tmp] = i;
				count_tmp = count_tmp + 1;
			}
			else
			//�µ�û��֮ǰ��ʱ���£�������
			{
				;
			}
		}
		location_arrary[0] = max_location;
		return;
	}
	else if ((location_arrary[0] != -1) && (location_arrary[1] == -1))
	{
		//ֱ��������
		return;
	}
	else
	{
		//�ܽ�����ֹһ��ǰ���Լ������;
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
					//����ICAO��һ����;
					if (airplane_location_array_tmp[location_arrary[i]].time <  airplane_location_array_tmp[j].time)
					{
						//�߶ȸ��ߣ�����;
						location_arrary[i] = j;
					}
					else
					{
						;
					}
				}
			}
		}
		//������������еĵ�λ�ö������µ�
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

void lead_tesk()	//��������������
{
	
	int i, j, k;
	int count = 0;//��¼���õ����ݵĸ���
	int isfirst = 0;//0��ʾ��һ�Σ�����0��ʾ���ǵ�һ��
	int V_time_last = 0;//��¼��һ�ε�ʱ�䣬����α��Ƿ��и���
	int P_time_last = 0;//��¼��һ�ε�ʱ�䣬����α��Ƿ��и���
	int ICAO_last = 0;//��¼��һ�ε�ICAO�ţ����ǲ�����ͬ��ÿ�θ���
	int v_count = -1;//��¼�õ����ٶȵ����
	int max_location = 0;//��¼��������ICAO�ŵ�λ�ã��Ǹ������Ϣ

	//Uint8 pbuf_yindao[36] = {0};//�������ݸ�ʽ
	//�������ݶ����ȫ�ֵģ�XSHҲҪ�á�
	float tmp1;
	float tmp2;
	float tmp3;
/*
	float a;
	int b;
	float c;
*/
	int second_30_count = 0;//������ÿ��һ�Σ�С��30���հ�������30ִ�к���Ĳ���
	int UTC_time_tmp;
	Uint8 tmp_change[4];
//	memcpy(&(UTC_time_tmp),&s_FPGA_YC,4);//��UTCʱ�����UTC_time_tmp��������

/*	
	a = 0.09;
	memcpy(&(b),&a,4);
	printf("b = %d\n",b);
	memcpy(&(c),&b,4);
	//c = (float *)(&b);
	printf("c = %f\n",c);
	
*/
//	TaskSleep(30 * 1000);//�ȵ�30s
	int count_tmp_zhang_for_test = 0;
	up_commend_count = 0;
	while (1)
	{
		count_tmp_zhang_for_test++;
		//zhangfulong add ʱ�����
/*		UTC_time_real[0] = UTC_time_all[13];
		UTC_time_real[1] = UTC_time_all[12];
		UTC_time_real[2] = UTC_time_all[15];
		UTC_time_real[3] = UTC_time_all[14];
		UTC_time_real[4] = UTC_time_all[17];
		UTC_time_real[5] = UTC_time_all[16];*/
		//zhangfulong add ʱ�����
		//����˳�����ݴ����ַ���lead����forѭ����ͷ״̬
		for (i = 0; i < 200; i = i + 2)
		{
			ICAOArray_real[i] = ICAOArray[i + 1 + 2];
			ICAOArray_real[i + 1] = ICAOArray[i + 2];
		}
		
		if ((ICAOArray_real[12] == 0xbb) && (ICAOArray_real[13] == 0x77) && (ICAOArray_real[14] == 0x3D) )
		{
			//��ͷ��ȷ ���������У��ͼ��� �� 12 ��00 04 ��ʼ  Uint16 check_sum_1,check_sum_2;
			check_sum_1 = 0;
			check_sum_2 = 0;
			for (i = 12; i < 151 - 2; i = i +2 )
			{
				check_sum_1 = (check_sum_1 ^ ( (Uint16)(ICAOArray_real[i] << 8)+(Uint16)(ICAOArray_real[i]) ) );
			}
			check_sum_2 = ( (Uint16)(ICAOArray_real[152 - 2] << 8)+(Uint16)(ICAOArray_real[153 - 2]) );
//			if (check_sum_1 == check_sum_2)		//У����ȥ�����Ժ����
			{
				//�ܽ���˵��У�����ȷ�����濪ʼ��ֵ���µ����ǵĽṹ������
				if (ICAOArray_real[15] == 0x33)//��������
				{
					yaokongcmd_all_data.mode = 1;
				}
				else if (ICAOArray_real[15] == 0xCC)//��ICAO�ŵ�ģʽ
				{
					yaokongcmd_all_data.mode = 2;
				}
				else if (ICAOArray_real[15] == 0xFF)//0xFF ���λ�÷ɻ���Ϣ
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
				memcpy(&(yaokongcmd_all_data.jingdu),&(tmp_change[0]),4);//����
				//yaokongcmd_all_data.jingdu = 90.0;//test ��ʱ
				yaokongcmd_all_data.jingdu = yaokongcmd_all_data.jingdu / 3.1415926 *180;//test ��ʱ
				yaokongcmd_all_data.jingdu = 90.0;
				//memcpy(&(yaokongcmd_all_data.jingdu),&(ICAOArray_real[16]),4);//����
				tmp_change[3] = ICAOArray_real[20];
				tmp_change[2] = ICAOArray_real[21];
				tmp_change[1] = ICAOArray_real[22];
				tmp_change[0] = ICAOArray_real[23];
				memcpy(&(yaokongcmd_all_data.weidu),&(tmp_change[0]),4);//γ��
				//yaokongcmd_all_data.weidu = 90.0;//test ��ʱ
				yaokongcmd_all_data.weidu = yaokongcmd_all_data.weidu /3.1415926 *180;							//20201112
				yaokongcmd_all_data.weidu = 90.0;
				//memcpy(&(yaokongcmd_all_data.weidu),&(ICAOArray_real[20]),4);//γ��
				tmp_change[3] = ICAOArray_real[24];
				tmp_change[2] = ICAOArray_real[25];
				tmp_change[1] = ICAOArray_real[26];
				tmp_change[0] = ICAOArray_real[27];
				memcpy(&(yaokongcmd_all_data.gaodu),&(tmp_change[0]),4);//�߶�
				//memcpy(&(yaokongcmd_all_data.gaodu),&(ICAOArray_real[24]),4);//�߶�
				yaokongcmd_all_data.banzhuijiao = ICAOArray_real[28];
				//yaokongcmd_all_data.banzhuijiao = 90;
				//yaokongcmd_all_data.chelue_1 = ICAOArray_real[29];
				//yaokongcmd_all_data.chelue_2 = ICAOArray_real[30];
				//yaokongcmd_all_data.chelue_3 = ICAOArray_real[31];
				if (ICAOArray_real[29] == 0x00)
				{
					//�߶� Ƶ��
					yaokongcmd_all_data.chelue_1 = 0xAA;
					yaokongcmd_all_data.chelue_2 = 0xFF;
					yaokongcmd_all_data.chelue_3 = 0x00;
				}
				else if (ICAOArray_real[29] == 0x02)
				{
					//Ƶ�� �߶� 
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
				
				isfirst = 0;//�µĲ��ԣ��ӵ�һ�ο�ʼ
				for (i = 0; i < 200; i++)
				{
					ICAOArray_real[i] = 0;
					ICAOArray[i] = 0;
				}
				up_commend_count = up_commend_count + 1;
			}			
		}
outclear:
#if 0		//ȥ��ǰ30s�ĵȴ�״̬�������ң�����ݺ�ʼִ�в���
		//ǰ30s��״̬
		if(second_30_count < 30)
		{
			for (k = 0; k < 34; k++)//��ʼ��
			{
				pbuf_yindao[k] = 0x00;
			}
			pbuf_yindao[0] = 0x6A;//��ͷ
			//UTCʱ��
			memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//��ʱ�临�ƹ�ȥ 4->6

			pbuf_yindao[8 + 2] = 0xF0;//��һ����F0  0-30״̬
			//У���
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
		//ǰ30s��״̬
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

		if (yaokongcmd_all_data.mode == 1)//����Ŀ������
		{
			count = 0;//��¼�Ƿ������������ڵ�����
			if(isfirst == 0)
			{
				count = 0;//��¼�Ƿ������������ڵ�����
				for (i = 0; i < position_BUFLEN; i++)
				{
					//tmp1 = fabs(airplane_location_static[i].coordinate[1] - yaokongcmd_all_data.jingdu);//γ�Ȳ�
					//tmp2 = fabs(airplane_location_static[i].coordinate[0] - yaokongcmd_all_data.weidu);//���Ȳ�
					//memcpy(&(tmp1),&(airplane_location_static[i].coordinate[1]),4);
					//memcpy(&(tmp2),&(airplane_location_static[i].coordinate[0]),4);
					//tmp1 = fabs(tmp1 - yaokongcmd_all_data.jingdu);//γ�Ȳ�
					//tmp2 = fabs(tmp2 - yaokongcmd_all_data.weidu);//���Ȳ�

					tmp1 = fabs(airplane_location_static[i].coordinate[0]/10000000 - yaokongcmd_all_data.jingdu);//γ�Ȳ�
					tmp2 = fabs(airplane_location_static[i].coordinate[1]/10000000 - yaokongcmd_all_data.weidu);//���Ȳ�
					if (airplane_location_static[i].ICAO_address != 0)
					{
						//printf("location ICAO is %d,i = %d ~ \n",airplane_location_static[i].ICAO_address, i);
					}		
					
					if ( (tmp1 <= yaokongcmd_all_data.banzhuijiao) && (tmp2 <= yaokongcmd_all_data.banzhuijiao) && (airplane_location_static[i].ICAO_address != 0))
					{
						//�������ӳ���,��Ӧ��ֵд��ȥ
						airplane_location_array_tmp[count].altitude = airplane_location_static[i].altitude;
						airplane_location_array_tmp[count].coordinate[1] = airplane_location_static[i].coordinate[1];
						airplane_location_array_tmp[count].coordinate[0] = airplane_location_static[i].coordinate[0];
						airplane_location_array_tmp[count].ICAO_address = airplane_location_static[i].ICAO_address;
						airplane_location_array_tmp[count].time = airplane_location_static[i].time;
						airplane_location_array_tmp[count].number = 0;//���������Ϊ0
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
#if 0	//���Ŀ��Ա仯˳��Ĳ����飬�����ߺ����else��֧
				//����ѭ���Ѿ������е����ӳ�������Ϣд��
				//������ͳ�Ƴ��ֵĸ�����Ϣ
				//for (i = 0; i < count; i++)//��ռ���
				//{
				//	airplane_location_array_tmp[i].number = 0;
				//}
				for (i = 0; i < count; i++)
				{
					for (j = i; j < count ; j++)
					{
						if(airplane_location_array_tmp[i].ICAO_address == airplane_location_array_tmp[j].ICAO_address)
						{
							//��һ�ο϶������
							airplane_location_array_tmp[i].number = airplane_location_array_tmp[i].number + 1;
						}
					}
				}
				//����ѭ����ÿһ��ICAO�ų��ֵĴ���д��airplane_location_array_tmp[count].number����
				//�����ҳ�����Ƶ����ߵ�
				max_location = 0;
				for (i = 1; i < count; i++)
				{
					if ((airplane_location_array_tmp[i].number > airplane_location_array_tmp[max_location].number)&& (airplane_location_array_tmp[i].ICAO_address != 0))
					{
						//��һ���ĳ��ִ�����࣬����max_locationλ��
						max_location = i;
					}
					else if (airplane_location_array_tmp[i].number == airplane_location_array_tmp[max_location].number)
					{
						//������ȣ����и߶��ж�
						if (airplane_location_array_tmp[i].altitude >= airplane_location_array_tmp[max_location].altitude)
						{
							//�µĸ߶ȸ��ߣ����и���
							max_location = i;
						}
						else
						{
							;
						}
						;
					}
					else//������ֵĴ���û��Ŀǰ���ֵ�࣬������
					{
						;
					}
				}
#else
				//����˳����Ψһ�Ĳ���
				//
				for(i= 0; i < 1500; i++)
				{
					location_arrary[i] = -1;//���г�ʼ��-1����
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
				//����ѭ�������һ�����ִ������ģ�����max_location����
				//������Ѱ��ָ��ICAO�ŵ�����λ����Ϣ
if (count > 0)//û�ҵ���������
{
				if (count > 0)//��� count ����0 ����������һ���ɻ�
				{
					max_location = P_get(airplane_location_array_tmp[max_location].ICAO_address);//��ʱmax_location����airplane_location_static��������
					airplane_location_tmp.ICAO_address = airplane_location_static[max_location].ICAO_address;
					airplane_location_tmp.altitude = airplane_location_static[max_location].altitude;
					airplane_location_tmp.coordinate[0] = airplane_location_static[max_location].coordinate[0];
					airplane_location_tmp.coordinate[1] = airplane_location_static[max_location].coordinate[1];
					airplane_location_tmp.time = airplane_location_static[max_location].time;
				}
				else//˵��û������
				{
					airplane_location_tmp.ICAO_address = 0;
					airplane_location_tmp.altitude = 0;
					airplane_location_tmp.coordinate[0] = 0;
					airplane_location_tmp.coordinate[1] = 0;
					airplane_location_tmp.time = 0;
				}
				//���ɰ�
				for (k = 0; k < 36; k++)//��ʼ��
				{
					pbuf_yindao[k] = 0x00;
				}
				pbuf_yindao[0] = 0x6A;//��ͷ
				pbuf_yindao[1] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x00FF0000)>>16);//ICAO��
				pbuf_yindao[2] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x0000FF00)>>8);
				pbuf_yindao[3] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x000000FF));
				//UTCʱ��
				//zhangfulong add ʱ�����
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add ʱ�����
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),6);//��ʱ�临�ƹ�ȥ 4->6
				pbuf_yindao[4] = UTC_time_real[0];
				pbuf_yindao[5] = UTC_time_real[1];
				pbuf_yindao[6] = UTC_time_real[2];
				pbuf_yindao[7] = UTC_time_real[3];
				pbuf_yindao[8] = UTC_time_real[4];
				pbuf_yindao[9] = UTC_time_real[5];
				//pbuf_yindao[9] = count_tmp_zhang_for_test & 0x000000FF;//��ʱҪȥ��
				//ʹ�ý��ʱ��
				pbuf_yindao[6] = (airplane_location_tmp.time & 0xFF000000)>>24;
				pbuf_yindao[7] = (airplane_location_tmp.time & 0x00FF0000)>>16;
				pbuf_yindao[8] = (airplane_location_tmp.time & 0x0000FF00)>>8;
				pbuf_yindao[9] = (airplane_location_tmp.time & 0x000000FF);
				//ʹ�ý��ʱ��
				//ʱ�����Ū��
				v_count = V_get(airplane_location_tmp.ICAO_address);
				//����״̬

				pbuf_yindao[8 + 2] = 0xFF;//��һ����FF
				//memcpy(&(pbuf_yindao[9+ 2]),&(airplane_location_tmp.coordinate[1]),4);//����
				//memcpy(&(pbuf_yindao[13+ 2]),&(airplane_location_tmp.coordinate[0]),4);//γ��
				pbuf_yindao[9+ 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0xFF000000) >> 24);
				pbuf_yindao[9+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x00FF0000) >> 16);
				pbuf_yindao[9+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x0000FF00) >> 8);
				pbuf_yindao[9+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x000000FF));

				pbuf_yindao[13+ 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0xFF000000) >> 24);
				pbuf_yindao[13+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x00FF0000) >> 16);
				pbuf_yindao[13+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x0000FF00) >> 8);
				pbuf_yindao[13+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x000000FF));

				if(v_count < 0)//û��ƥ���ٶ�
				{
					pbuf_yindao[17+ 2] = 0x00;
					pbuf_yindao[28+ 2] = 0x00;//�����Ѿ���0��
					V_time_last = 0;
					airplane_speed_tmp.E_W_velocity = 0;
					airplane_speed_tmp.ICAO_address = 0;
					airplane_speed_tmp.N_S_velocity = 0;
					airplane_speed_tmp.time = 0;
					airplane_speed_tmp.VERT_velocity = 0;
				}
				else//��ƥ���ٶ�
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
				//�߶�
				//memcpy(&(pbuf_yindao[29+ 2]),&(airplane_location_tmp.altitude),4);
				pbuf_yindao[29+ 2] = (Uint8)((airplane_location_tmp.altitude & 0xFF000000)>>24);
				pbuf_yindao[29+ 2 + 1] = (Uint8)((airplane_location_tmp.altitude & 0x00FF0000)>>16);
				pbuf_yindao[29+ 2 + 2] = (Uint8)((airplane_location_tmp.altitude & 0x0000FF00)>>8);
				pbuf_yindao[29+ 2 + 3] = (Uint8)((airplane_location_tmp.altitude & 0x000000FF));
				//
				//��¼��һ�ε�ʱ���ICAO
				memcpy(&P_time_last,&(airplane_location_tmp.time), 4);

				//У���
				pbuf_yindao[33+ 2] = 0;
				for( k = 0; k < 35; k++)
				{
					pbuf_yindao[33+ 2] = (pbuf_yindao[33+ 2] + pbuf_yindao[k]) & 0xFF;
				}
				//��ICAO�ű������һ�����
				ICAO_last = airplane_location_tmp.ICAO_address;
				;
				if (airplane_location_tmp.ICAO_address != 0)
				{
					isfirst = isfirst + 1;//��־��һ�ν�������������������
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
				//	pEDMA3CC_PaRAM->BCNT_ACNT =0x00660001; //1280*4	//0x003d0001D���?FPGA??��??��A0F	//51
				pEDMA3CC_PaRAM->BCNT_ACNT =0x00240001;	//0x22 = 34
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
			    pEDMA3CC_PaRAM->DST = (unsigned char *)DSP_TO_FPGA_YINDAO;
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
}//û�ҵ���������
				//�ȴ�15s
				//task_sleep(15);
			}
			else//��������һ�ν�������������������������������?5s���ڵ���������������
			{
				count = 0;//��¼�Ƿ������������ڵ�����
//if (ICAO_last == 0)
{
				for (i = 0; i < position_BUFLEN; i++)
				{
					//tmp1 = fabs(airplane_location_static[i].coordinate[1] - yaokongcmd_all_data.jingdu);//γ�Ȳ�
					//tmp2 = fabs(airplane_location_static[i].coordinate[0] - yaokongcmd_all_data.weidu);//���Ȳ�
					tmp3 = UTC_time_tmp - airplane_location_static[i].time;
					
					//memcpy(&(tmp1),&(airplane_location_static[i].coordinate[1]),4);
					//memcpy(&(tmp2),&(airplane_location_static[i].coordinate[0]),4);
					//tmp1 = fabs(tmp1 - yaokongcmd_all_data.jingdu);//γ�Ȳ�
					//tmp2 = fabs(tmp2 - yaokongcmd_all_data.weidu);//���Ȳ�

					tmp1 = fabs(airplane_location_static[i].coordinate[0]/10000000 - yaokongcmd_all_data.jingdu);//���Ȳ�
					tmp2 = fabs(airplane_location_static[i].coordinate[1]/10000000 - yaokongcmd_all_data.weidu);//γ�Ȳ�
					
					
				
					//if ( (tmp1 <= yaokongcmd_all_data.banzhuijiao) & (tmp2 <= yaokongcmd_all_data.banzhuijiao) & (tmp3 <= 15) )
					//��ʱȥ��ʱ�����ƣ�����ʱ���Ժ��ڼ���
					if ( (tmp1 <= yaokongcmd_all_data.banzhuijiao) & (tmp2 <= yaokongcmd_all_data.banzhuijiao)&& (airplane_location_static[i].ICAO_address != 0))
					{
						//�������ӳ���,��Ӧ��ֵд��ȥ
						airplane_location_array_tmp[count].altitude = airplane_location_static[i].altitude;
						airplane_location_array_tmp[count].coordinate[1] = airplane_location_static[i].coordinate[1];
						airplane_location_array_tmp[count].coordinate[0] = airplane_location_static[i].coordinate[0];
						airplane_location_array_tmp[count].ICAO_address = airplane_location_static[i].ICAO_address;
						airplane_location_array_tmp[count].time = airplane_location_static[i].time;
						airplane_location_array_tmp[count].number = 0;//���������Ϊ0
						count = count + 1;
						;
					}
				//	if ((airplane_location_static[i].ICAO_address & 0x00FFFFFF) == 0x123456)
				//	{
				//		printf("No.%d.altitude = %d;\n",i,airplane_location_static[i].altitude);
				//	}		
				}
				//����ѭ���Ѿ������е����ӳ�������Ϣд��
				//������ͳ�Ƴ��ֵĸ�����Ϣ
				//for (i = 0; i < count; i++)//��ռ���
				//{
				//	airplane_location_array_tmp[i].number = 0;
				//}
#if 0		//ȥ��������ԣ�����һ���ɻ�
				for (i = 0; i < count; i++)
				{
					for (j = i; j < count ; j++)
					{
						if(airplane_location_array_tmp[i].ICAO_address == airplane_location_array_tmp[j].ICAO_address)
						{
							//��һ�ο϶������
							airplane_location_array_tmp[i].number = airplane_location_array_tmp[i].number + 1;
							/*if (airplane_location_array_tmp[j].altitude != 0)
							{
								printf("airplane_location_array_tmp[%d].altitude = %d\n",j,airplane_location_array_tmp[j].altitude);
							}*/
							
						}
					}
				}
				//����ѭ����ÿһ��ICAO�ų��ֵĴ���д��airplane_location_array_tmp[count].number����
				//�����ҳ�����Ƶ����ߵ�
				max_location = 0;
				for (i = 1; i < count; i++)
				{
					if ((airplane_location_array_tmp[i].number > airplane_location_array_tmp[max_location].number) && (airplane_location_array_tmp[i].ICAO_address != 0) )
					{
						//��һ���ĳ��ִ�����࣬����max_locationλ��
						max_location = i;
					}
					else if (airplane_location_array_tmp[i].number == airplane_location_array_tmp[max_location].number)
					{
						//������ȣ����и߶��ж�
						if (airplane_location_array_tmp[i].altitude >= airplane_location_array_tmp[max_location].altitude)
						{
							//�µĸ߶ȸ��ߣ����и���
							max_location = i;
						}
						else
						{
							;
						}
						;
					}
					else//������ֵĴ���û��Ŀǰ���ֵ�࣬������
					{
						;
					}
				}
				//����ѭ�������һ�����ִ������ģ�����max_location����				
				//������Ѱ��ָ��ICAO�ŵ�����λ����Ϣ
				if (count > 0)//��� count ����0 ����������һ���ɻ�
				{
					max_location = P_get(airplane_location_array_tmp[max_location].ICAO_address);//��ʱmax_location����airplane_location_static��������
					airplane_location_tmp.ICAO_address = airplane_location_static[max_location].ICAO_address;
					airplane_location_tmp.altitude = airplane_location_static[max_location].altitude;
					airplane_location_tmp.coordinate[0] = airplane_location_static[max_location].coordinate[0];
					airplane_location_tmp.coordinate[1] = airplane_location_static[max_location].coordinate[1];
					airplane_location_tmp.time = airplane_location_static[max_location].time;
				}
				else//˵��û������
				{
					airplane_location_tmp.ICAO_address = 0;
					airplane_location_tmp.altitude = 0;
					airplane_location_tmp.coordinate[0] = 0;
					airplane_location_tmp.coordinate[1] = 0;
					airplane_location_tmp.time = 0;
				}
#endif			//ȥ��������ԣ�����һ���ɻ�
}
//else//�����֧�ǵ�һ��û�ҵ�����15s����������������ҵ���ICAO�����԰���ICAO���ң�
{
				for (i = 0; i < position_BUFLEN; i++)//�����յ��1500������
				{
					if(airplane_location_static[i].ICAO_address == airplane_location_tmp.ICAO_address)
					{
						//ICAO������,���濴�Ƿ���ʱ�����µģ�
						if(airplane_location_static[i].time > airplane_location_tmp.time)
						{
							memcpy(&airplane_location_tmp,&(airplane_location_static[i]), 20);
							count = count + 1;//count��ʾ�ٶ��и���
						}
						
					}
				//	if (airplane_location_static[i].ICAO_address == 0x00111111)
				//	{
				//		printf("No.%d.altitude = %d;\n",i,airplane_location_static[i].altitude);
				//	}
				}
}//���if elseѭ������
				//���ɰ�
				for (k = 0; k < 36; k++)//��ʼ��
				{
					pbuf_yindao[k] = 0x00;
				}
				pbuf_yindao[0] = 0x6A;//��ͷ
				pbuf_yindao[1] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x00FF0000)>>16);//ICAO��
				pbuf_yindao[2] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x0000FF00)>>8);
				pbuf_yindao[3] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x000000FF));
				//UTCʱ��
				//zhangfulong add ʱ�����
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add ʱ�����
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),4+ 2);//��ʱ�临�ƹ�ȥ
				pbuf_yindao[4] = UTC_time_real[0];
				pbuf_yindao[5] = UTC_time_real[1];
				pbuf_yindao[6] = UTC_time_real[2];
				pbuf_yindao[7] = UTC_time_real[3];
				pbuf_yindao[8] = UTC_time_real[4];
				pbuf_yindao[9] = UTC_time_real[5];
				//pbuf_yindao[9] = count_tmp_zhang_for_test & 0x000000FF;//��ʱҪȥ��
				//ʹ�ý��ʱ��
				pbuf_yindao[6] = (airplane_location_tmp.time & 0xFF000000)>>24;
				pbuf_yindao[7] = (airplane_location_tmp.time & 0x00FF0000)>>16;
				pbuf_yindao[8] = (airplane_location_tmp.time & 0x0000FF00)>>8;
				pbuf_yindao[9] = (airplane_location_tmp.time & 0x000000FF);
				//ʹ�ý��ʱ��
				//v_count = V_get(airplane_location_tmp.ICAO_address);
				//����״̬ �Ѿ����ǵ�һ���ˣ�����������ж���Ч��
#if 1
				if(airplane_location_tmp.ICAO_address == ICAO_last)//ICAO��ͬ
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] | 0xF0;
				}
				else//ICAO��ͬ
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0x0F;//��³?x0X ����һ����ͬ
				}
				
				if ((v_count == -1) && (P_time_last==airplane_location_tmp.time))
				//�ٶ�û�ҵ���λ��ʱ����ͬ
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
				}
				else//�ҵ����ٶ�,
				{
					if((V_time_last == airplane_velocity_three_static[v_count].time) && (P_time_last==airplane_location_tmp.time))
					{
						//�ҵ��ٶ� �ٶ��޸���,λ��Ҳû����
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
					}
					else
					{
						//�ٶ��и��£�����λ���и��£��������
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] | 0x0F;
						printf("PPPPPPPVVVVVVVV\n");
					}
				}
				/*if(v_count == -1)//û�ҵ�
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
				}
				else
				{
					if(V_time_last == airplane_velocity_three_static[v_count].time)
					{
						//�ҵ��ٶ��޸���
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
					}
					else
					{
						//�ٶ��и���
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] | 0x0F;
					}
					
				}*/
#endif
				//pbuf[8] = 0xFF;//��һ����FF
				//memcpy(&(pbuf_yindao[9+ 2]),&(airplane_location_tmp.coordinate[1]),4);//����
				//memcpy(&(pbuf_yindao[13+ 2]),&(airplane_location_tmp.coordinate[0]),4);//γ��
				pbuf_yindao[9+ 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0xFF000000) >> 24);
				pbuf_yindao[9+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x00FF0000) >> 16);
				pbuf_yindao[9+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x0000FF00) >> 8);
				pbuf_yindao[9+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x000000FF));

				pbuf_yindao[13+ 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0xFF000000) >> 24);
				pbuf_yindao[13+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x00FF0000) >> 16);
				pbuf_yindao[13+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x0000FF00) >> 8);
				pbuf_yindao[13+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x000000FF));
				if(v_count < 0)//û��ƥ���ٶ�
				{
					pbuf_yindao[17+ 2] = 0x00;
					pbuf_yindao[28+ 2] = 0x00;//�����Ѿ���0��
					V_time_last = 0;

					airplane_speed_tmp.E_W_velocity = 0;
					airplane_speed_tmp.ICAO_address = 0;
					airplane_speed_tmp.N_S_velocity = 0;
					airplane_speed_tmp.time = 0;
					airplane_speed_tmp.VERT_velocity = 0;
				}
				else//��ƥ���ٶ�
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

					//��¼��һ�ε�ʱ���ICAO �ٶ�
					memcpy(&V_time_last,&(airplane_velocity_three_static[v_count].time), 4);
					
					airplane_speed_tmp.E_W_velocity = airplane_velocity_three_static[v_count].E_W_velocity;
					airplane_speed_tmp.ICAO_address = airplane_velocity_three_static[v_count].ICAO_address;
					airplane_speed_tmp.N_S_velocity = airplane_velocity_three_static[v_count].N_S_velocity;
					airplane_speed_tmp.time = airplane_velocity_three_static[v_count].time;
					airplane_speed_tmp.VERT_velocity = airplane_velocity_three_static[v_count].VERT_velocity;

				}
				//�߶�
				//memcpy(&(pbuf_yindao[29+ 2]),&(airplane_location_tmp.altitude),4);
				pbuf_yindao[29+ 2] = (Uint8)((airplane_location_tmp.altitude & 0xFF000000)>>24);
				pbuf_yindao[29+ 2 + 1] = (Uint8)((airplane_location_tmp.altitude & 0x00FF0000)>>16);
				pbuf_yindao[29+ 2 + 2] = (Uint8)((airplane_location_tmp.altitude & 0x0000FF00)>>8);
				pbuf_yindao[29+ 2 + 3] = (Uint8)((airplane_location_tmp.altitude & 0x000000FF));
				//
				//��¼��һ�εı���ICAO λ��
				memcpy(&P_time_last,&(airplane_location_tmp.time), 4);

				//У���
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
				//	pEDMA3CC_PaRAM->BCNT_ACNT =0x00660001; //1280*4	//0x003d0001D���?FPGA??��??��A0F	//51
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
		else if (yaokongcmd_all_data.mode == 2)//Ŀ�����ģʽ
		{
			count = 0;
			if(isfirst == 0)
			{
				//��������ѭ������Ҫ��һ�£�Ӧ���Ȱ������ȼ��Ӹߵ��͵�һ�����ֵ�
				//for (i = 0; i < position_BUFLEN; i++)//�����յ���1500������
				for (j = 0; j < 30; j++)//�����ϴ���ICAO��
				{
					//for (j = 0; j < 30; j++)//�����ϴ���ICAO��
					for (i = 0; i < position_BUFLEN; i++)//�����յ���1500������
					{
						if((airplane_location_static[i].ICAO_address == yaokongcmd_all_data.ICAO[j]) && (yaokongcmd_all_data.ICAO[j] !=0))//ICAO ����
						{
							//tmp1 = fabs(airplane_location_static[i].coordinate[1] - yaokongcmd_all_data.jingdu);//γ�Ȳ�
							//tmp2 = fabs(airplane_location_static[i].coordinate[0] - yaokongcmd_all_data.weidu);//���Ȳ�
							
							//memcpy(&(tmp1),&(airplane_location_static[i].coordinate[1]),4);
							//memcpy(&(tmp2),&(airplane_location_static[i].coordinate[0]),4);
							tmp1 = fabs(airplane_location_static[i].coordinate[0]/10000000 - yaokongcmd_all_data.jingdu);//���Ȳ�
							tmp2 = fabs(airplane_location_static[i].coordinate[1]/10000000 - yaokongcmd_all_data.weidu);//γ�Ȳ�
							
							
							if ( (tmp1 <= yaokongcmd_all_data.banzhuijiao) & (tmp2 <= yaokongcmd_all_data.banzhuijiao) )
							{
								//�ڽǶȷ�Χ�ڣ������еķɻ�����airplane_location_tmp��������
								memcpy(&airplane_location_tmp,&(airplane_location_static[i]), 20);
								goto L1;//��������ѭ��
							}
						}
					}
				}
L1:
				
				//����ɻ���������Ϣ��û������Ҳ�����������������0
				for (k = 0; k < 34; k++)//��ʼ��
				{
					pbuf_yindao[k] = 0x00;
				}
				pbuf_yindao[0] = 0x6A;//��ͷ
				pbuf_yindao[1] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x00FF0000)>>16);//ICAO��
				pbuf_yindao[2] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x0000FF00)>>8);
				pbuf_yindao[3] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x000000FF));
				//UTCʱ��
				//zhangfulong add ʱ�����
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add ʱ�����
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),4+ 2);//��ʱ�临�ƹ�ȥ
				pbuf_yindao[4] = UTC_time_real[0];
				pbuf_yindao[5] = UTC_time_real[1];
				pbuf_yindao[6] = UTC_time_real[2];
				pbuf_yindao[7] = UTC_time_real[3];
				pbuf_yindao[8] = UTC_time_real[4];
				pbuf_yindao[9] = UTC_time_real[5];
				//pbuf_yindao[9] = count_tmp_zhang_for_test & 0x000000FF;//��ʱҪȥ��
				//ʹ�ý��ʱ��
				pbuf_yindao[6] = (airplane_location_tmp.time & 0xFF000000)>>24;
				pbuf_yindao[7] = (airplane_location_tmp.time & 0x00FF0000)>>16;
				pbuf_yindao[8] = (airplane_location_tmp.time & 0x0000FF00)>>8;
				pbuf_yindao[9] = (airplane_location_tmp.time & 0x000000FF);
				//ʹ�ý��ʱ��

				v_count = V_get(airplane_location_tmp.ICAO_address);
				//����״̬
#if 0
				if(airplane_location_tmp.ICAO_address == 0)//û�ҵ�ICAO
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0x0F;
				}
				else//�ҵ���ICAO
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0x0F;//���³�0x0X ����һ����ͬ
				}

				if(v_count == -1)//û�ҵ�
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
				}
				else
				{
					pbuf_yindao[8+ 2] = (pbuf_yindao[8+ 2] | 0x0F ) ;//���³�0x0X ����һ����ͬ
				}
#endif
				pbuf_yindao[8+ 2] = 0xFF;
				//memcpy(&(pbuf_yindao[9+ 2]),&(airplane_location_tmp.coordinate[1]),4);//����
				//memcpy(&(pbuf_yindao[13+ 2]),&(airplane_location_tmp.coordinate[0]),4);//γ��
				pbuf_yindao[9+ 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0xFF000000) >> 24);
				pbuf_yindao[9+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x00FF0000) >> 16);
				pbuf_yindao[9+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x0000FF00) >> 8);
				pbuf_yindao[9+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x000000FF));

				pbuf_yindao[13+ 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0xFF000000) >> 24);
				pbuf_yindao[13+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x00FF0000) >> 16);
				pbuf_yindao[13+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x0000FF00) >> 8);
				pbuf_yindao[13+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x000000FF));

				if(v_count < 0)//û��ƥ���ٶ�
				{
					pbuf_yindao[17+ 2] = 0x00;
					pbuf_yindao[28+ 2] = 0x00;//�����Ѿ���0��
					V_time_last = 0;

					airplane_speed_tmp.E_W_velocity = 0;
					airplane_speed_tmp.ICAO_address = 0;
					airplane_speed_tmp.N_S_velocity = 0;
					airplane_speed_tmp.time = 0;
					airplane_speed_tmp.VERT_velocity = 0;

				}
				else//��ƥ���ٶ�
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
				//�߶�
				//memcpy(&(pbuf_yindao[29+ 2]),&(airplane_location_tmp.altitude),4);
				pbuf_yindao[29+ 2] = (Uint8)((airplane_location_tmp.altitude & 0xFF000000)>>24);
				pbuf_yindao[29+ 2 + 1] = (Uint8)((airplane_location_tmp.altitude & 0x00FF0000)>>16);
				pbuf_yindao[29+ 2 + 2] = (Uint8)((airplane_location_tmp.altitude & 0x0000FF00)>>8);
				pbuf_yindao[29+ 2 + 3] = (Uint8)((airplane_location_tmp.altitude & 0x000000FF));

				//
				//�¼��һ�εʱ���ICAO
				memcpy(&P_time_last,&(airplane_location_tmp.time), 4);

				//У���
				for( k = 0; k < 36; k++)
				{
					pbuf_yindao[33+ 2] = (pbuf_yindao[33+ 2] + pbuf_yindao[k]) & 0xFF;
				}
				//����������ȥpbuf����û��

				//
				ICAO_last = airplane_location_tmp.ICAO_address;
				if(airplane_location_tmp.ICAO_address != 0)//������0���ҵ���һ���󣬸��٣��Ժ���?
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
				//	pEDMA3CC_PaRAM->BCNT_ACNT =0x00660001; //1280*4	//0x003d0001D���?FPGA??��??��A0F	//51
				pEDMA3CC_PaRAM->BCNT_ACNT =0x00240001;	//0x22 = 34
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
			    pEDMA3CC_PaRAM->DST = (unsigned char *)DSP_TO_FPGA_YINDAO;
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
				
			}
			else
			{
				for (i = 0; i < position_BUFLEN; i++)//�����յ���1500������
				{
					if(airplane_location_static[i].ICAO_address == airplane_location_tmp.ICAO_address)
					{
						//ICAO������,���濴�Ƿ���ʱ�����µģ�
						if(airplane_location_static[i].time > airplane_location_tmp.time)
						{
							memcpy(&airplane_location_tmp,&(airplane_location_static[i]), 20);
						}
						
					}
				}

				for (k = 0; k < 34; k++)//��ʼ��
				{
					pbuf_yindao[k] = 0x00;
				}
				pbuf_yindao[0] = 0x6A;//��ͷ
				pbuf_yindao[1] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x00FF0000)>>16);//ICAO��
				pbuf_yindao[2] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x0000FF00)>>8);
				pbuf_yindao[3] = (Uint8) ((airplane_location_tmp.ICAO_address & 0x000000FF));
				//UTCʱ��
				//zhangfulong add ʱ�����
				UTC_time_real[0] = UTC_time_all[13];
				UTC_time_real[1] = UTC_time_all[12];
				UTC_time_real[2] = UTC_time_all[15];
				UTC_time_real[3] = UTC_time_all[14];
				UTC_time_real[4] = UTC_time_all[17];
				UTC_time_real[5] = UTC_time_all[16];
				//zhangfulong add ʱ�����
				//memcpy(&(pbuf_yindao[4]),&(UTC_time_real[0]),4+ 2);//��ʱ�临�ƹ�ȥ
				pbuf_yindao[4] = UTC_time_real[0];
				pbuf_yindao[5] = UTC_time_real[1];
				pbuf_yindao[6] = UTC_time_real[2];
				pbuf_yindao[7] = UTC_time_real[3];
				pbuf_yindao[8] = UTC_time_real[4];
				pbuf_yindao[9] = UTC_time_real[5];
				//pbuf_yindao[9] = count_tmp_zhang_for_test & 0x000000FF;//��ʱҪȥ��
				//ʹ�ý��ʱ��
				pbuf_yindao[6] = (airplane_location_tmp.time & 0xFF000000)>>24;
				pbuf_yindao[7] = (airplane_location_tmp.time & 0x00FF0000)>>16;
				pbuf_yindao[8] = (airplane_location_tmp.time & 0x0000FF00)>>8;
				pbuf_yindao[9] = (airplane_location_tmp.time & 0x000000FF);
				//ʹ�ý��ʱ��
				//v_count = V_get(airplane_location_tmp.ICAO_address);
				//����״̬
#if 1
				if(airplane_location_tmp.ICAO_address == ICAO_last)//ICAO��ͬ
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] | 0xF0;
				}
				else//ICAO��ͬ
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0x0F;//���³�0x0X ����һ����ͬ
				}

				if ((v_count == -1) && (P_time_last==airplane_location_tmp.time))
				//�ٶ�û�ҵ���λ��ʱ����ͬ
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
					
				}
				else//�ҵ����ٶ�,
				{
					if((V_time_last == airplane_velocity_three_static[v_count].time) && (P_time_last==airplane_location_tmp.time))
					{
						//�ҵ��ٶ� �ٶ��޸���,λ��Ҳû����
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
					}
					else
					{
						//�ٶ��и��¬����λ���и��£�������?
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] | 0x0F;
						printf("new FFFFFF !!!");
					}
				}

				/*if(v_count == -1)//û�ҵ�
				{
					pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
				}
				else
				{
					if(V_time_last == airplane_velocity_three_static[v_count].time)
					{
						//�ҵ��ٶ��޸���
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] & 0xF0;
					}
					else
					{
						//�ٶ��и���
						pbuf_yindao[8+ 2] = pbuf_yindao[8+ 2] | 0x0F;
					}
					
				}*/
#endif
				//pbuf[8] = 0xFF;
				//memcpy(&(pbuf_yindao[9+ 2]),&(airplane_location_tmp.coordinate[1]),4);//����
				//memcpy(&(pbuf_yindao[13+ 2]),&(airplane_location_tmp.coordinate[0]),4);//γ��
				pbuf_yindao[9+ 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0xFF000000) >> 24);
				pbuf_yindao[9+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x00FF0000) >> 16);
				pbuf_yindao[9+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x0000FF00) >> 8);
				pbuf_yindao[9+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[0] & 0x000000FF));

				pbuf_yindao[13+ 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0xFF000000) >> 24);
				pbuf_yindao[13+ 2 + 1] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x00FF0000) >> 16);
				pbuf_yindao[13+ 2 + 2] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x0000FF00) >> 8);
				pbuf_yindao[13+ 2 + 3] = (Uint8)((airplane_location_tmp.coordinate[1] & 0x000000FF));
				if(v_count < 0)//û��ƥ���ٶ�
				{
					pbuf_yindao[17+ 2] = 0x00;
					pbuf_yindao[28+ 2] = 0x00;//�����Ѿ���0��
					V_time_last = 0;

					airplane_speed_tmp.E_W_velocity = 0;
					airplane_speed_tmp.ICAO_address = 0;
					airplane_speed_tmp.N_S_velocity = 0;
					airplane_speed_tmp.time = 0;
					airplane_speed_tmp.VERT_velocity = 0;
				}
				else//��ƥ���ٶ�
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
				//�߶�
				//memcpy(&(pbuf_yindao[29+ 2]),&(airplane_location_tmp.altitude),4);
				pbuf_yindao[29+ 2] = (Uint8)((airplane_location_tmp.altitude & 0xFF000000)>>24);
				pbuf_yindao[29+ 2 + 1] = (Uint8)((airplane_location_tmp.altitude & 0x00FF0000)>>16);
				pbuf_yindao[29+ 2 + 2] = (Uint8)((airplane_location_tmp.altitude & 0x0000FF00)>>8);
				pbuf_yindao[29+ 2 + 3] = (Uint8)((airplane_location_tmp.altitude & 0x000000FF));

				//
				//��¼��һ�ε�ʱ���ICAO
				memcpy(&P_time_last,&(airplane_location_tmp.time), 4);

				//У���
				for( k = 0; k < 36; k++)
				{
					pbuf_yindao[33+ 2] = (pbuf_yindao[33+ 2] + pbuf_yindao[k]) & 0xFF;
				}
				//����������ȥpbuf����û��
				
				//����ɻ���������Ϣ��û������Ҳ���������������?
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
				//	pEDMA3CC_PaRAM->BCNT_ACNT =0x00660001; //1280*4	//0x003d0001D���?FPGA??��??��A0F	//51
				pEDMA3CC_PaRAM->BCNT_ACNT =0x00240001;	//0x22 = 34
				pEDMA3CC_PaRAM->DSTBIDX_SRCBIDX = 0x00010001; //DSTBIDX=8;SRCBIDX=8;
			    pEDMA3CC_PaRAM->DST = (unsigned char *)DSP_TO_FPGA_YINDAO;
				EDMA3CC_ESRH = 0x00010000;
				TaskSleep(1);
			}
			
		}
		else
		{
			V_time_last = 0;//��¼��һ�ε�ʱ�䣬����α��Ƿ��и���
			P_time_last = 0;//��¼��һ�ε�ʱ�䣬����α��Ƿ�и��?
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
					//	pEDMA3CC_PaRAM->BCNT_ACNT =0x00660001; //1280*4	//0x003d0001D���?FPGA??��??��A0F	//51
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
	int cnt = 0;//��¼ICAOλ��
	for (i = 0; i < 1000; i++)
	{
		ICAO_task_tmp[i] = 0;
	}
	while (1)
	{
		for (i = 0; i < 1500; i++)//�������пռ�
		{
			//��ǰICAO�� airplane_location_static[i].ICAO_address
			for (j = 0; j < 1000; j++)//����ICAO������
			{
				if ((airplane_location_static[i].ICAO_address == ICAO_task_tmp[j]) && (airplane_location_static[i].ICAO_address != 0))
				{
					//�������ICAO���Ѿ�����
					goto ICAO_L;
				}
				;
			}
			//�����ܹ��ˣ�˵���Ǹ���ICAO
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
    DSP_TO_FPGA_HANDLE = 0x55aa; //��?
    MEM_initial();

	C62_disableIER(1<<8);  // edma�ж� 
	asm("nop");	
	C62_clearIFR(1<<8); // ��EDMA��	
	EDMA_init();
	
	C62_clearIFR(1<<9|1<<8);
	C62_enableIER(1<<9|1<<8);//9--fpga�ж� 8--edma�ж�	

//	gpio7_set_0();
#if 0
    C62_enableIER(1<<8);
	while(1)
	{
		   testEDMA();
		   DSK6455_wait(10);
    }
#endif

	//zhangfulong add �ʼ�?
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

   TaskCreate( get_Pulse ,"get_Pulse",OS_TASKPRINORM, 0x3000, 0, 0, 0); // 1280������ //OS_TASKPRILOW
#if 1
   TaskCreate( get_PosV ," get_PosV",OS_TASKPRINORM, 0x3000, 0, 0, 0);  //OS_TASKPRILOW
#endif
#if 1
   //TaskCreate( xsh_task ," xsh_task",OS_TASKPRINORM, 0x3000, 0, 0, 0); //20190211  //OS_TASKPRILOW
   //ϡ�軯
   TaskCreate( xsh_2_task ," xsh_2_task",OS_TASKPRINORM, 0x3000, 0, 0, 0); //20190211  //OS_TASKPRILOW
#endif

	//����EMIF��ͨ�����񣬴���ԭ����UDP����
	//ԭʼ���ں�
	TaskCreate( EMIF_sndPacket ,"EMIF_sndPacket",OS_TASKPRILOW, 0x5000, 0, 0, 0);

	//����һ���������� lead_tesk
	//����
	TaskCreate( lead_tesk ,"lead_tesk",OS_TASKPRILOW, 0x3000, 0, 0, 0);
	//check ICAO tesk
	//TaskCreate( ICAO_tesk ,"ICAO_tesk",OS_TASKPRILOW, 0x3000, 0, 0, 0);

#endif
#if 0
   TaskCreate( udp_sndPacket ,"udp_sndPacket",OS_TASKPRILOW, 0x3000, 0, 0, 0); //
#endif
   TaskBlock( TaskSelf() );
}
