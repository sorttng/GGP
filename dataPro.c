/*******************************************/
#include <math.h>
//�������
#define NZ 15
#define PI 3.141592653589793
#define LOCAL_TIME_THRESHOLD    (480*10000)
//��γ��Ϣ"�ֲ�����"ʱ�����ޣ���Ӧ��λΪ"��"
//#define GLOBAL_TIME_THRESHOLD   18 
#define GLOBAL_TIME_THRESHOLD   (15*10000) 
//��γ��Ϣ"ȫ�ֽ���"ʱ�����ޣ���Ӧ��λΪ"��"
//3.14159265358979323846
//���������������еĽṹ��
typedef struct FPGA_YC  //FPGA-DSP ң��
{
	unsigned int UTCtime;    //4
    unsigned short int temperature; //2
    unsigned short int YC_33;  //2
	unsigned short int YC_55;//2
//--
    unsigned char YK_cnt;//1
    unsigned char DSP_RST_cnt;//1
	unsigned char FPGA_RST_cnt;//1
	unsigned char FPGA_stat;//1-"2bFPGA����/2b״̬/2b����ģʽ/2b00"
    unsigned char reserve[2];  //
}FPGA_YC;
extern FPGA_YC s_FPGA_YC;  //16�ֽ�ң��
//*************************************************************************************************************************
//���������������еĽṹ��,����ȫ�ֶ���ṹ�����������
struct speed_struct_three
{
  unsigned int ICAO_address;
  unsigned int time;
  int  N_S_velocity;      
  int  E_W_velocity;  	  
  int  VERT_velocity;  
};
extern struct speed_struct_three airplane_velocity_three;//�����ٶ��ϱ�
/*********************************/
extern unsigned char velocity_subtype;//�ٶȽ���ȫ�ֱ��������ڴ�����Ϣʹ�ã��û����ù���
struct speed_code
{
  unsigned char bit46;
  unsigned short bit47_56;
  unsigned char bit57;
  unsigned short bit58_67;
  unsigned char bit69;
  unsigned short bit70_78;
};
extern struct speed_code velocity_code; //�ٶȽ���ȫ�ֽṹ����������ڴ�����Ϣʹ�ã��û����ù���
//*************************************************************************************************************************
struct data_struct
{
  unsigned int ICAO_address_with_mark;//����λ�� 190314 (ICAO_adress_with_mark)��(time)
  unsigned int time;
  union
  {
   double coordinate[2]; //����
   unsigned int CPR_code[4];
  }position;
};
extern struct data_struct data_save[1000],new_data;//�˽ṹ������ͽṹ�����ȫ��ʹ�ã�data_save[1000],new_data;
/*********************************/
struct location_struct
{
  unsigned int ICAO_address;
  unsigned int time;
  int coordinate[2];//��ʽ�ϱ�ʱ��ʹ��int�ͣ���ʽ�ϱ�ʱ��ʹ��int(1E-7Ϊ��λ)��
  int altitude;
};
extern struct location_struct airplane_location;//�˽ṹ������ͽṹ�����ȫ��ʹ��,airplane_location;
//���������������еĽṹ��,����ȫ�ֶ���ṹ�����������
struct message_struct
{
  unsigned int time;
  unsigned char data_demodulator[11];  //ADSB��ԭʼ88bit��Ϣ���ģ�11�ֽ�							
};
extern struct message_struct ADSB_message;//�ϱ���ADSBԭʼ����,�̶���ʽ�������ֱ������࣬�����������ݲ���
/*********************************/
#define pulse_BUFLEN 150
extern unsigned short   pulse_amp[112];
extern unsigned int pulse_time[pulse_BUFLEN];  //190521
extern unsigned int pulse_rd ;
unsigned short  DF_message=0;//���ڱ�����ȡ��֯Ϊ���㾭γ����Ҫ�ĸ�ʽʹ�� DF��Ϊ17,18,19 
unsigned short  CF_AF_message=0;//DF=18,19ʱ��ʹ�ã���DFΪ18��19��ʱ���6��8λ����Ϊ0����DF18ʱCFΪ0��DF19ʱΪAFΪ0	
unsigned short  TYPE_Code_message=0;//�����жϸñ����Ƿ��ǿ���λ�ñ���
/*********************************/
#define ICAO_BUFLEN 5000
extern unsigned int g_32_ICAO_new_cnt;
extern unsigned int g_32_ICAO_match_cnt;
unsigned int g_32_ICAO_match_2_cnt=0;
unsigned int g_32_ICAO_match_3_cnt=0;
extern unsigned char g_8_ICAO_NAME[ICAO_BUFLEN][4];  //int *
extern unsigned int  t1,t2,t3,t4,t5;
#define  htonl(a)            ( (((a)>>24)&0xff) + (((a)>>8)&0xff00) + \
                               (((a)<<8)&0xff0000) + (((a)<<24)&0xff000000) )


//zhangfulong add start
unsigned short int altitude_code_BARO = 0 , altitude_code_GNSS = 0;
int decode_altitude();
//zhangfulong add end

/**********************************************/
 //����ֵ2��ָʾ��֡���ݴ������������Ҹ�֡��λ�ñ��ģ�����ADSB_message���������ϴ����ģ�ͬʱ������new_data�ṹ�����
 //����ֵ1��ָʾ��֡���ݴ�������������֡����λ�ñ��ģ�������ADSB_message���������ϴ����ģ�
 //����ֵ0��ָʾ��֡���ݲ��������ǽ���ƺ����ݲ���ȷ, ADSB_message���鲻�ᱻ���£�
//------------------------
 //����ֵ2��ָʾ��֡���������ݱ��ģ����ǽ��㲻�ɹ�����������ADSB_message���飬�Լ����ݿ�
 //����ֵ3��ָʾ��֡���������ݱ��ģ�����ɹ���������ADSB_message���飬airplane_location�ṹ������Լ����ݿ�
/**********************************************/
unsigned int data_pro(unsigned short  * fifo)
{
     unsigned short  temp_message=0;//���ڱ���ƴ��Ϊ11�ֽڵ��޷����ַ�������
     unsigned int i,j;
     unsigned short  *  pulse_tmp;
      //�˶δ��밴��Ҫ�����ϱ�����,ͨ��CRCУ�飬����DFΪ17�������ϴ�ԭʼ88bit�����ݰ�,���ʱ����Ϣ������Ƿ�Ϊλ�û��ٶȱ��ģ���������׼����
            //***************************************************************************************************  
     pulse_tmp=fifo; //����һ��
     DF_message=0;
     for(i=0;i<5;i++)  //ǰ5λ��0������������ȫ0
     {   
        DF_message=DF_message|(pulse_tmp[i]<<(4-i));//ȡDFλ
	 }
	  if(DF_message==17)
	  {
          // ����Ƿ�Ϊ����λ�ñ��ģ������ٶȱ��ģ�����������
          //*****************************************************************************************  
          TYPE_Code_message=0;
          for(i=0;i<5;i++)
          {
              TYPE_Code_message=TYPE_Code_message|(pulse_tmp[i+32]<<(4-i));
		  }
           //TYPE����9-18����20-22��ʾ��λ�ñ���
          if(((TYPE_Code_message>=9)&&(TYPE_Code_message<=18))||((TYPE_Code_message>=20)&&(TYPE_Code_message<=22)))
          {   
			 memset(new_data, 0, sizeof(new_data));////���ڽṹ��new_data��ȫ�ֵģ��ʴ˴���Ҫ��0��
 #if 1   
		//	new_data.time=	htonl(s_FPGA_YC.UTCtime);  //ʱ�䴦��1��
            new_data.time=	htonl(pulse_time[pulse_rd]);  //190521
			
 #endif              
             //�˴������µ�λ�ñ��ĸ��£� new_data.ICAO_adress_with_mark�е�25λ��־λ������ᣬ��ʱһ�����ڽṹ���е��Ǳ�������                    
             for(i=0;i<24;i++)
			 {
		        new_data.ICAO_address_with_mark=new_data.ICAO_address_with_mark|(pulse_tmp[i+8]<<(23-i));//�Ѱ�BIT��ŵ�24λICAO��ַת����һ��24λ���ݴ��
             }
             //new_data.ICAO_adress_with_mark=new_data.ICAO_adress_with_mark|0x1000000;
             //new_data.ICAO_adress_with_mark��25λ�����24����ű�־λ����ʾ��ŵľ�γ�ȱ���
             for(i=0;i<17;i++)//����γ�ȱ������ڽṹ���еĹ�������
             {
                  new_data.position.CPR_code[0]=new_data.position.CPR_code[0]|(pulse_tmp[i+54]<<(16-i));//γ�ȱ���
                  new_data.position.CPR_code[1]=new_data.position.CPR_code[1]|(pulse_tmp[i+71]<<(16-i));//���ȱ���
             }
             new_data.position.CPR_code[2]=pulse_tmp[53];//������ż�����־λ�ڹ�������


			if((TYPE_Code_message>=9)&&(TYPE_Code_message<=18))  //
			 {
			 	altitude_code_BARO = 0;
				altitude_code_GNSS = 0;
				for ( i = 0; i < 12; i++)
				{
					altitude_code_BARO = altitude_code_BARO | (pulse_tmp[i+40]<<(11-i));
				}
			 }
			 else if((TYPE_Code_message>=20)&&(TYPE_Code_message<=22))//
			 {
			 	altitude_code_BARO = 0;
				altitude_code_GNSS = 0;
				for ( i = 0; i < 12; i++)
				{
					altitude_code_GNSS = altitude_code_GNSS | (pulse_tmp[i+40]<<(11-i));
				}
			 }

			 return(2);//����ֵ2��ָʾ��֡���ݴ�������,һ����λ�ñ���
           }  
           //TYPE����19��ʾ��λ�ñ��� //�ٶȱ��ģ���
           if(TYPE_Code_message==19)
           {
		         memset(airplane_velocity_three, 0, sizeof(airplane_velocity_three));////���ڽṹ��airplane_velocity��ȫ�ֵģ�����0��
				     
			     memset(velocity_code, 0, sizeof(velocity_code));////ȫ�ֱ������������ٶȽ��㺯�����ݲ���������0��
			     velocity_subtype=0;//ȫ�ֱ������������ٶȽ��㺯�����ݲ���������0
#if 1   
				 airplane_velocity_three.time=	htonl(s_FPGA_YC.UTCtime);  //ʱ�䴦��2��
#endif
                 for(i=0;i<24;i++)
			     {
			        airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|(pulse_tmp[i+8]<<(23-i));//�Ѱ�BIT��ŵ�24λICAO��ַת����һ��24λ���ݴ��
                 }      
                 for(i=0;i<3;i++)  
                 {
                    velocity_subtype=velocity_subtype|(pulse_tmp[i+37]<<(2-i));//�洢�ٶ�֡��������
				 } 	  
			   //��ȡ�ٶȱ�������֣����ں�������  
                 velocity_code.bit46=pulse_tmp[45];
                 velocity_code.bit57=pulse_tmp[56];
                 velocity_code.bit69=pulse_tmp[68];
				
				 for(i=0;i<10;i++) 
				 {
				      velocity_code.bit47_56=velocity_code.bit47_56|(pulse_tmp[i+46]<<(9-i));
                      velocity_code.bit58_67=velocity_code.bit58_67|(pulse_tmp[i+57]<<(9-i));
                 }
                 for(i=0;i<9;i++) 
				 { 
				      velocity_code.bit70_78=velocity_code.bit70_78|(pulse_tmp[i+69]<<(8-i));
                 }       
		         return(4);//����ֵ4��ָ��֡��ݴ�������,һ�����ٶȱ���				   
		   }            
           return(1);//����ֵ1��ָʾ��֡���ݴ�������,Ϊλ�ú��ٶȱ���֮�����������
	   } //  if(DF_message==17)
       else  //DF!=17
       {
          return(1);//����ֵ1��ָʾ��֡���ݴ����������������ͨ��У�飬���ǲ���DF17�ĸ�ʽ
       }  

}
//*************************************************************************************************************************************************************
//*                                                                                                                                                           *
//*  ����Ϊ��λ����Ϣ�漰���ĺ������漰NL������my_mod������lat_lon_local_calculate������lat_lon_global_calculate����  decode_position()5������                *                     
//*                                                                                                                                                           *
//*************************************************************************************************************************************************************
int NL(double  m)//���㾭�ȵ�ZONE�ָ���Ŀ,��ͬγ������¾��ȵķָ�����ǲ�ͬ�ģ���Ҫ����
{
  double cosine_value;
  int number_of_l;
  double temp;
  temp=m;
  if(temp<0)
    temp=-temp;

  cosine_value=1-(1-cos(PI/30.0))/pow(cos(PI*temp/180.0),2);
  number_of_l=(unsigned)floor(2.0*PI/acos(cosine_value));
  
  if(temp==0.0)
   number_of_l=59;
  if(temp==87.0)
   number_of_l=2;
  if(temp>87.0)
   number_of_l=1;
  
  return(number_of_l);
}
/*********************************/
double my_mod(double a,double b)//��׼��CPR�㷨�������ģ�������
{
  double reminder;
  reminder=a-b*floor(a/b);
  
  return(reminder);
}
/*********************************/
short int lat_lon_local_calculate(struct data_struct * st,struct data_struct * st_ref)
{
  int mark,number_of_lon;
  int j,m;
  double Dlat,Rlat,Dlon,Rlon,ref_lat,ref_lon;
//  double lat_delta,lon_delta,lat_average,lat_distance,lon_distance,distance,velocity;
  int XZ,YZ;
  //int time_delta;
  mark=st->position.CPR_code[2];//�浱ǰ���±��ĵ���ż��������

  ref_lon=st_ref->position.coordinate[1];//ǰһ��ʱ�̾��Ȳο�
  ref_lat=st_ref->position.coordinate[0];//ǰһ��ʱ��γ�Ȳο�
 
  XZ=st->position.CPR_code[1];//��ǰ�ľ��ȱ���
  YZ=st->position.CPR_code[0];//��ǰ��γ�ȱ���

  Dlat=360.0/((double)(4*NZ-mark));//�ɵ�ǰ����ż���뷽ʽ��ȷ��γ�Ȱ���6��6.1�ָ�

  j=floor(ref_lat/Dlat)+floor(0.5+(my_mod(ref_lat,Dlat)/Dlat)-(YZ/pow(2,17)));//��γ������
  Rlat=Dlat*(j+(YZ/pow(2,17)));//���㵱ǰγ�ȵľ���ֵ

  number_of_lon=NL(Rlat);//����γ�ȵ�ֵ�󾭶�����

  if((number_of_lon-mark)>0)
    Dlon=360.0/((double)(number_of_lon-mark));
  else
    Dlon=360.0;

  m=floor(ref_lon/Dlon)+floor(0.5+(my_mod(ref_lon,Dlon)/Dlon)-(XZ/pow(2,17)));
  Rlon=Dlon*(m+(XZ/pow(2,17)));//��ǰ�ľ��Ⱦ������

  //�������ݿ�
  st_ref->time=st->time;
  st_ref->ICAO_address_with_mark=st->ICAO_address_with_mark|0x1000000;
  st_ref->position.coordinate[0]=Rlat;//���浱ǰ�����γ��ֵ�����ݿ�
  st_ref->position.coordinate[1]=Rlon;//���浱ǰ����ľ���ֵ�����ݿ�
  
  //�����ϱ�λ����Ϣ�õĽṹ�����
  airplane_location.time=st->time;

  airplane_location.ICAO_address=st->ICAO_address_with_mark & 0xffffff;
  airplane_location.coordinate[1]=(int)(Rlat * 10000000);
  airplane_location.coordinate[0]=(int)(Rlon * 10000000);
  return(3);
  
}
/*********************************/
short int lat_lon_global_calculate(struct data_struct * st,struct data_struct * st_ref)
{
  int number_of_lon_0,number_of_lon_1;
  double Dlat_0,Rlat_0,Dlat_1,Rlat_1,Dlon_0,Rlon_0,Dlon_1,Rlon_1,n_i;
  int  XZ_0,YZ_0,XZ_1,YZ_1,j,m;//j��m�Ķ��巽ʽ���Ա�׼����������-59��58��Χ
  int  mark;
  double hh_tmp1;
  double hh_tmp2;
  int hh_tmp3;
  mark = st->position.CPR_code[2];//ȡ��ǰʱ���յ����ĵ���ż��������
  if(mark == 0)//��ǰΪż��������
  {
    XZ_0=st->position.CPR_code[1];
    YZ_0=st->position.CPR_code[0];
    XZ_1=st_ref->position.CPR_code[1];
    YZ_1=st_ref->position.CPR_code[0];
  }
  else//��ǰΪ���������
  {
    XZ_0=st_ref->position.CPR_code[1];
    YZ_0=st_ref->position.CPR_code[0];
    XZ_1=st->position.CPR_code[1];
    YZ_1=st->position.CPR_code[0];
  }

  Dlat_0=360.0/((double)4*NZ-0);//ż�������γ�ȵķָ���
  Dlat_1=360.0/((double)4*NZ-1);//��������γ�ȵķָ���
 
  j=floor(0.5+(59*YZ_0-60*YZ_1)/pow(2,17));//POW����ΪDOUBLE����

  Rlat_0=Dlat_0*(my_mod(j,60-0)+YZ_0/pow(2,17));//ż�������γ�ȵĽ�����
  Rlat_1=Dlat_1*(my_mod(j,60-1)+YZ_1/pow(2,17));//��������γ�ȵĽ�����
  if((Rlat_0>90&&Rlat_0<270)||(Rlat_1>90&&Rlat_1<270))//����������ܳ��֣���������Ӹ������
    return(2) ;//����2��ʾ���㲻�ɹ�
  
  if(Rlat_0 >= 270.0 && Rlat_0 <=360.0)//ż����γ�ȵĽ�����ͳһ���ϱ�γ���������ϸ�
    Rlat_0=my_mod(Rlat_0+180,360)-180.0;   
  
  if(Rlat_1 >= 270.0 && Rlat_1 <=360.0)//�����γ�ȵĽ�����ͳһ���ϱ�γ���������ϸ� 
    Rlat_1=my_mod(Rlat_1+180,360)-180.0;
  

  number_of_lon_0=NL(Rlat_0);//����ż����γ�ȣ����㾭�ȷָ���
  number_of_lon_1=NL(Rlat_1);//���������γ�ȣ����㾭�ȷָ���

   if(number_of_lon_0 != number_of_lon_1)//������ż����γ�ȼ�����ľ��ȷָ���Ŀ����ͬ�����޷����㾭����Ϣ���������㣬���浱ǰ���ĵ����ݿ�
   {
      st_ref->time=st->time;
	  st_ref->position.CPR_code[0]=st->position.CPR_code[0];
      st_ref->position.CPR_code[1]=st->position.CPR_code[1];
      st_ref->position.CPR_code[2]=st->position.CPR_code[2];
	  st_ref->position.CPR_code[3]=st->position.CPR_code[3];
    
      return(2) ;//����2��ʾ���㲻�ɹ�
   }
   else
   {
     if(number_of_lon_0 > 1)//number_of_lon_0��number_of_lon_1��ȣ�number_of_lon_0����1��number_of_lon_1Ҳ�ʹ���1
       {   
		   n_i=number_of_lon_0;  //n_i,��Ϊ���ȷָ���Ŀ
	       Dlon_0=360.0/n_i;     //ż����������ȵķָ���
	       Dlon_1=360.0/(n_i-1); //�����������ȵķָ���
	       m=floor(0.5+(XZ_0*(n_i-1)-XZ_1*(n_i))/pow(2,17));
	       Rlon_0=Dlon_0*(my_mod(m,n_i-0)+XZ_0/pow(2,17));
		   Rlon_1=Dlon_1*(my_mod(m,n_i-1)+XZ_1/pow(2,17));
	      
	       if(Rlon_0 > 180.0 && Rlon_0 <= 360.0)//ż���뾭�ȵĽ�����ͳһ��������������������	       
	         Rlon_0=my_mod(Rlon_0+180,360)-180;
	       
	       if(Rlon_1 > 180.0 && Rlon_1 <= 360.0)//����뾭�ȵĽ�����ͳһ��������������������	   
	         Rlon_1=my_mod(Rlon_1+180,360)-180;	       
       }
       else
       {	       
		   Dlon_0=360.0;
	       Dlon_1=360.0;
		   Rlon_0=Dlon_0*(XZ_0/pow(2,17));
	       Rlon_1=Dlon_1*(XZ_1/pow(2,17));

		   if(Rlon_0 > 180.0 && Rlon_0 <= 360.0)//ż���뾭�ȵĽ�����ͳһ��������������������
	       	  Rlon_0=my_mod(Rlon_0+180,360)-180;
	        
	       if(Rlon_1 > 180.0 && Rlon_1 <= 360.0)//����뾭�ȵĽ�����ͳһ��������������������
	       	  Rlon_1=my_mod(Rlon_1+180,360)-180;
	        
	   }
    }
  
   if(mark==0)//���ݵ�ǰ�յ����ĵ���ż����״ֵ̬������������ϱ����
   {
       //�������ݿ�
       st_ref->time=st->time;//?????????????????????????????????????????????????????
       st_ref->ICAO_address_with_mark=st->ICAO_address_with_mark | 0x1000000;
       st_ref->position.coordinate[0]=Rlat_0;
       st_ref->position.coordinate[1]=Rlon_0;

	   //�����ϱ�λ����Ϣ�õĽṹ�����
       airplane_location.time=st->time;
       airplane_location.ICAO_address=st->ICAO_address_with_mark & 0x00ffffff;
       airplane_location.coordinate[1]=(int)(Rlat_0 * 10000000);
       airplane_location.coordinate[0]=(int)(Rlon_0 * 10000000);
	    return(3) ;//����3��ʾ����ɹ�
   }
   else
   {
       //�������ݿ�
       st_ref->time=st->time;
       st_ref->ICAO_address_with_mark=st->ICAO_address_with_mark | 0x1000000;
       st_ref->position.coordinate[0]=Rlat_1;
       st_ref->position.coordinate[1]=Rlon_1;

	   //�����ϱ�λ����Ϣ�õĽṹ�����
	   airplane_location.time=st->time;
       airplane_location.ICAO_address=st->ICAO_address_with_mark & 0x00ffffff;
       airplane_location.coordinate[1]=(int)(Rlat_1 * 10000000);
       airplane_location.coordinate[0]=(int)(Rlon_1 * 10000000);
	   return(3) ;//����3��ʾ����ɹ�
   }

}
/*********************************/
//�ڶ����λ�ü�������ݿ����
unsigned short int decode_position()
{

	unsigned short int i,matching_address;
    unsigned short int address_message,address_position;
	unsigned int ICAO_temp,ICAO_now,time_save_message,time_save_position;//,time_temp,time_now;
	//unsigned int parity_type;  
    unsigned int data_save_type;
    unsigned short int Calculation_ok;//����ֵ��2��ʾ���㲻�ɹ���3��ʾ����ɹ���������ɹ�������������airplane_location�ṹ��ȫ�ֱ�����
    unsigned int ICAO_match=0;//�����ʼ��Ϊ0��ICAOƥ��ɹ���־���ɹ�Ϊ1�����ɹ�Ϊ0

//new_data.time++;//hh
    if(new_data.time==0)
         return(2);            //UTCʱ���ǩ����Ϊ0�����Ϊ0 ֻ�ܷ���2��������ǰ����                                       
    
    ICAO_now= new_data.ICAO_address_with_mark & 0x00ffffff;//ȡ��ǰλ�ñ��ĵ�ICAO��ַ��
   
   
    time_save_message= new_data.time;                     //time_save_message��ʼ����ǰλ�ñ��ĵ�ʱ��,�����Ƚϼ�¼���絽��ı���ʱ��
    time_save_position=new_data.time;                     //ime_save_position��ʼ����ǰλ�ñ��ĵ�ʱ��,�����Ƚϼ�¼���������ľ�γ����Ϣ��ʱ��
   
 
	for (i = 0; i < 1000; i++)//�������ݿ������Ѵ����Ϣ�������Ǳ��ı�����Ϣ���������Ѿ��������λ����Ϣ
	{
		
	       //�˴�����ICAO�Ĳ���ƥ�䣬һ��ƥ��ɹ���������ѭ����������������
	       //*****************************************************************************************************************************************
	       ICAO_temp = data_save[i].ICAO_address_with_mark & 0x00ffffff;//��ȡ���ݿ�ICAO��ַ
	        
	       if(ICAO_now == ICAO_temp)
		    {
		      ICAO_match=1;         //ƥ��ɹ���־����ʼΪ0����ʾ����ƥ�䲻�ɹ���ƥ��ɹ����޸�Ϊ1(��ʾ�����ݿ����ҵ���һ�µ�ICAO��ַ)
		      matching_address=i;   //ƥ��ɹ�ʱ�����ݿ��еĶ�Ӧһ��ICAO�ı��Ļ�γ�ȴ洢λ�ü�¼
		      
		      break;                // ƥ�䵽һ�µ�ICAO��ţ�������ѭ�������ټ������ң����Ҳ��ɹ���������ң�ֱ��������ݿ��е�1000��Ԫ�� 
		    }
	       //******************************************************************************************************************************************
	      //�˴������ݿ�ĸ��²��ԣ�ICAOƥ�䲻�ɹ������Ҽ�¼���ݿ���ʱ�����絽�ﱨ�ĵĴ洢��ַ�����Ҽ�¼����������Ŀ��λ����Ϣ�Ĵ洢��ַ���ֱ���Һʹ洢
		  //********************************************************************************************************************************************	 
		   data_save_type = data_save[i].ICAO_address_with_mark & 0x1000000;//��¼��ǰλ�ô洢���Ǳ��Ļ����Ǿ�γ����Ϣ����25λ�����24��Ϊ0��ʾ�Ǳ���ԭʼ��Ϣ��Ϊ1��ʾ�Ǿ�γ����Ϣ

	        if((data_save[i].time<time_save_message)&&(data_save_type==0))
			{
			  time_save_message = data_save[i].time;//��֤time_save_message�����һ���������ʱ�䣬��Ӧ����
			  address_message = i; //��֤address_message�д�ĵ�ַƫ����һ���Ƕ�Ӧ����ʱ��ı��ģ���ͬʱ��ı��ļ�¼��һ��λ��
			                       //��ʼ״̬�£�time_save_messageһ����Ϊ0��ȷ���½��յ��Ĳ�һ��ICAO�ı������δ��
								   //���ȫ����Ϊ����ľ�γ����Ϣ��address_message���ᱻ�޸ģ�Ϊ����ʱ�ĳ�ʼֵ-1���Ƿ�
			}

		    if((data_save[i].time<time_save_position)&&(data_save_type==0x1000000))
			{
			  time_save_position = data_save[i].time;//��֤time_save_position�����һ���������ʱ�䣬��λ����Ϣ
			  address_position = i;//��֤address_position�д�ĵ�ַƫ����һ���Ƕ�Ӧ����ʱ��Ľ������λ����Ϣ�洢��ַ����ͬʱ�����Ϣ��¼��һ��λ��
			                       //��ʼ״̬�£������ǹ��������У�ȫ��Ϊ������Ϣ��address_position��ֵ���ᱻ�޸ģ�Ϊ����ʱ�ĳ�ʼֵ-1���Ƿ�
			}
	     //*******************************************************************************************************************************************	
	 }	//for 1000
	
     if(ICAO_match==1)//ICAOƥ��ɹ���ɳ��Խ��н��㣬�ֲ������ȫ�ֽ��㣬����Ҫ����ʱ����������
	 {	    
		  data_save_type = data_save[matching_address].ICAO_address_with_mark & 0x1000000;//ƥ��ɹ�ʱ��ʶ�����ݿ��е��Ǳ��Ļ��Ǿ�γ����Ϣ����25λ�����24��Ϊ0��ʾ�Ǳ���ԭʼ��Ϣ��Ϊ1��ʾ�Ǿ�γ����Ϣ
		  g_32_ICAO_match_cnt++;
		  if(data_save_type == 0x1000000)//data_save_typeΪ0x1000000��ʹ�þֲ�����
		  {		
			  //ʱ���ֵ����Ϊ��λ��333��������ٶ�����1224����/Сʱ��1��գ������۲�ֵ���980�룬16.3���ӡ����������˶�����ʱ�����㣬��ֵȡ480��Ϊ�ֲ����������ʱ��		
			  if((new_data.time - data_save[matching_address].time) <= LOCAL_TIME_THRESHOLD)//480�����ֵ����Ϊ��λ
			  {
				  Calculation_ok=lat_lon_local_calculate(&new_data, &data_save[matching_address]);//���þֲ����㺯�����������洢��data_save[i]�У�ͬʱairplane_location�ṹ��ȫ�ֱ����ϱ�			
			        //����3
					decode_altitude();//zhangfulong add
			  }
			  else
			  {
				  Calculation_ok=2;//��ʾʱ�䲻����ֲ�����Ҫ�󣬷���ֵ�趨Ϊ2
				  	//�����ֵʱ�䲻����ֲ�����Ҫ��ͬʱICAO��ƥ��ģ���������ݿ��еľ�γ����Ϣ������ǰ�ı�����Ϣ�洢��ȥ����data_save[i]λ��
				  data_save[matching_address].time = new_data.time;
				  data_save[matching_address].ICAO_address_with_mark = new_data.ICAO_address_with_mark;//��25λΪ0����ʾ��λ�ô洢���Ǳ���
				  data_save[matching_address].position.CPR_code[0] = new_data.position.CPR_code[0];
				  data_save[matching_address].position.CPR_code[1] = new_data.position.CPR_code[1];
				  data_save[matching_address].position.CPR_code[2] = new_data.position.CPR_code[2];
				  data_save[matching_address].position.CPR_code[3] = 0;//��������λ�ÿ��Ժ��Բ��ܣ��������������0
			  }
		   }
		   else//data_save_type��Ϊ0x1000000���������ȫ�ֽ��� 
		   {
			  if((new_data.time - data_save[matching_address].time) <= GLOBAL_TIME_THRESHOLD)//ȫ�ֽ���ʱ�������ɻ��ٶȰ���1224���1��գ�,5.1km�ķ��о��룬��ζ��15��ʱ�䣬
			                                                              //��ȡ15��Ϊȫ�ֽ��������ʱ����ڿ��̣�ʵ�ʳ����У�90%�ķɻ�"����"��������1000������Էſ�18��ʱ������
			  
			  {
				  if (data_save[matching_address].position.CPR_code[2] != new_data.position.CPR_code[2])//�Ƚϵ�ǰ���ĺ��Ѵ洢�ı����Ƿ�����ż����ƥ��
				  {
						Calculation_ok=lat_lon_global_calculate(&new_data, &data_save[matching_address]);//����ȫ�ֽ��㺯�����������洢��data_save[matching_address]�У�ͬʱairplane_location�ṹ��ȫ�ֱ����ϱ�								
				  		decode_altitude();//zhangfulong add
				  }
				  else
				  {
						Calculation_ok=2;//��ʾ��ż���벻����ȫ�ֽ���Ҫ�󣬷���ֵ�趨Ϊ2
						//�����ż���벻ƥ�䣬����ȫ�ֽ���Ҫ��ͬʱICAO��ƥ��ģ���ֻ�������ݿ��еı�����Ϣ������ǰ�ı�����Ϣ�洢��ȥ����data_save[matching_address]λ��
						data_save[matching_address].time = new_data.time;
						data_save[matching_address].ICAO_address_with_mark = new_data.ICAO_address_with_mark;//��25Ϊ0����ʾ��λ�ô洢���Ǩ��
						data_save[matching_address].position.CPR_code[0] = new_data.position.CPR_code[0];
						data_save[matching_address].position.CPR_code[1] = new_data.position.CPR_code[1];
						data_save[matching_address].position.CPR_code[2] = new_data.position.CPR_code[2];
						data_save[matching_address].position.CPR_code[3] = 0;//��������λ�ÿ��Ժ��Բ��ܣ��������������0
				  }
			  }
			  else
			  {   
				   Calculation_ok=2;//��ʾʱ����������ȫ�ֽ���Ҫ�󣬷���ֵ�趨Ϊ2
				   //�����ֵʱ�䲻����ȫ�ֽ���Ҫ��ͬʱICAO��ƥ��ģ���ֻ�������ݿ��еı�����Ϣ������ǰ�ı����Ϣ�洢��ȥ����data_save[matching_address]λ��
				  	data_save[matching_address].time = new_data.time;
					data_save[matching_address].ICAO_address_with_mark = new_data.ICAO_address_with_mark;//��25Ϊ0����ʾ��λ�ô洢���Ǳ���
					data_save[matching_address].position.CPR_code[0] = new_data.position.CPR_code[0];
			    	data_save[matching_address].position.CPR_code[1] = new_data.position.CPR_code[1];
					data_save[matching_address].position.CPR_code[2] = new_data.position.CPR_code[2];
					data_save[matching_address].position.CPR_code[3] = 0;//��������λ�ÿ��Ժ��Ի�ܣ��������������0
			    }
		    }
#if 1
	   if(Calculation_ok==2)
	   {
	     g_32_ICAO_match_2_cnt++;
	   }
	   else
	   {
	    g_32_ICAO_match_3_cnt++;
	   }
#endif
	   return(Calculation_ok);//ICAOƥ��ɹ�������£���(ICAO_match==1)�������㣬������Ľ�����أ����ֵ��2����3		 
	 }
   if(ICAO_match==0)//�ڱ������ݿ��ICAOƥ�䲻�ɹ�����Ϊ��һ���·ɻ�������б������ݵı��棻�������ݿ��С�ǹ̶��ģ���Ҫ�ƶ�һ�׸������ݿ�Ĺ���
   {
#if 1
       memcpy((void *)(g_8_ICAO_NAME[g_32_ICAO_new_cnt]),(void *)&(new_data.ICAO_address_with_mark),4);
//     g_8_ICAO_NAME[g_32_ICAO_cnt]=&(new_data.ICAO_adress_with_mark);
       g_32_ICAO_new_cnt++;
	   if(g_32_ICAO_new_cnt>=400)
	   {
	    g_32_ICAO_new_cnt++;
	   }
#endif
   	   if(time_save_message==0)//time_save_messageΪ0����ζ�Ż��д洢�ռ�δ�����κα��ģ������Ȱ��µı��ķ��룬������ʱ��Ա�
	   {
	     data_save[address_message].time = new_data.time;
	     data_save[address_message].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		 data_save[address_message].position.CPR_code[0] = new_data.position.CPR_code[0];
		 data_save[address_message].position.CPR_code[1] = new_data.position.CPR_code[1];
		 data_save[address_message].position.CPR_code[2] = new_data.position.CPR_code[2];
		 data_save[address_message].position.CPR_code[3] = 0;  
		 return(2);  
	   }
	   else//time_save_message��Ϊ0����ζ�Ŵ洢�ռ��Ѿ�ȫ�����ñ��Ļ�γ������ʱ��Ա�
	   {
	   	// Calculation_ok=2;
	   	 //���߼��Ļ����Ǳ���ʱ����Чʱ��18�룬��γ�ȵ���������480�룬���Կ��ܹ����������
		 //1.��ǰ����ʱ��������洢����ʱ�̱Ƚϴ���18�룬ͬʱ�����������ľ�γ����Ϣʱ�̱Ƚϴ���480�룬��ʱ�������߾��ɣ�ѡ����±��ĵĴ洢
		 //2.��ǰ����ʱ��������洢����ʱ�̱Ƚϴ���18�룬ͬʱ�����������ľ�γ����Ϣʱ�̱Ƚ�С�ڵ���480�룬��ʱ���±��ĵĴ洢
		 //3.��ǰ����ʱ��������洢����ʱ�̱Ƚ�С�ڵ���18�룬ͬʱ�����������ľ�γ����Ϣʱ�̱Ƚϴ���480�룬��ʱ���¾�γ��Ϣ�Ĵ洢
	   	 //3.��ǰ����ʱ��������洢����ʱ�̱Ƚ�С�ڵ���18�룬ͬʱ�����������ľ�γ����Ϣʱ��Ƚ�С�ڵ���480�룬��ʱ���¾�γ����Ϣ�Ĵ洢 
	   	 
	   	 //��ʼ״̬������ʱ�����32λ���������ʱ�����UTCʱ2009��1��1�գ����Գ�ʼ״̬�£����ݿ�����ʱ��ȫ0��(������)
	   	 //������һ�У�Ĭ��Ϊȫ����"��Ч"�ı�����Ϣ��ֻҪ�յ���Ч���ģ�ʱ���ֵԶԶ����18�������2���������ʹʱ���С�ڵ���18�������4.1�����
	   	 //���������У�������ּ�������������ݿ�ȫ���Ǳ��ģ�һ���������2���4.1�����
	   	 //���������У�������ּ�������������ݿ�ȫ���Ǿ�γ����Ϣ��һ���������3���4.2����� 	 
	   	   if(((new_data.time-time_save_message)>GLOBAL_TIME_THRESHOLD)
	   	   &&((new_data.time-time_save_position)>LOCAL_TIME_THRESHOLD))
	   	   {
	   	     data_save[address_message].time = new_data.time;
		     data_save[address_message].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		     data_save[address_message].position.CPR_code[0] = new_data.position.CPR_code[0];
		     data_save[address_message].position.CPR_code[1] = new_data.position.CPR_code[1];
		     data_save[address_message].position.CPR_code[2] = new_data.position.CPR_code[2];
			 data_save[address_message].position.CPR_code[3] = 0;
	   	   }
	   	   
		   if(((new_data.time-time_save_message)>GLOBAL_TIME_THRESHOLD)
		   &&((new_data.time-time_save_position)<=LOCAL_TIME_THRESHOLD))//�п���new_data.time-time_save_positionΪ0�������ݿ���ȫ���洢�Ǳ���
	   	   {
	   	     data_save[address_message].time = new_data.time;
		     data_save[address_message].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		     data_save[address_message].position.CPR_code[0] = new_data.position.CPR_code[0];
		     data_save[address_message].position.CPR_code[1] = new_data.position.CPR_code[1];
		     data_save[address_message].position.CPR_code[2] = new_data.position.CPR_code[2];
			 data_save[address_message].position.CPR_code[3] = 0;
	   	   }
	   	 
	   	   if(((new_data.time-time_save_message)<=GLOBAL_TIME_THRESHOLD)
	   	   &&((new_data.time-time_save_position)>LOCAL_TIME_THRESHOLD))//�п���new_data.time-time_save_messageΪ0�������ݿ���ȫ���洢�Ǿ�γ����Ϣ
	   	   {
	   	     data_save[address_position].time = new_data.time;
		     data_save[address_position].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		     data_save[address_position].position.CPR_code[0] = new_data.position.CPR_code[0];
		     data_save[address_position].position.CPR_code[1] = new_data.position.CPR_code[1];
		     data_save[address_position].position.CPR_code[2] = new_data.position.CPR_code[2];
	         data_save[address_position].position.CPR_code[3] = 0;
	   	   }

	       if(((new_data.time-time_save_message)<=GLOBAL_TIME_THRESHOLD)
	       &&((new_data.time-time_save_position)<=LOCAL_TIME_THRESHOLD))//������ֵ������һ����Ϊ0
	   	   {
	   	      if(((new_data.time-time_save_message)!=0)&&((new_data.time-time_save_position)==0))//���ݿ�ȫ�����Ǳ��ģ�û��λ����Ϣ��ֻ�ܸ���ʱ������ı��Ĵ洢
			  {
			     data_save[address_message].time = new_data.time;
		         data_save[address_message].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		         data_save[address_message].position.CPR_code[0] = new_data.position.CPR_code[0];
		         data_save[address_message].position.CPR_code[1] = new_data.position.CPR_code[1];
		         data_save[address_message].position.CPR_code[2] = new_data.position.CPR_code[2];
				 data_save[address_message].position.CPR_code[3] = 0;
	          }
	          if(((new_data.time-time_save_message)==0)&&((new_data.time-time_save_position)!=0))//���ݿ�ȫ������λ����Ϣ��û�б��ģ�ֻ�ܸ���ʱ�������λ�ô洢
	   	      {
	   	         data_save[address_position].time = new_data.time;
		         data_save[address_position].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		         data_save[address_position].position.CPR_code[0] = new_data.position.CPR_code[0];
		         data_save[address_position].position.CPR_code[1] = new_data.position.CPR_code[1];
		         data_save[address_position].position.CPR_code[2] = new_data.position.CPR_code[2];
			     data_save[address_position].position.CPR_code[3] = 0;
			  }
			  if(((new_data.time-time_save_message)!=0)&&((new_data.time-time_save_position)!=0))//���ݿ�������ı��ĺ�λ��ʱ�䶼�������ڣ�ѡ�����ʱ�������λ�ô洢
	          {
	   	         data_save[address_position].time = new_data.time;
		         data_save[address_position].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		         data_save[address_position].position.CPR_code[0] = new_data.position.CPR_code[0];
		         data_save[address_position].position.CPR_code[1] = new_data.position.CPR_code[1];
		         data_save[address_position].position.CPR_code[2] = new_data.position.CPR_code[2];
			     data_save[address_position].position.CPR_code[3] = 0;
			  }
			  //������ڳ���ִ�й����У��������������ȫ����ȷ������£��ǲ�����ֵģ�д�������Ŀ��ֻ��Ϊ�˱����������Ӧ��ʱ����Ϣ�������
			  if(((new_data.time-time_save_message)==0)&&((new_data.time-time_save_position)==0))////���4.4�������ϲ������
			  {
				 data_save[address_message].time = new_data.time;
			     data_save[address_message].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
			     data_save[address_message].position.CPR_code[0] = new_data.position.CPR_code[0];
			     data_save[address_message].position.CPR_code[1] = new_data.position.CPR_code[1];
			     data_save[address_message].position.CPR_code[2] = new_data.position.CPR_code[2];
			     data_save[address_message].position.CPR_code[3] = 0;
		      }
	   	    }
			return(2);
    		// return(Calculation_ok);//ICAOƥ��ɹ�������£���(ICAO_match==0)�������㣬ֻ�ܷ���2	
       }////time_save_messageΪ0��else
   }
}   	 
//*************************************************************************************************************************************************************
//*                                                                                                                                                           *
//*  ����Ϊ���ٶ���Ϣ�漰���ĺ�����   decode_velocity()                                                                     
//*                                                                                                                                                           *
//*************************************************************************************************************************************************************
#if 1
unsigned short decode_velocity() 
{
  
  float velocity_polar_coordinates=0;
  //temp_code=velocity_code;
  airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address & 0xffffff;//��8λ��0�������ٶ�״̬��Ϣ�洢
  if((velocity_subtype>=1)&&(velocity_subtype<=4))
  {
       //------------------------1---------------------------- 
      if(velocity_subtype==1)//���ٷǳ�����ģʽ
      {
	       airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x1000000;/*airplane_velocity_three.ICAO_addressΪ32bit���޷������ͣ���24��ICAO����27��26��25λ���ٶ�״̬��Ϊ001*/
	       if(velocity_code.bit47_56==0)
	       {
		         airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x40000000;//airplane_velocity.ICAO_addressΪ32bit���޷������ͣ���24��ICAO��
                                                                                // ���������ٶ���Ч��1��ʼ��ŵĵ�31λ(0��ʼ��ŵĵ�30λ)����Ϊ1
				 airplane_velocity_three.E_W_velocity=0; 
		   }
	       else
	       {
	            if(velocity_code.bit46==0)
	 		    {
	 		        if(velocity_code.bit47_56<1023)
			              airplane_velocity_three.E_W_velocity=(int)(1.852*(velocity_code.bit47_56-1)*10000);//����ֵ����ʾ��,�ٶ�KM/H,����Ϊ1E-4
				    else
				          airplane_velocity_three.E_W_velocity=18918180;                       //Ϊ1023ʱ�����⴦������ֵ����ʾ��,�ٶ�KM/H,����Ϊ1E-4
		        }  
		        else
		        {
		            if(velocity_code.bit47_56<1023)
				         airplane_velocity_three.E_W_velocity=(int)(-1.852*(velocity_code.bit47_56-1)*10000);//����ֵ����ʾ����,�ٶ�KM/H������Ϊ1E-4
		            else
					     airplane_velocity_three.E_W_velocity=-18918180;                      //Ϊ1023ʱ�����⴦������ֵ����ʾ����,�ٶ�KM/H,����Ϊ1E-4		     
		        }
		  }
		  if(velocity_code.bit58_67==0)
		  {
		           airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x20000000;//airplane_velocity.ICAO_addressΪ32bit���޷������ͣ���24��ICAO��
			                                                                                // �ϱ������ٶ���Ч��1��ʼ��ŵĵ�30λ(0��ʼ��ŵĵ�29λ)����Ϊ1
				   airplane_velocity_three.N_S_velocity=0;  
		  }
	      else
		  {
				   if(velocity_code.bit57==0) 
				   {
	                   if(velocity_code.bit58_67<1023)         
	                       airplane_velocity_three.N_S_velocity=(int)(1.852*(velocity_code.bit58_67-1)*10000);//����ֵ����ʾ�򱱣��ٶ�KM/H,����Ϊ1E-4
	                   else
	                       airplane_velocity_three.N_S_velocity=18918180;                       //Ϊ1023ʱ�����⴦������ֵ����ʾ��,�ٶ�KM/H,����Ϊ1E-4           
		           }
		           else
	               {
	                   if(velocity_code.bit58_67<1023) 
	                       airplane_velocity_three.N_S_velocity=(int)(-1.852*(velocity_code.bit58_67-1)*10000);//����ֵ����ʾ���ϣ��ٶ�KM/H,����Ϊ1E-4
	                   else
	                       airplane_velocity_three.N_S_velocity=-18918180;                       //Ϊ1023ʱ�����⴦������ֵ����ʾ����,�ٶ�KM/H,����Ϊ1E-4  
	               }  
	      }
          if(velocity_code.bit70_78==0)
          {
                   airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x10000000;//airplane_velocity_three.ICAO_addressΪ32bit���޷������ͣ���24��ICAO��
			                                                                               // ��ֱ�����ٶ���Ч��1��ʼ��ŵĵ�29λ(0��ʼ��ŵĵ�28λ)����Ϊ1
				   airplane_velocity_three.VERT_velocity=0; 
          }
          else
          {
                  if(velocity_code.bit69==0)
	              {
	                   if(velocity_code.bit70_78<511)   
	                       airplane_velocity_three.VERT_velocity=(int)(0.3048*(velocity_code.bit70_78-1)*64*10000);   //����ֵ����ʾ���ϣ��ٶ���/����,����Ϊ1E-4
				       else
				           airplane_velocity_three.VERT_velocity=99389184;                          //Ϊ511ʱ�����⴦��,����ֵ����ʾ���ϣ��ٶ���/����,����Ϊ1E-4
	              }
	              else 
			      {
			           if(velocity_code.bit70_78<511) 
			               airplane_velocity_three.VERT_velocity=(int)(-0.3048*(velocity_code.bit70_78-1)*64*10000);  //����ֵ����ʾ���£��ٶ���/����,����Ϊ1E-4
	                   else
				           airplane_velocity_three.VERT_velocity=-99389184;                          //Ϊ511ʱ�����⴦,����ֵ����ʾ���£��ٶ���/����,����Ϊ1E-4
                  }
           }  
         return(5);//���ٷǳ�����ģʽ��ά�ٶȣ�����ֵ5
        }
		 //------------------------2---------------------------- 
        if(velocity_subtype==2)////���ٳ�����ģʽ
	    {
    	       airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x2000000;/* airplane_velocity_three.ICAO_addressΪ32bit���޷������ͣ���24��ICAO��
		                                                                                ��27��26��25λ���ٶ�״̬��Ϊ010*/
		       if(velocity_code.bit47_56==0)
		       {
		           airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x40000000;//airplane_velocity.ICAO_addressΪ32bit���޷������ͣ���24��ICAO��
			                                                                               // ���������ٶ���Ч��1��ʼ��ŵĵ�31λ(0��ʼ��ŵĵ�30λ)����Ϊ1
				   airplane_velocity_three.E_W_velocity=0; 
			   }
		       else
			   {
			       if(velocity_code.bit46==0)
			       {
			           if(velocity_code.bit47_56<1023)
			               airplane_velocity_three.E_W_velocity=(int)(7.408*(velocity_code.bit47_56-1)*10000);//����ֵ����ʾ��,�ٶ�KM/H,����Ϊ1E-4
					   else
					       airplane_velocity_three.E_W_velocity=75672720;                       //Ϊ1023ʱ�����⴦������ֵ����ʾ��,�ٶ�KM/H,����Ϊ1E-4
		           }
		           else
				   {
				       if(velocity_code.bit47_56<1023)
				           airplane_velocity_three.E_W_velocity=(int)(-7.408*(velocity_code.bit47_56-1)*10000);//����ֵ����ʾ����,�ٶ�KM/H,����Ϊ1E-4
		               else
					       airplane_velocity_three.E_W_velocity=-75672720;                      //Ϊ1023ʱ�����⴦������ֵ����ʾ����,�ٶ�KM/H,����Ϊ1E-4		     
			       }
		       }
		       if(velocity_code.bit58_67==0)
		       {
		           airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x20000000;//airplane_velocity.ICAO_addressΪ32bit���޷������ͣ���24��ICAO��
			                                                                               // �ϱ������ٶ���Ч��1��ʼ��ŵĵ�30λ(0��ʼ��ŵĵ�29λ)����Ϊ1
				   airplane_velocity_three.N_S_velocity=0; 
			   }
	           else
			   {
				   if(velocity_code.bit57==0) 
                   {
	                   if(velocity_code.bit58_67<1023)         
	                     airplane_velocity_three.N_S_velocity=(int)(7.408*(velocity_code.bit58_67-1)*10000);//����ֵ����ʾ�򱱣��ٶ�KM/H,����Ϊ1E-4
	                   else
	                     airplane_velocity_three.N_S_velocity=75672720;                       //Ϊ1023ʱ�����⴦������ֵ����ʾ��,�ٶ�KM/H,����Ϊ1E-4           
		           }
		           else
	               {
	                   if(velocity_code.bit58_67<1023) 
	                     airplane_velocity_three.N_S_velocity=(int)(-7.408*(velocity_code.bit58_67-1)*10000);//����ֵ����ʾ���ϣ��ٶ�KM/H,����Ϊ1E-4
	                   else
	                     airplane_velocity_three.N_S_velocity=-75672720;                       //Ϊ1023ʱ�����⴦������ֵ����ʾ����,�ٶ�KM/H,����Ϊ1E-4  
	               }
               }
               if(velocity_code.bit70_78==0)
               {
                    airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x10000000;//airplane_velocity.ICAO_addressΪ32bit���޷������ͣ���24��ICAO��
			                                                                                // ��ֱ�����ٶ���Ч��1��ʼ��ŵĵ�29λ(0��ʼ��ŵĵ�28λ)����Ϊ1
				    airplane_velocity_three.VERT_velocity=0; 
               }
               else
			   {
                   if(velocity_code.bit69==0)
                   { 
	                   if(velocity_code.bit70_78<511)   
	                       airplane_velocity_three.VERT_velocity=(int)(0.3048*(velocity_code.bit70_78-1)*64*10000);   //����ֵ����ʾ���ϣ��ٶ���/����,����Ϊ1E-4  
				       else
				           airplane_velocity_three.VERT_velocity=99389184;                          //Ϊ511ʱ�����⴦��,����ֵ����ʾ���ϣ��ٶ���/����,����Ϊ1E-4  
	               }
	               else 
			       {
			           if(velocity_code.bit70_78<511) 
			               airplane_velocity_three.VERT_velocity=(int)(-0.3048*(velocity_code.bit70_78-1)*64*10000);  //����ֵ����ʾ���£��ٶ���/����,����Ϊ1E-4  
	                   else
				           airplane_velocity_three.VERT_velocity=-99389184;                          //Ϊ511ʱ�����⴦��,����ֵ����ʾ���£��ٶ���/����,����Ϊ1E-4  
                   }
                }
                return(5);//���ٳ�����ģʽ,��ά�ٶȣ�����ֵ5
        }
		 //------------------------3---------------------------- 
        if(velocity_subtype==3)//���ٷǳ�����ģʽ
        {
	          //airplane_velocity_two.ICAO_address=airplane_velocity_two.ICAO_address|0x3000000;
		      airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x3000000;         //��27��26��25λ���ٶ�״̬��Ϊ011*/
              if((velocity_code.bit46==0)||(velocity_code.bit58_67==0))//ƽ���ٶȷ���δ��֪����ƽ���ٶ�δ��֪
              {
		           airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x60000000;//airplane_velocity.ICAO_addressΪ32bit���޷������ͣ���24��ICAO��
			                                                                               // ƽ���ٶȷ���δ��֪��1��ʼ��ŵĵ�30��31λ(0��ʼ��ŵ�29��30λ)����Ϊ1
				   airplane_velocity_three.N_S_velocity=0;
				   airplane_velocity_three.E_W_velocity=0; 
			  }
              else
			  {
                  if(velocity_code.bit58_67<1023)
			           velocity_polar_coordinates=1.852*(velocity_code.bit58_67-1);
                  else
			           velocity_polar_coordinates=1891.818;

				  airplane_velocity_three.E_W_velocity=(int)(velocity_polar_coordinates*sin(0.3515625*velocity_code.bit47_56*PI/180.0)*10000);//��������Ϊ0�ȣ�˳ʱ����ת����ֵΪ������ֵΪ��������Ϊ1E-4  
                  airplane_velocity_three.N_S_velocity=(int)(velocity_polar_coordinates*cos(0.3515625*velocity_code.bit47_56*PI/180.0)*10000);//��������Ϊ0�ȣ�˳ʱ����ת����ֵΪ������ֵΪ�ϣ�����Ϊ1E-4 
			  }
              if(velocity_code.bit70_78==0)
              {
                   airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x10000000;//airplane_velocity.ICAO_addressΪ32bit���޷������ͣ���24��ICAO��
			                                                                               // ��ֱ�����ٶ���Ч��1��ʼ��ŵĵ�29λ(0��ʼ��ŵĵ�28λ)����Ϊ1
				   airplane_velocity_three.VERT_velocity=0; 
              }
              else
			  {
                   if(velocity_code.bit69==0)
	                   if(velocity_code.bit70_78<511)   
	                     airplane_velocity_three.VERT_velocity=(int)(0.3048*(velocity_code.bit70_78-1)*64*10000);   //����ֵ����ʾ���ϣ��ٶ���/����,����Ϊ1E-4 
				       else
				         airplane_velocity_three.VERT_velocity=99389184;                          //Ϊ511������⴦�?����ֵ����ʾ���ϣ��ٶ���/����,����Ϊ1E-4 
	               else 
			           if(velocity_code.bit70_78<511) 
			             airplane_velocity_three.VERT_velocity=(int)(-0.3048*(velocity_code.bit70_78-1)*64*10000);  //����ֵ����ʾ���£��ٶ���/����,����Ϊ1E-4 
	                   else
				         airplane_velocity_three.VERT_velocity=-99389184;                          //Ϊ511ʱ�����⴦��,����ֵ����ʾ���£��ٶ���/����,����Ϊ1E-4 
              }
	          return(5);//���ٷǳ�����ģʽ,ƽ���ٶȼӴ�ֱ�ٶ���ת��Ϊ3ά�ٶȣ�����ֵ5
	    }
	    //------------------------4---------------------------- 
	    if(velocity_subtype==4)//���ٳ�����ģʽ
        {
	          airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x4000000;/* airplane_velocity.ICAO_addressΪ32bit���޷������ͣ���24��ICAO��
		                                                                                ��27��26��25λ���ٶ�״̬��Ϊ100*/
            
              if((velocity_code.bit46==0)||(velocity_code.bit58_67==0))//ƽ���ٶȷ���δ��֪����ƽ���ٶ�δ��֪
              {
		           airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x60000000;//airplane_velocity.ICAO_addressΪ32bit���޷������ͣ���24��ICAO��
			                                                                               // ƽ���ٷ���δ��֪��1��ʼ��ŵĵ�30��31λ(0��ʼ��ŵ�29��30λ)����Ϊ1
				   airplane_velocity_three.N_S_velocity=0;
				   airplane_velocity_three.E_W_velocity=0; 
			  }
              else
			  {
                  if(velocity_code.bit58_67<1023)
			           velocity_polar_coordinates=7.408*(velocity_code.bit58_67-1);
                  else
			           velocity_polar_coordinates=7567.272;

				   airplane_velocity_three.E_W_velocity=(int)(velocity_polar_coordinates*sin(0.3515625*velocity_code.bit47_56*PI/180.0)*10000);//��������Ϊ0�ȣ�˳ʱ����ת����ֵΪ������ֵΪ��,����Ϊ1E-4 
                   airplane_velocity_three.N_S_velocity=(int)(velocity_polar_coordinates*cos(0.3515625*velocity_code.bit47_56*PI/180.0)*10000);//��������Ϊ0�ȣ�˳ʱ����ת����ֵΪ������ֵΪ��,����Ϊ1E-4 
			  }
              if(velocity_code.bit70_78==0)
              {
                   airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x10000000;//airplane_velocity.ICAO_addressΪ32bit���޷������ͣ���24��ICAO��
			                                                                               // ��ֱ�����ٶ���Ч��1��ʼ��ŵĵ�29λ(0��ʼ��ŵĵ�28λ)����Ϊ1
				   airplane_velocity_three.VERT_velocity=0; 
              }
              else
		      {
                   if(velocity_code.bit69==0)
	               {
	                   if(velocity_code.bit70_78<511)   
	                        airplane_velocity_three.VERT_velocity=(int)(0.3048*(velocity_code.bit70_78-1)*64*10000);   //����ֵ�������ϣ��ٶ���/����,����Ϊ1E-4 
				       else
				            airplane_velocity_three.VERT_velocity=99389184;                          //Ϊ511ʱ�����⴦��,����ֵ����ʾ���ϣ��ٶ���/����,����Ϊ1E-4 
	               }
	               else 
			       {
			           if(velocity_code.bit70_78<511) 
			               airplane_velocity_three.VERT_velocity=(int)(-0.3048*(velocity_code.bit70_78-1)*64*10000);  //����ֵ����ʾ���£��ٶ���/����,����Ϊ1E-4 
	                   else
				           airplane_velocity_three.VERT_velocity=-99389184; 
				   }                                 //Ϊ511ʱ�����⴦��,����ֵ����ʾ���£��ٶ���/����,����Ϊ1E-4 
                }
	            return(5);//���ٳ�����ģʽ,ƽ���ٶȼӴ�ֱ�ٶ���ת��Ϊ3ά�ٶȣ�����ֵ5
	    }   
	       
	}
    else
    {
	    return(4);
    }
}   
#endif
/****************************************/

//*************************************************************************************************************************************************************
//*                                                                                                                                                           *
//*  ����Ϊ��߶���Ϣ�ĺ����� decode_altitude()   
//zhangfulong add from chengdian li                                                                
//*                                                                                                                                                           *
//*************************************************************************************************************************************************************

int decode_altitude() 
{
   int i,j,k;
   unsigned short int  altitude_code;
   unsigned short int altitude_code_temp_low,altitude_code_temp_high,altitude_code_temp;
   unsigned short int altitude_decode_DAB,altitude_code_DAB[8];
   unsigned short int altitude_code_temp_C,altitude_decode_C;

   int altitude_foot;

  

   if((altitude_code_BARO==0)&&(altitude_code_GNSS==0))
     {
     airplane_location.altitude=0;
     airplane_location.ICAO_address=airplane_location.ICAO_address|0x1000000;//��24λΪ0,����߶���Ч,Ϊ1����߶�������Ч��
     return(0);     
     }
   
    if(altitude_code_GNSS==0)
     {
      altitude_code=altitude_code_BARO;	 
     }
   
    if(altitude_code_BARO==0)
      {
      altitude_code=altitude_code_GNSS;
	  airplane_location.ICAO_address=airplane_location.ICAO_address|0x2000000;//��25λΪ0,����ǰ����ѹ�߶�,Ϊ1������ǰ��GNSS�߶ȡ�
     }

   if((altitude_code&0x10)==0x10)
    {

      altitude_code_temp_low=altitude_code&0xF;//�߶ȱ����4BIT
	  altitude_code_temp_high=(altitude_code>>1)&0x7F0;//�߶ȱ����7BIT

	  altitude_code_temp=altitude_code_temp_low|altitude_code_temp_high;////�߶ȱ���11bit
      altitude_foot=altitude_code_temp*25-1000;//��ʽ����߶�ֵ����λӢ��
      airplane_location.altitude=(3048*altitude_foot);//����Ϊ��Ϊ��λ�����ҷŴ�10000��
    }
   else
    {
       airplane_location.ICAO_address=airplane_location.ICAO_address|0x4000000;//��26λΪ0,����ǰ�߶Ⱦ���Ϊ��3.81�ף���12.5Ӣ�ߣ�,Ϊ1����ǰ�߶Ⱦ���Ϊ��15.24�ף���50Ӣ�ߣ�

      memset(altitude_code_DAB, 0, sizeof(altitude_code_DAB));

      altitude_code_DAB[0]=(altitude_code>>2)&0x1;//�ƶ�D2
	  altitude_code_DAB[1]=(altitude_code>>0)&0x1;//�ƶ�D4
      altitude_code_DAB[1]=(altitude_code_DAB[0]^altitude_code_DAB[1])&0x1;//����������
	  altitude_code_DAB[2]=(altitude_code>>10)&0x1;//�ƶ�A1
      altitude_code_DAB[2]=(altitude_code_DAB[1]^altitude_code_DAB[2])&0x1;//����������
      altitude_code_DAB[3]=(altitude_code>>8)&0x1;//�ƶ�A2
      altitude_code_DAB[3]=(altitude_code_DAB[2]^altitude_code_DAB[3])&0x1;//����������
      altitude_code_DAB[4]=((altitude_code>>6)&0x08);//�ƶ�A4
      altitude_code_DAB[4]=(altitude_code_DAB[3]^altitude_code_DAB[4])&0x1;//����������
      altitude_code_DAB[5]=(altitude_code>>5)&0x1;//�ƶ�B1
      altitude_code_DAB[5]=(altitude_code_DAB[4]^altitude_code_DAB[5])&0x1;//����������
      altitude_code_DAB[6]=(altitude_code>>3)&0x1;//�ƶ�B2
      altitude_code_DAB[6]=(altitude_code_DAB[5]^altitude_code_DAB[6])&0x1;//����������
	  altitude_code_DAB[7]=(altitude_code>>1)&0x1;//�ƶ�B4
      altitude_code_DAB[7]=(altitude_code_DAB[6]^altitude_code_DAB[7])&0x1;//����������

      altitude_decode_DAB=0;
      
      for(i=0;i<8;i++) 
	      altitude_decode_DAB=altitude_decode_DAB|(altitude_code_DAB[i]<<(7-i));//�õ�������������Ӧ�Ķ�������ֵ

      altitude_code_temp_C=0;
      altitude_code_temp_C=((altitude_code>>9)&0x04)|altitude_code_temp_C;//�ƶ�C1
      altitude_code_temp_C=((altitude_code>>8)&0x02)|altitude_code_temp_C;//�ƶ�C2
	  altitude_code_temp_C=((altitude_code>>7)&0x01)|altitude_code_temp_C;//�ƶ�C4
      
       if((altitude_decode_DAB%2)==0)//����DAB�������������ֵ��ż����C��������ѭ��������
         {
		   switch(altitude_code_temp_C)
		    { 
               case 1:altitude_decode_C=0;break;
			   case 3:altitude_decode_C=1;break;
			   case 2:altitude_decode_C=2;break;
			   case 6:altitude_decode_C=3;break;
			   case 4:altitude_decode_C=4;break;
			   default:altitude_decode_C=5;
            }
         }
       else
	     {
	       switch(altitude_code_temp_C)
		    { 
               case 4:altitude_decode_C=0;break;
			   case 6:altitude_decode_C=1;break;
			   case 2:altitude_decode_C=2;break;
			   case 3:altitude_decode_C=3;break;
			   case 1:altitude_decode_C=4;break;
			   default:altitude_decode_C=5;
            } 
	     }
      
       if((altitude_decode_C<=4)&&(altitude_decode_C>=0))
	     {
	      altitude_foot=altitude_decode_DAB*500+altitude_decode_C*100-1200;
	      airplane_location.altitude=3048*altitude_foot;//����Ϊ��Ϊ��λ�����ҷŴ�10000��
	     }
	   else
	     {
	     airplane_location.altitude=0;
	     airplane_location.ICAO_address=airplane_location.ICAO_address|0x1000000;//��24λΪ0,����߶���Ч,Ϊ1����߶�������Ч��
                                                                                 //altitude_decode_Cֻ����0-4�е�������������ֵ���߶ȱ�����󣬸߶���Ч
	     }
      
	      
    }
	
}

