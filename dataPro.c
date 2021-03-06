/*******************************************/
#include <math.h>
//必须包含
#define NZ 15
#define PI 3.141592653589793
#define LOCAL_TIME_THRESHOLD    (480*10000)
//经纬信息"局部解算"时间门限，对应单位为"秒"
//#define GLOBAL_TIME_THRESHOLD   18 
#define GLOBAL_TIME_THRESHOLD   (15*10000) 
//经纬信息"全局解算"时间门限，对应单位为"秒"
//3.14159265358979323846
//定义在主调函数中的结构体
typedef struct FPGA_YC  //FPGA-DSP 遥测
{
	unsigned int UTCtime;    //4
    unsigned short int temperature; //2
    unsigned short int YC_33;  //2
	unsigned short int YC_55;//2
//--
    unsigned char YK_cnt;//1
    unsigned char DSP_RST_cnt;//1
	unsigned char FPGA_RST_cnt;//1
	unsigned char FPGA_stat;//1-"2bFPGA配置/2b状态/2b工作模式/2b00"
    unsigned char reserve[2];  //
}FPGA_YC;
extern FPGA_YC s_FPGA_YC;  //16字节遥测
//*************************************************************************************************************************
//定义在主调函数中的结构体,用于全局定义结构体变量的声明
struct speed_struct_three
{
  unsigned int ICAO_address;
  unsigned int time;
  int  N_S_velocity;      
  int  E_W_velocity;  	  
  int  VERT_velocity;  
};
extern struct speed_struct_three airplane_velocity_three;//用于速度上报
/*********************************/
extern unsigned char velocity_subtype;//速度解算全局变量，用于传递信息使用，用户不用关心
struct speed_code
{
  unsigned char bit46;
  unsigned short bit47_56;
  unsigned char bit57;
  unsigned short bit58_67;
  unsigned char bit69;
  unsigned short bit70_78;
};
extern struct speed_code velocity_code; //速度解算全局结构体变量，用于传递信息使用，用户不用关心
//*************************************************************************************************************************
struct data_struct
{
  unsigned int ICAO_address_with_mark;//换了位置 190314 (ICAO_adress_with_mark)和(time)
  unsigned int time;
  union
  {
   double coordinate[2]; //坐标
   unsigned int CPR_code[4];
  }position;
};
extern struct data_struct data_save[1000],new_data;//此结构体数组和结构体变量全局使用，data_save[1000],new_data;
/*********************************/
struct location_struct
{
  unsigned int ICAO_address;
  unsigned int time;
  int coordinate[2];//正式上报时候使用int型，正式上报时候使用int(1E-7为单位)。
  int altitude;
};
extern struct location_struct airplane_location;//此结构体数组和结构体变量全局使用,airplane_location;
//定义在主调函数中的结构体,用于全局定义结构体变量的声明
struct message_struct
{
  unsigned int time;
  unsigned char data_demodulator[11];  //ADSB的原始88bit消息报文，11字节							
};
extern struct message_struct ADSB_message;//上报的ADSB原始报文,固定格式，不区分报文种类，向主函数传递参数
/*********************************/
#define pulse_BUFLEN 150
extern unsigned short   pulse_amp[112];
extern unsigned int pulse_time[pulse_BUFLEN];  //190521
extern unsigned int pulse_rd ;
unsigned short  DF_message=0;//用于报文提取组织为解算经纬度需要的格式使用 DF可为17,18,19 
unsigned short  CF_AF_message=0;//DF=18,19时候使用，在DF为18和19的时候第6到8位必须为0，即DF18时CF为0或DF19时为AF为0	
unsigned short  TYPE_Code_message=0;//用于判断该报文是否是空中位置报文
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
 //返回值2，指示该帧数据处理正常，并且该帧是位置报文，更新ADSB_message数组用于上传报文，同时更新了new_data结构体变量
 //返回值1，指示该帧数据处理正常，但该帧不是位置报文，仅更新ADSB_message数组用于上传报文；
 //返回值0，指示该帧数据不正常或是解调制后数据不正确, ADSB_message数组不会被更新；
//------------------------
 //返回值2，指示该帧处理是数据报文，但是解算不成功，仅更新了ADSB_message数组，以及数据库
 //返回值3，指示该帧处理是数据报文，解算成功，更新了ADSB_message数组，airplane_location结构体变量以及数据库
/**********************************************/
unsigned int data_pro(unsigned short  * fifo)
{
     unsigned short  temp_message=0;//用于报文拼接为11字节的无符号字符型数据
     unsigned int i,j;
     unsigned short  *  pulse_tmp;
      //此段代码按照要求打包上报数据,通过CRC校验，并且DF为17即可以上传原始88bit的数据包,添加时间信息，检查是否为位置或速度报文，是则将数据准备好
            //***************************************************************************************************  
     pulse_tmp=fifo; //传递一下
     DF_message=0;
     for(i=0;i<5;i++)  //前5位是0，基本可能是全0
     {   
        DF_message=DF_message|(pulse_tmp[i]<<(4-i));//取DF位
	 }
	  if(DF_message==17)
	  {
          // 检查是否为数据位置报文，或者速度报文，是则打包数据
          //*****************************************************************************************  
          TYPE_Code_message=0;
          for(i=0;i<5;i++)
          {
              TYPE_Code_message=TYPE_Code_message|(pulse_tmp[i+32]<<(4-i));
		  }
           //TYPE类型9-18或者20-22表示是位置报文
          if(((TYPE_Code_message>=9)&&(TYPE_Code_message<=18))||((TYPE_Code_message>=20)&&(TYPE_Code_message<=22)))
          {   
			 memset(new_data, 0, sizeof(new_data));////由于结构体new_data是全局的，故此处需要清0；
 #if 1   
		//	new_data.time=	htonl(s_FPGA_YC.UTCtime);  //时间处理（1）
            new_data.time=	htonl(pulse_time[pulse_rd]);  //190521
			
 #endif              
             //此处是最新的位置报文更新， new_data.ICAO_adress_with_mark中第25位标志位不用理会，此时一定存在结构体中的是报文数据                    
             for(i=0;i<24;i++)
			 {
		        new_data.ICAO_address_with_mark=new_data.ICAO_address_with_mark|(pulse_tmp[i+8]<<(23-i));//把按BIT存放的24位ICAO地址转换成一个24位数据存放
             }
             //new_data.ICAO_adress_with_mark=new_data.ICAO_adress_with_mark|0x1000000;
             //new_data.ICAO_adress_with_mark第25位（编号24）存放标志位，表示存放的经纬度编码
             for(i=0;i<17;i++)//将经纬度编码存放在结构体中的共用体中
             {
                  new_data.position.CPR_code[0]=new_data.position.CPR_code[0]|(pulse_tmp[i+54]<<(16-i));//纬度编码
                  new_data.position.CPR_code[1]=new_data.position.CPR_code[1]|(pulse_tmp[i+71]<<(16-i));//经度编码
             }
             new_data.position.CPR_code[2]=pulse_tmp[53];//放置奇偶编码标志位在共用体中


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

			 return(2);//返回值2，指示该帧数据处理正常,一定是位置报文
           }  
           //TYPE类型19表示是位置报文 //速度报文？？
           if(TYPE_Code_message==19)
           {
		         memset(airplane_velocity_three, 0, sizeof(airplane_velocity_three));////由于结构体airplane_velocity是全局的，需清0；
				     
			     memset(velocity_code, 0, sizeof(velocity_code));////全局变量，用于向速度解算函数传递参数，需清0；
			     velocity_subtype=0;//全局变量，用于向速度解算函数传递参数，需清0
#if 1   
				 airplane_velocity_three.time=	htonl(s_FPGA_YC.UTCtime);  //时间处理（2）
#endif
                 for(i=0;i<24;i++)
			     {
			        airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|(pulse_tmp[i+8]<<(23-i));//把按BIT存放的24位ICAO地址转换成一个24位数据存放
                 }      
                 for(i=0;i<3;i++)  
                 {
                    velocity_subtype=velocity_subtype|(pulse_tmp[i+37]<<(2-i));//存储速度帧的子类型
				 } 	  
			   //提取速度编码的码字，用于后续处理  
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
		         return(4);//返回值4，指该帧荽碚�,一定是速度报文				   
		   }            
           return(1);//返回值1，指示该帧数据处理正常,为位置和速度报文之外的其他报文
	   } //  if(DF_message==17)
       else  //DF!=17
       {
          return(1);//返回值1，指示该帧数据传输正常，处理后能通过校验，但是不是DF17的格式
       }  

}
//*************************************************************************************************************************************************************
//*                                                                                                                                                           *
//*  以下为解位置信息涉及到的函数：涉及NL（），my_mod（），lat_lon_local_calculate（），lat_lon_global_calculate（）  decode_position()5个函数                *                     
//*                                                                                                                                                           *
//*************************************************************************************************************************************************************
int NL(double  m)//解算经度的ZONE分割数目,不同纬度情况下经度的分割渴遣煌模枰扑�
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
double my_mod(double a,double b)//标准中CPR算法定义的求模运算规则
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
  mark=st->position.CPR_code[2];//存当前最新报文的奇偶编码类型

  ref_lon=st_ref->position.coordinate[1];//前一个时刻经度参考
  ref_lat=st_ref->position.coordinate[0];//前一个时刻纬度参考
 
  XZ=st->position.CPR_code[1];//当前的经度编码
  YZ=st->position.CPR_code[0];//当前的纬度编码

  Dlat=360.0/((double)(4*NZ-mark));//由当前的奇偶编码方式，确定纬度按照6或6.1分割

  j=floor(ref_lat/Dlat)+floor(0.5+(my_mod(ref_lat,Dlat)/Dlat)-(YZ/pow(2,17)));//求纬度索引
  Rlat=Dlat*(j+(YZ/pow(2,17)));//解算当前纬度的具体值

  number_of_lon=NL(Rlat);//根据纬度的值求经度索引

  if((number_of_lon-mark)>0)
    Dlon=360.0/((double)(number_of_lon-mark));
  else
    Dlon=360.0;

  m=floor(ref_lon/Dlon)+floor(0.5+(my_mod(ref_lon,Dlon)/Dlon)-(XZ/pow(2,17)));
  Rlon=Dlon*(m+(XZ/pow(2,17)));//当前的经度具体解算

  //更新数据库
  st_ref->time=st->time;
  st_ref->ICAO_address_with_mark=st->ICAO_address_with_mark|0x1000000;
  st_ref->position.coordinate[0]=Rlat;//保存当前解算的纬度值到数据库
  st_ref->position.coordinate[1]=Rlon;//保存当前解算的经度值到数据库
  
  //更新上报位置信息用的结构体变量
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
  int  XZ_0,YZ_0,XZ_1,YZ_1,j,m;//j和m的定义方式来自标准，整数，从-59到58范围
  int  mark;
  double hh_tmp1;
  double hh_tmp2;
  int hh_tmp3;
  mark = st->position.CPR_code[2];//取当前时刻收到报文的奇偶编码类型
  if(mark == 0)//当前为偶编码的情况
  {
    XZ_0=st->position.CPR_code[1];
    YZ_0=st->position.CPR_code[0];
    XZ_1=st_ref->position.CPR_code[1];
    YZ_1=st_ref->position.CPR_code[0];
  }
  else//当前为奇编码的情况
  {
    XZ_0=st_ref->position.CPR_code[1];
    YZ_0=st_ref->position.CPR_code[0];
    XZ_1=st->position.CPR_code[1];
    YZ_1=st->position.CPR_code[0];
  }

  Dlat_0=360.0/((double)4*NZ-0);//偶编码情况纬度的分割间隔
  Dlat_1=360.0/((double)4*NZ-1);//奇编码情况纬度的分割间隔
 
  j=floor(0.5+(59*YZ_0-60*YZ_1)/pow(2,17));//POW返回为DOUBLE类型

  Rlat_0=Dlat_0*(my_mod(j,60-0)+YZ_0/pow(2,17));//偶编码情况纬度的解算结果
  Rlat_1=Dlat_1*(my_mod(j,60-1)+YZ_1/pow(2,17));//奇编码情况纬度的解算结果
  if((Rlat_0>90&&Rlat_0<270)||(Rlat_1>90&&Rlat_1<270))//该情况不可能出现，避免出错，加该条语句
    return(2) ;//返回2表示解算不成功
  
  if(Rlat_0 >= 270.0 && Rlat_0 <=360.0)//偶编码纬度的解算结果统一到南北纬，北正，南负
    Rlat_0=my_mod(Rlat_0+180,360)-180.0;   
  
  if(Rlat_1 >= 270.0 && Rlat_1 <=360.0)//奇编码纬度的解算结果统一到南北纬，北正，南负 
    Rlat_1=my_mod(Rlat_1+180,360)-180.0;
  

  number_of_lon_0=NL(Rlat_0);//根据偶编码纬度，计算经度分割数
  number_of_lon_1=NL(Rlat_1);//根据奇编码纬度，计算经度分割数

   if(number_of_lon_0 != number_of_lon_1)//根据奇偶编码纬度计算出的经度分割数目不相同，则无法解算经度信息，放弃解算，保存当前报文到数据库
   {
      st_ref->time=st->time;
	  st_ref->position.CPR_code[0]=st->position.CPR_code[0];
      st_ref->position.CPR_code[1]=st->position.CPR_code[1];
      st_ref->position.CPR_code[2]=st->position.CPR_code[2];
	  st_ref->position.CPR_code[3]=st->position.CPR_code[3];
    
      return(2) ;//返回2表示解算不成功
   }
   else
   {
     if(number_of_lon_0 > 1)//number_of_lon_0与number_of_lon_1相等，number_of_lon_0大于1，number_of_lon_1也就大于1
       {   
		   n_i=number_of_lon_0;  //n_i,即为经度分割数目
	       Dlon_0=360.0/n_i;     //偶编码情况经度的分割间隔
	       Dlon_1=360.0/(n_i-1); //奇编码情况经度的分割间隔
	       m=floor(0.5+(XZ_0*(n_i-1)-XZ_1*(n_i))/pow(2,17));
	       Rlon_0=Dlon_0*(my_mod(m,n_i-0)+XZ_0/pow(2,17));
		   Rlon_1=Dlon_1*(my_mod(m,n_i-1)+XZ_1/pow(2,17));
	      
	       if(Rlon_0 > 180.0 && Rlon_0 <= 360.0)//偶编码经度的解算结果统一到东西经，东正，西负	       
	         Rlon_0=my_mod(Rlon_0+180,360)-180;
	       
	       if(Rlon_1 > 180.0 && Rlon_1 <= 360.0)//奇编码经度的解算结果统一到东西经，东正，西负	   
	         Rlon_1=my_mod(Rlon_1+180,360)-180;	       
       }
       else
       {	       
		   Dlon_0=360.0;
	       Dlon_1=360.0;
		   Rlon_0=Dlon_0*(XZ_0/pow(2,17));
	       Rlon_1=Dlon_1*(XZ_1/pow(2,17));

		   if(Rlon_0 > 180.0 && Rlon_0 <= 360.0)//偶编码经度的解算结果统一到东西经，东正，西负
	       	  Rlon_0=my_mod(Rlon_0+180,360)-180;
	        
	       if(Rlon_1 > 180.0 && Rlon_1 <= 360.0)//奇编码经度的解算结果统一到东西经，东正，西负
	       	  Rlon_1=my_mod(Rlon_1+180,360)-180;
	        
	   }
    }
  
   if(mark==0)//根据当前收到报文的奇偶编码状态值，决定保存和上报结果
   {
       //更新数据库
       st_ref->time=st->time;//?????????????????????????????????????????????????????
       st_ref->ICAO_address_with_mark=st->ICAO_address_with_mark | 0x1000000;
       st_ref->position.coordinate[0]=Rlat_0;
       st_ref->position.coordinate[1]=Rlon_0;

	   //更新上报位置信息用的结构体变量
       airplane_location.time=st->time;
       airplane_location.ICAO_address=st->ICAO_address_with_mark & 0x00ffffff;
       airplane_location.coordinate[1]=(int)(Rlat_0 * 10000000);
       airplane_location.coordinate[0]=(int)(Rlon_0 * 10000000);
	    return(3) ;//返回3表示解算成功
   }
   else
   {
       //更新数据库
       st_ref->time=st->time;
       st_ref->ICAO_address_with_mark=st->ICAO_address_with_mark | 0x1000000;
       st_ref->position.coordinate[0]=Rlat_1;
       st_ref->position.coordinate[1]=Rlon_1;

	   //更新上报位置信息用的结构体变量
	   airplane_location.time=st->time;
       airplane_location.ICAO_address=st->ICAO_address_with_mark & 0x00ffffff;
       airplane_location.coordinate[1]=(int)(Rlat_1 * 10000000);
       airplane_location.coordinate[0]=(int)(Rlon_1 * 10000000);
	   return(3) ;//返回3表示解算成功
   }

}
/*********************************/
//第二版的位置计算和数据库更新
unsigned short int decode_position()
{

	unsigned short int i,matching_address;
    unsigned short int address_message,address_position;
	unsigned int ICAO_temp,ICAO_now,time_save_message,time_save_position;//,time_temp,time_now;
	//unsigned int parity_type;  
    unsigned int data_save_type;
    unsigned short int Calculation_ok;//返回值，2表示解算不成功；3表示解算成功；若解算成功，解算结果放在airplane_location结构体全局变量中
    unsigned int ICAO_match=0;//必须初始化为0，ICAO匹配成功标志，成功为1，不成功为0

//new_data.time++;//hh
    if(new_data.time==0)
         return(2);            //UTC时间标签不能为0，如果为0 只能返回2，放弃当前处理                                       
    
    ICAO_now= new_data.ICAO_address_with_mark & 0x00ffffff;//取当前位置报文的ICAO地址码
   
   
    time_save_message= new_data.time;                     //time_save_message初始化当前位置报文的时刻,后续比较记录最早到达的报文时刻
    time_save_position=new_data.time;                     //ime_save_position初始化当前位置报文的时刻,后续比较记录最早解算出的经纬度信息包时刻
   
 
	for (i = 0; i < 1000; i++)//查找数据库里面已存的信息，可能是报文编码信息，可能是已经解算出的位置信息
	{
		
	       //此处是做ICAO的查找匹配，一旦匹配成功，就跳出循环，不做后续处理
	       //*****************************************************************************************************************************************
	       ICAO_temp = data_save[i].ICAO_address_with_mark & 0x00ffffff;//读取数据库ICAO地址
	        
	       if(ICAO_now == ICAO_temp)
		    {
		      ICAO_match=1;         //匹配成功标志，初始为0，表示查找匹配不成功，匹配成功则修改为1(表示在数据库中找到了一致的ICAO地址)
		      matching_address=i;   //匹配成功时，数据库中的对应一致ICAO的报文或经纬度存储位置记录
		      
		      break;                // 匹配到一致的ICAO编号，则跳出循环，不再继续查找，查找不成功则继续查找，直达遍历数据库中的1000个元素 
		    }
	       //******************************************************************************************************************************************
	      //此处是数据库的更新策略，ICAO匹配不成功，查找记录数据库中时间最早到达报文的存储地址，查找记录最早解算出的目标位置信息的存储地址，分别查找和存储
		  //********************************************************************************************************************************************	 
		   data_save_type = data_save[i].ICAO_address_with_mark & 0x1000000;//记录当前位置存储的是报文或者是经纬度信息，第25位（编号24）为0表示是报文原始信息，为1表示是经纬度信息

	        if((data_save[i].time<time_save_message)&&(data_save_type==0))
			{
			  time_save_message = data_save[i].time;//保证time_save_message保存的一定是最早的时间，对应报文
			  address_message = i; //保证address_message中存的地址偏移量一定是对应最早时间的报文，相同时间的报文记录第一个位置
			                       //初始状态下，time_save_message一定会为0，确保新接收到的不一样ICAO的报文依次存放
								   //如果全部的为解算的经纬度信息，address_message不会被修改，为定义时的初始值-1，非法
			}

		    if((data_save[i].time<time_save_position)&&(data_save_type==0x1000000))
			{
			  time_save_position = data_save[i].time;//保证time_save_position保存的一定是最早的时间，对位置信息
			  address_position = i;//保证address_position中存的地址偏移量一定是对应最早时间的解算出的位置信息存储地址，相同时间的信息记录第一个位置
			                       //初始状态下，或者是工作过程中，全部为报文信息，address_position的值不会被修改，为定义时的初始值-1，非法
			}
	     //*******************************************************************************************************************************************	
	 }	//for 1000
	
     if(ICAO_match==1)//ICAO匹配成功则可尝试进行解算，局部解算或全局解算，都需要考虑时间门限因素
	 {	    
		  data_save_type = data_save[matching_address].ICAO_address_with_mark & 0x1000000;//匹配成功时，识别数据库中的是报文或是经纬度信息，第25位（编号24）为0表示是报文原始信息，为1表示是经纬度信息
		  g_32_ICAO_match_cnt++;
		  if(data_save_type == 0x1000000)//data_save_type为0x1000000，使用局部解算
		  {		
			  //时间差值，秒为单位，333公里，飞行速度折算1224公里/小时（1马赫），理论差值最大980秒，16.3分钟。考虑卫星运动过顶时间折算，差值取480秒为局部解算的门限时间		
			  if((new_data.time - data_save[matching_address].time) <= LOCAL_TIME_THRESHOLD)//480代表差值，秒为单位
			  {
				  Calculation_ok=lat_lon_local_calculate(&new_data, &data_save[matching_address]);//调用局部解算函数，解算结果存储到data_save[i]中，同时airplane_location结构体全局变量上报			
			        //返回3
					decode_altitude();//zhangfulong add
			  }
			  else
			  {
				  Calculation_ok=2;//表示时间不满足局部解算要求，返回值设定为2
				  	//如果差值时间不满足局部解算要求，同时ICAO是匹配的，则擦除数据库中的经纬度信息，将当前的报文信息存储进去，在data_save[i]位置
				  data_save[matching_address].time = new_data.time;
				  data_save[matching_address].ICAO_address_with_mark = new_data.ICAO_address_with_mark;//第25位为0，表示该位置存储的是报文
				  data_save[matching_address].position.CPR_code[0] = new_data.position.CPR_code[0];
				  data_save[matching_address].position.CPR_code[1] = new_data.position.CPR_code[1];
				  data_save[matching_address].position.CPR_code[2] = new_data.position.CPR_code[2];
				  data_save[matching_address].position.CPR_code[3] = 0;//数组的这个位置可以忽略不管，避免出错，建议清0
			  }
		   }
		   else//data_save_type不为0x1000000，则需进行全局解算 
		   {
			  if((new_data.time - data_save[matching_address].time) <= GLOBAL_TIME_THRESHOLD)//全局解算时间间隔，飞机速度按照1224公里（1马赫）,5.1km的飞行距离，意味着15秒时间，
			                                                              //但取15秒为全局解算的门限时间过于苛刻，实际场景中，90%的飞机"地速"都不大于1000公里，可以放宽到18秒时间门限
			  
			  {
				  if (data_save[matching_address].position.CPR_code[2] != new_data.position.CPR_code[2])//比较当前报文和已存储的报文是否是奇偶编码匹配
				  {
						Calculation_ok=lat_lon_global_calculate(&new_data, &data_save[matching_address]);//调用全局解算函数，解算结果存储到data_save[matching_address]中，同时airplane_location结构体全局变量上报								
				  		decode_altitude();//zhangfulong add
				  }
				  else
				  {
						Calculation_ok=2;//表示奇偶编码不满足全局解算要求，返回值设定为2
						//如果奇偶编码不匹配，则不能全局解算要求，同时ICAO是匹配的，则只更新数据库中的报文信息，将当前的报文信息存储进去，在data_save[matching_address]位置
						data_save[matching_address].time = new_data.time;
						data_save[matching_address].ICAO_address_with_mark = new_data.ICAO_address_with_mark;//第25为0，表示该位置存储的是ㄎ�
						data_save[matching_address].position.CPR_code[0] = new_data.position.CPR_code[0];
						data_save[matching_address].position.CPR_code[1] = new_data.position.CPR_code[1];
						data_save[matching_address].position.CPR_code[2] = new_data.position.CPR_code[2];
						data_save[matching_address].position.CPR_code[3] = 0;//数组的这个位置可以忽略不管，避免出错，建议清0
				  }
			  }
			  else
			  {   
				   Calculation_ok=2;//表示时间间隔不满足全局解算要求，返回值设定为2
				   //如果差值时间不满足全局解算要求，同时ICAO是匹配的，则只更新数据库中的报文信息，将当前的报男畔⒋娲⒔ィ赿ata_save[matching_address]位置
				  	data_save[matching_address].time = new_data.time;
					data_save[matching_address].ICAO_address_with_mark = new_data.ICAO_address_with_mark;//第25为0，表示该位置存储的是报文
					data_save[matching_address].position.CPR_code[0] = new_data.position.CPR_code[0];
			    	data_save[matching_address].position.CPR_code[1] = new_data.position.CPR_code[1];
					data_save[matching_address].position.CPR_code[2] = new_data.position.CPR_code[2];
					data_save[matching_address].position.CPR_code[3] = 0;//数组的这个位置可以忽略还埽苊獬龃恚ㄒ榍�0
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
	   return(Calculation_ok);//ICAO匹配成功的情况下，即(ICAO_match==1)条件满足，将解算的结果返回，返刂凳�2或者3		 
	 }
   if(ICAO_match==0)//在遍历数据库后ICAO匹配不成功则认为是一架新飞机，需进行报文数据的保存；由于数据库大小是固定的，需要制定一套更新数据库的规则
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
   	   if(time_save_message==0)//time_save_message为0，意味着还有存储空间未放置任何报文，所以先把新的报文放入，不用做时间对比
	   {
	     data_save[address_message].time = new_data.time;
	     data_save[address_message].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		 data_save[address_message].position.CPR_code[0] = new_data.position.CPR_code[0];
		 data_save[address_message].position.CPR_code[1] = new_data.position.CPR_code[1];
		 data_save[address_message].position.CPR_code[2] = new_data.position.CPR_code[2];
		 data_save[address_message].position.CPR_code[3] = 0;  
		 return(2);  
	   }
	   else//time_save_message不为0，意味着存储空间已经全部放置报文或经纬，需做时间对比
	   {
	   	// Calculation_ok=2;
	   	 //此逻辑的基础是报文时间有效时间18秒，经纬度的生存周期480秒，所以可能构成四种情况
		 //1.当前报文时刻与最早存储报文时刻比较大于18秒，同时与最早解算出的经纬度信息时刻比较大于480秒，此时更新两者均可，选择更新报文的存储
		 //2.当前报文时刻与最早存储报文时刻比较大于18秒，同时与最早解算出的经纬度信息时刻比较小于等于480秒，此时更新报文的存储
		 //3.当前报文时刻与最早存储报文时刻比较小于等于18秒，同时与最早解算出的经纬度信息时刻比较大于480秒，此时更新经纬信息的存储
	   	 //3.当前报文时刻与最早存储报文时刻比较小于等于18秒，同时与最早解算出的经纬度信息时刻冉闲∮诘扔�480秒，此时更新经纬度信息的存储 
	   	 
	   	 //初始状态：由于时间采用32位的秒计数，时间起点UTC时2009年1月1日，所以初始状态下，数据库里面时刻全0，(续下行)
	   	 //（接上一行）默认为全部是"无效"的报文信息，只要收到有效报文，时间差值远远大于18，满足第2种情况；即使时间差小于等于18，满足第4.1种情况
	   	 //工作过程中，如果出现极端情况，即数据库全部是报文，一定会满足第2或第4.1种情况
	   	 //工作过程中，如果出现极端情况，即数据库全部是经纬度信息，一定会满足第3或第4.2种情况 	 
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
		   &&((new_data.time-time_save_position)<=LOCAL_TIME_THRESHOLD))//有可能new_data.time-time_save_position为0，即数据库中全部存储是报文
	   	   {
	   	     data_save[address_message].time = new_data.time;
		     data_save[address_message].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		     data_save[address_message].position.CPR_code[0] = new_data.position.CPR_code[0];
		     data_save[address_message].position.CPR_code[1] = new_data.position.CPR_code[1];
		     data_save[address_message].position.CPR_code[2] = new_data.position.CPR_code[2];
			 data_save[address_message].position.CPR_code[3] = 0;
	   	   }
	   	 
	   	   if(((new_data.time-time_save_message)<=GLOBAL_TIME_THRESHOLD)
	   	   &&((new_data.time-time_save_position)>LOCAL_TIME_THRESHOLD))//有可能new_data.time-time_save_message为0，即数据库中全部存储是经纬度信息
	   	   {
	   	     data_save[address_position].time = new_data.time;
		     data_save[address_position].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		     data_save[address_position].position.CPR_code[0] = new_data.position.CPR_code[0];
		     data_save[address_position].position.CPR_code[1] = new_data.position.CPR_code[1];
		     data_save[address_position].position.CPR_code[2] = new_data.position.CPR_code[2];
	         data_save[address_position].position.CPR_code[3] = 0;
	   	   }

	       if(((new_data.time-time_save_message)<=GLOBAL_TIME_THRESHOLD)
	       &&((new_data.time-time_save_position)<=LOCAL_TIME_THRESHOLD))//两个差值至少有一个不为0
	   	   {
	   	      if(((new_data.time-time_save_message)!=0)&&((new_data.time-time_save_position)==0))//数据库全部都是报文，没有位置信息，只能更新时间最早的报文存储
			  {
			     data_save[address_message].time = new_data.time;
		         data_save[address_message].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		         data_save[address_message].position.CPR_code[0] = new_data.position.CPR_code[0];
		         data_save[address_message].position.CPR_code[1] = new_data.position.CPR_code[1];
		         data_save[address_message].position.CPR_code[2] = new_data.position.CPR_code[2];
				 data_save[address_message].position.CPR_code[3] = 0;
	          }
	          if(((new_data.time-time_save_message)==0)&&((new_data.time-time_save_position)!=0))//数据库全部都是位置信息，没有报文，只能更新时间最早的位置存储
	   	      {
	   	         data_save[address_position].time = new_data.time;
		         data_save[address_position].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		         data_save[address_position].position.CPR_code[0] = new_data.position.CPR_code[0];
		         data_save[address_position].position.CPR_code[1] = new_data.position.CPR_code[1];
		         data_save[address_position].position.CPR_code[2] = new_data.position.CPR_code[2];
			     data_save[address_position].position.CPR_code[3] = 0;
			  }
			  if(((new_data.time-time_save_message)!=0)&&((new_data.time-time_save_position)!=0))//数据库中最早的报文和位置时间都在门限内，选择更新时间最早的位置存储
	          {
	   	         data_save[address_position].time = new_data.time;
		         data_save[address_position].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		         data_save[address_position].position.CPR_code[0] = new_data.position.CPR_code[0];
		         data_save[address_position].position.CPR_code[1] = new_data.position.CPR_code[1];
		         data_save[address_position].position.CPR_code[2] = new_data.position.CPR_code[2];
			     data_save[address_position].position.CPR_code[3] = 0;
			  }
			  //该情况在程序执行过程中，如果所有输入量全部正确的情况下，是不会出现的，写在这里的目的只是为了避免出错。（对应无时间信息的情况）
			  if(((new_data.time-time_save_message)==0)&&((new_data.time-time_save_position)==0))////情况4.4，理论上不会出现
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
    		// return(Calculation_ok);//ICAO匹配成功的情况下，即(ICAO_match==0)条件满足，只能返回2	
       }////time_save_message为0的else
   }
}   	 
//*************************************************************************************************************************************************************
//*                                                                                                                                                           *
//*  以下为解速度信息涉及到的函数：   decode_velocity()                                                                     
//*                                                                                                                                                           *
//*************************************************************************************************************************************************************
#if 1
unsigned short decode_velocity() 
{
  
  float velocity_polar_coordinates=0;
  //temp_code=velocity_code;
  airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address & 0xffffff;//高8位清0，用于速度状态信息存储
  if((velocity_subtype>=1)&&(velocity_subtype<=4))
  {
       //------------------------1---------------------------- 
      if(velocity_subtype==1)//地速非超音速模式
      {
	       airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x1000000;/*airplane_velocity_three.ICAO_address为32bit的无符号整型，低24存ICAO，第27，26，25位存速度状态，为001*/
	       if(velocity_code.bit47_56==0)
	       {
		         airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x40000000;//airplane_velocity.ICAO_address为32bit的无符号整型，低24存ICAO，
                                                                                // 东西方向速度无效，1开始编号的第31位(0开始编号的第30位)设置为1
				 airplane_velocity_three.E_W_velocity=0; 
		   }
	       else
	       {
	            if(velocity_code.bit46==0)
	 		    {
	 		        if(velocity_code.bit47_56<1023)
			              airplane_velocity_three.E_W_velocity=(int)(1.852*(velocity_code.bit47_56-1)*10000);//正数值，表示向东,速度KM/H,精度为1E-4
				    else
				          airplane_velocity_three.E_W_velocity=18918180;                       //为1023时，特殊处理，正数值，表示向东,速度KM/H,精度为1E-4
		        }  
		        else
		        {
		            if(velocity_code.bit47_56<1023)
				         airplane_velocity_three.E_W_velocity=(int)(-1.852*(velocity_code.bit47_56-1)*10000);//负数值，表示向西,速度KM/H，精度为1E-4
		            else
					     airplane_velocity_three.E_W_velocity=-18918180;                      //为1023时，特殊处理，负数值，表示向西,速度KM/H,精度为1E-4		     
		        }
		  }
		  if(velocity_code.bit58_67==0)
		  {
		           airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x20000000;//airplane_velocity.ICAO_address为32bit的无符号整型，低24存ICAO，
			                                                                                // 南北方向速度无效，1开始编号的第30位(0开始编号的第29位)设置为1
				   airplane_velocity_three.N_S_velocity=0;  
		  }
	      else
		  {
				   if(velocity_code.bit57==0) 
				   {
	                   if(velocity_code.bit58_67<1023)         
	                       airplane_velocity_three.N_S_velocity=(int)(1.852*(velocity_code.bit58_67-1)*10000);//正数值，表示向北，速度KM/H,精度为1E-4
	                   else
	                       airplane_velocity_three.N_S_velocity=18918180;                       //为1023时，特殊处理，正数值，表示向北,速度KM/H,精度为1E-4           
		           }
		           else
	               {
	                   if(velocity_code.bit58_67<1023) 
	                       airplane_velocity_three.N_S_velocity=(int)(-1.852*(velocity_code.bit58_67-1)*10000);//负数值，表示向南，速度KM/H,精度为1E-4
	                   else
	                       airplane_velocity_three.N_S_velocity=-18918180;                       //为1023时，特殊处理，负数值，表示向南,速度KM/H,精度为1E-4  
	               }  
	      }
          if(velocity_code.bit70_78==0)
          {
                   airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x10000000;//airplane_velocity_three.ICAO_address为32bit的无符号整型，低24存ICAO，
			                                                                               // 垂直方向速度无效，1开始编号的第29位(0开始编号的第28位)设置为1
				   airplane_velocity_three.VERT_velocity=0; 
          }
          else
          {
                  if(velocity_code.bit69==0)
	              {
	                   if(velocity_code.bit70_78<511)   
	                       airplane_velocity_three.VERT_velocity=(int)(0.3048*(velocity_code.bit70_78-1)*64*10000);   //正数值，表示向上，速度米/分钟,精度为1E-4
				       else
				           airplane_velocity_three.VERT_velocity=99389184;                          //为511时，特殊处理,正数值，表示向上，速度米/分钟,精度为1E-4
	              }
	              else 
			      {
			           if(velocity_code.bit70_78<511) 
			               airplane_velocity_three.VERT_velocity=(int)(-0.3048*(velocity_code.bit70_78-1)*64*10000);  //负数值，表示向下，速度米/分钟,精度为1E-4
	                   else
				           airplane_velocity_three.VERT_velocity=-99389184;                          //为511时，特殊处,负数值，表示向下，速度米/分钟,精度为1E-4
                  }
           }  
         return(5);//地速非超音速模式三维速度，返回值5
        }
		 //------------------------2---------------------------- 
        if(velocity_subtype==2)////地速超音速模式
	    {
    	       airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x2000000;/* airplane_velocity_three.ICAO_address为32bit的无符号整型，低24存ICAO，
		                                                                                第27，26，25位存速度状态，为010*/
		       if(velocity_code.bit47_56==0)
		       {
		           airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x40000000;//airplane_velocity.ICAO_address为32bit的无符号整型，低24存ICAO，
			                                                                               // 东西方向速度无效，1开始编号的第31位(0开始编号的第30位)设置为1
				   airplane_velocity_three.E_W_velocity=0; 
			   }
		       else
			   {
			       if(velocity_code.bit46==0)
			       {
			           if(velocity_code.bit47_56<1023)
			               airplane_velocity_three.E_W_velocity=(int)(7.408*(velocity_code.bit47_56-1)*10000);//正数值，表示向东,速度KM/H,精度为1E-4
					   else
					       airplane_velocity_three.E_W_velocity=75672720;                       //为1023时，特殊处理，正数值，表示向东,速度KM/H,精度为1E-4
		           }
		           else
				   {
				       if(velocity_code.bit47_56<1023)
				           airplane_velocity_three.E_W_velocity=(int)(-7.408*(velocity_code.bit47_56-1)*10000);//负数值，表示向西,速度KM/H,精度为1E-4
		               else
					       airplane_velocity_three.E_W_velocity=-75672720;                      //为1023时，特殊处理，负数值，表示向西,速度KM/H,精度为1E-4		     
			       }
		       }
		       if(velocity_code.bit58_67==0)
		       {
		           airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x20000000;//airplane_velocity.ICAO_address为32bit的无符号整型，低24存ICAO，
			                                                                               // 南北方向速度无效，1开始编号的第30位(0开始编号的第29位)设置为1
				   airplane_velocity_three.N_S_velocity=0; 
			   }
	           else
			   {
				   if(velocity_code.bit57==0) 
                   {
	                   if(velocity_code.bit58_67<1023)         
	                     airplane_velocity_three.N_S_velocity=(int)(7.408*(velocity_code.bit58_67-1)*10000);//正数值，表示向北，速度KM/H,精度为1E-4
	                   else
	                     airplane_velocity_three.N_S_velocity=75672720;                       //为1023时，特殊处理，正数值，表示向北,速度KM/H,精度为1E-4           
		           }
		           else
	               {
	                   if(velocity_code.bit58_67<1023) 
	                     airplane_velocity_three.N_S_velocity=(int)(-7.408*(velocity_code.bit58_67-1)*10000);//负数值，表示向南，速度KM/H,精度为1E-4
	                   else
	                     airplane_velocity_three.N_S_velocity=-75672720;                       //为1023时，特殊处理，负数值，表示向南,速度KM/H,精度为1E-4  
	               }
               }
               if(velocity_code.bit70_78==0)
               {
                    airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x10000000;//airplane_velocity.ICAO_address为32bit的无符号整型，低24存ICAO，
			                                                                                // 垂直方向速度无效，1开始编号的第29位(0开始编号的第28位)设置为1
				    airplane_velocity_three.VERT_velocity=0; 
               }
               else
			   {
                   if(velocity_code.bit69==0)
                   { 
	                   if(velocity_code.bit70_78<511)   
	                       airplane_velocity_three.VERT_velocity=(int)(0.3048*(velocity_code.bit70_78-1)*64*10000);   //正数值，表示向上，速度米/分钟,精度为1E-4  
				       else
				           airplane_velocity_three.VERT_velocity=99389184;                          //为511时，特殊处理,正数值，表示向上，速度米/分钟,精度为1E-4  
	               }
	               else 
			       {
			           if(velocity_code.bit70_78<511) 
			               airplane_velocity_three.VERT_velocity=(int)(-0.3048*(velocity_code.bit70_78-1)*64*10000);  //负数值，表示向下，速度米/分钟,精度为1E-4  
	                   else
				           airplane_velocity_three.VERT_velocity=-99389184;                          //为511时，特殊处理,负数值，表示向下，速度米/分钟,精度为1E-4  
                   }
                }
                return(5);//地速超音速模式,三维速度，返回值5
        }
		 //------------------------3---------------------------- 
        if(velocity_subtype==3)//空速非超音速模式
        {
	          //airplane_velocity_two.ICAO_address=airplane_velocity_two.ICAO_address|0x3000000;
		      airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x3000000;         //第27，26，25位存速度状态，为011*/
              if((velocity_code.bit46==0)||(velocity_code.bit58_67==0))//平面速度方向未可知或者平面速度未可知
              {
		           airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x60000000;//airplane_velocity.ICAO_address为32bit的无符号整型，低24存ICAO，
			                                                                               // 平面速度方向未可知，1开始编号的第30，31位(0开始编号的29，30位)设置为1
				   airplane_velocity_three.N_S_velocity=0;
				   airplane_velocity_three.E_W_velocity=0; 
			  }
              else
			  {
                  if(velocity_code.bit58_67<1023)
			           velocity_polar_coordinates=1.852*(velocity_code.bit58_67-1);
                  else
			           velocity_polar_coordinates=1891.818;

				  airplane_velocity_three.E_W_velocity=(int)(velocity_polar_coordinates*sin(0.3515625*velocity_code.bit47_56*PI/180.0)*10000);//正北方向为0度，顺时针旋转，正值为东，负值为西，精度为1E-4  
                  airplane_velocity_three.N_S_velocity=(int)(velocity_polar_coordinates*cos(0.3515625*velocity_code.bit47_56*PI/180.0)*10000);//正北方向为0度，顺时针旋转，正值为北，负值为南，精度为1E-4 
			  }
              if(velocity_code.bit70_78==0)
              {
                   airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x10000000;//airplane_velocity.ICAO_address为32bit的无符号整型，低24存ICAO，
			                                                                               // 垂直方向速度无效，1开始编号的第29位(0开始编号的第28位)设置为1
				   airplane_velocity_three.VERT_velocity=0; 
              }
              else
			  {
                   if(velocity_code.bit69==0)
	                   if(velocity_code.bit70_78<511)   
	                     airplane_velocity_three.VERT_velocity=(int)(0.3048*(velocity_code.bit70_78-1)*64*10000);   //正数值，表示向上，速度米/分钟,精度为1E-4 
				       else
				         airplane_velocity_three.VERT_velocity=99389184;                          //为511保厥獯?正数值，表示向上，速度米/分钟,精度为1E-4 
	               else 
			           if(velocity_code.bit70_78<511) 
			             airplane_velocity_three.VERT_velocity=(int)(-0.3048*(velocity_code.bit70_78-1)*64*10000);  //负数值，表示向下，速度米/分钟,精度为1E-4 
	                   else
				         airplane_velocity_three.VERT_velocity=-99389184;                          //为511时，特殊处理,负数值，表示向下，速度米/分钟,精度为1E-4 
              }
	          return(5);//空速非超音速模式,平面速度加垂直速度已转换为3维速度，返回值5
	    }
	    //------------------------4---------------------------- 
	    if(velocity_subtype==4)//空速超音速模式
        {
	          airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x4000000;/* airplane_velocity.ICAO_address为32bit的无符号整型，低24存ICAO，
		                                                                                第27，26，25位存速度状态，为100*/
            
              if((velocity_code.bit46==0)||(velocity_code.bit58_67==0))//平面速度方向未可知或者平面速度未可知
              {
		           airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x60000000;//airplane_velocity.ICAO_address为32bit的无符号整型，低24存ICAO，
			                                                                               // 平面速方向未可知，1开始编号的第30，31位(0开始编号的29，30位)设置为1
				   airplane_velocity_three.N_S_velocity=0;
				   airplane_velocity_three.E_W_velocity=0; 
			  }
              else
			  {
                  if(velocity_code.bit58_67<1023)
			           velocity_polar_coordinates=7.408*(velocity_code.bit58_67-1);
                  else
			           velocity_polar_coordinates=7567.272;

				   airplane_velocity_three.E_W_velocity=(int)(velocity_polar_coordinates*sin(0.3515625*velocity_code.bit47_56*PI/180.0)*10000);//正北反向为0度，顺时针旋转，正值为东，负值为西,精度为1E-4 
                   airplane_velocity_three.N_S_velocity=(int)(velocity_polar_coordinates*cos(0.3515625*velocity_code.bit47_56*PI/180.0)*10000);//正北反向为0度，顺时针旋转，正值为北，负值为南,精度为1E-4 
			  }
              if(velocity_code.bit70_78==0)
              {
                   airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x10000000;//airplane_velocity.ICAO_address为32bit的无符号整型，低24存ICAO，
			                                                                               // 垂直方向速度无效，1开始编号的第29位(0开始编号的第28位)设置为1
				   airplane_velocity_three.VERT_velocity=0; 
              }
              else
		      {
                   if(velocity_code.bit69==0)
	               {
	                   if(velocity_code.bit70_78<511)   
	                        airplane_velocity_three.VERT_velocity=(int)(0.3048*(velocity_code.bit70_78-1)*64*10000);   //正数值，表鞠蛏希俣让�/分钟,精度为1E-4 
				       else
				            airplane_velocity_three.VERT_velocity=99389184;                          //为511时，特殊处理,正数值，表示向上，速度米/分钟,精度为1E-4 
	               }
	               else 
			       {
			           if(velocity_code.bit70_78<511) 
			               airplane_velocity_three.VERT_velocity=(int)(-0.3048*(velocity_code.bit70_78-1)*64*10000);  //负数值，表示向下，速度米/分钟,精度为1E-4 
	                   else
				           airplane_velocity_three.VERT_velocity=-99389184; 
				   }                                 //为511时，特殊处理,负数值，表示向下，速度米/分钟,精度为1E-4 
                }
	            return(5);//空速超音速模式,平面速度加垂直速度已转换为3维速度，返回值5
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
//*  以下为解高度信息的函数： decode_altitude()   
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
     airplane_location.ICAO_address=airplane_location.ICAO_address|0x1000000;//第24位为0,代表高度有效,为1代表高度数据无效。
     return(0);     
     }
   
    if(altitude_code_GNSS==0)
     {
      altitude_code=altitude_code_BARO;	 
     }
   
    if(altitude_code_BARO==0)
      {
      altitude_code=altitude_code_GNSS;
	  airplane_location.ICAO_address=airplane_location.ICAO_address|0x2000000;//第25位为0,代表当前是气压高度,为1，代表当前是GNSS高度。
     }

   if((altitude_code&0x10)==0x10)
    {

      altitude_code_temp_low=altitude_code&0xF;//高度编码低4BIT
	  altitude_code_temp_high=(altitude_code>>1)&0x7F0;//高度编码高7BIT

	  altitude_code_temp=altitude_code_temp_low|altitude_code_temp_high;////高度编码11bit
      altitude_foot=altitude_code_temp*25-1000;//公式计算高度值，单位英尺
      airplane_location.altitude=(3048*altitude_foot);//换算为米为单位，并且放大10000倍
    }
   else
    {
       airplane_location.ICAO_address=airplane_location.ICAO_address|0x4000000;//第26位为0,代表当前高度精度为±3.81米（±12.5英尺）,为1代表当前高度精度为±15.24米（±50英尺）

      memset(altitude_code_DAB, 0, sizeof(altitude_code_DAB));

      altitude_code_DAB[0]=(altitude_code>>2)&0x1;//移动D2
	  altitude_code_DAB[1]=(altitude_code>>0)&0x1;//移动D4
      altitude_code_DAB[1]=(altitude_code_DAB[0]^altitude_code_DAB[1])&0x1;//格雷码译码
	  altitude_code_DAB[2]=(altitude_code>>10)&0x1;//移动A1
      altitude_code_DAB[2]=(altitude_code_DAB[1]^altitude_code_DAB[2])&0x1;//格雷码译码
      altitude_code_DAB[3]=(altitude_code>>8)&0x1;//移动A2
      altitude_code_DAB[3]=(altitude_code_DAB[2]^altitude_code_DAB[3])&0x1;//格雷码译码
      altitude_code_DAB[4]=((altitude_code>>6)&0x08);//移动A4
      altitude_code_DAB[4]=(altitude_code_DAB[3]^altitude_code_DAB[4])&0x1;//格雷码译码
      altitude_code_DAB[5]=(altitude_code>>5)&0x1;//移动B1
      altitude_code_DAB[5]=(altitude_code_DAB[4]^altitude_code_DAB[5])&0x1;//格雷码译码
      altitude_code_DAB[6]=(altitude_code>>3)&0x1;//移动B2
      altitude_code_DAB[6]=(altitude_code_DAB[5]^altitude_code_DAB[6])&0x1;//格雷码译码
	  altitude_code_DAB[7]=(altitude_code>>1)&0x1;//移动B4
      altitude_code_DAB[7]=(altitude_code_DAB[6]^altitude_code_DAB[7])&0x1;//格雷码译码

      altitude_decode_DAB=0;
      
      for(i=0;i<8;i++) 
	      altitude_decode_DAB=altitude_decode_DAB|(altitude_code_DAB[i]<<(7-i));//得到格雷码译码后对应的二进制数值

      altitude_code_temp_C=0;
      altitude_code_temp_C=((altitude_code>>9)&0x04)|altitude_code_temp_C;//移动C1
      altitude_code_temp_C=((altitude_code>>8)&0x02)|altitude_code_temp_C;//移动C2
	  altitude_code_temp_C=((altitude_code>>7)&0x01)|altitude_code_temp_C;//移动C4
      
       if((altitude_decode_DAB%2)==0)//根据DAB格雷码译码后数值奇偶进行C的五周期循环码译码
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
	      airplane_location.altitude=3048*altitude_foot;//换算为米为单位，并且放大10000倍
	     }
	   else
	     {
	     airplane_location.altitude=0;
	     airplane_location.ICAO_address=airplane_location.ICAO_address|0x1000000;//第24位为0,代表高度有效,为1代表高度数据无效。
                                                                                 //altitude_decode_C只能是0-4中的数，出现其他值，高度编码错误，高度无效
	     }
      
	      
    }
	
}

