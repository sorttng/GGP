/*******************************************/
#include <math.h>
//±ØĞë°üº¬
#define NZ 15
#define PI 3.141592653589793
#define LOCAL_TIME_THRESHOLD    (480*10000)
//¾­Î³ĞÅÏ¢"¾Ö²¿½âËã"Ê±¼äÃÅÏŞ£¬¶ÔÓ¦µ¥Î»Îª"Ãë"
//#define GLOBAL_TIME_THRESHOLD   18 
#define GLOBAL_TIME_THRESHOLD   (15*10000) 
//¾­Î³ĞÅÏ¢"È«¾Ö½âËã"Ê±¼äÃÅÏŞ£¬¶ÔÓ¦µ¥Î»Îª"Ãë"
//3.14159265358979323846
//¶¨ÒåÔÚÖ÷µ÷º¯ÊıÖĞµÄ½á¹¹Ìå
typedef struct FPGA_YC  //FPGA-DSP Ò£²â
{
	unsigned int UTCtime;    //4
    unsigned short int temperature; //2
    unsigned short int YC_33;  //2
	unsigned short int YC_55;//2
//--
    unsigned char YK_cnt;//1
    unsigned char DSP_RST_cnt;//1
	unsigned char FPGA_RST_cnt;//1
	unsigned char FPGA_stat;//1-"2bFPGAÅäÖÃ/2b×´Ì¬/2b¹¤×÷Ä£Ê½/2b00"
    unsigned char reserve[2];  //
}FPGA_YC;
extern FPGA_YC s_FPGA_YC;  //16×Ö½ÚÒ£²â
//*************************************************************************************************************************
//¶¨ÒåÔÚÖ÷µ÷º¯ÊıÖĞµÄ½á¹¹Ìå,ÓÃÓÚÈ«¾Ö¶¨Òå½á¹¹Ìå±äÁ¿µÄÉùÃ÷
struct speed_struct_three
{
  unsigned int ICAO_address;
  unsigned int time;
  int  N_S_velocity;      
  int  E_W_velocity;  	  
  int  VERT_velocity;  
};
extern struct speed_struct_three airplane_velocity_three;//ÓÃÓÚËÙ¶ÈÉÏ±¨
/*********************************/
extern unsigned char velocity_subtype;//ËÙ¶È½âËãÈ«¾Ö±äÁ¿£¬ÓÃÓÚ´«µİĞÅÏ¢Ê¹ÓÃ£¬ÓÃ»§²»ÓÃ¹ØĞÄ
struct speed_code
{
  unsigned char bit46;
  unsigned short bit47_56;
  unsigned char bit57;
  unsigned short bit58_67;
  unsigned char bit69;
  unsigned short bit70_78;
};
extern struct speed_code velocity_code; //ËÙ¶È½âËãÈ«¾Ö½á¹¹Ìå±äÁ¿£¬ÓÃÓÚ´«µİĞÅÏ¢Ê¹ÓÃ£¬ÓÃ»§²»ÓÃ¹ØĞÄ
//*************************************************************************************************************************
struct data_struct
{
  unsigned int ICAO_address_with_mark;//»»ÁËÎ»ÖÃ 190314 (ICAO_adress_with_mark)ºÍ(time)
  unsigned int time;
  union
  {
   double coordinate[2]; //×ø±ê
   unsigned int CPR_code[4];
  }position;
};
extern struct data_struct data_save[1000],new_data;//´Ë½á¹¹ÌåÊı×éºÍ½á¹¹Ìå±äÁ¿È«¾ÖÊ¹ÓÃ£¬data_save[1000],new_data;
/*********************************/
struct location_struct
{
  unsigned int ICAO_address;
  unsigned int time;
  int coordinate[2];//ÕıÊ½ÉÏ±¨Ê±ºòÊ¹ÓÃintĞÍ£¬ÕıÊ½ÉÏ±¨Ê±ºòÊ¹ÓÃint(1E-7Îªµ¥Î»)¡£
  int altitude;
};
extern struct location_struct airplane_location;//´Ë½á¹¹ÌåÊı×éºÍ½á¹¹Ìå±äÁ¿È«¾ÖÊ¹ÓÃ,airplane_location;
//¶¨ÒåÔÚÖ÷µ÷º¯ÊıÖĞµÄ½á¹¹Ìå,ÓÃÓÚÈ«¾Ö¶¨Òå½á¹¹Ìå±äÁ¿µÄÉùÃ÷
struct message_struct
{
  unsigned int time;
  unsigned char data_demodulator[11];  //ADSBµÄÔ­Ê¼88bitÏûÏ¢±¨ÎÄ£¬11×Ö½Ú							
};
extern struct message_struct ADSB_message;//ÉÏ±¨µÄADSBÔ­Ê¼±¨ÎÄ,¹Ì¶¨¸ñÊ½£¬²»Çø·Ö±¨ÎÄÖÖÀà£¬ÏòÖ÷º¯Êı´«µİ²ÎÊı
/*********************************/
#define pulse_BUFLEN 150
extern unsigned short   pulse_amp[112];
extern unsigned int pulse_time[pulse_BUFLEN];  //190521
extern unsigned int pulse_rd ;
unsigned short  DF_message=0;//ÓÃÓÚ±¨ÎÄÌáÈ¡×éÖ¯Îª½âËã¾­Î³¶ÈĞèÒªµÄ¸ñÊ½Ê¹ÓÃ DF¿ÉÎª17,18,19 
unsigned short  CF_AF_message=0;//DF=18,19Ê±ºòÊ¹ÓÃ£¬ÔÚDFÎª18ºÍ19µÄÊ±ºòµÚ6µ½8Î»±ØĞëÎª0£¬¼´DF18Ê±CFÎª0»òDF19Ê±ÎªAFÎª0	
unsigned short  TYPE_Code_message=0;//ÓÃÓÚÅĞ¶Ï¸Ã±¨ÎÄÊÇ·ñÊÇ¿ÕÖĞÎ»ÖÃ±¨ÎÄ
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
 //·µ»ØÖµ2£¬Ö¸Ê¾¸ÃÖ¡Êı¾İ´¦ÀíÕı³££¬²¢ÇÒ¸ÃÖ¡ÊÇÎ»ÖÃ±¨ÎÄ£¬¸üĞÂADSB_messageÊı×éÓÃÓÚÉÏ´«±¨ÎÄ£¬Í¬Ê±¸üĞÂÁËnew_data½á¹¹Ìå±äÁ¿
 //·µ»ØÖµ1£¬Ö¸Ê¾¸ÃÖ¡Êı¾İ´¦ÀíÕı³££¬µ«¸ÃÖ¡²»ÊÇÎ»ÖÃ±¨ÎÄ£¬½ö¸üĞÂADSB_messageÊı×éÓÃÓÚÉÏ´«±¨ÎÄ£»
 //·µ»ØÖµ0£¬Ö¸Ê¾¸ÃÖ¡Êı¾İ²»Õı³£»òÊÇ½âµ÷ÖÆºóÊı¾İ²»ÕıÈ·, ADSB_messageÊı×é²»»á±»¸üĞÂ£»
//------------------------
 //·µ»ØÖµ2£¬Ö¸Ê¾¸ÃÖ¡´¦ÀíÊÇÊı¾İ±¨ÎÄ£¬µ«ÊÇ½âËã²»³É¹¦£¬½ö¸üĞÂÁËADSB_messageÊı×é£¬ÒÔ¼°Êı¾İ¿â
 //·µ»ØÖµ3£¬Ö¸Ê¾¸ÃÖ¡´¦ÀíÊÇÊı¾İ±¨ÎÄ£¬½âËã³É¹¦£¬¸üĞÂÁËADSB_messageÊı×é£¬airplane_location½á¹¹Ìå±äÁ¿ÒÔ¼°Êı¾İ¿â
/**********************************************/
unsigned int data_pro(unsigned short  * fifo)
{
     unsigned short  temp_message=0;//ÓÃÓÚ±¨ÎÄÆ´½ÓÎª11×Ö½ÚµÄÎŞ·ûºÅ×Ö·ûĞÍÊı¾İ
     unsigned int i,j;
     unsigned short  *  pulse_tmp;
      //´Ë¶Î´úÂë°´ÕÕÒªÇó´ò°üÉÏ±¨Êı¾İ,Í¨¹ıCRCĞ£Ñé£¬²¢ÇÒDFÎª17¼´¿ÉÒÔÉÏ´«Ô­Ê¼88bitµÄÊı¾İ°ü,Ìí¼ÓÊ±¼äĞÅÏ¢£¬¼ì²éÊÇ·ñÎªÎ»ÖÃ»òËÙ¶È±¨ÎÄ£¬ÊÇÔò½«Êı¾İ×¼±¸ºÃ
            //***************************************************************************************************  
     pulse_tmp=fifo; //´«µİÒ»ÏÂ
     DF_message=0;
     for(i=0;i<5;i++)  //Ç°5Î»ÊÇ0£¬»ù±¾¿ÉÄÜÊÇÈ«0
     {   
        DF_message=DF_message|(pulse_tmp[i]<<(4-i));//È¡DFÎ»
	 }
	  if(DF_message==17)
	  {
          // ¼ì²éÊÇ·ñÎªÊı¾İÎ»ÖÃ±¨ÎÄ£¬»òÕßËÙ¶È±¨ÎÄ£¬ÊÇÔò´ò°üÊı¾İ
          //*****************************************************************************************  
          TYPE_Code_message=0;
          for(i=0;i<5;i++)
          {
              TYPE_Code_message=TYPE_Code_message|(pulse_tmp[i+32]<<(4-i));
		  }
           //TYPEÀàĞÍ9-18»òÕß20-22±íÊ¾ÊÇÎ»ÖÃ±¨ÎÄ
          if(((TYPE_Code_message>=9)&&(TYPE_Code_message<=18))||((TYPE_Code_message>=20)&&(TYPE_Code_message<=22)))
          {   
			 memset(new_data, 0, sizeof(new_data));////ÓÉÓÚ½á¹¹Ìånew_dataÊÇÈ«¾ÖµÄ£¬¹Ê´Ë´¦ĞèÒªÇå0£»
 #if 1   
		//	new_data.time=	htonl(s_FPGA_YC.UTCtime);  //Ê±¼ä´¦Àí£¨1£©
            new_data.time=	htonl(pulse_time[pulse_rd]);  //190521
			
 #endif              
             //´Ë´¦ÊÇ×îĞÂµÄÎ»ÖÃ±¨ÎÄ¸üĞÂ£¬ new_data.ICAO_adress_with_markÖĞµÚ25Î»±êÖ¾Î»²»ÓÃÀí»á£¬´ËÊ±Ò»¶¨´æÔÚ½á¹¹ÌåÖĞµÄÊÇ±¨ÎÄÊı¾İ                    
             for(i=0;i<24;i++)
			 {
		        new_data.ICAO_address_with_mark=new_data.ICAO_address_with_mark|(pulse_tmp[i+8]<<(23-i));//°Ñ°´BIT´æ·ÅµÄ24Î»ICAOµØÖ·×ª»»³ÉÒ»¸ö24Î»Êı¾İ´æ·Å
             }
             //new_data.ICAO_adress_with_mark=new_data.ICAO_adress_with_mark|0x1000000;
             //new_data.ICAO_adress_with_markµÚ25Î»£¨±àºÅ24£©´æ·Å±êÖ¾Î»£¬±íÊ¾´æ·ÅµÄ¾­Î³¶È±àÂë
             for(i=0;i<17;i++)//½«¾­Î³¶È±àÂë´æ·ÅÔÚ½á¹¹ÌåÖĞµÄ¹²ÓÃÌåÖĞ
             {
                  new_data.position.CPR_code[0]=new_data.position.CPR_code[0]|(pulse_tmp[i+54]<<(16-i));//Î³¶È±àÂë
                  new_data.position.CPR_code[1]=new_data.position.CPR_code[1]|(pulse_tmp[i+71]<<(16-i));//¾­¶È±àÂë
             }
             new_data.position.CPR_code[2]=pulse_tmp[53];//·ÅÖÃÆæÅ¼±àÂë±êÖ¾Î»ÔÚ¹²ÓÃÌåÖĞ


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

			 return(2);//·µ»ØÖµ2£¬Ö¸Ê¾¸ÃÖ¡Êı¾İ´¦ÀíÕı³£,Ò»¶¨ÊÇÎ»ÖÃ±¨ÎÄ
           }  
           //TYPEÀàĞÍ19±íÊ¾ÊÇÎ»ÖÃ±¨ÎÄ //ËÙ¶È±¨ÎÄ£¿£¿
           if(TYPE_Code_message==19)
           {
		         memset(airplane_velocity_three, 0, sizeof(airplane_velocity_three));////ÓÉÓÚ½á¹¹Ìåairplane_velocityÊÇÈ«¾ÖµÄ£¬ĞèÇå0£»
				     
			     memset(velocity_code, 0, sizeof(velocity_code));////È«¾Ö±äÁ¿£¬ÓÃÓÚÏòËÙ¶È½âËãº¯Êı´«µİ²ÎÊı£¬ĞèÇå0£»
			     velocity_subtype=0;//È«¾Ö±äÁ¿£¬ÓÃÓÚÏòËÙ¶È½âËãº¯Êı´«µİ²ÎÊı£¬ĞèÇå0
#if 1   
				 airplane_velocity_three.time=	htonl(s_FPGA_YC.UTCtime);  //Ê±¼ä´¦Àí£¨2£©
#endif
                 for(i=0;i<24;i++)
			     {
			        airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|(pulse_tmp[i+8]<<(23-i));//°Ñ°´BIT´æ·ÅµÄ24Î»ICAOµØÖ·×ª»»³ÉÒ»¸ö24Î»Êı¾İ´æ·Å
                 }      
                 for(i=0;i<3;i++)  
                 {
                    velocity_subtype=velocity_subtype|(pulse_tmp[i+37]<<(2-i));//´æ´¢ËÙ¶ÈÖ¡µÄ×ÓÀàĞÍ
				 } 	  
			   //ÌáÈ¡ËÙ¶È±àÂëµÄÂë×Ö£¬ÓÃÓÚºóĞø´¦Àí  
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
		         return(4);//·µ»ØÖµ4£¬Ö¸¸ÃÖ¡ı¾İ´¦ÀíÕı³£,Ò»¶¨ÊÇËÙ¶È±¨ÎÄ				   
		   }            
           return(1);//·µ»ØÖµ1£¬Ö¸Ê¾¸ÃÖ¡Êı¾İ´¦ÀíÕı³£,ÎªÎ»ÖÃºÍËÙ¶È±¨ÎÄÖ®ÍâµÄÆäËû±¨ÎÄ
	   } //  if(DF_message==17)
       else  //DF!=17
       {
          return(1);//·µ»ØÖµ1£¬Ö¸Ê¾¸ÃÖ¡Êı¾İ´«ÊäÕı³££¬´¦ÀíºóÄÜÍ¨¹ıĞ£Ñé£¬µ«ÊÇ²»ÊÇDF17µÄ¸ñÊ½
       }  

}
//*************************************************************************************************************************************************************
//*                                                                                                                                                           *
//*  ÒÔÏÂÎª½âÎ»ÖÃĞÅÏ¢Éæ¼°µ½µÄº¯Êı£ºÉæ¼°NL£¨£©£¬my_mod£¨£©£¬lat_lon_local_calculate£¨£©£¬lat_lon_global_calculate£¨£©  decode_position()5¸öº¯Êı                *                     
//*                                                                                                                                                           *
//*************************************************************************************************************************************************************
int NL(double  m)//½âËã¾­¶ÈµÄZONE·Ö¸îÊıÄ¿,²»Í¬Î³¶ÈÇé¿öÏÂ¾­¶ÈµÄ·Ö¸îıÁ¿ÊÇ²»Í¬µÄ£¬ĞèÒª¼ÆËã
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
double my_mod(double a,double b)//±ê×¼ÖĞCPRËã·¨¶¨ÒåµÄÇóÄ£ÔËËã¹æÔò
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
  mark=st->position.CPR_code[2];//´æµ±Ç°×îĞÂ±¨ÎÄµÄÆæÅ¼±àÂëÀàĞÍ

  ref_lon=st_ref->position.coordinate[1];//Ç°Ò»¸öÊ±¿Ì¾­¶È²Î¿¼
  ref_lat=st_ref->position.coordinate[0];//Ç°Ò»¸öÊ±¿ÌÎ³¶È²Î¿¼
 
  XZ=st->position.CPR_code[1];//µ±Ç°µÄ¾­¶È±àÂë
  YZ=st->position.CPR_code[0];//µ±Ç°µÄÎ³¶È±àÂë

  Dlat=360.0/((double)(4*NZ-mark));//ÓÉµ±Ç°µÄÆæÅ¼±àÂë·½Ê½£¬È·¶¨Î³¶È°´ÕÕ6»ò6.1·Ö¸î

  j=floor(ref_lat/Dlat)+floor(0.5+(my_mod(ref_lat,Dlat)/Dlat)-(YZ/pow(2,17)));//ÇóÎ³¶ÈË÷Òı
  Rlat=Dlat*(j+(YZ/pow(2,17)));//½âËãµ±Ç°Î³¶ÈµÄ¾ßÌåÖµ

  number_of_lon=NL(Rlat);//¸ù¾İÎ³¶ÈµÄÖµÇó¾­¶ÈË÷Òı

  if((number_of_lon-mark)>0)
    Dlon=360.0/((double)(number_of_lon-mark));
  else
    Dlon=360.0;

  m=floor(ref_lon/Dlon)+floor(0.5+(my_mod(ref_lon,Dlon)/Dlon)-(XZ/pow(2,17)));
  Rlon=Dlon*(m+(XZ/pow(2,17)));//µ±Ç°µÄ¾­¶È¾ßÌå½âËã

  //¸üĞÂÊı¾İ¿â
  st_ref->time=st->time;
  st_ref->ICAO_address_with_mark=st->ICAO_address_with_mark|0x1000000;
  st_ref->position.coordinate[0]=Rlat;//±£´æµ±Ç°½âËãµÄÎ³¶ÈÖµµ½Êı¾İ¿â
  st_ref->position.coordinate[1]=Rlon;//±£´æµ±Ç°½âËãµÄ¾­¶ÈÖµµ½Êı¾İ¿â
  
  //¸üĞÂÉÏ±¨Î»ÖÃĞÅÏ¢ÓÃµÄ½á¹¹Ìå±äÁ¿
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
  int  XZ_0,YZ_0,XZ_1,YZ_1,j,m;//jºÍmµÄ¶¨Òå·½Ê½À´×Ô±ê×¼£¬ÕûÊı£¬´Ó-59µ½58·¶Î§
  int  mark;
  double hh_tmp1;
  double hh_tmp2;
  int hh_tmp3;
  mark = st->position.CPR_code[2];//È¡µ±Ç°Ê±¿ÌÊÕµ½±¨ÎÄµÄÆæÅ¼±àÂëÀàĞÍ
  if(mark == 0)//µ±Ç°ÎªÅ¼±àÂëµÄÇé¿ö
  {
    XZ_0=st->position.CPR_code[1];
    YZ_0=st->position.CPR_code[0];
    XZ_1=st_ref->position.CPR_code[1];
    YZ_1=st_ref->position.CPR_code[0];
  }
  else//µ±Ç°ÎªÆæ±àÂëµÄÇé¿ö
  {
    XZ_0=st_ref->position.CPR_code[1];
    YZ_0=st_ref->position.CPR_code[0];
    XZ_1=st->position.CPR_code[1];
    YZ_1=st->position.CPR_code[0];
  }

  Dlat_0=360.0/((double)4*NZ-0);//Å¼±àÂëÇé¿öÎ³¶ÈµÄ·Ö¸î¼ä¸ô
  Dlat_1=360.0/((double)4*NZ-1);//Ææ±àÂëÇé¿öÎ³¶ÈµÄ·Ö¸î¼ä¸ô
 
  j=floor(0.5+(59*YZ_0-60*YZ_1)/pow(2,17));//POW·µ»ØÎªDOUBLEÀàĞÍ

  Rlat_0=Dlat_0*(my_mod(j,60-0)+YZ_0/pow(2,17));//Å¼±àÂëÇé¿öÎ³¶ÈµÄ½âËã½á¹û
  Rlat_1=Dlat_1*(my_mod(j,60-1)+YZ_1/pow(2,17));//Ææ±àÂëÇé¿öÎ³¶ÈµÄ½âËã½á¹û
  if((Rlat_0>90&&Rlat_0<270)||(Rlat_1>90&&Rlat_1<270))//¸ÃÇé¿ö²»¿ÉÄÜ³öÏÖ£¬±ÜÃâ³ö´í£¬¼Ó¸ÃÌõÓï¾ä
    return(2) ;//·µ»Ø2±íÊ¾½âËã²»³É¹¦
  
  if(Rlat_0 >= 270.0 && Rlat_0 <=360.0)//Å¼±àÂëÎ³¶ÈµÄ½âËã½á¹ûÍ³Ò»µ½ÄÏ±±Î³£¬±±Õı£¬ÄÏ¸º
    Rlat_0=my_mod(Rlat_0+180,360)-180.0;   
  
  if(Rlat_1 >= 270.0 && Rlat_1 <=360.0)//Ææ±àÂëÎ³¶ÈµÄ½âËã½á¹ûÍ³Ò»µ½ÄÏ±±Î³£¬±±Õı£¬ÄÏ¸º 
    Rlat_1=my_mod(Rlat_1+180,360)-180.0;
  

  number_of_lon_0=NL(Rlat_0);//¸ù¾İÅ¼±àÂëÎ³¶È£¬¼ÆËã¾­¶È·Ö¸îÊı
  number_of_lon_1=NL(Rlat_1);//¸ù¾İÆæ±àÂëÎ³¶È£¬¼ÆËã¾­¶È·Ö¸îÊı

   if(number_of_lon_0 != number_of_lon_1)//¸ù¾İÆæÅ¼±àÂëÎ³¶È¼ÆËã³öµÄ¾­¶È·Ö¸îÊıÄ¿²»ÏàÍ¬£¬ÔòÎŞ·¨½âËã¾­¶ÈĞÅÏ¢£¬·ÅÆú½âËã£¬±£´æµ±Ç°±¨ÎÄµ½Êı¾İ¿â
   {
      st_ref->time=st->time;
	  st_ref->position.CPR_code[0]=st->position.CPR_code[0];
      st_ref->position.CPR_code[1]=st->position.CPR_code[1];
      st_ref->position.CPR_code[2]=st->position.CPR_code[2];
	  st_ref->position.CPR_code[3]=st->position.CPR_code[3];
    
      return(2) ;//·µ»Ø2±íÊ¾½âËã²»³É¹¦
   }
   else
   {
     if(number_of_lon_0 > 1)//number_of_lon_0Óënumber_of_lon_1ÏàµÈ£¬number_of_lon_0´óÓÚ1£¬number_of_lon_1Ò²¾Í´óÓÚ1
       {   
		   n_i=number_of_lon_0;  //n_i,¼´Îª¾­¶È·Ö¸îÊıÄ¿
	       Dlon_0=360.0/n_i;     //Å¼±àÂëÇé¿ö¾­¶ÈµÄ·Ö¸î¼ä¸ô
	       Dlon_1=360.0/(n_i-1); //Ææ±àÂëÇé¿ö¾­¶ÈµÄ·Ö¸î¼ä¸ô
	       m=floor(0.5+(XZ_0*(n_i-1)-XZ_1*(n_i))/pow(2,17));
	       Rlon_0=Dlon_0*(my_mod(m,n_i-0)+XZ_0/pow(2,17));
		   Rlon_1=Dlon_1*(my_mod(m,n_i-1)+XZ_1/pow(2,17));
	      
	       if(Rlon_0 > 180.0 && Rlon_0 <= 360.0)//Å¼±àÂë¾­¶ÈµÄ½âËã½á¹ûÍ³Ò»µ½¶«Î÷¾­£¬¶«Õı£¬Î÷¸º	       
	         Rlon_0=my_mod(Rlon_0+180,360)-180;
	       
	       if(Rlon_1 > 180.0 && Rlon_1 <= 360.0)//Ææ±àÂë¾­¶ÈµÄ½âËã½á¹ûÍ³Ò»µ½¶«Î÷¾­£¬¶«Õı£¬Î÷¸º	   
	         Rlon_1=my_mod(Rlon_1+180,360)-180;	       
       }
       else
       {	       
		   Dlon_0=360.0;
	       Dlon_1=360.0;
		   Rlon_0=Dlon_0*(XZ_0/pow(2,17));
	       Rlon_1=Dlon_1*(XZ_1/pow(2,17));

		   if(Rlon_0 > 180.0 && Rlon_0 <= 360.0)//Å¼±àÂë¾­¶ÈµÄ½âËã½á¹ûÍ³Ò»µ½¶«Î÷¾­£¬¶«Õı£¬Î÷¸º
	       	  Rlon_0=my_mod(Rlon_0+180,360)-180;
	        
	       if(Rlon_1 > 180.0 && Rlon_1 <= 360.0)//Ææ±àÂë¾­¶ÈµÄ½âËã½á¹ûÍ³Ò»µ½¶«Î÷¾­£¬¶«Õı£¬Î÷¸º
	       	  Rlon_1=my_mod(Rlon_1+180,360)-180;
	        
	   }
    }
  
   if(mark==0)//¸ù¾İµ±Ç°ÊÕµ½±¨ÎÄµÄÆæÅ¼±àÂë×´Ì¬Öµ£¬¾ö¶¨±£´æºÍÉÏ±¨½á¹û
   {
       //¸üĞÂÊı¾İ¿â
       st_ref->time=st->time;//?????????????????????????????????????????????????????
       st_ref->ICAO_address_with_mark=st->ICAO_address_with_mark | 0x1000000;
       st_ref->position.coordinate[0]=Rlat_0;
       st_ref->position.coordinate[1]=Rlon_0;

	   //¸üĞÂÉÏ±¨Î»ÖÃĞÅÏ¢ÓÃµÄ½á¹¹Ìå±äÁ¿
       airplane_location.time=st->time;
       airplane_location.ICAO_address=st->ICAO_address_with_mark & 0x00ffffff;
       airplane_location.coordinate[1]=(int)(Rlat_0 * 10000000);
       airplane_location.coordinate[0]=(int)(Rlon_0 * 10000000);
	    return(3) ;//·µ»Ø3±íÊ¾½âËã³É¹¦
   }
   else
   {
       //¸üĞÂÊı¾İ¿â
       st_ref->time=st->time;
       st_ref->ICAO_address_with_mark=st->ICAO_address_with_mark | 0x1000000;
       st_ref->position.coordinate[0]=Rlat_1;
       st_ref->position.coordinate[1]=Rlon_1;

	   //¸üĞÂÉÏ±¨Î»ÖÃĞÅÏ¢ÓÃµÄ½á¹¹Ìå±äÁ¿
	   airplane_location.time=st->time;
       airplane_location.ICAO_address=st->ICAO_address_with_mark & 0x00ffffff;
       airplane_location.coordinate[1]=(int)(Rlat_1 * 10000000);
       airplane_location.coordinate[0]=(int)(Rlon_1 * 10000000);
	   return(3) ;//·µ»Ø3±íÊ¾½âËã³É¹¦
   }

}
/*********************************/
//µÚ¶ş°æµÄÎ»ÖÃ¼ÆËãºÍÊı¾İ¿â¸üĞÂ
unsigned short int decode_position()
{

	unsigned short int i,matching_address;
    unsigned short int address_message,address_position;
	unsigned int ICAO_temp,ICAO_now,time_save_message,time_save_position;//,time_temp,time_now;
	//unsigned int parity_type;  
    unsigned int data_save_type;
    unsigned short int Calculation_ok;//·µ»ØÖµ£¬2±íÊ¾½âËã²»³É¹¦£»3±íÊ¾½âËã³É¹¦£»Èô½âËã³É¹¦£¬½âËã½á¹û·ÅÔÚairplane_location½á¹¹ÌåÈ«¾Ö±äÁ¿ÖĞ
    unsigned int ICAO_match=0;//±ØĞë³õÊ¼»¯Îª0£¬ICAOÆ¥Åä³É¹¦±êÖ¾£¬³É¹¦Îª1£¬²»³É¹¦Îª0

//new_data.time++;//hh
    if(new_data.time==0)
         return(2);            //UTCÊ±¼ä±êÇ©²»ÄÜÎª0£¬Èç¹ûÎª0 Ö»ÄÜ·µ»Ø2£¬·ÅÆúµ±Ç°´¦Àí                                       
    
    ICAO_now= new_data.ICAO_address_with_mark & 0x00ffffff;//È¡µ±Ç°Î»ÖÃ±¨ÎÄµÄICAOµØÖ·Âë
   
   
    time_save_message= new_data.time;                     //time_save_message³õÊ¼»¯µ±Ç°Î»ÖÃ±¨ÎÄµÄÊ±¿Ì,ºóĞø±È½Ï¼ÇÂ¼×îÔçµ½´ïµÄ±¨ÎÄÊ±¿Ì
    time_save_position=new_data.time;                     //ime_save_position³õÊ¼»¯µ±Ç°Î»ÖÃ±¨ÎÄµÄÊ±¿Ì,ºóĞø±È½Ï¼ÇÂ¼×îÔç½âËã³öµÄ¾­Î³¶ÈĞÅÏ¢°üÊ±¿Ì
   
 
	for (i = 0; i < 1000; i++)//²éÕÒÊı¾İ¿âÀïÃæÒÑ´æµÄĞÅÏ¢£¬¿ÉÄÜÊÇ±¨ÎÄ±àÂëĞÅÏ¢£¬¿ÉÄÜÊÇÒÑ¾­½âËã³öµÄÎ»ÖÃĞÅÏ¢
	{
		
	       //´Ë´¦ÊÇ×öICAOµÄ²éÕÒÆ¥Åä£¬Ò»µ©Æ¥Åä³É¹¦£¬¾ÍÌø³öÑ­»·£¬²»×öºóĞø´¦Àí
	       //*****************************************************************************************************************************************
	       ICAO_temp = data_save[i].ICAO_address_with_mark & 0x00ffffff;//¶ÁÈ¡Êı¾İ¿âICAOµØÖ·
	        
	       if(ICAO_now == ICAO_temp)
		    {
		      ICAO_match=1;         //Æ¥Åä³É¹¦±êÖ¾£¬³õÊ¼Îª0£¬±íÊ¾²éÕÒÆ¥Åä²»³É¹¦£¬Æ¥Åä³É¹¦ÔòĞŞ¸ÄÎª1(±íÊ¾ÔÚÊı¾İ¿âÖĞÕÒµ½ÁËÒ»ÖÂµÄICAOµØÖ·)
		      matching_address=i;   //Æ¥Åä³É¹¦Ê±£¬Êı¾İ¿âÖĞµÄ¶ÔÓ¦Ò»ÖÂICAOµÄ±¨ÎÄ»ò¾­Î³¶È´æ´¢Î»ÖÃ¼ÇÂ¼
		      
		      break;                // Æ¥Åäµ½Ò»ÖÂµÄICAO±àºÅ£¬ÔòÌø³öÑ­»·£¬²»ÔÙ¼ÌĞø²éÕÒ£¬²éÕÒ²»³É¹¦Ôò¼ÌĞø²éÕÒ£¬Ö±´ï±éÀúÊı¾İ¿âÖĞµÄ1000¸öÔªËØ 
		    }
	       //******************************************************************************************************************************************
	      //´Ë´¦ÊÇÊı¾İ¿âµÄ¸üĞÂ²ßÂÔ£¬ICAOÆ¥Åä²»³É¹¦£¬²éÕÒ¼ÇÂ¼Êı¾İ¿âÖĞÊ±¼ä×îÔçµ½´ï±¨ÎÄµÄ´æ´¢µØÖ·£¬²éÕÒ¼ÇÂ¼×îÔç½âËã³öµÄÄ¿±êÎ»ÖÃĞÅÏ¢µÄ´æ´¢µØÖ·£¬·Ö±ğ²éÕÒºÍ´æ´¢
		  //********************************************************************************************************************************************	 
		   data_save_type = data_save[i].ICAO_address_with_mark & 0x1000000;//¼ÇÂ¼µ±Ç°Î»ÖÃ´æ´¢µÄÊÇ±¨ÎÄ»òÕßÊÇ¾­Î³¶ÈĞÅÏ¢£¬µÚ25Î»£¨±àºÅ24£©Îª0±íÊ¾ÊÇ±¨ÎÄÔ­Ê¼ĞÅÏ¢£¬Îª1±íÊ¾ÊÇ¾­Î³¶ÈĞÅÏ¢

	        if((data_save[i].time<time_save_message)&&(data_save_type==0))
			{
			  time_save_message = data_save[i].time;//±£Ö¤time_save_message±£´æµÄÒ»¶¨ÊÇ×îÔçµÄÊ±¼ä£¬¶ÔÓ¦±¨ÎÄ
			  address_message = i; //±£Ö¤address_messageÖĞ´æµÄµØÖ·Æ«ÒÆÁ¿Ò»¶¨ÊÇ¶ÔÓ¦×îÔçÊ±¼äµÄ±¨ÎÄ£¬ÏàÍ¬Ê±¼äµÄ±¨ÎÄ¼ÇÂ¼µÚÒ»¸öÎ»ÖÃ
			                       //³õÊ¼×´Ì¬ÏÂ£¬time_save_messageÒ»¶¨»áÎª0£¬È·±£ĞÂ½ÓÊÕµ½µÄ²»Ò»ÑùICAOµÄ±¨ÎÄÒÀ´Î´æ·Å
								   //Èç¹ûÈ«²¿µÄÎª½âËãµÄ¾­Î³¶ÈĞÅÏ¢£¬address_message²»»á±»ĞŞ¸Ä£¬Îª¶¨ÒåÊ±µÄ³õÊ¼Öµ-1£¬·Ç·¨
			}

		    if((data_save[i].time<time_save_position)&&(data_save_type==0x1000000))
			{
			  time_save_position = data_save[i].time;//±£Ö¤time_save_position±£´æµÄÒ»¶¨ÊÇ×îÔçµÄÊ±¼ä£¬¶ÔÎ»ÖÃĞÅÏ¢
			  address_position = i;//±£Ö¤address_positionÖĞ´æµÄµØÖ·Æ«ÒÆÁ¿Ò»¶¨ÊÇ¶ÔÓ¦×îÔçÊ±¼äµÄ½âËã³öµÄÎ»ÖÃĞÅÏ¢´æ´¢µØÖ·£¬ÏàÍ¬Ê±¼äµÄĞÅÏ¢¼ÇÂ¼µÚÒ»¸öÎ»ÖÃ
			                       //³õÊ¼×´Ì¬ÏÂ£¬»òÕßÊÇ¹¤×÷¹ı³ÌÖĞ£¬È«²¿Îª±¨ÎÄĞÅÏ¢£¬address_positionµÄÖµ²»»á±»ĞŞ¸Ä£¬Îª¶¨ÒåÊ±µÄ³õÊ¼Öµ-1£¬·Ç·¨
			}
	     //*******************************************************************************************************************************************	
	 }	//for 1000
	
     if(ICAO_match==1)//ICAOÆ¥Åä³É¹¦Ôò¿É³¢ÊÔ½øĞĞ½âËã£¬¾Ö²¿½âËã»òÈ«¾Ö½âËã£¬¶¼ĞèÒª¿¼ÂÇÊ±¼äÃÅÏŞÒòËØ
	 {	    
		  data_save_type = data_save[matching_address].ICAO_address_with_mark & 0x1000000;//Æ¥Åä³É¹¦Ê±£¬Ê¶±ğÊı¾İ¿âÖĞµÄÊÇ±¨ÎÄ»òÊÇ¾­Î³¶ÈĞÅÏ¢£¬µÚ25Î»£¨±àºÅ24£©Îª0±íÊ¾ÊÇ±¨ÎÄÔ­Ê¼ĞÅÏ¢£¬Îª1±íÊ¾ÊÇ¾­Î³¶ÈĞÅÏ¢
		  g_32_ICAO_match_cnt++;
		  if(data_save_type == 0x1000000)//data_save_typeÎª0x1000000£¬Ê¹ÓÃ¾Ö²¿½âËã
		  {		
			  //Ê±¼ä²îÖµ£¬ÃëÎªµ¥Î»£¬333¹«Àï£¬·ÉĞĞËÙ¶ÈÕÛËã1224¹«Àï/Ğ¡Ê±£¨1ÂíºÕ£©£¬ÀíÂÛ²îÖµ×î´ó980Ãë£¬16.3·ÖÖÓ¡£¿¼ÂÇÎÀĞÇÔË¶¯¹ı¶¥Ê±¼äÕÛËã£¬²îÖµÈ¡480ÃëÎª¾Ö²¿½âËãµÄÃÅÏŞÊ±¼ä		
			  if((new_data.time - data_save[matching_address].time) <= LOCAL_TIME_THRESHOLD)//480´ú±í²îÖµ£¬ÃëÎªµ¥Î»
			  {
				  Calculation_ok=lat_lon_local_calculate(&new_data, &data_save[matching_address]);//µ÷ÓÃ¾Ö²¿½âËãº¯Êı£¬½âËã½á¹û´æ´¢µ½data_save[i]ÖĞ£¬Í¬Ê±airplane_location½á¹¹ÌåÈ«¾Ö±äÁ¿ÉÏ±¨			
			        //·µ»Ø3
					decode_altitude();//zhangfulong add
			  }
			  else
			  {
				  Calculation_ok=2;//±íÊ¾Ê±¼ä²»Âú×ã¾Ö²¿½âËãÒªÇó£¬·µ»ØÖµÉè¶¨Îª2
				  	//Èç¹û²îÖµÊ±¼ä²»Âú×ã¾Ö²¿½âËãÒªÇó£¬Í¬Ê±ICAOÊÇÆ¥ÅäµÄ£¬Ôò²Á³ıÊı¾İ¿âÖĞµÄ¾­Î³¶ÈĞÅÏ¢£¬½«µ±Ç°µÄ±¨ÎÄĞÅÏ¢´æ´¢½øÈ¥£¬ÔÚdata_save[i]Î»ÖÃ
				  data_save[matching_address].time = new_data.time;
				  data_save[matching_address].ICAO_address_with_mark = new_data.ICAO_address_with_mark;//µÚ25Î»Îª0£¬±íÊ¾¸ÃÎ»ÖÃ´æ´¢µÄÊÇ±¨ÎÄ
				  data_save[matching_address].position.CPR_code[0] = new_data.position.CPR_code[0];
				  data_save[matching_address].position.CPR_code[1] = new_data.position.CPR_code[1];
				  data_save[matching_address].position.CPR_code[2] = new_data.position.CPR_code[2];
				  data_save[matching_address].position.CPR_code[3] = 0;//Êı×éµÄÕâ¸öÎ»ÖÃ¿ÉÒÔºöÂÔ²»¹Ü£¬±ÜÃâ³ö´í£¬½¨ÒéÇå0
			  }
		   }
		   else//data_save_type²»Îª0x1000000£¬ÔòĞè½øĞĞÈ«¾Ö½âËã 
		   {
			  if((new_data.time - data_save[matching_address].time) <= GLOBAL_TIME_THRESHOLD)//È«¾Ö½âËãÊ±¼ä¼ä¸ô£¬·É»úËÙ¶È°´ÕÕ1224¹«Àï£¨1ÂíºÕ£©,5.1kmµÄ·ÉĞĞ¾àÀë£¬ÒâÎ¶×Å15ÃëÊ±¼ä£¬
			                                                              //µ«È¡15ÃëÎªÈ«¾Ö½âËãµÄÃÅÏŞÊ±¼ä¹ıÓÚ¿Á¿Ì£¬Êµ¼Ê³¡¾°ÖĞ£¬90%µÄ·É»ú"µØËÙ"¶¼²»´óÓÚ1000¹«Àï£¬¿ÉÒÔ·Å¿íµ½18ÃëÊ±¼äÃÅÏŞ
			  
			  {
				  if (data_save[matching_address].position.CPR_code[2] != new_data.position.CPR_code[2])//±È½Ïµ±Ç°±¨ÎÄºÍÒÑ´æ´¢µÄ±¨ÎÄÊÇ·ñÊÇÆæÅ¼±àÂëÆ¥Åä
				  {
						Calculation_ok=lat_lon_global_calculate(&new_data, &data_save[matching_address]);//µ÷ÓÃÈ«¾Ö½âËãº¯Êı£¬½âËã½á¹û´æ´¢µ½data_save[matching_address]ÖĞ£¬Í¬Ê±airplane_location½á¹¹ÌåÈ«¾Ö±äÁ¿ÉÏ±¨								
				  		decode_altitude();//zhangfulong add
				  }
				  else
				  {
						Calculation_ok=2;//±íÊ¾ÆæÅ¼±àÂë²»Âú×ãÈ«¾Ö½âËãÒªÇó£¬·µ»ØÖµÉè¶¨Îª2
						//Èç¹ûÆæÅ¼±àÂë²»Æ¥Åä£¬Ôò²»ÄÜÈ«¾Ö½âËãÒªÇó£¬Í¬Ê±ICAOÊÇÆ¥ÅäµÄ£¬ÔòÖ»¸üĞÂÊı¾İ¿âÖĞµÄ±¨ÎÄĞÅÏ¢£¬½«µ±Ç°µÄ±¨ÎÄĞÅÏ¢´æ´¢½øÈ¥£¬ÔÚdata_save[matching_address]Î»ÖÃ
						data_save[matching_address].time = new_data.time;
						data_save[matching_address].ICAO_address_with_mark = new_data.ICAO_address_with_mark;//µÚ25Îª0£¬±íÊ¾¸ÃÎ»ÖÃ´æ´¢µÄÊÇ¨ÎÄ
						data_save[matching_address].position.CPR_code[0] = new_data.position.CPR_code[0];
						data_save[matching_address].position.CPR_code[1] = new_data.position.CPR_code[1];
						data_save[matching_address].position.CPR_code[2] = new_data.position.CPR_code[2];
						data_save[matching_address].position.CPR_code[3] = 0;//Êı×éµÄÕâ¸öÎ»ÖÃ¿ÉÒÔºöÂÔ²»¹Ü£¬±ÜÃâ³ö´í£¬½¨ÒéÇå0
				  }
			  }
			  else
			  {   
				   Calculation_ok=2;//±íÊ¾Ê±¼ä¼ä¸ô²»Âú×ãÈ«¾Ö½âËãÒªÇó£¬·µ»ØÖµÉè¶¨Îª2
				   //Èç¹û²îÖµÊ±¼ä²»Âú×ãÈ«¾Ö½âËãÒªÇó£¬Í¬Ê±ICAOÊÇÆ¥ÅäµÄ£¬ÔòÖ»¸üĞÂÊı¾İ¿âÖĞµÄ±¨ÎÄĞÅÏ¢£¬½«µ±Ç°µÄ±¨ÄĞÅÏ¢´æ´¢½øÈ¥£¬ÔÚdata_save[matching_address]Î»ÖÃ
				  	data_save[matching_address].time = new_data.time;
					data_save[matching_address].ICAO_address_with_mark = new_data.ICAO_address_with_mark;//µÚ25Îª0£¬±íÊ¾¸ÃÎ»ÖÃ´æ´¢µÄÊÇ±¨ÎÄ
					data_save[matching_address].position.CPR_code[0] = new_data.position.CPR_code[0];
			    	data_save[matching_address].position.CPR_code[1] = new_data.position.CPR_code[1];
					data_save[matching_address].position.CPR_code[2] = new_data.position.CPR_code[2];
					data_save[matching_address].position.CPR_code[3] = 0;//Êı×éµÄÕâ¸öÎ»ÖÃ¿ÉÒÔºöÂÔ»¹Ü£¬±ÜÃâ³ö´í£¬½¨ÒéÇå0
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
	   return(Calculation_ok);//ICAOÆ¥Åä³É¹¦µÄÇé¿öÏÂ£¬¼´(ICAO_match==1)Ìõ¼şÂú×ã£¬½«½âËãµÄ½á¹û·µ»Ø£¬·µØÖµÊÇ2»òÕß3		 
	 }
   if(ICAO_match==0)//ÔÚ±éÀúÊı¾İ¿âºóICAOÆ¥Åä²»³É¹¦ÔòÈÏÎªÊÇÒ»¼ÜĞÂ·É»ú£¬Ğè½øĞĞ±¨ÎÄÊı¾İµÄ±£´æ£»ÓÉÓÚÊı¾İ¿â´óĞ¡ÊÇ¹Ì¶¨µÄ£¬ĞèÒªÖÆ¶¨Ò»Ì×¸üĞÂÊı¾İ¿âµÄ¹æÔò
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
   	   if(time_save_message==0)//time_save_messageÎª0£¬ÒâÎ¶×Å»¹ÓĞ´æ´¢¿Õ¼äÎ´·ÅÖÃÈÎºÎ±¨ÎÄ£¬ËùÒÔÏÈ°ÑĞÂµÄ±¨ÎÄ·ÅÈë£¬²»ÓÃ×öÊ±¼ä¶Ô±È
	   {
	     data_save[address_message].time = new_data.time;
	     data_save[address_message].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		 data_save[address_message].position.CPR_code[0] = new_data.position.CPR_code[0];
		 data_save[address_message].position.CPR_code[1] = new_data.position.CPR_code[1];
		 data_save[address_message].position.CPR_code[2] = new_data.position.CPR_code[2];
		 data_save[address_message].position.CPR_code[3] = 0;  
		 return(2);  
	   }
	   else//time_save_message²»Îª0£¬ÒâÎ¶×Å´æ´¢¿Õ¼äÒÑ¾­È«²¿·ÅÖÃ±¨ÎÄ»ò¾­Î³£¬Ğè×öÊ±¼ä¶Ô±È
	   {
	   	// Calculation_ok=2;
	   	 //´ËÂß¼­µÄ»ù´¡ÊÇ±¨ÎÄÊ±¼äÓĞĞ§Ê±¼ä18Ãë£¬¾­Î³¶ÈµÄÉú´æÖÜÆÚ480Ãë£¬ËùÒÔ¿ÉÄÜ¹¹³ÉËÄÖÖÇé¿ö
		 //1.µ±Ç°±¨ÎÄÊ±¿ÌÓë×îÔç´æ´¢±¨ÎÄÊ±¿Ì±È½Ï´óÓÚ18Ãë£¬Í¬Ê±Óë×îÔç½âËã³öµÄ¾­Î³¶ÈĞÅÏ¢Ê±¿Ì±È½Ï´óÓÚ480Ãë£¬´ËÊ±¸üĞÂÁ½Õß¾ù¿É£¬Ñ¡Ôñ¸üĞÂ±¨ÎÄµÄ´æ´¢
		 //2.µ±Ç°±¨ÎÄÊ±¿ÌÓë×îÔç´æ´¢±¨ÎÄÊ±¿Ì±È½Ï´óÓÚ18Ãë£¬Í¬Ê±Óë×îÔç½âËã³öµÄ¾­Î³¶ÈĞÅÏ¢Ê±¿Ì±È½ÏĞ¡ÓÚµÈÓÚ480Ãë£¬´ËÊ±¸üĞÂ±¨ÎÄµÄ´æ´¢
		 //3.µ±Ç°±¨ÎÄÊ±¿ÌÓë×îÔç´æ´¢±¨ÎÄÊ±¿Ì±È½ÏĞ¡ÓÚµÈÓÚ18Ãë£¬Í¬Ê±Óë×îÔç½âËã³öµÄ¾­Î³¶ÈĞÅÏ¢Ê±¿Ì±È½Ï´óÓÚ480Ãë£¬´ËÊ±¸üĞÂ¾­Î³ĞÅÏ¢µÄ´æ´¢
	   	 //3.µ±Ç°±¨ÎÄÊ±¿ÌÓë×îÔç´æ´¢±¨ÎÄÊ±¿Ì±È½ÏĞ¡ÓÚµÈÓÚ18Ãë£¬Í¬Ê±Óë×îÔç½âËã³öµÄ¾­Î³¶ÈĞÅÏ¢Ê±¿ÌÈ½ÏĞ¡ÓÚµÈÓÚ480Ãë£¬´ËÊ±¸üĞÂ¾­Î³¶ÈĞÅÏ¢µÄ´æ´¢ 
	   	 
	   	 //³õÊ¼×´Ì¬£ºÓÉÓÚÊ±¼ä²ÉÓÃ32Î»µÄÃë¼ÆÊı£¬Ê±¼äÆğµãUTCÊ±2009Äê1ÔÂ1ÈÕ£¬ËùÒÔ³õÊ¼×´Ì¬ÏÂ£¬Êı¾İ¿âÀïÃæÊ±¿ÌÈ«0£¬(ĞøÏÂĞĞ)
	   	 //£¨½ÓÉÏÒ»ĞĞ£©Ä¬ÈÏÎªÈ«²¿ÊÇ"ÎŞĞ§"µÄ±¨ÎÄĞÅÏ¢£¬Ö»ÒªÊÕµ½ÓĞĞ§±¨ÎÄ£¬Ê±¼ä²îÖµÔ¶Ô¶´óÓÚ18£¬Âú×ãµÚ2ÖÖÇé¿ö£»¼´Ê¹Ê±¼ä²îĞ¡ÓÚµÈÓÚ18£¬Âú×ãµÚ4.1ÖÖÇé¿ö
	   	 //¹¤×÷¹ı³ÌÖĞ£¬Èç¹û³öÏÖ¼«¶ËÇé¿ö£¬¼´Êı¾İ¿âÈ«²¿ÊÇ±¨ÎÄ£¬Ò»¶¨»áÂú×ãµÚ2»òµÚ4.1ÖÖÇé¿ö
	   	 //¹¤×÷¹ı³ÌÖĞ£¬Èç¹û³öÏÖ¼«¶ËÇé¿ö£¬¼´Êı¾İ¿âÈ«²¿ÊÇ¾­Î³¶ÈĞÅÏ¢£¬Ò»¶¨»áÂú×ãµÚ3»òµÚ4.2ÖÖÇé¿ö 	 
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
		   &&((new_data.time-time_save_position)<=LOCAL_TIME_THRESHOLD))//ÓĞ¿ÉÄÜnew_data.time-time_save_positionÎª0£¬¼´Êı¾İ¿âÖĞÈ«²¿´æ´¢ÊÇ±¨ÎÄ
	   	   {
	   	     data_save[address_message].time = new_data.time;
		     data_save[address_message].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		     data_save[address_message].position.CPR_code[0] = new_data.position.CPR_code[0];
		     data_save[address_message].position.CPR_code[1] = new_data.position.CPR_code[1];
		     data_save[address_message].position.CPR_code[2] = new_data.position.CPR_code[2];
			 data_save[address_message].position.CPR_code[3] = 0;
	   	   }
	   	 
	   	   if(((new_data.time-time_save_message)<=GLOBAL_TIME_THRESHOLD)
	   	   &&((new_data.time-time_save_position)>LOCAL_TIME_THRESHOLD))//ÓĞ¿ÉÄÜnew_data.time-time_save_messageÎª0£¬¼´Êı¾İ¿âÖĞÈ«²¿´æ´¢ÊÇ¾­Î³¶ÈĞÅÏ¢
	   	   {
	   	     data_save[address_position].time = new_data.time;
		     data_save[address_position].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		     data_save[address_position].position.CPR_code[0] = new_data.position.CPR_code[0];
		     data_save[address_position].position.CPR_code[1] = new_data.position.CPR_code[1];
		     data_save[address_position].position.CPR_code[2] = new_data.position.CPR_code[2];
	         data_save[address_position].position.CPR_code[3] = 0;
	   	   }

	       if(((new_data.time-time_save_message)<=GLOBAL_TIME_THRESHOLD)
	       &&((new_data.time-time_save_position)<=LOCAL_TIME_THRESHOLD))//Á½¸ö²îÖµÖÁÉÙÓĞÒ»¸ö²»Îª0
	   	   {
	   	      if(((new_data.time-time_save_message)!=0)&&((new_data.time-time_save_position)==0))//Êı¾İ¿âÈ«²¿¶¼ÊÇ±¨ÎÄ£¬Ã»ÓĞÎ»ÖÃĞÅÏ¢£¬Ö»ÄÜ¸üĞÂÊ±¼ä×îÔçµÄ±¨ÎÄ´æ´¢
			  {
			     data_save[address_message].time = new_data.time;
		         data_save[address_message].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		         data_save[address_message].position.CPR_code[0] = new_data.position.CPR_code[0];
		         data_save[address_message].position.CPR_code[1] = new_data.position.CPR_code[1];
		         data_save[address_message].position.CPR_code[2] = new_data.position.CPR_code[2];
				 data_save[address_message].position.CPR_code[3] = 0;
	          }
	          if(((new_data.time-time_save_message)==0)&&((new_data.time-time_save_position)!=0))//Êı¾İ¿âÈ«²¿¶¼ÊÇÎ»ÖÃĞÅÏ¢£¬Ã»ÓĞ±¨ÎÄ£¬Ö»ÄÜ¸üĞÂÊ±¼ä×îÔçµÄÎ»ÖÃ´æ´¢
	   	      {
	   	         data_save[address_position].time = new_data.time;
		         data_save[address_position].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		         data_save[address_position].position.CPR_code[0] = new_data.position.CPR_code[0];
		         data_save[address_position].position.CPR_code[1] = new_data.position.CPR_code[1];
		         data_save[address_position].position.CPR_code[2] = new_data.position.CPR_code[2];
			     data_save[address_position].position.CPR_code[3] = 0;
			  }
			  if(((new_data.time-time_save_message)!=0)&&((new_data.time-time_save_position)!=0))//Êı¾İ¿âÖĞ×îÔçµÄ±¨ÎÄºÍÎ»ÖÃÊ±¼ä¶¼ÔÚÃÅÏŞÄÚ£¬Ñ¡Ôñ¸üĞÂÊ±¼ä×îÔçµÄÎ»ÖÃ´æ´¢
	          {
	   	         data_save[address_position].time = new_data.time;
		         data_save[address_position].ICAO_address_with_mark = new_data.ICAO_address_with_mark;
		         data_save[address_position].position.CPR_code[0] = new_data.position.CPR_code[0];
		         data_save[address_position].position.CPR_code[1] = new_data.position.CPR_code[1];
		         data_save[address_position].position.CPR_code[2] = new_data.position.CPR_code[2];
			     data_save[address_position].position.CPR_code[3] = 0;
			  }
			  //¸ÃÇé¿öÔÚ³ÌĞòÖ´ĞĞ¹ı³ÌÖĞ£¬Èç¹ûËùÓĞÊäÈëÁ¿È«²¿ÕıÈ·µÄÇé¿öÏÂ£¬ÊÇ²»»á³öÏÖµÄ£¬Ğ´ÔÚÕâÀïµÄÄ¿µÄÖ»ÊÇÎªÁË±ÜÃâ³ö´í¡££¨¶ÔÓ¦ÎŞÊ±¼äĞÅÏ¢µÄÇé¿ö£©
			  if(((new_data.time-time_save_message)==0)&&((new_data.time-time_save_position)==0))////Çé¿ö4.4£¬ÀíÂÛÉÏ²»»á³öÏÖ
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
    		// return(Calculation_ok);//ICAOÆ¥Åä³É¹¦µÄÇé¿öÏÂ£¬¼´(ICAO_match==0)Ìõ¼şÂú×ã£¬Ö»ÄÜ·µ»Ø2	
       }////time_save_messageÎª0µÄelse
   }
}   	 
//*************************************************************************************************************************************************************
//*                                                                                                                                                           *
//*  ÒÔÏÂÎª½âËÙ¶ÈĞÅÏ¢Éæ¼°µ½µÄº¯Êı£º   decode_velocity()                                                                     
//*                                                                                                                                                           *
//*************************************************************************************************************************************************************
#if 1
unsigned short decode_velocity() 
{
  
  float velocity_polar_coordinates=0;
  //temp_code=velocity_code;
  airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address & 0xffffff;//¸ß8Î»Çå0£¬ÓÃÓÚËÙ¶È×´Ì¬ĞÅÏ¢´æ´¢
  if((velocity_subtype>=1)&&(velocity_subtype<=4))
  {
       //------------------------1---------------------------- 
      if(velocity_subtype==1)//µØËÙ·Ç³¬ÒôËÙÄ£Ê½
      {
	       airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x1000000;/*airplane_velocity_three.ICAO_addressÎª32bitµÄÎŞ·ûºÅÕûĞÍ£¬µÍ24´æICAO£¬µÚ27£¬26£¬25Î»´æËÙ¶È×´Ì¬£¬Îª001*/
	       if(velocity_code.bit47_56==0)
	       {
		         airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x40000000;//airplane_velocity.ICAO_addressÎª32bitµÄÎŞ·ûºÅÕûĞÍ£¬µÍ24´æICAO£¬
                                                                                // ¶«Î÷·½ÏòËÙ¶ÈÎŞĞ§£¬1¿ªÊ¼±àºÅµÄµÚ31Î»(0¿ªÊ¼±àºÅµÄµÚ30Î»)ÉèÖÃÎª1
				 airplane_velocity_three.E_W_velocity=0; 
		   }
	       else
	       {
	            if(velocity_code.bit46==0)
	 		    {
	 		        if(velocity_code.bit47_56<1023)
			              airplane_velocity_three.E_W_velocity=(int)(1.852*(velocity_code.bit47_56-1)*10000);//ÕıÊıÖµ£¬±íÊ¾Ïò¶«,ËÙ¶ÈKM/H,¾«¶ÈÎª1E-4
				    else
				          airplane_velocity_three.E_W_velocity=18918180;                       //Îª1023Ê±£¬ÌØÊâ´¦Àí£¬ÕıÊıÖµ£¬±íÊ¾Ïò¶«,ËÙ¶ÈKM/H,¾«¶ÈÎª1E-4
		        }  
		        else
		        {
		            if(velocity_code.bit47_56<1023)
				         airplane_velocity_three.E_W_velocity=(int)(-1.852*(velocity_code.bit47_56-1)*10000);//¸ºÊıÖµ£¬±íÊ¾ÏòÎ÷,ËÙ¶ÈKM/H£¬¾«¶ÈÎª1E-4
		            else
					     airplane_velocity_three.E_W_velocity=-18918180;                      //Îª1023Ê±£¬ÌØÊâ´¦Àí£¬¸ºÊıÖµ£¬±íÊ¾ÏòÎ÷,ËÙ¶ÈKM/H,¾«¶ÈÎª1E-4		     
		        }
		  }
		  if(velocity_code.bit58_67==0)
		  {
		           airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x20000000;//airplane_velocity.ICAO_addressÎª32bitµÄÎŞ·ûºÅÕûĞÍ£¬µÍ24´æICAO£¬
			                                                                                // ÄÏ±±·½ÏòËÙ¶ÈÎŞĞ§£¬1¿ªÊ¼±àºÅµÄµÚ30Î»(0¿ªÊ¼±àºÅµÄµÚ29Î»)ÉèÖÃÎª1
				   airplane_velocity_three.N_S_velocity=0;  
		  }
	      else
		  {
				   if(velocity_code.bit57==0) 
				   {
	                   if(velocity_code.bit58_67<1023)         
	                       airplane_velocity_three.N_S_velocity=(int)(1.852*(velocity_code.bit58_67-1)*10000);//ÕıÊıÖµ£¬±íÊ¾Ïò±±£¬ËÙ¶ÈKM/H,¾«¶ÈÎª1E-4
	                   else
	                       airplane_velocity_three.N_S_velocity=18918180;                       //Îª1023Ê±£¬ÌØÊâ´¦Àí£¬ÕıÊıÖµ£¬±íÊ¾Ïò±±,ËÙ¶ÈKM/H,¾«¶ÈÎª1E-4           
		           }
		           else
	               {
	                   if(velocity_code.bit58_67<1023) 
	                       airplane_velocity_three.N_S_velocity=(int)(-1.852*(velocity_code.bit58_67-1)*10000);//¸ºÊıÖµ£¬±íÊ¾ÏòÄÏ£¬ËÙ¶ÈKM/H,¾«¶ÈÎª1E-4
	                   else
	                       airplane_velocity_three.N_S_velocity=-18918180;                       //Îª1023Ê±£¬ÌØÊâ´¦Àí£¬¸ºÊıÖµ£¬±íÊ¾ÏòÄÏ,ËÙ¶ÈKM/H,¾«¶ÈÎª1E-4  
	               }  
	      }
          if(velocity_code.bit70_78==0)
          {
                   airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x10000000;//airplane_velocity_three.ICAO_addressÎª32bitµÄÎŞ·ûºÅÕûĞÍ£¬µÍ24´æICAO£¬
			                                                                               // ´¹Ö±·½ÏòËÙ¶ÈÎŞĞ§£¬1¿ªÊ¼±àºÅµÄµÚ29Î»(0¿ªÊ¼±àºÅµÄµÚ28Î»)ÉèÖÃÎª1
				   airplane_velocity_three.VERT_velocity=0; 
          }
          else
          {
                  if(velocity_code.bit69==0)
	              {
	                   if(velocity_code.bit70_78<511)   
	                       airplane_velocity_three.VERT_velocity=(int)(0.3048*(velocity_code.bit70_78-1)*64*10000);   //ÕıÊıÖµ£¬±íÊ¾ÏòÉÏ£¬ËÙ¶ÈÃ×/·ÖÖÓ,¾«¶ÈÎª1E-4
				       else
				           airplane_velocity_three.VERT_velocity=99389184;                          //Îª511Ê±£¬ÌØÊâ´¦Àí,ÕıÊıÖµ£¬±íÊ¾ÏòÉÏ£¬ËÙ¶ÈÃ×/·ÖÖÓ,¾«¶ÈÎª1E-4
	              }
	              else 
			      {
			           if(velocity_code.bit70_78<511) 
			               airplane_velocity_three.VERT_velocity=(int)(-0.3048*(velocity_code.bit70_78-1)*64*10000);  //¸ºÊıÖµ£¬±íÊ¾ÏòÏÂ£¬ËÙ¶ÈÃ×/·ÖÖÓ,¾«¶ÈÎª1E-4
	                   else
				           airplane_velocity_three.VERT_velocity=-99389184;                          //Îª511Ê±£¬ÌØÊâ´¦,¸ºÊıÖµ£¬±íÊ¾ÏòÏÂ£¬ËÙ¶ÈÃ×/·ÖÖÓ,¾«¶ÈÎª1E-4
                  }
           }  
         return(5);//µØËÙ·Ç³¬ÒôËÙÄ£Ê½ÈıÎ¬ËÙ¶È£¬·µ»ØÖµ5
        }
		 //------------------------2---------------------------- 
        if(velocity_subtype==2)////µØËÙ³¬ÒôËÙÄ£Ê½
	    {
    	       airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x2000000;/* airplane_velocity_three.ICAO_addressÎª32bitµÄÎŞ·ûºÅÕûĞÍ£¬µÍ24´æICAO£¬
		                                                                                µÚ27£¬26£¬25Î»´æËÙ¶È×´Ì¬£¬Îª010*/
		       if(velocity_code.bit47_56==0)
		       {
		           airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x40000000;//airplane_velocity.ICAO_addressÎª32bitµÄÎŞ·ûºÅÕûĞÍ£¬µÍ24´æICAO£¬
			                                                                               // ¶«Î÷·½ÏòËÙ¶ÈÎŞĞ§£¬1¿ªÊ¼±àºÅµÄµÚ31Î»(0¿ªÊ¼±àºÅµÄµÚ30Î»)ÉèÖÃÎª1
				   airplane_velocity_three.E_W_velocity=0; 
			   }
		       else
			   {
			       if(velocity_code.bit46==0)
			       {
			           if(velocity_code.bit47_56<1023)
			               airplane_velocity_three.E_W_velocity=(int)(7.408*(velocity_code.bit47_56-1)*10000);//ÕıÊıÖµ£¬±íÊ¾Ïò¶«,ËÙ¶ÈKM/H,¾«¶ÈÎª1E-4
					   else
					       airplane_velocity_three.E_W_velocity=75672720;                       //Îª1023Ê±£¬ÌØÊâ´¦Àí£¬ÕıÊıÖµ£¬±íÊ¾Ïò¶«,ËÙ¶ÈKM/H,¾«¶ÈÎª1E-4
		           }
		           else
				   {
				       if(velocity_code.bit47_56<1023)
				           airplane_velocity_three.E_W_velocity=(int)(-7.408*(velocity_code.bit47_56-1)*10000);//¸ºÊıÖµ£¬±íÊ¾ÏòÎ÷,ËÙ¶ÈKM/H,¾«¶ÈÎª1E-4
		               else
					       airplane_velocity_three.E_W_velocity=-75672720;                      //Îª1023Ê±£¬ÌØÊâ´¦Àí£¬¸ºÊıÖµ£¬±íÊ¾ÏòÎ÷,ËÙ¶ÈKM/H,¾«¶ÈÎª1E-4		     
			       }
		       }
		       if(velocity_code.bit58_67==0)
		       {
		           airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x20000000;//airplane_velocity.ICAO_addressÎª32bitµÄÎŞ·ûºÅÕûĞÍ£¬µÍ24´æICAO£¬
			                                                                               // ÄÏ±±·½ÏòËÙ¶ÈÎŞĞ§£¬1¿ªÊ¼±àºÅµÄµÚ30Î»(0¿ªÊ¼±àºÅµÄµÚ29Î»)ÉèÖÃÎª1
				   airplane_velocity_three.N_S_velocity=0; 
			   }
	           else
			   {
				   if(velocity_code.bit57==0) 
                   {
	                   if(velocity_code.bit58_67<1023)         
	                     airplane_velocity_three.N_S_velocity=(int)(7.408*(velocity_code.bit58_67-1)*10000);//ÕıÊıÖµ£¬±íÊ¾Ïò±±£¬ËÙ¶ÈKM/H,¾«¶ÈÎª1E-4
	                   else
	                     airplane_velocity_three.N_S_velocity=75672720;                       //Îª1023Ê±£¬ÌØÊâ´¦Àí£¬ÕıÊıÖµ£¬±íÊ¾Ïò±±,ËÙ¶ÈKM/H,¾«¶ÈÎª1E-4           
		           }
		           else
	               {
	                   if(velocity_code.bit58_67<1023) 
	                     airplane_velocity_three.N_S_velocity=(int)(-7.408*(velocity_code.bit58_67-1)*10000);//¸ºÊıÖµ£¬±íÊ¾ÏòÄÏ£¬ËÙ¶ÈKM/H,¾«¶ÈÎª1E-4
	                   else
	                     airplane_velocity_three.N_S_velocity=-75672720;                       //Îª1023Ê±£¬ÌØÊâ´¦Àí£¬¸ºÊıÖµ£¬±íÊ¾ÏòÄÏ,ËÙ¶ÈKM/H,¾«¶ÈÎª1E-4  
	               }
               }
               if(velocity_code.bit70_78==0)
               {
                    airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x10000000;//airplane_velocity.ICAO_addressÎª32bitµÄÎŞ·ûºÅÕûĞÍ£¬µÍ24´æICAO£¬
			                                                                                // ´¹Ö±·½ÏòËÙ¶ÈÎŞĞ§£¬1¿ªÊ¼±àºÅµÄµÚ29Î»(0¿ªÊ¼±àºÅµÄµÚ28Î»)ÉèÖÃÎª1
				    airplane_velocity_three.VERT_velocity=0; 
               }
               else
			   {
                   if(velocity_code.bit69==0)
                   { 
	                   if(velocity_code.bit70_78<511)   
	                       airplane_velocity_three.VERT_velocity=(int)(0.3048*(velocity_code.bit70_78-1)*64*10000);   //ÕıÊıÖµ£¬±íÊ¾ÏòÉÏ£¬ËÙ¶ÈÃ×/·ÖÖÓ,¾«¶ÈÎª1E-4  
				       else
				           airplane_velocity_three.VERT_velocity=99389184;                          //Îª511Ê±£¬ÌØÊâ´¦Àí,ÕıÊıÖµ£¬±íÊ¾ÏòÉÏ£¬ËÙ¶ÈÃ×/·ÖÖÓ,¾«¶ÈÎª1E-4  
	               }
	               else 
			       {
			           if(velocity_code.bit70_78<511) 
			               airplane_velocity_three.VERT_velocity=(int)(-0.3048*(velocity_code.bit70_78-1)*64*10000);  //¸ºÊıÖµ£¬±íÊ¾ÏòÏÂ£¬ËÙ¶ÈÃ×/·ÖÖÓ,¾«¶ÈÎª1E-4  
	                   else
				           airplane_velocity_three.VERT_velocity=-99389184;                          //Îª511Ê±£¬ÌØÊâ´¦Àí,¸ºÊıÖµ£¬±íÊ¾ÏòÏÂ£¬ËÙ¶ÈÃ×/·ÖÖÓ,¾«¶ÈÎª1E-4  
                   }
                }
                return(5);//µØËÙ³¬ÒôËÙÄ£Ê½,ÈıÎ¬ËÙ¶È£¬·µ»ØÖµ5
        }
		 //------------------------3---------------------------- 
        if(velocity_subtype==3)//¿ÕËÙ·Ç³¬ÒôËÙÄ£Ê½
        {
	          //airplane_velocity_two.ICAO_address=airplane_velocity_two.ICAO_address|0x3000000;
		      airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x3000000;         //µÚ27£¬26£¬25Î»´æËÙ¶È×´Ì¬£¬Îª011*/
              if((velocity_code.bit46==0)||(velocity_code.bit58_67==0))//Æ½ÃæËÙ¶È·½ÏòÎ´¿ÉÖª»òÕßÆ½ÃæËÙ¶ÈÎ´¿ÉÖª
              {
		           airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x60000000;//airplane_velocity.ICAO_addressÎª32bitµÄÎŞ·ûºÅÕûĞÍ£¬µÍ24´æICAO£¬
			                                                                               // Æ½ÃæËÙ¶È·½ÏòÎ´¿ÉÖª£¬1¿ªÊ¼±àºÅµÄµÚ30£¬31Î»(0¿ªÊ¼±àºÅµÄ29£¬30Î»)ÉèÖÃÎª1
				   airplane_velocity_three.N_S_velocity=0;
				   airplane_velocity_three.E_W_velocity=0; 
			  }
              else
			  {
                  if(velocity_code.bit58_67<1023)
			           velocity_polar_coordinates=1.852*(velocity_code.bit58_67-1);
                  else
			           velocity_polar_coordinates=1891.818;

				  airplane_velocity_three.E_W_velocity=(int)(velocity_polar_coordinates*sin(0.3515625*velocity_code.bit47_56*PI/180.0)*10000);//Õı±±·½ÏòÎª0¶È£¬Ë³Ê±ÕëĞı×ª£¬ÕıÖµÎª¶«£¬¸ºÖµÎªÎ÷£¬¾«¶ÈÎª1E-4  
                  airplane_velocity_three.N_S_velocity=(int)(velocity_polar_coordinates*cos(0.3515625*velocity_code.bit47_56*PI/180.0)*10000);//Õı±±·½ÏòÎª0¶È£¬Ë³Ê±ÕëĞı×ª£¬ÕıÖµÎª±±£¬¸ºÖµÎªÄÏ£¬¾«¶ÈÎª1E-4 
			  }
              if(velocity_code.bit70_78==0)
              {
                   airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x10000000;//airplane_velocity.ICAO_addressÎª32bitµÄÎŞ·ûºÅÕûĞÍ£¬µÍ24´æICAO£¬
			                                                                               // ´¹Ö±·½ÏòËÙ¶ÈÎŞĞ§£¬1¿ªÊ¼±àºÅµÄµÚ29Î»(0¿ªÊ¼±àºÅµÄµÚ28Î»)ÉèÖÃÎª1
				   airplane_velocity_three.VERT_velocity=0; 
              }
              else
			  {
                   if(velocity_code.bit69==0)
	                   if(velocity_code.bit70_78<511)   
	                     airplane_velocity_three.VERT_velocity=(int)(0.3048*(velocity_code.bit70_78-1)*64*10000);   //ÕıÊıÖµ£¬±íÊ¾ÏòÉÏ£¬ËÙ¶ÈÃ×/·ÖÖÓ,¾«¶ÈÎª1E-4 
				       else
				         airplane_velocity_three.VERT_velocity=99389184;                          //Îª511±£¬ÌØÊâ´¦À?ÕıÊıÖµ£¬±íÊ¾ÏòÉÏ£¬ËÙ¶ÈÃ×/·ÖÖÓ,¾«¶ÈÎª1E-4 
	               else 
			           if(velocity_code.bit70_78<511) 
			             airplane_velocity_three.VERT_velocity=(int)(-0.3048*(velocity_code.bit70_78-1)*64*10000);  //¸ºÊıÖµ£¬±íÊ¾ÏòÏÂ£¬ËÙ¶ÈÃ×/·ÖÖÓ,¾«¶ÈÎª1E-4 
	                   else
				         airplane_velocity_three.VERT_velocity=-99389184;                          //Îª511Ê±£¬ÌØÊâ´¦Àí,¸ºÊıÖµ£¬±íÊ¾ÏòÏÂ£¬ËÙ¶ÈÃ×/·ÖÖÓ,¾«¶ÈÎª1E-4 
              }
	          return(5);//¿ÕËÙ·Ç³¬ÒôËÙÄ£Ê½,Æ½ÃæËÙ¶È¼Ó´¹Ö±ËÙ¶ÈÒÑ×ª»»Îª3Î¬ËÙ¶È£¬·µ»ØÖµ5
	    }
	    //------------------------4---------------------------- 
	    if(velocity_subtype==4)//¿ÕËÙ³¬ÒôËÙÄ£Ê½
        {
	          airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x4000000;/* airplane_velocity.ICAO_addressÎª32bitµÄÎŞ·ûºÅÕûĞÍ£¬µÍ24´æICAO£¬
		                                                                                µÚ27£¬26£¬25Î»´æËÙ¶È×´Ì¬£¬Îª100*/
            
              if((velocity_code.bit46==0)||(velocity_code.bit58_67==0))//Æ½ÃæËÙ¶È·½ÏòÎ´¿ÉÖª»òÕßÆ½ÃæËÙ¶ÈÎ´¿ÉÖª
              {
		           airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x60000000;//airplane_velocity.ICAO_addressÎª32bitµÄÎŞ·ûºÅÕûĞÍ£¬µÍ24´æICAO£¬
			                                                                               // Æ½ÃæËÙ·½ÏòÎ´¿ÉÖª£¬1¿ªÊ¼±àºÅµÄµÚ30£¬31Î»(0¿ªÊ¼±àºÅµÄ29£¬30Î»)ÉèÖÃÎª1
				   airplane_velocity_three.N_S_velocity=0;
				   airplane_velocity_three.E_W_velocity=0; 
			  }
              else
			  {
                  if(velocity_code.bit58_67<1023)
			           velocity_polar_coordinates=7.408*(velocity_code.bit58_67-1);
                  else
			           velocity_polar_coordinates=7567.272;

				   airplane_velocity_three.E_W_velocity=(int)(velocity_polar_coordinates*sin(0.3515625*velocity_code.bit47_56*PI/180.0)*10000);//Õı±±·´ÏòÎª0¶È£¬Ë³Ê±ÕëĞı×ª£¬ÕıÖµÎª¶«£¬¸ºÖµÎªÎ÷,¾«¶ÈÎª1E-4 
                   airplane_velocity_three.N_S_velocity=(int)(velocity_polar_coordinates*cos(0.3515625*velocity_code.bit47_56*PI/180.0)*10000);//Õı±±·´ÏòÎª0¶È£¬Ë³Ê±ÕëĞı×ª£¬ÕıÖµÎª±±£¬¸ºÖµÎªÄÏ,¾«¶ÈÎª1E-4 
			  }
              if(velocity_code.bit70_78==0)
              {
                   airplane_velocity_three.ICAO_address=airplane_velocity_three.ICAO_address|0x10000000;//airplane_velocity.ICAO_addressÎª32bitµÄÎŞ·ûºÅÕûĞÍ£¬µÍ24´æICAO£¬
			                                                                               // ´¹Ö±·½ÏòËÙ¶ÈÎŞĞ§£¬1¿ªÊ¼±àºÅµÄµÚ29Î»(0¿ªÊ¼±àºÅµÄµÚ28Î»)ÉèÖÃÎª1
				   airplane_velocity_three.VERT_velocity=0; 
              }
              else
		      {
                   if(velocity_code.bit69==0)
	               {
	                   if(velocity_code.bit70_78<511)   
	                        airplane_velocity_three.VERT_velocity=(int)(0.3048*(velocity_code.bit70_78-1)*64*10000);   //ÕıÊıÖµ£¬±í¾ÏòÉÏ£¬ËÙ¶ÈÃ×/·ÖÖÓ,¾«¶ÈÎª1E-4 
				       else
				            airplane_velocity_three.VERT_velocity=99389184;                          //Îª511Ê±£¬ÌØÊâ´¦Àí,ÕıÊıÖµ£¬±íÊ¾ÏòÉÏ£¬ËÙ¶ÈÃ×/·ÖÖÓ,¾«¶ÈÎª1E-4 
	               }
	               else 
			       {
			           if(velocity_code.bit70_78<511) 
			               airplane_velocity_three.VERT_velocity=(int)(-0.3048*(velocity_code.bit70_78-1)*64*10000);  //¸ºÊıÖµ£¬±íÊ¾ÏòÏÂ£¬ËÙ¶ÈÃ×/·ÖÖÓ,¾«¶ÈÎª1E-4 
	                   else
				           airplane_velocity_three.VERT_velocity=-99389184; 
				   }                                 //Îª511Ê±£¬ÌØÊâ´¦Àí,¸ºÊıÖµ£¬±íÊ¾ÏòÏÂ£¬ËÙ¶ÈÃ×/·ÖÖÓ,¾«¶ÈÎª1E-4 
                }
	            return(5);//¿ÕËÙ³¬ÒôËÙÄ£Ê½,Æ½ÃæËÙ¶È¼Ó´¹Ö±ËÙ¶ÈÒÑ×ª»»Îª3Î¬ËÙ¶È£¬·µ»ØÖµ5
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
//*  ÒÔÏÂÎª½â¸ß¶ÈĞÅÏ¢µÄº¯Êı£º decode_altitude()   
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
     airplane_location.ICAO_address=airplane_location.ICAO_address|0x1000000;//µÚ24Î»Îª0,´ú±í¸ß¶ÈÓĞĞ§,Îª1´ú±í¸ß¶ÈÊı¾İÎŞĞ§¡£
     return(0);     
     }
   
    if(altitude_code_GNSS==0)
     {
      altitude_code=altitude_code_BARO;	 
     }
   
    if(altitude_code_BARO==0)
      {
      altitude_code=altitude_code_GNSS;
	  airplane_location.ICAO_address=airplane_location.ICAO_address|0x2000000;//µÚ25Î»Îª0,´ú±íµ±Ç°ÊÇÆøÑ¹¸ß¶È,Îª1£¬´ú±íµ±Ç°ÊÇGNSS¸ß¶È¡£
     }

   if((altitude_code&0x10)==0x10)
    {

      altitude_code_temp_low=altitude_code&0xF;//¸ß¶È±àÂëµÍ4BIT
	  altitude_code_temp_high=(altitude_code>>1)&0x7F0;//¸ß¶È±àÂë¸ß7BIT

	  altitude_code_temp=altitude_code_temp_low|altitude_code_temp_high;////¸ß¶È±àÂë11bit
      altitude_foot=altitude_code_temp*25-1000;//¹«Ê½¼ÆËã¸ß¶ÈÖµ£¬µ¥Î»Ó¢³ß
      airplane_location.altitude=(3048*altitude_foot);//»»ËãÎªÃ×Îªµ¥Î»£¬²¢ÇÒ·Å´ó10000±¶
    }
   else
    {
       airplane_location.ICAO_address=airplane_location.ICAO_address|0x4000000;//µÚ26Î»Îª0,´ú±íµ±Ç°¸ß¶È¾«¶ÈÎª¡À3.81Ã×£¨¡À12.5Ó¢³ß£©,Îª1´ú±íµ±Ç°¸ß¶È¾«¶ÈÎª¡À15.24Ã×£¨¡À50Ó¢³ß£©

      memset(altitude_code_DAB, 0, sizeof(altitude_code_DAB));

      altitude_code_DAB[0]=(altitude_code>>2)&0x1;//ÒÆ¶¯D2
	  altitude_code_DAB[1]=(altitude_code>>0)&0x1;//ÒÆ¶¯D4
      altitude_code_DAB[1]=(altitude_code_DAB[0]^altitude_code_DAB[1])&0x1;//¸ñÀ×ÂëÒëÂë
	  altitude_code_DAB[2]=(altitude_code>>10)&0x1;//ÒÆ¶¯A1
      altitude_code_DAB[2]=(altitude_code_DAB[1]^altitude_code_DAB[2])&0x1;//¸ñÀ×ÂëÒëÂë
      altitude_code_DAB[3]=(altitude_code>>8)&0x1;//ÒÆ¶¯A2
      altitude_code_DAB[3]=(altitude_code_DAB[2]^altitude_code_DAB[3])&0x1;//¸ñÀ×ÂëÒëÂë
      altitude_code_DAB[4]=((altitude_code>>6)&0x08);//ÒÆ¶¯A4
      altitude_code_DAB[4]=(altitude_code_DAB[3]^altitude_code_DAB[4])&0x1;//¸ñÀ×ÂëÒëÂë
      altitude_code_DAB[5]=(altitude_code>>5)&0x1;//ÒÆ¶¯B1
      altitude_code_DAB[5]=(altitude_code_DAB[4]^altitude_code_DAB[5])&0x1;//¸ñÀ×ÂëÒëÂë
      altitude_code_DAB[6]=(altitude_code>>3)&0x1;//ÒÆ¶¯B2
      altitude_code_DAB[6]=(altitude_code_DAB[5]^altitude_code_DAB[6])&0x1;//¸ñÀ×ÂëÒëÂë
	  altitude_code_DAB[7]=(altitude_code>>1)&0x1;//ÒÆ¶¯B4
      altitude_code_DAB[7]=(altitude_code_DAB[6]^altitude_code_DAB[7])&0x1;//¸ñÀ×ÂëÒëÂë

      altitude_decode_DAB=0;
      
      for(i=0;i<8;i++) 
	      altitude_decode_DAB=altitude_decode_DAB|(altitude_code_DAB[i]<<(7-i));//µÃµ½¸ñÀ×ÂëÒëÂëºó¶ÔÓ¦µÄ¶ş½øÖÆÊıÖµ

      altitude_code_temp_C=0;
      altitude_code_temp_C=((altitude_code>>9)&0x04)|altitude_code_temp_C;//ÒÆ¶¯C1
      altitude_code_temp_C=((altitude_code>>8)&0x02)|altitude_code_temp_C;//ÒÆ¶¯C2
	  altitude_code_temp_C=((altitude_code>>7)&0x01)|altitude_code_temp_C;//ÒÆ¶¯C4
      
       if((altitude_decode_DAB%2)==0)//¸ù¾İDAB¸ñÀ×ÂëÒëÂëºóÊıÖµÆæÅ¼½øĞĞCµÄÎåÖÜÆÚÑ­»·ÂëÒëÂë
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
	      airplane_location.altitude=3048*altitude_foot;//»»ËãÎªÃ×Îªµ¥Î»£¬²¢ÇÒ·Å´ó10000±¶
	     }
	   else
	     {
	     airplane_location.altitude=0;
	     airplane_location.ICAO_address=airplane_location.ICAO_address|0x1000000;//µÚ24Î»Îª0,´ú±í¸ß¶ÈÓĞĞ§,Îª1´ú±í¸ß¶ÈÊı¾İÎŞĞ§¡£
                                                                                 //altitude_decode_CÖ»ÄÜÊÇ0-4ÖĞµÄÊı£¬³öÏÖÆäËûÖµ£¬¸ß¶È±àÂë´íÎó£¬¸ß¶ÈÎŞĞ§
	     }
      
	      
    }
	
}

