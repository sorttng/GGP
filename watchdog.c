#include <stdio.h>
#include <netmain.h>
#include "macro.h"
/*****************************************/
void watchdog()  //���Ź� GPIO3
{
	while(1)
	{
	    GP_CLR_DATA = 0x04;//GPIO
		TaskSleep(300);
		GP_SET_DATA = 0x04;
		TaskSleep(300);
	}
}
/*****************************************/