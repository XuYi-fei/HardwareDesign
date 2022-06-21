/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   HC05蓝牙模块测试程序
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 霸道 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
 
 
#include "stm32f10x.h"
#include "./usart/bsp_usart.h"
#include "./usart/bsp_usart_blt.h"
#include "./systick/bsp_SysTick.h"
#include "./hc05/bsp_hc05.h"
#include "./led/bsp_led.h"
#include "./key/bsp_key.h" 
#include "./lcd/bsp_ili9341_lcd.h"
#include <string.h>
#include <stdlib.h>
#include "./dwt_delay/core_delay.h"   
#include "letters.h"
#define ENABLE_LCD_DISPLAY 1


extern uint32_t Task_Delay_Group[]; //此处作为他用：用于产生一个随机数
extern int hc05_inquery_connect;
extern int hc05_check_recvbuff;


char sendData[1024];
char linebuff[1024];

char hc05_nameCMD[40];
char disp_buff[200];

uint8_t hc05_role=0;
char hc05_mode[10]="SLAVE";
char hc05_name[30]="xhf";
char * linkhint = "请先断开蓝牙连接!!!\r\n";
char * linkhint_en = "Please disconnect the Bluetooth connection first";


//测试函数声明
void CheckConnect_LinkHC05_Test(void);
void CheckRecvBltBuff_Test(void);
void Switch_HC05Mode_Test(void);
void Generate_Modify_HC05Name(void);

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */

unsigned  char (*letters)[32];
int size;
int main(void)
{
	 int time;
	 letters = letters1;
		size = size1;
	/* 延时函数初始化 */
  CPU_TS_TmrInit();

  /* LCD初始化 */
  #ifdef ENABLE_LCD_DISPLAY
	ILI9341_Init();	
	ILI9341_GramScan( 6 );
	LCD_SetFont(&Font8x16);
	LCD_SetColors(WHITE,WHITE);
  ILI9341_Clear(0,0,LCD_X_LENGTH,LCD_Y_LENGTH);	/* 清屏，显示全黑 */
  #endif
	
  /* 调试串口初始化 USART1 配置模式为 115200 8-N-1 接收中断 */
	USART_Config();
  /* HC05蓝牙模块初始化：GPIO 和 USART3 配置模式为 38400 8-N-1 接收中断 */
	if(HC05_Init() == 0)
	{
		HC05_INFO("HC05模块检测正常。");
	}
	else
	{
		HC05_ERROR("HC05模块检测不正常，请检查模块与开发板的连接，然后复位开发板重新测试。");
		while(1);
	}

	HC05_INFO("**********HC05模块实验************");
	HC05_INFO("本模块名字为:%s ,模块已准备就绪。",hc05_name);
	sprintf(disp_buff,"Device name:%s",hc05_name);
  /* SysTick 10ms中断初始化 */
  SysTick_Init();
	time = 0;
	while(1)
	{

    //每 5s 检查一次蓝牙连接，搜索蓝牙模块，并进行连接
    if( 1 == hc05_inquery_connect )
    {
      hc05_inquery_connect = 0; //清零标志位
      CheckConnect_LinkHC05_Test();
    }
    //连接后每隔一段时间检查接收缓冲区
		if(1 == hc05_check_recvbuff)
		{
      hc05_check_recvbuff = 0; //清零标志位
      
      CheckRecvBltBuff_Test();
		}
		time ++;
		if(time % 10000 == 0){
			time = 0;
			Chinese_Show(letters, size);
		}
	}
}

/**
  * @brief  检查蓝牙连接
  *         作为主机时，搜索蓝牙并连接名字含有“HC05”的蓝牙模块
  *         作为从机时，连接后通过蓝牙模块发送字符串
  * @param  无
  * @retval 无
  */

void CheckConnect_LinkHC05_Test(void)
{
  {
    static uint8_t testdata=0;
    HC05_INFO("蓝牙已连接");
    //连接后每隔一段时间通过蓝牙模块发送字符串
    sprintf(sendData,"<%s> send data test,testdata=%d\r\n",hc05_name,testdata++);
    HC05_SendString(sendData);	
  }
}


/**
  * @brief  检查蓝牙接收缓冲区、处理接收数据
  * @param  无
  * @retval 无
  */
void CheckRecvBltBuff_Test(void)
{
  char* redata;
	uint16_t len;
	char c;
	int i;
  if( IS_HC05_CONNECTED() )
  {
    uint16_t linelen;

    /*获取数据*/
    redata = get_rebuff(&len); 
    linelen = get_line(linebuff,redata,len);
  
    /*检查数据是否有更新*/
    if(linelen<200 && linelen != 0)
    {
      if(strcmp(redata,"AT+LED1=ON")==0)
      {
        LED1_ON;						
        HC05_SendString("+LED1:ON\r\nOK\r\n");	
      }
      else if(strcmp(redata,"AT+LED1=OFF")==0)
      {
        LED1_OFF;
        HC05_SendString("+LED1:OFF\r\nOK\r\n");
      }
      else
      {
        /*这里只演示显示单行的数据，如果想显示完整的数据，可直接使用redata数组*/
        HC05_INFO("receive:\r\n%s",linebuff);
				c = linebuff[i];
				switch(c){
					case '0':{
						mode = 0; break;
					}
					case '1':{
						mode = 1; break;	
					}
					case '2':{
						mode = 2; break;
					}
					case '3':{
						letters = letters1; size = size1; break;
					}
					case '4':{
						letters = letters2; size = size2; break;
					}
					case '5':{
						letters = letters3; size = size3; break;
					}
					case '6':{
						letters = letters4; size = size4; break;
					}
					case '7':{
						letters = letters5; size = size5; break;
					}
					case '8':{
						letters = letters6; size = size6; break;
					}
					default:
						break;
				}
        ILI9341_Clear(0,120,240,200);
        LCD_SetColors(RED,WHITE);
        ILI9341_DispString_EN( 5, 120,"Choice :" );
        LCD_SetColors(BLUE,WHITE);
        ILI9341_DispString_EN( 5, 140,linebuff );
      }
      clean_rebuff();    
    }
  }

//  else
//  {
//        Usart_SendString( DEBUG_USARTx, "\r\n蓝牙未连接。接收到蓝牙返回数据：\r\n" );
//        redata = get_rebuff(&len); 
//        Usart_SendString( DEBUG_USARTx, (uint8_t *)redata );
//        Usart_SendString( DEBUG_USARTx, "\r\n\r\n" );
//  }
}

/*********************************************END OF FILE**********************/

