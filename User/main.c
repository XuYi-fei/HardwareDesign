/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   HC05����ģ����Գ���
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� �Ե� STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
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


extern uint32_t Task_Delay_Group[]; //�˴���Ϊ���ã����ڲ���һ�������
extern int hc05_inquery_connect;
extern int hc05_check_recvbuff;


char sendData[1024];
char linebuff[1024];

char hc05_nameCMD[40];
char disp_buff[200];

uint8_t hc05_role=0;
char hc05_mode[10]="SLAVE";
char hc05_name[30]="xhf";
char * linkhint = "���ȶϿ���������!!!\r\n";
char * linkhint_en = "Please disconnect the Bluetooth connection first";


//���Ժ�������
void CheckConnect_LinkHC05_Test(void);
void CheckRecvBltBuff_Test(void);
void Switch_HC05Mode_Test(void);
void Generate_Modify_HC05Name(void);

/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */

unsigned  char (*letters)[32];
int size;
int main(void)
{
	 int time;
	 letters = letters1;
		size = size1;
	/* ��ʱ������ʼ�� */
  CPU_TS_TmrInit();

  /* LCD��ʼ�� */
  #ifdef ENABLE_LCD_DISPLAY
	ILI9341_Init();	
	ILI9341_GramScan( 6 );
	LCD_SetFont(&Font8x16);
	LCD_SetColors(WHITE,WHITE);
  ILI9341_Clear(0,0,LCD_X_LENGTH,LCD_Y_LENGTH);	/* ��������ʾȫ�� */
  #endif
	
  /* ���Դ��ڳ�ʼ�� USART1 ����ģʽΪ 115200 8-N-1 �����ж� */
	USART_Config();
  /* HC05����ģ���ʼ����GPIO �� USART3 ����ģʽΪ 38400 8-N-1 �����ж� */
	if(HC05_Init() == 0)
	{
		HC05_INFO("HC05ģ����������");
	}
	else
	{
		HC05_ERROR("HC05ģ���ⲻ����������ģ���뿪��������ӣ�Ȼ��λ���������²��ԡ�");
		while(1);
	}

	HC05_INFO("**********HC05ģ��ʵ��************");
	HC05_INFO("��ģ������Ϊ:%s ,ģ����׼��������",hc05_name);
	sprintf(disp_buff,"Device name:%s",hc05_name);
  /* SysTick 10ms�жϳ�ʼ�� */
  SysTick_Init();
	time = 0;
	while(1)
	{

    //ÿ 5s ���һ���������ӣ���������ģ�飬����������
    if( 1 == hc05_inquery_connect )
    {
      hc05_inquery_connect = 0; //�����־λ
      CheckConnect_LinkHC05_Test();
    }
    //���Ӻ�ÿ��һ��ʱ������ջ�����
		if(1 == hc05_check_recvbuff)
		{
      hc05_check_recvbuff = 0; //�����־λ
      
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
  * @brief  �����������
  *         ��Ϊ����ʱ�������������������ֺ��С�HC05��������ģ��
  *         ��Ϊ�ӻ�ʱ�����Ӻ�ͨ������ģ�鷢���ַ���
  * @param  ��
  * @retval ��
  */

void CheckConnect_LinkHC05_Test(void)
{
  {
    static uint8_t testdata=0;
    HC05_INFO("����������");
    //���Ӻ�ÿ��һ��ʱ��ͨ������ģ�鷢���ַ���
    sprintf(sendData,"<%s> send data test,testdata=%d\r\n",hc05_name,testdata++);
    HC05_SendString(sendData);	
  }
}


/**
  * @brief  ����������ջ������������������
  * @param  ��
  * @retval ��
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

    /*��ȡ����*/
    redata = get_rebuff(&len); 
    linelen = get_line(linebuff,redata,len);
  
    /*��������Ƿ��и���*/
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
        /*����ֻ��ʾ��ʾ���е����ݣ��������ʾ���������ݣ���ֱ��ʹ��redata����*/
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
//        Usart_SendString( DEBUG_USARTx, "\r\n����δ���ӡ����յ������������ݣ�\r\n" );
//        redata = get_rebuff(&len); 
//        Usart_SendString( DEBUG_USARTx, (uint8_t *)redata );
//        Usart_SendString( DEBUG_USARTx, "\r\n\r\n" );
//  }
}

/*********************************************END OF FILE**********************/

