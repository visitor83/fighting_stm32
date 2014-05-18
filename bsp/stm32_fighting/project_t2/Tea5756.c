/**
  ******************************************************************************
  * @file I2C/M24C08_EEPROM/i2c_ee.c 
  * @author  MCD Application Team
  * @version  V3.0.0
  * @date  04/06/2009
  * @brief  This file provides a set of functions needed to manage the
  *         communication between I2C peripheral and I2C M24C08 EEPROM.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include <rtthread.h>
#include "stm32f10x_i2c.h"
#include "stm32f10x.h"
/** @addtogroup StdPeriph_Examples
  * @{
  */

/** @addtogroup I2C_M24C08_EEPROM
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2C_Speed              400000
#define I2C1_SLAVE_ADDRESS7    0xA0
#define I2C_PageSize           16

#define SCL_H         GPIOB->BSRR = GPIO_Pin_10
#define SCL_L         GPIOB->BRR  = GPIO_Pin_10 
   
#define SDA_H         GPIOB->BSRR = GPIO_Pin_11
#define SDA_L         GPIOB->BRR  = GPIO_Pin_11

#define SCL_read      GPIOB->IDR  & GPIO_Pin_10
#define SDA_read      GPIOB->IDR  & GPIO_Pin_11

void I2C_GPIO_Config(void);
bool I2C_WriteByte(u8 SendByte, u16 WriteAddress, u8 DeviceAddress);
bool I2C_BufferWrite(u8* pBuffer, u8 length,     u16 WriteAddress, u8 DeviceAddress);
void I2C_PageWrite(  u8* pBuffer, u8 length,     u16 WriteAddress, u8 DeviceAddress);
bool I2C_ReadByte(u8* pBuffer,   u8 length,   u8 DeviceAddress);

void I2C_delay(void);
bool I2C_Start(void);

void I2C_Stop(void);

void I2C_Ack(void);

void I2C_NoAck(void);

bool I2C_WaitAck(void);
void I2C_SendByte(u8 SendByte);
extern void Delay(__IO uint32_t nCount);


/* Private function prototypes -----------------------------------------------*/
//void GPIO_Configuration(void);
void I2C_Configuration(void);



/* Private functions ---------------------------------------------------------*/

/**
  * @brief  I2C Configuration
  * @param  None
  * @retval : None
  */
void I2C_Configuration(void)
{
  I2C_InitTypeDef  I2C_InitStructure; 

  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C1_SLAVE_ADDRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;
  
  /* I2C Peripheral Enable */
  I2C_Cmd(I2C1, ENABLE);
  /* Apply I2C configuration after enabling it */
  I2C_Init(I2C1, &I2C_InitStructure);
}

/**
  * @brief  Initializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval : None
  */


bool I2C_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
{
  	if(!I2C_Start())return FALSE;
    I2C_SendByte(WriteAddr);//器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
		while(NumByteToWrite--)
		{
		  I2C_SendByte(* pBuffer);
		  I2C_WaitAck();
          pBuffer++;
		}
	  I2C_Stop(); 	
	  return TRUE;
}

/**
  * @brief  Reads a block of data from the EEPROM.
  * @param pBuffer : pointer to the buffer that receives the data read 
  *   from the EEPROM.
  * @param ReadAddr : EEPROM's internal address to read from.
  * @param NumByteToRead : number of bytes to read from the EEPROM.
  * @retval : None
  */

void I2C_delay(void)
{	
   u8 i=100; //这里可以优化速度	，经测试最低到5还能写入
   while(i) 
   { 
     i--; 
   } 
}

bool I2C_Start(void)
{
	SDA_H;
	SCL_H;
	I2C_delay();
	if(!SDA_read)return FALSE;	//SDA线为低电平则总线忙,退出
	SDA_L;
	I2C_delay();
	if(SDA_read) return FALSE;	//SDA线为高电平则总线出错,退出
	SDA_L;
	I2C_delay();
	return TRUE;
}

void I2C_Stop(void)
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
}

void I2C_Ack(void)
{	
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}

void I2C_NoAck(void)
{	
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}

bool I2C_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
{
	SCL_L;
	I2C_delay();
	SDA_H;			
	I2C_delay();
	SCL_H;
	I2C_delay();
	if(SDA_read)
	{
      SCL_L;
      return FALSE;
	}
	SCL_L;
	return TRUE;
}

void I2C_SendByte(u8 SendByte) //数据从高位到低位//
{
    u8 i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
      if(SendByte&0x80)
        SDA_H;  
      else 
        SDA_L;   
        SendByte<<=1;
        I2C_delay();
		SCL_H;
        I2C_delay();
    }
    SCL_L;
}

u8 I2C_ReceiveByte(void)  //数据从高位到低位//
{ 
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      I2C_delay();
	  SCL_H;
      I2C_delay();	
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;
    return ReceiveByte;
}

//写入1字节数据       待写入数据    待写入地址       器件类型(24c16或SD2403)
bool I2C_WriteByte(u8 SendByte, u16 WriteAddress, u8 DeviceAddress)
{		
    if(!I2C_Start())return FALSE;
    I2C_SendByte(WriteAddress);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
    I2C_SendByte((u8)(WriteAddress & 0x00FF));   //设置低起始地址      
    I2C_WaitAck();	
    I2C_SendByte(SendByte);
    I2C_WaitAck();   
    I2C_Stop(); 
	//注意：因为这里要等待EEPROM写完，可以采用查询或延时方式(10ms)
    //Systick_Delay_1ms(10);
    return TRUE;
}

bool I2C_BufferWrite(u8* pBuffer, u8 length,     u16 WriteAddress, u8 DeviceAddress)
{
    if(!I2C_Start())return FALSE;
    I2C_SendByte(((WriteAddress & 0x0700) >>7) | DeviceAddress & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
    I2C_SendByte((u8)(WriteAddress & 0x00FF));   //设置低起始地址      
	  I2C_WaitAck();	
	  
		while(length--)
		{
		  I2C_SendByte(* pBuffer);
		  I2C_WaitAck();
          pBuffer++;
		}
	  I2C_Stop();
	  //注意：因为这里要等待EEPROM写完，可以采用查询或延时方式(10ms)
	  //Systick_Delay_1ms(10);
	  return TRUE;
}

//读出1串数据         存放读出数据  待读出长度   器件类型(24c16或SD2403)	
bool I2C_ReadByte(u8* pBuffer,   u8 length,   u8 DeviceAddress)
{		
    if(!I2C_Start())return FALSE;
    I2C_SendByte(DeviceAddress);//器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
	    
		while(length--)
		{
		  *pBuffer = I2C_ReceiveByte();
     	  if(length == 1)I2C_NoAck();
     	  else I2C_Ack(); 
          pBuffer++;
        
		}
	  I2C_Stop(); 	
	  return TRUE;
}

void tea5756_setpll(unsigned long freq, int opermode)
{
    /*
     * fRef = 32768 == 32.768KHz for 32.768kHz crystal
     */
    rt_uint32_t  fRef = 32768, fIf =  225;     
    rt_uint8_t buffer[] = {0XF0,0X2C,0XD0, 0x12,0X40};
    rt_uint32_t  pll;

//  rt_sem_take(&tea5756_device.sem_lock, RT_WAITING_FOREVER);
    pll =(rt_uint32_t)((4000 *(freq * 1000 + fIf))/fRef);

    if(opermode ==  2) {
        buffer[0] = (rt_uint8_t)((pll >> 8) & 0x3f | (0x80 | 0x40));
    } else {
        buffer[0] = (rt_uint8_t)((pll >> 8) & 0x3f);
    }
    buffer[1]= (rt_uint8_t) (pll & 0xff);                       //I2C第二字节值
    I2C_Write(&buffer[0], 0xc0, 5); 
//  rt_sem_release(&tea5756_device.sem_lock);
}



