//#include "includes.h"
#include <STM32F10X_SPI.h>
#include <STM32F10X_GPIO.h>
#include <STM32F10X_RCC.h>


static void GPIO_Configuration(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 |RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE, ENABLE);
  	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				     //LED1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);					 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_3;		 //LED2, LED3
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;		
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_7;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_ResetBits(GPIOE, GPIO_Pin_0);	   
  //GPIO_SetBits(GPIOE, GPIO_Pin_1);
  GPIO_ResetBits(GPIOE, GPIO_Pin_1);
  //Delay(0xAFFF);					   
  GPIO_SetBits(GPIOE, GPIO_Pin_1 );		 	 
  //Delay(0xAFFF);		  	 	

  
  GPIO_SetBits(GPIOC, GPIO_Pin_4);			//SPI CS1
  GPIO_SetBits(GPIOB, GPIO_Pin_12|GPIO_Pin_7);			//SPI CS4   CS3
  //GPIO_SetBits(GPIOA, GPIO_Pin_4);			//SPI NSS
  //GPIO_PinRemapConfig(AFIO_MAPR_USART1_REMAP, ENABLE);
}
static void RCC_Configuration(void)
{
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
     initialize the PLL and update the SystemFrequency variable. */
  //SystemInit();
  
  
}
//SPI1初始化
void	SPI1_Init(void)
	{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

      RCC_Configuration();
       
  

  /* Configure the GPIO ports */
  GPIO_Configuration();

  
	/* Enable SPI1 and GPIOA clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	/* Configure SPI1 pins: NSS, SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//SPI1 NSS 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_SetBits(GPIOA, GPIO_Pin_4);   

	/* SPI1 configuration */ 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	
	/* Enable SPI1  */
	SPI_Cmd(SPI1, ENABLE); 
	}

//SPI1读写一字节数据
unsigned char	SPI1_ReadWrite(unsigned char writedat)
	{
	/* Loop while DR register in not emplty */
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET);
	
	/* Send byte through the SPI1 peripheral */
	SPI_I2S_SendData(SPI1, writedat);
	
	/* Wait to receive a byte */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	
	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI1);
	}

