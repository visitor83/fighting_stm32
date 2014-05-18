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

#include <rtthread.h>

/* Includes ------------------------------------------------------------------*/

#include "stm32f10x_i2c.h"
#include "stm32f10x.h"


//#define TEA5756_DEBUG		1
#if (TEA5756_DEBUG == 1)
#define TEA5756_TRACE	rt_kprintf
#else
#define TEA5756_TRACE(...)
#endif

/** @addtogroup StdPeriph_Examples
  * @{
  */

/** @addtogroup I2C_M24C08_EEPROM
  * @{
  */ 

#define MHz  (1000000)

#define Tea5767_WriteAddress1    0xc0
#define Tea5767_ReadAddress1     0xc1


/******************************
 * Write mode register values *
 ******************************/

/* First register */
#define TEA5767_MUTE            0x80    /* Mutes output */
#define TEA5767_SEARCH          0x40    /* Activates station search */
/* Bits 0-5 for divider MSB */

/* Second register */
/* Bits 0-7 for divider LSB */

/* Third register */

/* Station search from botton to up */
#define TEA5767_SEARCH_UP       0x80

/* Searches with ADC output = 10 */
#define TEA5767_SRCH_HIGH_LVL   0x60

/* Searches with ADC output = 10 */
#define TEA5767_SRCH_MID_LVL    0x40

/* Searches with ADC output = 5 */
#define TEA5767_SRCH_LOW_LVL    0x20

/* if on, div=4*(Frf+Fif)/Fref otherwise, div=4*(Frf-Fif)/Freq) */
#define TEA5767_HIGH_LO_INJECT  0x10

/* Disable stereo */
#define TEA5767_MONO            0x08

/* Disable right channel and turns to mono */
#define TEA5767_MUTE_RIGHT      0x04

/* Disable left channel and turns to mono */
#define TEA5767_MUTE_LEFT       0x02

#define TEA5767_PORT1_HIGH      0x01

/* Fourth register */
#define TEA5767_PORT2_HIGH      0x80
/* Chips stops working. Only I2C bus remains on */
#define TEA5767_STDBY           0x40

/* Japan freq (76-108 MHz. If disabled, 87.5-108 MHz */
#define TEA5767_JAPAN_BAND      0x20

/* Unselected means 32.768 KHz freq as reference. Otherwise Xtal at 13 MHz */
#define TEA5767_XTAL_32768      0x10

/* Cuts weak signals */
#define TEA5767_SOFT_MUTE       0x08

/* Activates high cut control */
#define TEA5767_HIGH_CUT_CTRL   0x04

/* Activates stereo noise control */
#define TEA5767_ST_NOISE_CTL    0x02

/* If activate PORT 1 indicates SEARCH or else it is used as PORT1 */
#define TEA5767_SRCH_IND        0x01

/* Fifth register */

/* By activating, it will use Xtal at 13 MHz as reference for divider */
#define TEA5767_PLLREF_ENABLE   0x80

/* By activating, deemphasis=50, or else, deemphasis of 50us */
#define TEA5767_DEEMPH_75       0X40

/*****************************
 * Read mode register values *
 *****************************/

/* First register */
#define TEA5767_READY_FLAG_MASK 0x80
#define TEA5767_BAND_LIMIT_MASK 0X40
/* Bits 0-5 for divider MSB after search or preset */

/* Second register */
/* Bits 0-7 for divider LSB after search or preset */

/* Third register */
#define TEA5767_STEREO_MASK     0x80
#define TEA5767_IF_CNTR_MASK    0x7f

/* Fourth register */
#define TEA5767_ADC_LEVEL_MASK  0xf0

/* should be 0 */
#define TEA5767_CHIP_ID_MASK    0x0f

/* Fifth register */
/* Reserved for future extensions */
#define TEA5767_RESERVED_MASK   0xff


enum tea5756_mode 
{
     TEA5756_IDLE = 0, TEA5756_SEARCH = 0, TEA5756_PRESET, TEA5756_MODE_MAX
};

#define TEA5756_FM_CHANNEL_MAX  50
typedef struct _tea5756_device {
    enum tea5756_mode  operMode;
    rt_uint32_t FM_FREQ;         //默认西安交通广播98.8MHz
    //unsigned long   FM_FREQ=91600000;
    rt_uint32_t FM_PLL;
    rt_uint8_t PLL_HIGH;
    rt_uint8_t PLL_LOW;
    rt_uint32_t fmCh[TEA5756_FM_CHANNEL_MAX];

    struct rt_semaphore sem_ack, sem_lock;
} tea5756_device_st;

static tea5756_device_st tea5756_device;
static rt_uint8_t Tx1_Buffer[] = {0XF0,0X2C,0XD0, TEA5767_XTAL_32768 << 4 | 0x02,0X40};
static rt_uint8_t Rx1_Buffer[] = {0XF0,0X2C,0XD0,0X12,0X40};
static rt_uint8_t Tx2_Buffer[] = {0X00,0Xc0,0xe0,0X41,0X6e,0X7e};


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
rt_bool_t I2C_WriteByte(u8 SendByte, u16 WriteAddress, u8 DeviceAddress);
rt_bool_t I2C_BufferWrite(u8* pBuffer, u8 length,     u16 WriteAddress, u8 DeviceAddress);
void I2C_PageWrite(  u8* pBuffer, u8 length,     u16 WriteAddress, u8 DeviceAddress);
rt_bool_t I2C_ReadByte(u8* pBuffer,   u8 length,   u8 DeviceAddress);

void I2C_delay(void);
rt_bool_t I2C_Start(void);

void I2C_Stop(void);

void I2C_Ack(void);

void I2C_NoAck(void);

rt_bool_t I2C_WaitAck(void);
void I2C_SendByte(u8 SendByte);
//extern void Delay(__IO uint32_t nCount);


/* Private function prototypes -----------------------------------------------*/
static void GPIO_Configuration(void);
static void I2C_Configuration(void);


static void delay_ms(rt_uint32_t ms)
{
    rt_uint32_t len;
    for (; ms > 0; ms --)
        for (len = 0; len < 100; len++ );
}


/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Initializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval : None
  */
rt_bool_t I2C_Write(rt_uint8_t* pBuffer, rt_uint8_t WriteAddr, rt_uint8_t NumByteToWrite)
{
    if(!I2C_Start())return RT_FALSE;
    I2C_SendByte(WriteAddr);//器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return RT_FALSE;}
    while(NumByteToWrite--)
    {
        I2C_SendByte(* pBuffer);
        I2C_WaitAck();
        pBuffer++;
    }
    I2C_Stop(); 	
    return RT_TRUE;
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

rt_bool_t I2C_Start(void)
{
	SDA_H;
	SCL_H;
	I2C_delay();
	if(!SDA_read)return RT_FALSE;	//SDA线为低电平则总线忙,退出
	SDA_L;
	I2C_delay();
	if(SDA_read) return RT_FALSE;	//SDA线为高电平则总线出错,退出
	SDA_L;
	I2C_delay();
	return RT_TRUE;
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

rt_bool_t I2C_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
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
        return RT_FALSE;
    }
    SCL_L;
    return RT_TRUE;
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
rt_bool_t I2C_WriteByte(u8 SendByte, u16 WriteAddress, u8 DeviceAddress)
{		
    if(!I2C_Start())return RT_FALSE;
    I2C_SendByte(WriteAddress);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return RT_FALSE;}
    I2C_SendByte((u8)(WriteAddress & 0x00FF));   //设置低起始地址      
    I2C_WaitAck();	
    I2C_SendByte(SendByte);
    I2C_WaitAck();   
    I2C_Stop(); 
	//注意：因为这里要等待EEPROM写完，可以采用查询或延时方式(10ms)
    //Systick_Delay_1ms(10);
    return RT_TRUE;
}

rt_bool_t I2C_BufferWrite(u8* pBuffer, u8 length,     u16 WriteAddress, u8 DeviceAddress)
{
    if(!I2C_Start())return RT_FALSE;
    I2C_SendByte(((WriteAddress & 0x0700) >>7) | DeviceAddress & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return RT_FALSE;}
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
    return RT_TRUE;
}

//读出1串数据         存放读出数据  待读出长度   器件类型(24c16或SD2403)	
rt_bool_t I2C_ReadByte(u8* pBuffer,   u8 length,   u8 DeviceAddress)
{		
    if(!I2C_Start())return RT_FALSE;
    I2C_SendByte(DeviceAddress);//器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return RT_FALSE;}

    while(length--)
    {
        *pBuffer = I2C_ReceiveByte();
        if(length == 1)I2C_NoAck();
        else I2C_Ack(); 
        pBuffer++;

    }
    I2C_Stop(); 	
    return RT_TRUE;
}

#if 0
static void SetPLL(void)
{
   //FM_PLL=(unsigned long)((4000*(FM_FREQ/1000+225))/32768); 	//计算ＰＬＬ值
   FM_PLL=(unsigned long)((4000*(FM_FREQ/1000+225))/32768); 	//计算ＰＬＬ值
   if(rec_f==2) PLL_HIGH=(unsigned char)(((FM_PLL >> 8)&0X3f)|0xc0);	 //PLL高字节值
   else PLL_HIGH=(unsigned char)((FM_PLL >> 8)&0X3f);	 //PLL高字节值
   //Tx1_Buffer[0]=(Tx1_Buffer[0]&0XC0)|PLL_HIGH;		 //I2C第一字节值
   Tx1_Buffer[0]=PLL_HIGH;		 //I2C第一字节值
   PLL_LOW=(unsigned char)FM_PLL;			      		 //PLL低字节值
   Tx1_Buffer[1]= PLL_LOW;						 //I2C第二字节值
   I2C_Write(Tx1_Buffer, Tea5767_WriteAddress1, 5); 
}
#endif

/**
 * @brief 
 *
 * @param freq Unit MHz
 */
void tea5756_tune_pll(unsigned long freq, enum tea5756_mode opermode)
{
    /*
     * fRef = 32768 == 32.768KHz for 32.768kHz crystal
     */
    rt_uint32_t  fRef = 32768, fIf =  225;     
    rt_uint8_t buffer[] = {0XF0,0X2C,0XD0, 0x12,0X40};
    rt_uint32_t  pll;

    //_sem_take(&tea5756_device.sem_lock, RT_WAITING_FOREVER);
    pll =(rt_uint32_t)((4000 *(freq * 1000 + fIf))/fRef);

    if(opermode ==  TEA5756_SEARCH) {
        buffer[0] = (rt_uint8_t)((pll >> 8) & 0x3f | (TEA5767_MUTE | TEA5767_SEARCH));
    } else {
        buffer[0] = (rt_uint8_t)((pll >> 8) & 0x3f);
    }
    buffer[1]= (rt_uint8_t) (pll & 0xff);                       //I2C第二字节值
    I2C_Write(&buffer[0], Tea5767_WriteAddress1, 5); 
    //_sem_release(&tea5756_device.sem_lock);
}

void tea5756_fix_freq(rt_uint32_t freq_MHz)
{
    tea5756_tune_pll(freq_MHz, TEA5756_PRESET);
    //USART_OUT(USART1,&TxBuffer1[0],len+2);
    TEA5756_TRACE("\r 当前FM频率是:   %g\n MHz \n",a); 
}

void tea5756_auto_search()
{
    int ch1, ch2;
    unsigned long freq, fmCh[TEA5756_FM_CHANNEL_MAX];
    float a;

    //USART_OUT(USART1,"Search......",15+2);
    printf("\n 搜索FM节目! \n");
    I2C_Write(Tx1_Buffer, Tea5767_WriteAddress1, 5); 
    //SetPLL();
    //Delay(0xffffff);
    //delay_ms(0x6fffff);
    Tx1_Buffer[0] = 0xf0;
    freq = 87500000;
    freq = 87500000 + 100000;
    //SetPLL();
    tea5756_tune_pll(freq, TEA5756_IDLE);
    
    I2C_ReadByte(Rx1_Buffer,5,Tea5767_ReadAddress1);
    ch2=0;
    while(1){
fm_pub:;
    //IF<51  IF>55  LEVEL<9
        freq = freq + 100000;
        if(freq > 108000000)  { freq = 98800000; break;}  
        //SetPLL();
        tea5756_tune_pll(freq, TEA5756_SEARCH);
        
        I2C_ReadByte(Rx1_Buffer,5,Tea5767_ReadAddress1);                
        //a = freq;
        //if((Rx1_Buffer[0]&0x3f)!=(Tx1_Buffer[0]&0x3f)||(Rx1_Buffer[1]!=Tx1_Buffer[1])||Rx1_Buffer[2]<51||Rx1_Buffer[2]>=55||(Rx1_Buffer[3]>>4)<9){    
        if((Rx1_Buffer[0]&0x3f)!=(Tx1_Buffer[0]&0x3f)||(Rx1_Buffer[1]!=Tx1_Buffer[1])||(Rx1_Buffer[1]&0x80!=0x80)||Rx1_Buffer[2]<50||Rx1_Buffer[2]>=56||(Rx1_Buffer[3]>>4)<7||(Rx1_Buffer[3]>>4)>14){
            //printf("\n 当前FM频率是:   %g MHz     有信号!!!  %u  %u\n",a,Rx1_Buffer[2],Rx1_Buffer[3]>>4);
            printf("\n 当前FM频率是:   %u MHz \n", freq); 
            //printf("\n 当前FM频率是:   %g MHz     无信号!!!    %u  %u\n",a, Rx1_Buffer[2],Rx1_Buffer[3]>>4);              
        }
        else {                  
            //printf("\n 当前FM频率是:   %g MHz     有信号!!!  %u    %u  %u\n",a,Rx1_Buffer[1], Rx1_Buffer[2],Rx1_Buffer[3]>>4);
            printf("\n 当前FM频率是:   %u MHz     有信号!!!    %u  %u\r\n", 
                freq, Rx1_Buffer[2],Rx1_Buffer[3]>>4);
            fmCh[ch2]= freq;                    
            ch2++;
            continue;
        }
        
        
    }
    if(freq != 98800000) 
        goto fm_pub;


    printf("\r\n 有效的FM频率总共有:   %u 个 \n",ch2); 
    ch1 = ch2;
    while(ch1--){
        a = fmCh[ch1];
        printf("\r\n %u  FM频率: %u  MHz \n",ch1, fmCh[ch1]); 
    }
}

void __tea5756_auto_search()
{
    int ch1, ch2;
    float a, fmreq, fmCh[TEA5756_FM_CHANNEL_MAX];
    //USART_OUT(USART1,"Search......",15+2);
    printf("\n 搜索FM节目! \n");
    Tx1_Buffer[0] = 0XF0; 
    //I2C_Write(Tx1_Buffer, Tea5767_WriteAddress1, 5); 
    //Delay(0xffffff);
    fmreq = 87.5f;
    fmreq = fmreq + 0.1f;             

    tea5756_tune_pll(fmreq, TEA5756_SEARCH);
    delay_ms(0x6fffff);
    I2C_ReadByte(Rx1_Buffer, 5, Tea5767_ReadAddress1);

    ch2=0;
    while(1){
    //IF<51  IF>55  LEVEL<9
        fmreq = fmreq+ 0.1f;         
        if(fmreq > 108)  { fmreq = 98.8; break;}  

        tea5756_tune_pll(fmreq, TEA5756_SEARCH);
/*         delay_ms(0x0fffff);
 */
        I2C_ReadByte(Rx1_Buffer,5,Tea5767_ReadAddress1);                
        a = fmreq;
        //if((Rx1_Buffer[0]&0x3f)!=(Tx1_Buffer[0]&0x3f)||(Rx1_Buffer[1]!=Tx1_Buffer[1])||Rx1_Buffer[2]<51||Rx1_Buffer[2]>=55||(Rx1_Buffer[3]>>4)<9){    
        if((Rx1_Buffer[0]&0x3f)!=(Tx1_Buffer[0]&0x3f)||(Rx1_Buffer[1]!=Tx1_Buffer[1])||(Rx1_Buffer[1]&0x80!=0x80)||Rx1_Buffer[2]<50||Rx1_Buffer[2]>=56||(Rx1_Buffer[3]>>4)<7||(Rx1_Buffer[3]>>4)>14){
            //printf("\n 当前FM频率是:   %g MHz     有信号!!!  %u  %u\n",a,Rx1_Buffer[2],Rx1_Buffer[3]>>4);
            printf("\n 当前FM频率是:   %g MHz \n",a); 
            //printf("\n 当前FM频率是:   %g MHz     无信号!!!    %u  %u\n",a, Rx1_Buffer[2],Rx1_Buffer[3]>>4);              
        }
        else {                  
            //printf("\n 当前FM频率是:   %g MHz     有信号!!!  %u    %u  %u\n",a,Rx1_Buffer[1], Rx1_Buffer[2],Rx1_Buffer[3]>>4);
            printf("\n 当前FM频率是:   %g MHz     有信号!!!    %u  %u\n",a, Rx1_Buffer[2],Rx1_Buffer[3]>>4);
            fmCh[ch2]= fmreq;                    
            ch2++;
            continue;
        }
        
        
    }
/*     if(fmreq != 98.8) 
 *         continue;
 */

    printf("\r\n 有效的FM频率总共有:   %u 个 \n",ch2); 
    ch1 = ch2;
    while(ch1--){
        a = fmCh[ch1];
        printf("\r\n %u  FM频率: %g  MHz \n",ch1,a); 
    }
}


rt_bool_t tea5756_read_status(char *buffer) 
{
    if (buffer == RT_NULL) return RT_FALSE;

    memset(buffer, 0, 5);
    if (5 != I2C_ReadByte(buffer, 5, Tea5767_ReadAddress1)) {
        return RT_FALSE;
    }

    return RT_TRUE;
}

int tea5756_g_rf_signal(rt_uint16_t *strength)
{
   rt_uint8_t buffer[5];

   *strength = 0;
   if (0 == tea5756_read_status(&buffer[0]))
       *strength = (buffer[3] & TEA5767_ADC_LEVEL_MASK) << 8;

   return 0;
}

rt_bool_t tea5756_stereo(const char *buffer)
{
    int stereo = buffer[2] & TEA5767_STEREO_MASK;

    TEA5756_TRACE("Radio ST Get=%02x\n", stereo);

    return stereo ? RT_TRUE : RT_FALSE;
}

void tea5767_status_dump(rt_uint8_t *buffer)
{
    unsigned int pll_div, freq;

    if (TEA5767_READY_FLAG_MASK & buffer[0])
        TEA5756_TRACE("Ready Flag on\n");
    else {
        TEA5756_TRACE("Ready Flag off\n");
    }

    pll_div = (buffer[1] & 0x3f) << 8 | buffer[1];

    /* TEA5767_HIGH_LO32768 */
    freq = (pll_div * 32768 - 700000 - 225000) / 4;
    buffer[0] = (pll_div >> 8) & 0x3f;
    buffer[1] = pll_div & 0xff;

    TEA5756_TRACE("Frequency %d.%03 KHz (divider = 0x%04x)\n", frq / 1000, freq % 1000, div);

    //(TEA5767_STEREO_MASK & buffer[2] ) ? TEA5756_TRACE("Stereo\n") : TEA5756_TRACE("Mono\n");

    TEA5756_TRACE("IF Counter=%d\n", buffer[2] & TEA5767_IF_CNTR_MASK);
    TEA5756_TRACE("ADC Level=%d\n", (buffer[3] & TEA5767_ADC_LEVEL_MASK) >> 4);
    TEA5756_TRACE("Chip ID=%d\n", (buffer[3] & TEA5767_CHIP_ID_MASK));
    TEA5756_TRACE("Reserved=0x%02x\n", (buffer[4] & TEA5767_RESERVED_MASK));
}

int tea5767_standby()
{
    rt_uint8_t buffer[5];
    unsigned div, rc;

    /* set frequency to 87.5 Mhz*/
    div = (87500 * 4 + 700 + 225 + 25) / 50;
    buffer[0] = (div >> 8) & 0x3f;
    buffer[1] = div & 0xff;
    // ? PORT1 
    buffer[2] = TEA5767_PORT1_HIGH;
    buffer[3] = TEA5767_PORT2_HIGH | TEA5767_HIGH_CUT_CTRL |
        TEA5767_ST_NOISE_CTL | TEA5767_JAPAN_BAND | TEA5767_STDBY;
    buffer[4] = 0;

    I2C_Write(buffer, I2C1_SLAVE_ADDRESS7, 5);
    return 0;
}


/**
  * @brief  Configure the used I/O ports pin
  * @param  None
  * @retval : None
  */
static void GPIO_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | 
                         RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE, ENABLE);
  /* Configure I2C1 pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;		 //LCD 背光控制
  //GPIO_Init(GPIOD, &GPIO_InitStructure);
  //GPIO_SetBits(GPIOD, GPIO_Pin_13);	   	 	

  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;		 //LCD 背光控制
  //GPIO_Init(GPIOD, &GPIO_InitStructure);
  //GPIO_SetBits(GPIOD, GPIO_Pin_13);	   	 	
}

/**
  * @brief  I2C Configuration
  * @param  None
  * @retval : None
  */
static void I2C_Configuration(void)
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

static void RCC_Configuration(void)
{
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
     initialize the PLL and update the SystemFrequency variable. */
  
/* Enable peripheral clocks --------------------------------------------------*/
  /* GPIOB Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);
  /* I2C1 Periph clock enable */
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
}


/**
 * @brief Initializes tea5756 hardware configuration
 */
void tea5756_hardware_init()
{
    RCC_Configuration();
    GPIO_Configuration();
 //   I2C_Configuration();

/*
    tea5756_device.FM_FREQ = 988 * FREQ_1MHz;
    tea5756_device.FM_PLL = 0;
    tea5756_device.PLL_HIGH= tea5756_device.PLL_LOW = 0;
    tea5756_device.operMode = TEA5756_IDLE;
*/    
 //   rt_sem_init(&tea5756_device.sem_lock, "dev_lock", 1, RT_IPC_FLAG_FIFO);

	tea5756_auto_search();
}



