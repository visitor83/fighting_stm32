/*
 * File      : enc28j60.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-05-05     Bernard      the first version
 */
#include "enc28j60.h"

#include <netif/ethernetif.h>
#include <stm32f10x.h>
#include <stm32f10x_spi.h>

#define MAX_ADDR_LEN    6

#define 	ENC28J60_CS	 	GPIO_Pin_4
#define 	ENC28J60_CS_L		GPIOA->BRR = ENC28J60_CS;
#define 	ENC28J60_CS_H		GPIOA->BSRR = ENC28J60_CS;

#define FILTER_PROMISC 0x00

#define ENC28J60_DBG    rt_kprintf

/* ENC28J60 Receive Status Vector */
#define RSV_RXLONGEVDROPEV	16
#define RSV_CARRIEREV		18
#define RSV_CRCERROR		20
#define RSV_LENCHECKERR		21
#define RSV_LENOUTOFRANGE	22
#define RSV_RXOK		23
#define RSV_RXMULTICAST		24
#define RSV_RXBROADCAST		25
#define RSV_DRIBBLENIBBLE	26
#define RSV_RXCONTROLFRAME	27
#define RSV_RXPAUSEFRAME	28
#define RSV_RXUNKNOWNOPCODE	29
#define RSV_RXTYPEVLAN		30

#define RSV_SIZE		6
#define RSV_BITMASK(x)		(1 << ((x) - 16))
#define RSV_GETBIT(x, y)	(((x) & RSV_BITMASK(y)) ? 1 : 0)


struct net_device
{
	/* inherit from ethernet device */
	struct eth_device parent;

	/* interface address info. */
	rt_uint8_t  dev_addr[MAX_ADDR_LEN];			/* hw address	*/
};

static struct net_device  enc28j60_dev_entry;
static struct net_device *enc28j60_dev =&enc28j60_dev_entry;
static rt_uint8_t  Enc28j60Bank;
static rt_uint16_t NextPacketPtr;
static struct rt_semaphore lock_sem;

void _delay_us(rt_uint32_t us)
{
	rt_uint32_t len;
	for (;us > 0; us --)
		for (len = 0; len < 20; len++ );
}

void delay_ms(rt_uint32_t ms)
{
	rt_uint32_t len;
	for (;ms > 0; ms --)
		for (len = 0; len < 100; len++ );
}

static rt_uint8_t spi_read_op(rt_uint8_t op, rt_uint8_t address)
{
	int temp=0;

	ENC28J60_CS_L;
	SPI_I2S_SendData(SPI1, (op | (address & ADDR_MASK)));
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET);
	SPI_I2S_ReceiveData(SPI1);
	SPI_I2S_SendData(SPI1, 0x00);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET);

	// do dummy read if needed (for mac and mii, see datasheet page 29)
	if(address & 0x80)
	{
		SPI_I2S_ReceiveData(SPI1);
		SPI_I2S_SendData(SPI1, 0x00);
		while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET);
	}
	// release CS

	temp=SPI_I2S_ReceiveData(SPI1);
	// for(t=0;t<20;t++);
	ENC28J60_CS_H;
	return (temp);
}

// ����: ����,��ַ,����
static void spi_write_op(rt_uint8_t op, rt_uint8_t address, rt_uint8_t data)
{
	rt_uint32_t level;

	level = rt_hw_interrupt_disable();

	ENC28J60_CS_L;
	SPI_I2S_SendData(SPI1, op | (address & ADDR_MASK));
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET);
	SPI_I2S_SendData(SPI1,data);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET);
	ENC28J60_CS_H;

	rt_hw_interrupt_enable(level);
}

void enc28j60_set_bank(rt_uint8_t address)
{
	// set the bank (if needed)
	if((address & BANK_MASK) != Enc28j60Bank)
	{
		// set the bank
		spi_write_op(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
		spi_write_op(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK)>>5);
		Enc28j60Bank = (address & BANK_MASK);
	}
}

rt_uint8_t spi_read(rt_uint8_t address)
{
	// set the bank
	enc28j60_set_bank(address);
	// do the read
	return spi_read_op(ENC28J60_READ_CTRL_REG, address);
}

static void enc28j60_membuf_read(rt_uint8_t* data, rt_size_t len)
{
	ENC28J60_CS_L;

	SPI_I2S_SendData(SPI1,ENC28J60_READ_BUF_MEM);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET);

	SPI_I2S_ReceiveData(SPI1);

	while(len)
	{
	    len--;
	    SPI_I2S_SendData(SPI1,0x00)	;
	    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET);

	    *data= SPI_I2S_ReceiveData(SPI1);
	    data++;
	}

	ENC28J60_CS_H;
}

void spi_write(rt_uint8_t address, rt_uint8_t data)
{
	// set the bank
	enc28j60_set_bank(address);
	// do the write
	spi_write_op(ENC28J60_WRITE_CTRL_REG, address, data);
}

void enc28j60_phy_write(rt_uint8_t address, rt_uint16_t data)
{
	// set the PHY register address
	spi_write(MIREGADR, address);

	// write the PHY data
	spi_write(MIWRL, data);
	spi_write(MIWRH, data>>8);

	// wait until the PHY write completes
	while(spi_read(MISTAT) & MISTAT_BUSY)
	{
		_delay_us(15);
	}
}

// read upper 8 bits
rt_uint16_t enc28j60_phy_read(rt_uint8_t address)
{
	// Set the right address and start the register read operation
	spi_write(MIREGADR, address);
	spi_write(MICMD, MICMD_MIIRD);

	_delay_us(15);

	// wait until the PHY read completes
	while(spi_read(MISTAT) & MISTAT_BUSY);

	// reset reading bit
	spi_write(MICMD, 0x00);

	return (spi_read(MIRDH));
}

void enc28j60_clkout(rt_uint8_t clk)
{
	//setup clkout: 2 is 12.5MHz:
	spi_write(ECOCON, clk & 0x7);
}

rt_inline rt_uint32_t enc28j60_interrupt_disable()
{
	rt_uint32_t level;

    /* switch to bank 0 */
    enc28j60_set_bank(EIE);

    /* get last interrupt level */
	level = spi_read(EIE);
    /* disable interrutps */
    spi_write_op(ENC28J60_BIT_FIELD_CLR, EIE, level);

    return level;
}

rt_inline void enc28j60_interrupt_enable(rt_uint32_t level)
{
    /* switch to bank 0 */
    enc28j60_set_bank(EIE);
    spi_write_op(ENC28J60_BIT_FIELD_SET, EIE, level);
}

/*
 * Access the PHY to determine link status
 */
static rt_bool_t enc28j60_check_link_status()
{
	rt_uint16_t reg;
	int duplex;

	reg = enc28j60_phy_read(PHSTAT2);
	duplex = reg & PHSTAT2_DPXSTAT;

	if (reg & PHSTAT2_LSTAT)
	{
	    /* on */
        return RT_TRUE;
	}
	else
	{
	    /* off */
        return RT_FALSE;
	}
}

#ifdef RT_USING_FINSH
/*
 * Debug routine to dump useful register contents
 */
static void enc28j60(void)
{
	rt_kprintf("-- enc28j60 registers:\n");
	rt_kprintf("HwRevID: 0x%02x\n", spi_read(EREVID));
	rt_kprintf("Cntrl: ECON1 ECON2 ESTAT  EIR  EIE\n");
	rt_kprintf("       0x%02x  0x%02x  0x%02x  0x%02x  0x%02x\n",spi_read(ECON1), spi_read(ECON2), spi_read(ESTAT), spi_read(EIR), spi_read(EIE));
	rt_kprintf("MAC  : MACON1 MACON3 MACON4\n");
	rt_kprintf("       0x%02x   0x%02x   0x%02x\n", spi_read(MACON1), spi_read(MACON3), spi_read(MACON4));
	rt_kprintf("Rx   : ERXST  ERXND  ERXWRPT ERXRDPT ERXFCON EPKTCNT MAMXFL\n");
	rt_kprintf("       0x%04x 0x%04x 0x%04x  0x%04x  ",
		(spi_read(ERXSTH) << 8) | spi_read(ERXSTL),
		(spi_read(ERXNDH) << 8) | spi_read(ERXNDL),
		(spi_read(ERXWRPTH) << 8) | spi_read(ERXWRPTL),
		(spi_read(ERXRDPTH) << 8) | spi_read(ERXRDPTL));
	rt_kprintf("0x%02x    0x%02x    0x%04x\n", spi_read(ERXFCON), spi_read(EPKTCNT),
		(spi_read(MAMXFLH) << 8) | spi_read(MAMXFLL));

	rt_kprintf("Tx   : ETXST  ETXND  MACLCON1 MACLCON2 MAPHSUP\n");
	rt_kprintf("       0x%04x 0x%04x 0x%02x     0x%02x     0x%02x\n",
		(spi_read(ETXSTH) << 8) | spi_read(ETXSTL),
		(spi_read(ETXNDH) << 8) | spi_read(ETXNDL),
		spi_read(MACLCON1), spi_read(MACLCON2), spi_read(MAPHSUP));
}
#include <finsh.h>
FINSH_FUNCTION_EXPORT(enc28j60, dump enc28j60 registers);
#endif

rt_timer_t strPollTimer;
rt_uint32_t gulPollTimerCnt = 0;
/*****************************************************************************
 �� �� ��  : ENC28J60_TimOut
 ��������  : OS 10 Tick
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��5��16��
    ��    ��   : peter
    �޸�����   : �����ɺ���

*****************************************************************************/
void ENC28J60_TimOut(void)
{
    volatile rt_uint32_t ulPktCnt;
    
    gulPollTimerCnt++;

    ulPktCnt= enc28j60Read(EPKTCNT);
	if (ulPktCnt)
    {
        /* a frame has been received */
        eth_device_ready((struct eth_device*)&(enc28j60_dev->parent));
        //rt_kprintf("x\r\n");

	    // switch to bank 0
	    //enc28j60_set_bank(EIE);
		// disable rx interrutps
		//spi_write_op(ENC28J60_BIT_FIELD_CLR, EIE, EIE_PKTIE);
	}
}


/*****************************************************************************
 �� �� ��  : ENC28J60_PollRoutine
 ��������  : 10 OS tick to emit receive event
 �������  : void
 �������  : ��
 �� �� ֵ  : void
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��5��16��
    ��    ��   : peter
    �޸�����   : �����ɺ���

*****************************************************************************/
void ENC28J60_PollRoutine( void )
{
    strPollTimer = rt_timer_create("tEnc28j60", 
        ENC28J60_TimOut, RT_NULL, 1,  RT_TIMER_FLAG_PERIODIC);
    if (RT_NULL != strPollTimer)
    {
        rt_timer_start(strPollTimer);
    }

    return ;
}


/*****************************************************************************
 �� �� ��  : ENC28J60_PollRx
 ��������  : Poll method Recieve
 �������  : rt_device_t dev  
 �������  : ��
 �� �� ֵ  : struct
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��5��17��
    ��    ��   : peter
    �޸�����   : �����ɺ���

*****************************************************************************/
struct pbuf *ENC28J60_PollRx(rt_device_t dev)
{
    struct pbuf *pBuf = RT_NULL;
    
    enc28j60PacketReceive(1500, pBuf);

    return RT_NULL;
}



/*
 * RX handler
 * ignore PKTIF because is unreliable! (look at the errata datasheet)
 * check EPKTCNT is the suggested workaround.
 * We don't need to clear interrupt flag, automatically done when
 * enc28j60_hw_rx() decrements the packet counter.
 */
void enc28j60_isr()
{
    /* Variable definitions can be made now. */
    volatile rt_uint32_t eir, pk_counter;
    volatile rt_bool_t rx_activiated;

    rx_activiated = RT_FALSE;

    /* get EIR */
    eir = spi_read(EIR);
    //rt_kprintf("\neir: 0x%08x\n", eir);
    do
    {
        /* clear DMAIF */
/*         if (eir & EIR_DMAIF)
 *         {
 *             enc28j60_set_bank(EIR);
 *             spi_write_op(ENC28J60_BIT_FIELD_CLR, EIR, EIR_DMAIF);
 *         }
 */
         /* errata #4, PKTIF does not reliable */
	    pk_counter = spi_read(EPKTCNT);
	    if (pk_counter)
	    {
	        /* a frame has been received */
	        eth_device_ready((struct eth_device*)&(enc28j60_dev->parent));

			// switch to bank 0
			enc28j60_set_bank(EIE);
			// disable rx interrutps
			spi_write_op(ENC28J60_BIT_FIELD_CLR, EIE, EIE_PKTIE);
	    }

		/* clear PKTIF */
		if (eir & EIR_PKTIF)
		{
			enc28j60_set_bank(EIR);
			spi_write_op(ENC28J60_BIT_FIELD_CLR, EIR, EIR_PKTIF);

			rx_activiated = RT_TRUE;
		}


        /* LINK changed handler (not used currently) */
        if ( eir & EIR_LINKIF)
        {
            enc28j60_check_link_status();

            /* read PHIR to clear the flag */
            enc28j60_phy_read(PHIR);

            enc28j60_set_bank(EIR);
            spi_write_op(ENC28J60_BIT_FIELD_CLR, EIR, EIR_LINKIF);
        }

        if (eir & EIR_TXIF)
        {
            /* A frame has been transmitted. */
            enc28j60_set_bank(EIR);
            spi_write_op(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXIF);
        }

        /* TX Error handler */
        if ((eir & EIR_TXERIF) != 0)
        {
            enc28j60_set_bank(ECON1);
            spi_write_op(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
            spi_write_op(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
            enc28j60_set_bank(EIR);
            spi_write_op(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXERIF);
        }

        eir = spi_read(EIR);
        //rt_kprintf("inner eir: 0x%08x\n", eir);
    } while ((rx_activiated != RT_TRUE && eir != 0));
}

/* RT-Thread Device Interface */


/*****************************************************************************
 �� �� ��  : ENC28J60_Init
 ��������  : call demo's initlization and start recieve polling task
 �������  : rt_device_t dev  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��5��17��
    ��    ��   : peter
    �޸�����   : �����ɺ���

*****************************************************************************/
rt_err_t ENC28J60_Init(rt_device_t dev)
{
    rt_uint8_t aucMacAddr[MAX_ADDR_LEN] = {0x04,0x02,0x35,0x00,0x00,0x01};

    enc28j60Init(&(aucMacAddr[0]));

    rt_kprintf("ENC28J60_Init() done\r\n");

    return RT_EOK;
}

/* initialize the interface */
rt_err_t enc28j60_init(rt_device_t dev)
{
    rt_uint8_t ucRegVal;
    ENC28J60_CS_H;

    // perform system reset
    spi_write_op(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
    delay_ms(50);

    // check CLKRDY bit to see if reset is complete 
    // THE CLKRDY does not work. See Rev.B4 Silicon Errata.
    while (!(spi_read(ESTAT) & ESTAT_CLKRDY))   ;
#if 0
    ENC28J60_Init(dev);
#else
    NextPacketPtr = RXSTART_INIT;

    // Rx start
    spi_write(ERXSTL, RXSTART_INIT&0xFF);
    spi_write(ERXSTH, RXSTART_INIT>>8);
    // set receive pointer address
    spi_write(ERXRDPTL, RXSTOP_INIT&0xFF);
    spi_write(ERXRDPTH, RXSTOP_INIT>>8);
    // RX end
    spi_write(ERXNDL, RXSTOP_INIT&0xFF);
    spi_write(ERXNDH, RXSTOP_INIT>>8);

    // TX start
    spi_write(ETXSTL, TXSTART_INIT&0xFF);
    spi_write(ETXSTH, TXSTART_INIT>>8);
    // set transmission pointer address
    spi_write(EWRPTL, TXSTART_INIT&0xFF);
    spi_write(EWRPTH, TXSTART_INIT>>8);
    // TX end
    spi_write(ETXNDL, TXSTOP_INIT&0xFF);
    spi_write(ETXNDH, TXSTOP_INIT>>8);

	// do bank 1 stuff, packet filter:
    // For broadcast packets we allow only ARP packtets
    // All other packets should be unicast only for our mac (MAADR)
    //
    // The pattern to match on is therefore
    // Type     ETH.DST
    // ARP      BROADCAST
    // 06 08 -- ff ff ff ff ff ff -> ip checksum for theses bytes=f7f9
    // in binary these poitions are:11 0000 0011 1111
    // This is hex 303F->EPMM0=0x3f,EPMM1=0x30
    //ucRegVal = spi_read(ERXFCON);
    //ucRegVal &= ~ERXFCON_PMEN;
    spi_write(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_BCEN);
    rt_kprintf("\nERXFCON:%02x", spi_read(ERXFCON));

    // do bank 2 stuff
    // enable MAC receive
    spi_write(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);

    /* ��MACON2 ��MARSTλ����, ʹMAC�˳���λ״̬ */
    spi_write(MACON2, 0x00);

    // enable automatic padding to 60bytes and CRC operations
    // spi_write_op(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
    spi_write_op(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN | MACON3_FULDPX);
    // bring MAC out of reset

    // set inter-frame gap (back-to-back)
    // spi_write(MABBIPG, 0x12);
    spi_write(MABBIPG, 0x15);

    spi_write(MACON4, MACON4_DEFER);
    spi_write(MACLCON2, 63);

    // set inter-frame gap (non-back-to-back)
    spi_write(MAIPGL, 0x12);
    spi_write(MAIPGH, 0x0C);

    // Set the maximum packet size which the controller will accept
    // Do not send packets longer than MAX_FRAMELEN:
    // ���֡�� 1500 B
    spi_write(MAMXFLL, MAX_FRAMELEN&0xFF);
    spi_write(MAMXFLH, MAX_FRAMELEN>>8);

    // do bank 3 stuff
    // write MAC address
    // NOTE: MAC address in ENC28J60 is byte-backward
    spi_write(MAADR0, enc28j60_dev->dev_addr[5]);
    spi_write(MAADR1, enc28j60_dev->dev_addr[4]);
    spi_write(MAADR2, enc28j60_dev->dev_addr[3]);
    spi_write(MAADR3, enc28j60_dev->dev_addr[2]);
    spi_write(MAADR4, enc28j60_dev->dev_addr[1]);
    spi_write(MAADR5, enc28j60_dev->dev_addr[0]);
    if (spi_read(MAADR5) == enc28j60_dev->dev_addr[0])
    {
        rt_kprintf("Setting mac addres 00-04-A3-11-22-33\r\n");
        rt_kprintf("%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
                spi_read(MAADR0), spi_read(MAADR1), spi_read(MAADR2),
                spi_read(MAADR3), spi_read(MAADR4), spi_read(MAADR5));
    }

    /* output off */
    spi_write(ECOCON, 0x00);

    // enc28j60_phy_write(PHCON1, 0x00);
    // ����PHYΪȫ˫��, LEDBλ������
	enc28j60_phy_write(PHCON1, PHCON1_PDPXMD); // full duplex
    // no loopback of transmitted frames
    enc28j60_phy_write(PHCON2, PHCON2_HDLDIS);

    enc28j60_set_bank(ECON2);
    spi_write_op(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_AUTOINC);

    // enable the filters specifed in the 
//    spi_write(ERXFCON, FILTER_PROMISC);
    // switch to bank 0
    // phy interrupt 
    //        enc28j60_phy_write(PHIE, PHIE_PGEIE | PHIE_PLNKIE);


    enc28j60_set_bank(ECON1);
    // enable interrutps
    //spi_write_op(ENC28J60_BIT_FIELD_CLR, EIR, EIE_INTIE|EIE_PKTIE|EIR_TXIF | EIR_RXERIF);
    // �����ж��¼�����INT�� ����������ݰ����жϴ���
	spi_write_op(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE|EIR_TXIF);
    // enable packet reception
    spi_write_op(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);

    enc28j60_phy_write(PHLCON,0x0476);	
	//enc28j60_clkout(2); // change clkout from 6.25MHz to 12.5MHz
    delay_ms(20);
#endif
    rt_kprintf("enc28j60_init() done\r\n");

    //ENC28J60_PollRoutine();
    
    return RT_EOK;
}

/* control the interface */
rt_err_t enc28j60_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	switch(cmd)
	{
	case NIOCTL_GADDR:
		/* get mac address */
		if(args) rt_memcpy(args, enc28j60_dev_entry.dev_addr, 6);
		else return -RT_ERROR;
		break;

	default :
		break;
	}

	return RT_EOK;
}

/* Open the ethernet interface */
rt_err_t enc28j60_open(rt_device_t dev, rt_uint16_t oflag)
{
	return RT_EOK;
}

/* Close the interface */
rt_err_t enc28j60_close(rt_device_t dev)
{
	return RT_EOK;
}

/* Read */
rt_size_t enc28j60_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
	rt_set_errno(-RT_ENOSYS);
	return 0;
}

/* Write */
rt_size_t enc28j60_write(rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
	rt_set_errno(-RT_ENOSYS);
	return 0;
}

/* ethernet device interface */
/*
 * Transmit packet.
 */
rt_err_t enc28j60_tx( rt_device_t dev, struct pbuf* p)
{
    struct pbuf* q;
    rt_uint32_t len;
    rt_uint8_t* ptr;
    rt_uint32_t level;

    rt_kprintf("tx pbuf: 0x%08x, total len %d\n", p, p->tot_len);

    /* lock enc28j60 */
    rt_sem_take(&lock_sem, RT_WAITING_FOREVER);
    /* disable enc28j60 interrupt */
    level = enc28j60_interrupt_disable();

    // Set the write pointer to start of transmit buffer area
    spi_write(EWRPTL, TXSTART_INIT&0xFF);
    spi_write(EWRPTH, TXSTART_INIT>>8);
    // Set the TXND pointer to correspond to the packet size given
    spi_write(ETXNDL, (TXSTART_INIT+ p->tot_len + 1)&0xFF);
    spi_write(ETXNDH, (TXSTART_INIT+ p->tot_len + 1)>>8);

    // write per-packet control byte (0x00 means use macon3 settings)
    spi_write_op(ENC28J60_WRITE_BUF_MEM, 0, 0x00);

    for (q = p; q != NULL; q = q->next)
    {
        ENC28J60_CS_L;

        SPI_I2S_SendData(SPI1, ENC28J60_WRITE_BUF_MEM);
        while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET);

        len = q->len;
        ptr = q->payload;
        while(len)
        {
            SPI_I2S_SendData(SPI1,*ptr) ;
            while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET);;
            ptr++;

            len--;
        }

        ENC28J60_CS_H;
    }

    // send the contents of the transmit buffer onto the network
    spi_write_op(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
    // Reset the transmit logic problem. See Rev. B4 Silicon Errata point 12.
    if( (spi_read(EIR) & EIR_TXERIF) )
    {
        spi_write_op(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
    }

    /* enable enc28j60 interrupt */
    enc28j60_interrupt_enable(level);
    rt_sem_release(&lock_sem);

    return RT_EOK;
}

static void enc28j60_dump_rsv(const char *msg, uint16_t pkt_addr, int len, uint16_t sts)
{
    ENC28J60_DBG(":%s - NextPkt:0x%04x - RSV\n", msg, pkt_addr);

    ENC28J60_DBG( ": ByteCount: %d, DribbleNibble: %d\n", len,
            RSV_GETBIT(sts, RSV_DRIBBLENIBBLE));
    ENC28J60_DBG( ": RxOK: %d, CRCErr:%d, LenChkErr: %d,"
            " LenOutOfRange: %d\n", RSV_GETBIT(sts, RSV_RXOK),
            RSV_GETBIT(sts, RSV_CRCERROR),
            RSV_GETBIT(sts, RSV_LENCHECKERR),
            RSV_GETBIT(sts, RSV_LENOUTOFRANGE));
    ENC28J60_DBG( ": Multicast: %d, Broadcast: %d, "
            "LongDropEvent: %d, CarrierEvent: %d\n",
            RSV_GETBIT(sts, RSV_RXMULTICAST),
            RSV_GETBIT(sts, RSV_RXBROADCAST),
            RSV_GETBIT(sts, RSV_RXLONGEVDROPEV),
            RSV_GETBIT(sts, RSV_CARRIEREV));
    ENC28J60_DBG( ": ControlFrame: %d, PauseFrame: %d,"
            " UnknownOp: %d, VLanTagFrame: %d\n",
            RSV_GETBIT(sts, RSV_RXCONTROLFRAME),
            RSV_GETBIT(sts, RSV_RXPAUSEFRAME),
            RSV_GETBIT(sts, RSV_RXUNKNOWNOPCODE),
            RSV_GETBIT(sts, RSV_RXTYPEVLAN));   
}

struct pbuf *enc28j60_rx(rt_device_t dev)
{
    struct pbuf* p;
    rt_uint16_t usLen;
    rt_uint16_t rxstat;
    rt_uint32_t pk_counter;
    rt_uint32_t level;

    p = RT_NULL;

    //rt_kprintf("In enc28j60_rx()\r\n");

    /* lock enc28j60 */
    rt_sem_take(&lock_sem, RT_WAITING_FOREVER);
    /* disable enc28j60 interrupt */
    level = enc28j60_interrupt_disable();
    
    pk_counter = spi_read(EPKTCNT);
    if (pk_counter)
    {
        // Set the read pointer to the start of the received packet
        spi_write(ERDPTL, (NextPacketPtr));
        spi_write(ERDPTH, (NextPacketPtr)>>8);

        // read the next packet pointer
        NextPacketPtr  = spi_read_op(ENC28J60_READ_BUF_MEM, 0);
        NextPacketPtr |= spi_read_op(ENC28J60_READ_BUF_MEM, 0)<<8;

        // read the packet length (see datasheet page 43)
        usLen  = spi_read_op(ENC28J60_READ_BUF_MEM, 0);	    //0x54
        usLen |= spi_read_op(ENC28J60_READ_BUF_MEM, 0) <<8;	//5554

      
        usLen -=4; //remove the CRC count
#if 1
        // read the receive status (see datasheet page 43)
        rxstat  = spi_read_op(ENC28J60_READ_BUF_MEM, 0);
        rxstat |= spi_read_op(ENC28J60_READ_BUF_MEM, 0)<<8;

//enc28j60_dump_rsv(__FUNCTION__, NextPacketPtr,  usLen, rxstat);
        // check CRC and symbol errors (see datasheet page 44, table 7-3):
        // The ERXFCON.CRCEN is set by default. Normally we should not
        // need to check this.

        if ((rxstat & 0x80)==0)
        {
            // invalid
            rt_kprintf("---invalid, rxstat=0x%02x,pktlen=%d\n", rxstat,usLen);
            usLen=0;
        }
        else
#endif            
        {
            //rt_kprintf("\nIn enc28j60_rx() ulLen:%u \r\n", usLen);
            /* allocation pbuf */
            p = pbuf_alloc(PBUF_LINK, usLen, PBUF_RAM);
            if (p != RT_NULL)
            {
                rt_uint8_t* data;
                struct pbuf* q;

                for (q = p; q != RT_NULL; q= q->next)
                {
                    data = q->payload;
                    usLen = q->len;

                    ENC28J60_CS_L;

                    SPI_I2S_SendData(SPI1,ENC28J60_READ_BUF_MEM);
                    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET);

                    SPI_I2S_ReceiveData(SPI1);

                    while(usLen)
                    {
                        usLen--;
                        SPI_I2S_SendData(SPI1,0x00)	;
                        while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET);

                        *data= SPI_I2S_ReceiveData(SPI1);
                        //rt_kprintf("%02x ", *data);
                        data++;
                    }

                    ENC28J60_CS_H;
                }
            }
        }

        // Move the RX read pointer to the start of the next received packet
        // This frees the memory we just read out
        spi_write(ERXRDPTL, (NextPacketPtr));
        spi_write(ERXRDPTH, (NextPacketPtr)>>8);

        // decrement the packet counter indicate we are done with this packet
        spi_write_op(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
    }
    else
    {
        // switch to bank 0
        enc28j60_set_bank(ECON1);
        // enable packet reception
        spi_write_op(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);

        level |= EIE_PKTIE;
    }

    /* enable enc28j60 interrupt */
    enc28j60_interrupt_enable(level);
    rt_sem_release(&lock_sem);

    return p;

 failed:
            /* enable enc28j60 interrupt */
    enc28j60_interrupt_enable(level | EIE_PKTIE);
    rt_sem_release(&lock_sem);
    return RT_NULL;

}

static void RCC_Configuration(void)
{
    /* enable SPI1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    /* enable gpiob port clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB 
        | RCC_APB2Periph_AFIO, ENABLE);
}

static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Configure one bit for preemption priority */
    /* ���ȼ��� ˵������ռ���ȼ����õ�λ�����������ȼ����õ�λ��   ��������1�� 7 */    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    
    /* Enable the EXTI2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;      //��ռ���ȼ� 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;             //�����ȼ�0  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                //ʹ��
    NVIC_Init(&NVIC_InitStructure);
}

/*****************************************************************************
 �� �� ��  : GPIO_Configuration
 ��������  : ��ʼ��ENC28J60 �ж� SPI1
 �������  : ��
 �������  : ��
 �� �� ֵ  : static
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2014��5��14��
    ��    ��   : peter
    �޸�����   : �����ɺ���

*****************************************************************************/
static void GPIO_Configuration()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
#if 0

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);		

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
  
    //-------------------------------------------------  
    /* configure PA1 as external interrupt */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure SPI1 pins:  SCK, MISO and MOSI ----------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // CS  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

#if 1  
    /* Connect ENC28J60 EXTI Line to GPIOB Pin 2 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);

    /* Configure ENC28J60 EXTI Line to generate an interrupt on falling edge */
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Clear the Key Button EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line1);
#endif    

}

static void SetupSPI (void)
{
    SPI_InitTypeDef SPI_InitStructure;

    GPIO_SetBits(GPIOA, GPIO_Pin_4);   
    
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
    SPI_Cmd(SPI1, ENABLE);
}


void Enc28j60Init()
{
    /* configuration PB5 as INT */
    /*
    RCC_Configuration();
    NVIC_Configuration();
    GPIO_Configuration();
    SetupSPI();
    */
    SPI1_Init();
    NVIC_Configuration();
    GPIO_Configuration();

    
    delay_ms(50);

    #if 1

    /* init rt-thread device interface */
    enc28j60_dev_entry.parent.parent.init		= enc28j60_init;
    enc28j60_dev_entry.parent.parent.open		= enc28j60_open;
    enc28j60_dev_entry.parent.parent.close		= enc28j60_close;
    enc28j60_dev_entry.parent.parent.read		= enc28j60_read;
    enc28j60_dev_entry.parent.parent.write		= enc28j60_write;
    enc28j60_dev_entry.parent.parent.control	= enc28j60_control;
    enc28j60_dev_entry.parent.eth_rx			= enc28j60_rx;
    enc28j60_dev_entry.parent.eth_tx			= enc28j60_tx;

    /* Update MAC address */
    /* OUI 00-04-A3 Microchip Technology, Inc. */
    enc28j60_dev_entry.dev_addr[0] = 0x00;
    enc28j60_dev_entry.dev_addr[1] = 0x04;
    enc28j60_dev_entry.dev_addr[2] = 0xA3;
    /* generate MAC addr (only for test) */
    enc28j60_dev_entry.dev_addr[3] = 0x11;
    enc28j60_dev_entry.dev_addr[4] = 0x22;
    enc28j60_dev_entry.dev_addr[5] = 0x33;

    rt_sem_init(&lock_sem, "lock", 1, RT_IPC_FLAG_FIFO);

    eth_device_init(&(enc28j60_dev->parent), "e0");
    #else

    ENC28J60_Init(RT_NULL);
    #endif
}

#include <finsh.h>
void show_reg(void)
{
    
}
FINSH_FUNCTION_EXPORT(show_reg,show en28j60 regs)
