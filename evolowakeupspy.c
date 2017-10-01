/*
 * evolowakeupspy.c:
 * 
 * This is the code for a DC supply with integrated high current
 * trigger used to monitor the wake-up circuit of a "evolo" digital
 * door lock. Hardware is based on the "High performance AD/DA 
 * converter" addon board for Raspberry-Pi. 
 */
 
#include <bcm2835.h>  
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#define DRDY  RPI_GPIO_P1_11         	//P0
#define RST  RPI_GPIO_P1_12     		//P1
#define	SPICS	RPI_GPIO_P1_15			//P3   ADS1256 chip select
#define	SPICS1	RPI_GPIO_P1_16			//P4   DAC8552 chip select

#define CS_1() bcm2835_gpio_write(SPICS,HIGH)
#define CS_0()  bcm2835_gpio_write(SPICS,LOW)
#define CS1_1() bcm2835_gpio_write(SPICS1,HIGH)
#define CS1_0()  bcm2835_gpio_write(SPICS1,LOW)
#define DRDY_IS_LOW()	((bcm2835_gpio_lev(DRDY)==0))
#define RST_1() 	bcm2835_gpio_write(RST,HIGH);
#define RST_0() 	bcm2835_gpio_write(RST,LOW);

/* Unsigned integer types  */
#define uint8_t unsigned char
#define uint16_t unsigned short    
#define uint32_t unsigned long     

#define DAC_CHANNELA   0x30
#define DAC_CHANNELB   0x34

void ADS1256_WaitDRDY(void);

typedef enum {FALSE = 0, TRUE = !FALSE} bool;

/* gain channelÓ */
typedef enum
{
	ADS1256_GAIN_1			= (0),	/* GAIN   1 */
	ADS1256_GAIN_2			= (1),	/*GAIN   2 */
	ADS1256_GAIN_4			= (2),	/*GAIN   4 */
	ADS1256_GAIN_8			= (3),	/*GAIN   8 */
	ADS1256_GAIN_16			= (4),	/* GAIN  16 */
	ADS1256_GAIN_32			= (5),	/*GAIN    32 */
	ADS1256_GAIN_64			= (6),	/*GAIN    64 */
}ADS1256_GAIN_E;

/*Register definition */
enum
{
	/*Register address, followed by reset the default values */
	REG_STATUS = 0,	// x1H
	REG_MUX    = 1, // 01H
	REG_ADCON  = 2, // 20H
	REG_DRATE  = 3, // F0H
	REG_IO     = 4, // E0H
	REG_OFC0   = 5, // xxH
	REG_OFC1   = 6, // xxH
	REG_OFC2   = 7, // xxH
	REG_FSC0   = 8, // xxH
	REG_FSC1   = 9, // xxH
	REG_FSC2   = 10, // xxH
};

/* Command definition */
enum
{
	CMD_WAKEUP  = 0x00,	// Completes SYNC and Exits Standby Mode 0000  0000 (00h)
	CMD_RDATA   = 0x01, // Read Data 0000  0001 (01h)
	CMD_RDATAC  = 0x03, // Read Data Continuously 0000   0011 (03h)
	CMD_SDATAC  = 0x0F, // Stop Read Data Continuously 0000   1111 (0Fh)
	CMD_RREG    = 0x10, // Read from REG rrr 0001 rrrr (1xh)
	CMD_WREG    = 0x50, // Write to REG rrr 0101 rrrr (5xh)
	CMD_SELFCAL = 0xF0, // Offset and Gain Self-Calibration 1111    0000 (F0h)
	CMD_SELFOCAL= 0xF1, // Offset Self-Calibration 1111    0001 (F1h)
	CMD_SELFGCAL= 0xF2, // Gain Self-Calibration 1111    0010 (F2h)
	CMD_SYSOCAL = 0xF3, // System Offset Calibration 1111   0011 (F3h)
	CMD_SYSGCAL = 0xF4, // System Gain Calibration 1111    0100 (F4h)
	CMD_SYNC    = 0xFC, // Synchronize the A/D Conversion 1111   1100 (FCh)
	CMD_STANDBY = 0xFD, // Begin Standby Mode 1111   1101 (FDh)
	CMD_RESET   = 0xFE, // Reset to Power-Up Values 1111   1110 (FEh)
};

void  bsp_DelayUS(uint64_t micros)
{
		bcm2835_delayMicroseconds (micros);
}

/***********************************************************************
*	name: ADS1256_Send8Bit
*	function: SPI bus to send 8 bit data
*	parameter: _data:  data
*	The return value: NULL
***********************************************************************/
static void ADS1256_Send8Bit(uint8_t _data)
{
	bsp_DelayUS(2);
	bcm2835_spi_transfer(_data);
}

/***********************************************************************
*	name: ADS1256_CfgADC
*	function: The configuration parameters of ADC
***********************************************************************/
void ADS1256_CfgADC(ADS1256_GAIN_E _gain)
{	
	ADS1256_WaitDRDY();

	CS_0();
	ADS1256_Send8Bit(CMD_WREG | 0);			// Write command register, send the register address
	ADS1256_Send8Bit(0x03);	    			// Number of bytes to be written -1
	ADS1256_Send8Bit(0x00);					// @0x00 STATUS: default 
	ADS1256_Send8Bit(0x08);					// @0x01 MUX: AIN0 & AINCOM 
	ADS1256_Send8Bit(0x20|(_gain << 0));	// @0x02 ADCON: control register,gain 
	ADS1256_Send8Bit(0xF0);					// @0x03 DRATE: Set the output rate 30k SPS (default) 
	CS_1();

	bsp_DelayUS(50);
}

/***********************************************************************
*	name: ADS1256_DelayDATA
*	function: delay
*	parameter: NULL
*	The return value: NULL
***********************************************************************/
static void ADS1256_DelayDATA(void)
{
	/*
		Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands
		min  50   CLK = 50 * 0.13uS = 6.5uS
	*/
	bsp_DelayUS(10);	/* The minimum time delay 6.5us */
}

/***********************************************************************
*	name: ADS1256_Recive8Bit
*	function: SPI bus receive function
*	parameter: NULL
*	The return value: NULL
***********************************************************************/
static uint8_t ADS1256_Recive8Bit(void)
{
	uint8_t read = 0;
	read = bcm2835_spi_transfer(0xff);
	return read;
}

/***********************************************************************
*	name: ADS1256_WriteReg
*	function: Write the corresponding register
*	parameter: _RegID: register  ID
*			 _RegValue: register Value
*	The return value: NULL
***********************************************************************/
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue)
{
	CS_0();	/* SPI  cs  = 0 */
	ADS1256_Send8Bit(CMD_WREG | _RegID);	// CMD + RegNr
	ADS1256_Send8Bit(0x00);					// Number of data bytes -1 
	ADS1256_Send8Bit(_RegValue);			// Send register data	
	CS_1();	/* SPI   cs = 1 */
}

/***********************************************************************
*	name: ADS1256_ReadReg
*	function: Read  the corresponding register
*	parameter: _RegID: register  ID
*	The return value: read register value
***********************************************************************/
static uint8_t ADS1256_ReadReg(uint8_t _RegID)
{
	uint8_t read;

	CS_0();	/* SPI  cs  = 0 */
	ADS1256_Send8Bit(CMD_RREG | _RegID);	/* Write command register */
	ADS1256_Send8Bit(0x00);					// Number of data bytes -1

	ADS1256_DelayDATA();	/*delay time */

	read = ADS1256_Recive8Bit();	/* Read the register values */
	CS_1();	/* SPI   cs  = 1 */

	return read;
}

/***********************************************************************
*	name: ADS1256_WriteCmd
*	function: Sending a single byte order
*	parameter: _cmd : command
*	The return value: NULL
***********************************************************************/
static void ADS1256_WriteCmd(uint8_t _cmd)
{
	CS_0();	/* SPI   cs = 0 */
	ADS1256_Send8Bit(_cmd);
	CS_1();	/* SPI  cs  = 1 */
}

/***********************************************************************
*	name: ADS1256_ReadChipID
*	function: Read the chip ID
*	parameter: _cmd : NULL
*	The return value: four high status register
***********************************************************************/
uint8_t ADS1256_ReadChipID(void)
{
	uint8_t id;

	ADS1256_WaitDRDY();
	id = ADS1256_ReadReg(REG_STATUS);
	return (id >> 4);
}

/***********************************************************************
*	name: ADS1256_WaitDRDY
*	function: delay time  wait for automatic calibration
*	parameter:  NULL
*	The return value:  NULL
***********************************************************************/
void ADS1256_WaitDRDY(void)
{
	uint32_t i;

	for (i = 0; i < 400000; i++)
	{
		if (DRDY_IS_LOW())
		{
			break;
		}
	}
	if (i >= 400000)
	{
		printf("ADS1256_WaitDRDY() Time Out ...\r\n");		
	}
}

/***********************************************************************
*	name: ADS1256_ReadData
*	function: read ADC value
*	parameter: NULL
*	The return value:  NULL
***********************************************************************/
static int32_t ADS1256_ReadData(void)
{
	uint32_t read = 0;
    static uint8_t buf[3];

	CS_0();	/* SPI   cs = 0 */

	ADS1256_Send8Bit(CMD_RDATA);	/* read ADC command  */
	ADS1256_DelayDATA();	/*delay time  */

	/*Read the sample results 24bit*/
    buf[0] = ADS1256_Recive8Bit();
    buf[1] = ADS1256_Recive8Bit();
    buf[2] = ADS1256_Recive8Bit();

    read = ((uint32_t)buf[0] << 16) & 0x00FF0000;
    read |= ((uint32_t)buf[1] << 8) & 0x0000FF00;
    read |= buf[2];

	CS_1();

	/* Extend a signed number*/
    /*
    if (read & 0x800000)
    {
	    read |= 0xFF000000;
    }
    */

	return (int32_t)read;
}

/***********************************************************************
*	name: Write_DAC8552
*	function:  DAC send data 
*	parameter: channel : output channel number 
*			   data : output DAC value 
*	The return value:  NULL
***********************************************************************/
void Write_DAC8552(uint8_t channel, uint16_t Data)
{
	uint8_t i;

	 CS1_1() ;
	 CS1_0() ;
      bcm2835_spi_transfer(channel);
      bcm2835_spi_transfer((Data>>8));
      bcm2835_spi_transfer((Data&0xff));  
      CS1_1() ;
}
/***********************************************************************
*	name: VoltToHex16
***********************************************************************/
uint16_t VoltToHex16(float volt)
{
	// reference voltage must be 5.0V!
	return (uint16_t)(0xFFFF*volt/5.0);
}
/***********************************************************************
*	name: Hex16ToVolt
***********************************************************************/
float Hex16ToVolt(uint16_t hex16)
{
	return (float)      ((int16_t)hex16)/0x8000*5.0;
}
/***********************************************************************
*	name: main
*	function:  
*	parameter: NULL
*	The return value:  NULL
***********************************************************************/

uint16_t SINE_LUT[80]={
0x8000,0x8a0a,0x9405,0x9de1,0xa78d,0xb0fb,0xba1c,0xc2e0,
0xcb3c,0xd320,0xda82,0xe154,0xe78d,0xed22,0xf20c,0xf641,
0xf9bb,0xfc76,0xfe6c,0xff9a,0xffff,0xff9a,0xfe6c,0xfc76,
0xf9bb,0xf641,0xf20c,0xed22,0xe78d,0xe154,0xda82,0xd320,
0xcb3c,0xc2e0,0xba1c,0xb0fb,0xa78d,0x9de1,0x9405,0x8a0a,
0x8000,0x75f5,0x6bfa,0x621e,0x5872,0x4f04,0x45e3,0x3d1f,
0x34c3,0x2cdf,0x257d,0x1eab,0x1872,0x12dd,0xdf3,0x9be,
0x644,0x389,0x193,0x65,0x0,0x65,0x193,0x389,
0x644,0x9be,0xdf3,0x12dd,0x1872,0x1eab,0x257d,0x2cdf,
0x34c3,0x3d1f,0x45e3,0x4f04,0x5872,0x621e,0x6bfa,0x75f5
};

#define VOLT_UPPER	3.7
#define VOLT_LOWER	2.5
#define VOLT_STEP	0.1
#define DVOLT_TRIGGER	0.1

#define TIME_LEVEL 	10000000
#define TIME_SAMPLE	10000
int  main()
{
	uint8_t id;
  	uint32_t adc;
	uint32_t i;	
	float adVolt,daVolt;
    uint16_t daVal,adVal;
    
    printf("Evolo Wake-Up Spy Version 1.0\r\n");
    printf("-----------------------------\r\n");
    printf("STEP_TIME=%dms SAMPLE_TIME=%dms\r\n",TIME_LEVEL/1000,TIME_SAMPLE/1000);
    
    if (!bcm2835_init())
        return 1;
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_LSBFIRST );      // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                   // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_8192); // The default
    bcm2835_gpio_fsel(SPICS, BCM2835_GPIO_FSEL_OUTP);//
    bcm2835_gpio_write(SPICS, HIGH);
    bcm2835_gpio_fsel(DRDY, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(DRDY, BCM2835_GPIO_PUD_UP);    	

	id = ADS1256_ReadChipID(); 
	if (id != 3)
		printf("Error, ASD1256 Chip ID = 0x%d\r\n", (int)id);
	else
		printf("Ok, ASD1256 Chip ID = 0x%d\r\n", (int)id);

	ADS1256_CfgADC(ADS1256_GAIN_1);
    
    daVolt = VOLT_UPPER;
    
    while(1)
    {	
		// Setting new output voltage
		daVal = VoltToHex16(daVolt);
		Write_DAC8552(0x30,daVal);    	//Write channel A buffer (0x30)  
		printf("Setting new output voltage %.02f V\r\n",daVolt);
		
		i=0;
		while(TIME_SAMPLE * i < TIME_LEVEL)
		{
			// get new AD data
			adc = ADS1256_ReadData();
			adVal = (adc>>8);
			adVolt = Hex16ToVolt(adVal);
			// printf("-> adVal=0x%04X adVolt=%f\r\n",adVal,adVolt);
		
			// check if data is inside window
			if (daVolt - adVolt > DVOLT_TRIGGER)
			{
				printf("Wakeup Trigger: Soll=%f Ist=%f\r\n",daVolt,adVolt);
			}
			
			// wait
			bsp_DelayUS(TIME_SAMPLE);
			i++;
		}
		
		daVolt = daVolt - VOLT_STEP;
		if (daVolt < VOLT_LOWER) 
			daVolt = VOLT_UPPER;
		
	}
    
    bcm2835_spi_end();
    bcm2835_close();
	
    return 0;
}
