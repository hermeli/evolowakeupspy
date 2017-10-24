/***********************************************************************
 * evolowakeupspy.c:
 * 
 * This is the code for a DC supply with integrated high current
 * trigger used to monitor the wake-up circuit of a "evolo" digital
 * door lock. Hardware is based on the "High performance AD/DA 
 * converter" addon board for Raspberry-Pi.
 * 
 * A TLV4112 Op-Amp with a gain of 2 is used as a high current driver.
 * DAC0: Source voltage output (1/2 of Output Voltage)
 * ADC0: Feedback from Output Voltage (6 Ohms sense resistor) 
 **********************************************************************/
#include <bcm2835.h>  
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <time.h>

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
* ADS1256_Send8Bit(uint8_t _data)
***********************************************************************/
static void ADS1256_Send8Bit(uint8_t _data)
{
	bsp_DelayUS(2);
	bcm2835_spi_transfer(_data);
}

/***********************************************************************
* ADS1256_CfgADC(ADS1256_GAIN_E _gain)
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
* ADS1256_DelayDATA(void)
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
* ADS1256_Recive8Bit(void)
***********************************************************************/
static uint8_t ADS1256_Recive8Bit(void)
{
	uint8_t read = 0;
	read = bcm2835_spi_transfer(0xff);
	return read;
}

/***********************************************************************
* ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue)
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
* ADS1256_ReadReg(uint8_t _RegID)
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
* ADS1256_WriteCmd(uint8_t _cmd)
***********************************************************************/
static void ADS1256_WriteCmd(uint8_t _cmd)
{
	CS_0();	/* SPI   cs = 0 */
	ADS1256_Send8Bit(_cmd);
	CS_1();	/* SPI  cs  = 1 */
}

/***********************************************************************
* ADS1256_ReadChipID(void)
***********************************************************************/
uint8_t ADS1256_ReadChipID(void)
{
	uint8_t id;

	ADS1256_WaitDRDY();
	id = ADS1256_ReadReg(REG_STATUS);
	return (id >> 4);
}

/***********************************************************************
* ADS1256_WaitDRDY(void)
***********************************************************************/
void ADS1256_WaitDRDY(void)
{
	uint32_t i;

	for (i = 0; i < 400000; i++)
	{
		if (DRDY_IS_LOW())
			break;
	}
	if (i >= 400000)
		printf("ADS1256_WaitDRDY() Time Out ...\r\n");		
}

/***********************************************************************
* ADS1256_ReadData(void)
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
	return (int32_t)read;
}
/***********************************************************************
* Write_DAC8552(uint8_t channel, uint16_t Data)
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
*	name: Hex16ToVolt
***********************************************************************/
float Hex16ToVolt(uint16_t hex16)
{
	return (float) ((int16_t)hex16)/0x8000*5.0;
}
/***********************************************************************
* VoltageConvert(float Vref, float voltage)
*
* Map voltage from 0 to Vref to uint16 Value
* (_D_ seems to be necessary (compiler optimization?))
***********************************************************************/
uint16_t VoltageConvert(float Vref, float voltage)
{
	uint16_t _D_;
	_D_ = (uint16_t)(65536 * voltage / Vref);
	return _D_;
}
/***********************************************************************
* SetOutputVoltage(float Vout)
* 
* Output OP-AMP has a gain of 2, so we need to set the DAC to 1/2
* voltage. 
***********************************************************************/
void SetOutputVoltage(float Vout)
{
	float daVolt = Vout/2.0;
	uint16_t daVal = VoltageConvert(3.3, daVolt);
	Write_DAC8552(0x30, daVal);
}
/***********************************************************************
*	main
***********************************************************************/
#define VOLT_UPPER	3.7
#define VOLT_LOWER	2.5
#define VOLT_STEP	0.01
#define DVOLT_TRIGGER	0.1

#define TIME_LEVEL 	1000000		// 1 Sek for each voltage level
#define TIME_SAMPLE	10000   	// 10 msec sample time for supervisor
#define TIME_SETTLE 1000 		// 1 msec until voltage should be stable

int  main()
{
	uint8_t id;
	uint32_t adc;
	uint32_t i;	
	float adVolt,outVolt;
	float step;
	uint16_t daVal,adVal;
	time_t t;
   
	printf("Evolo Wake-Up Spy Version 1.0\r\n");
	printf("-----------------------------\r\n");
	printf("STEP_TIME=%dms SAMPLE_TIME=%dms SETTLE_TIME=%dms\r\n",TIME_LEVEL/1000,TIME_SETTLE/1000);
	
	if (!bcm2835_init())
		return 1;
	bcm2835_spi_begin();
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_LSBFIRST );      // The default
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                   // The default
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16); // The default
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
	outVolt = VOLT_UPPER;
	step = -VOLT_STEP;
    
    SetOutputVoltage(outVolt); 
    printf("Waiting 5 Secs...\n");
    bsp_DelayUS(5000000);
    
    time(&t);
    printf("Starting test at %s",ctime(&t));
    
	while(1)
	{	
		// Setting new output voltage
		SetOutputVoltage(outVolt); 
		
		// time(&t);
		// printf("%.02fV | %s",outVolt,ctime(&t));
		
		bsp_DelayUS(TIME_SETTLE);
		
		i=0;
		while(TIME_SAMPLE * i < TIME_LEVEL)
		{
			// get new AD data
			adc = ADS1256_ReadData();
			adVal = (adc>>8);
			adVolt = Hex16ToVolt(adVal);
			// printf("-> adVal=0x%04X adVolt=%f\r\n",adVal,adVolt);
		
			// check if data is inside window
			if ( outVolt - adVolt > DVOLT_TRIGGER)
			{
				time(&t);
				printf("Wakeup Trigger: Soll=%fV Ist=%fV | %s",outVolt,adVolt,ctime(&t));
			}
			
			// wait
			bsp_DelayUS(TIME_SAMPLE);
			i++;
		}
		
		// set next output voltage
		outVolt = outVolt + step;
		
		// eventually set direction of output voltage positive or negative
		if ( (outVolt < VOLT_LOWER) || (outVolt > VOLT_UPPER) )
			step *= -1;
	}
	bcm2835_spi_end();
	bcm2835_close();
	return 0;
}
