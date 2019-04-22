/*
 * ADS1256_test.c:
 *	Very simple program to test the serial port. Expects
 *	the port to be looped back to itself
 *
 */
 
/*
             define from bcm2835.h                       define from Board DVK511
                 3.3V | | 5V               ->                 3.3V | | 5V
    RPI_V2_GPIO_P1_03 | | 5V               ->                  SDA | | 5V 
    RPI_V2_GPIO_P1_05 | | GND              ->                  SCL | | GND
       RPI_GPIO_P1_07 | | RPI_GPIO_P1_08   ->                  IO7 | | TX
                  GND | | RPI_GPIO_P1_10   ->                  GND | | RX
       RPI_GPIO_P1_11 | | RPI_GPIO_P1_12   ->                  IO0 | | IO1
    RPI_V2_GPIO_P1_13 | | GND              ->                  IO2 | | GND
       RPI_GPIO_P1_15 | | RPI_GPIO_P1_16   ->                  IO3 | | IO4
                  VCC | | RPI_GPIO_P1_18   ->                  VCC | | IO5
       RPI_GPIO_P1_19 | | GND              ->                 MOSI | | GND
       RPI_GPIO_P1_21 | | RPI_GPIO_P1_22   ->                 MISO | | IO6
       RPI_GPIO_P1_23 | | RPI_GPIO_P1_24   ->                  SCK | | CE0
                  GND | | RPI_GPIO_P1_26   ->                  GND | | CE1

::if your raspberry Pi is version 1 or rev 1 or rev A
RPI_V2_GPIO_P1_03->RPI_GPIO_P1_03
RPI_V2_GPIO_P1_05->RPI_GPIO_P1_05
RPI_V2_GPIO_P1_13->RPI_GPIO_P1_13
::
*/


////
#include <python3.5/Python.h>
#include <bcm2835.h>  
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <time.h>
#include <stdlib.h>
//CS      -----   SPICS  
//DIN     -----   MOSI
//DOUT  -----   MISO
//SCLK   -----   SCLK
//DRDY  -----   ctl_IO     data  starting
//RST     -----   ctl_IO     reset


//#define  DRDY  RPI_V2_GPIO_P1_11         //P0
#define  DRDY  17
//#define  RST  RPI_V2_GPIO_P1_12     //P1
#define  RST  18
//#define	SPICS	RPI_V2_GPIO_P1_15	//P3
#define  SPICS  22

#define CS_1() bcm2835_gpio_write(SPICS,HIGH)
#define CS_0()  bcm2835_gpio_write(SPICS,LOW)
#define DRDY_IS_LOW()	((bcm2835_gpio_lev(DRDY)==0))
#define RST_1() 	bcm2835_gpio_write(RST,HIGH)
#define RST_0() 	bcm2835_gpio_write(RST,LOW)

/* Unsigned integer types  */
#define uint8_t unsigned char
#define uint16_t unsigned short    
#define uint32_t unsigned long     


void waitDRDY(void);
uint8_t initializeSPI();
void endSPI();
uint8_t readByteFromReg(uint8_t registerID);
void writeByteToReg(uint8_t registerID, uint8_t value);
uint8_t writeCMD(uint8_t command);
uint8_t setBuffer(bool val);
uint8_t readChipID(void);
void setDIFFChannel(uint8_t positiveCh, uint8_t NegativeCh);
void setSEChannel(uint8_t channel);
void setPGA(uint8_t pga);
void setDataRate(uint8_t drate);
int32_t readData(void);
int32_t getValSEChannel(uint8_t channel);
int32_t getValDIFFChannel(uint8_t positiveCh, uint8_t negativeCh);
void scanSEChannels(uint8_t channels[], uint8_t numOfChannels, uint32_t *values);
void scanDIFFChannels(uint8_t positiveChs[], uint8_t negativeChs[], uint8_t numOfChannels, int32_t *values, int32_t dataLength);
void scanSEChannelContinuous(uint8_t channel, uint32_t numOfMeasure, uint32_t *values, uint32_t *currentTime);
void scanDIFFChannelContinuous(uint8_t positiveCh, uint8_t negativeCh, uint32_t numOfMeasure, int32_t *values, uint32_t *currentTime);
void saveCsvFile(int32_t* values_DIFF_CONT);
PyObject * Convert_big_Array(double array[],int length);
/* gain channelî */

enum
{
	PGA_GAIN1	= 0, // Input voltage range: +- 5 V
	PGA_GAIN2	= 1, // Input voltage range: +- 2.5 V
	PGA_GAIN4	= 2, // Input voltage range: +- 1.25 V
	PGA_GAIN8	= 3, // Input voltage range: +- 0.625 V
	PGA_GAIN16	= 4, // Input voltage range: +- 0.3125 V
	PGA_GAIN32	= 5, // Input voltage range: +- 0.15625 V
	PGA_GAIN64	= 6  // Input voltage range: +- 0.078125 V
};

enum
{
	DRATE_30000 = 0xF0, 
	DRATE_15000 = 0xE0,
	DRATE_7500  = 0xD0,
	DRATE_3750  = 0xC0,
	DRATE_2000  = 0xB0,
	DRATE_1000  = 0xA1,
	DRATE_500   = 0x92,
	DRATE_100   = 0x82,
	DRATE_60    = 0x72,
	DRATE_50    = 0x63,
	DRATE_30    = 0x53,
	DRATE_25    = 0x43,
	DRATE_15    = 0x33,
	DRATE_10    = 0x20,
	DRATE_5     = 0x13,
	DRATE_2d5   = 0x03
};

enum
{
	AIN0   = 0, //Binary value: 0000 0000
	AIN1   = 1, //Binary value: 0000 0001
	AIN2   = 2, //Binary value: 0000 0010
	AIN3   = 3, //Binary value: 0000 0011
	AIN4   = 4, //Binary value: 0000 0100
	AIN5   = 5, //Binary value: 0000 0101
	AIN6   = 6, //Binary value: 0000 0110
	AIN7   = 7, //Binary value: 0000 0111
	AINCOM = 8  //Binary value: 0000 1000
};
/* Sampling speed choice*/
/* 
	11110000 = 30,000SPS (default)
	11100000 = 15,000SPS
	11010000 = 7,500SPS
	11000000 = 3,750SPS
	10110000 = 2,000SPS
	10100001 = 1,000SPS
	10010010 = 500SPS
	10000010 = 100SPS
	01110010 = 60SPS
	01100011 = 50SPS
	01010011 = 30SPS
	01000011 = 25SPS
	00110011 = 15SPS
	00100011 = 10SPS
	00010011 = 5SPS
	00000011 = 2.5SPS
*/

/*Register definition£º Table 23. Register Map --- ADS1256 datasheet Page 30*/
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

/* Command definition£º TTable 24. Command Definitions --- ADS1256 datasheet Page 34 */
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

// Docstrings
static char module_docstring[] = "This is a python wrapper for the Waveshare AD-DA board.";

// Functions
static PyObject * initialSPI(PyObject * self, PyObject * args);
static PyObject * setADC1256BaseParameter(PyObject * self, PyObject * args);
static PyObject * readDiffChannelVoltsContinous(PyObject * self, PyObject * args);
static PyObject * readDiffChannelVolts(PyObject * self, PyObject * args);
static PyObject * endSPIfunc(PyObject * self, PyObject * args);


// Method specification
static PyMethodDef module_methods[] = {
	{"initialSPI", initialSPI, METH_VARARGS, "Read single channel and convert to volts."},
	{"endSPIfunc", endSPIfunc, METH_VARARGS, "end SPI function & wiring"},
	{"setADC1256BaseParameter", setADC1256BaseParameter, METH_VARARGS, "set ADC1256 Based Parameter"},	
	{"readDiffChannelVolts", readDiffChannelVolts, METH_VARARGS, "Read Diff channel and convert to volts."},
	{"readDiffChannelVoltsContinous", readDiffChannelVoltsContinous, METH_VARARGS, "Read Diff channel and convert to volts continous."},
	{NULL, NULL, 0, NULL}
};

// Module specification
static struct PyModuleDef SMG_ADS1256PyLib_module = {
    PyModuleDef_HEAD_INIT,
    "pyadda",   /* name of module */
    module_docstring, /* module documentation, may be NULL */
    -1,       /* size of per-interpreter state of the module,
                 or -1 if the module keeps state in global variables. */
    module_methods
};

PyMODINIT_FUNC PyInit_SMG_ADS1256PyLib(void) {

    return PyModule_Create(&SMG_ADS1256PyLib_module);
}

PyObject * Convert_big_Array(int32_t array[],int length)
{
	PyObject *pylist, *item;
	int i;
	pylist = PyList_New(length);
	if(pylist != NULL){
	for(i=0;i<length;i++){
		item = PyFloat_FromDouble(array[i]);
		
		PyList_SET_ITEM(pylist, i, item);
	}
    }
    printf("Convert array finished");
	return pylist;
}

static PyObject * readDiffChannelVolts(PyObject * self, PyObject * args){
	
	PyObject * voltageData;
	int32_t num_measure_DIFF = 500;
	if (!PyArg_ParseTuple(args, "i", &num_measure_DIFF)) {
		return NULL;
	}
	if(num_measure_DIFF ==0){
	num_measure_DIFF = 500;
	}
	//int num_ch_DIFF = 4;
	uint8_t num_ch_DIFF = 1;
	
	int32_t values_DIFF [num_measure_DIFF];
	//uint8_t  posChannels [4] = {AIN0, AIN2, AIN4, AIN6};
	//uint8_t  negChannels [4] = {AIN1, AIN3, AIN5, AIN7};
	uint8_t  posChannels [1] = {AIN0}; // TESTING
	uint8_t  negChannels [1] = {AIN1}; // TESTING
	

	scanDIFFChannels(posChannels, negChannels, num_ch_DIFF, values_DIFF,num_measure_DIFF);

	voltageData = Convert_big_Array(values_DIFF,num_measure_DIFF);
	
	return voltageData;
}


static PyObject * readDiffChannelVoltsContinous(PyObject * self, PyObject * args) {
	int channel;
	PyObject * voltageData;
	// parse arguments as long integers
	if (!PyArg_ParseTuple(args, "i", &channel)) {
		return NULL;
	}

	// run read_channel_volts()
	int num_measure_DIFF_CONT = 10000; // 30x measurements because it works with much higher sample rate
	int32_t values_DIFF_CONT [num_measure_DIFF_CONT];
	uint32_t time_DIFF_CONT [num_measure_DIFF_CONT];
	scanDIFFChannelContinuous(AIN0, AIN1, num_measure_DIFF_CONT, values_DIFF_CONT, time_DIFF_CONT);
	int32_t dataLength =  sizeof(values_DIFF_CONT) / sizeof(values_DIFF_CONT[0]);
	voltageData = Convert_big_Array(values_DIFF_CONT,dataLength);
	
    return voltageData;
}

static PyObject * initialSPI(PyObject * self, PyObject * args)
{
	double volts = 1;
	initializeSPI();
	PyObject * ret = Py_BuildValue("d",volts);
    return ret;
}

static PyObject * endSPIfunc(PyObject * self, PyObject * args)
{
	double volts = 1;
	endSPI();
	PyObject * ret = Py_BuildValue("d",volts);
    return ret;
}

static PyObject * setADC1256BaseParameter(PyObject * self, PyObject * args)
{
	double volts = 1;
	int32_t DateRate = 0;
	int8_t pgaGain = 0;
	int BufferboolArg = 0;
	
	
	if (!PyArg_ParseTuple(args, "pii",&BufferboolArg,&pgaGain, &DateRate)) {
	return NULL;
	}

	setBuffer(BufferboolArg);
	setPGA(pgaGain);
	setDataRate(DateRate);
	
	PyObject * ret = Py_BuildValue("d",volts);
    return ret;
}




void  bsp_DelayUS(uint64_t micros);

/*
 Define New Method  
 */
void send8bit(uint8_t data);
uint8_t receive8bit(void);
void  Delay_us(uint64_t micros);
 
 /* 
 Define New Method End
*/




void  Delay_us(uint64_t micros)
{
	bcm2835_delayMicroseconds (micros);
}



/*
*********************************************************************************************************
*	name: Send8bit
*	function: SPI send data to SPI slave 
*	parameter: data
*	The return value: NULL
*********************************************************************************************************
*/
void send8bit(uint8_t data)
{
	bcm2835_spi_transfer(data);
}

/*
*********************************************************************************************************
*	name: receive8bit
*	function: receive data from SPI slave
*	parameter: data
*	The return value: NULL
*********************************************************************************************************
*/
uint8_t receive8bit(void)
{
	uint8_t read = 0;
	read  = bcm2835_spi_transfer(0xff);
	return read;
}

/*
*********************************************************************************************************
*	name: waitDRDY
*	function: Wait for DRDY is Low
*	parameter: data
*	The return value: None
*********************************************************************************************************
*/
void waitDRDY(void)
{
	while(!DRDY_IS_LOW())
	{
		continue;
	}
}


/*
*********************************************************************************************************
*	name: initializeSPI
*	function: Initial SPI bus, set BitOrder & SPI MODE, Clock, GPIO.
*	parameter: None
*	The return value: NULL
*********************************************************************************************************
*/
uint8_t initializeSPI()
{
	if(!bcm2835_init())
	{
		return -1;
	}
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);   
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128);

    bcm2835_gpio_fsel(SPICS, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(SPICS, HIGH);
    bcm2835_gpio_fsel(DRDY, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(DRDY, BCM2835_GPIO_PUD_UP); 
    printf("Initial BCM2835 SPI finished\n");
    return 1;	
}
/*
*********************************************************************************************************
*	name: endSPI
*	function: Stop SPI Bus
*	parameter: None
*	The return value: NULL
*********************************************************************************************************
*/
void endSPI(void)
{
	bcm2835_spi_end();
	bcm2835_close();
}
/*
*********************************************************************************************************
*	name: readByteFromReg
*	function: read 1 byte from register address registerID.
*	parameter: register ID
*	The return value: 
*********************************************************************************************************
*/
uint8_t readByteFromReg(uint8_t registerID)
{
	CS_0();
	send8bit(CMD_RREG | registerID); //1st byte:address of the first register to read 
	send8bit(0x00);						//2nd byte : number of bytes to read = 1
	Delay_us(7);						//min delay :t6 = 50* 1 
	uint8_t read = receive8bit();
	CS_1();
	
	return read;
}
/*
*********************************************************************************************************
*	name: writeByteToReg
*	function: read 1 byte from register address registerID.
*	parameter: register ID
*	The return value: 
*********************************************************************************************************
*/
void writeByteToReg(uint8_t registerID, uint8_t value)
{
	CS_0();
	send8bit(CMD_WREG | registerID);		//1syt byte: address of the first register to write
	send8bit(0x00);							//2nd byte: number of byte to write = 1.
	send8bit(value);						//3rd byte: value to write to register
	CS_1();
	
}
/*
*********************************************************************************************************
*	name: writeCMD
*	function: Send Standalone commands to register
*	parameter: command
*	The return value: None
*********************************************************************************************************
*/
uint8_t writeCMD(uint8_t command)
{
	CS_0();
	send8bit(command);
	CS_1();
}

/*
*********************************************************************************************************
*	name: setBuffer
*	function: Set the internal buffer (True-enable), (Fasle-disable)
*	parameter: bool val
*	The return value: val
*********************************************************************************************************
*/
uint8_t setBuffer(bool val)
{
	CS_0();
	send8bit(CMD_WREG | REG_STATUS);
	send8bit((0 <<3) | (1 << 2) | (val << 1));
	CS_1();
	printf("Set buffer finsihed\n");
}
/*
*********************************************************************************************************
*	name: readChipID
*	function: Get data from Status register - chipID "check" 
*	parameter: 
*	The return value: val
*********************************************************************************************************
*/
uint8_t readChipID(void)
{
	waitDRDY();
	uint8_t id  = readByteFromReg(REG_STATUS);
	
	return (id >> 4);
}
/*
*********************************************************************************************************
*	name: setSEChannel
*	function: Write to MUX register - set channel to read from in single-ended mode 
*   Bit 7,6,5,4 determine the positive input channel (AINp).
*   Bit 3,2,1,0 determine the negative input channel (AINn). True Ground
*	parameter: 
*	The return value: val
*********************************************************************************************************
*/
void setSEChannel(uint8_t channel)
{
	writeByteToReg(REG_MUX, channel << 4 | 1 << 3); //xxxx1000 - AINp = channel, AINn = AINCOM
}
/*
*********************************************************************************************************
*	name: setDIFFChannel
*	function: Write to MUX register - set channel to read from in single-ended mode 
*   Bit 7,6,5,4 determine the positive input channel (AINp).
*   Bit 3,2,1,0 determine the negative input channel (AINn). e.g. (0-1, 2,3 - 4,5 - 6,7)
*	parameter: 
*	The return value: val
*********************************************************************************************************
*/
void setDIFFChannel(uint8_t positiveCh, uint8_t NegativeCh)
{
	writeByteToReg(REG_MUX, positiveCh <<4 | NegativeCh); //xxxx1000 - AINp = positiveCh, AINn = NegativeCh
}
/*
*********************************************************************************************************
*	name: setPGA
*	function: Set gain of amplifier 
*	parameter: pga
*	The return value: None
*********************************************************************************************************
*/
void setPGA(uint8_t pga)
{
	writeByteToReg(REG_ADCON,pga); 
	printf("Set PGA finsihed\n");
}

/*
*********************************************************************************************************
*	name: setDataRate
*	function: sampling rate of collection
*	parameter: pga
*	The return value: None
*********************************************************************************************************
*/
void setDataRate(uint8_t drate)
{
	writeByteToReg(REG_DRATE,drate); 
	printf("Set data rate finsihed\n");
}

/*
*********************************************************************************************************
*	name: readData
*	function: Read 24 bit value from SPI bus, when DRDY goes low to read signle conversion result
* 	Allows reading data from multiple different channels.
*	parameter: None
*	The return value: None
*********************************************************************************************************
*/
int32_t readData(void)
{
		uint32_t read = 0;
	uint8_t buffer[3];

	CS_0();
	send8bit(CMD_RDATA);
	Delay_us(7);// min delay: t6 = 50 * 1/freq.clkin = 50 * 1 / 7,68 Mhz = 6.5 micro sec

	buffer[0] = receive8bit();
	buffer[1] = receive8bit();
	buffer[2] = receive8bit();
	// DRDY goes high here

	// construct 24 bit value
	read =  ((uint32_t)buffer[0] << 16) & 0x00FF0000;
	read |= ((uint32_t)buffer[1] << 8);
	read |= buffer[2];
	if (read & 0x800000){
		read |= 0xFF000000;
	}

	CS_1();

	return (int32_t)read;
}

/*
*********************************************************************************************************
*	name: getValSEChannel
*	Get one single-ended analog input value by issuing command to input multiplexer.
*	It reads a value from previous conversion!
* 	DRDY needs to be low!
*	parameter: 
*	The return value: val
*********************************************************************************************************
*/
int32_t getValSEChannel(uint8_t channel)
{
	int32_t read = 0;
	setSEChannel(channel); // MUX command
	Delay_us(3); // min delay: t11 = 24 * 1 / 7,68 Mhz = 3,125 micro sec
	writeCMD(CMD_SYNC);    // SYNC command
	Delay_us(3);
	writeCMD(CMD_WAKEUP);  // WAKEUP command
	Delay_us(1); // min delay: t11 = 4 * 1 / 7,68 Mhz = 0,52 micro sec
	read = readData();
	return read;
}
/*
*********************************************************************************************************
*	name: getValDIFFChannel
*	Get one differential analog input value by issuing command to input multiplexer.
* 	It reads a value from previous conversion!
*	DRDY needs to be low!
*	parameter: 
*	The return value: val
*********************************************************************************************************
*/
int32_t getValDIFFChannel(uint8_t positiveCh, uint8_t negativeCh)
{
	int32_t read = 0;
	setDIFFChannel(positiveCh, negativeCh);
	Delay_us(3); // min delayus: t11 = 24 * 1 / 7,68 Mhz = 3,125 micro sec
	writeCMD(CMD_SYNC);
	Delay_us(3);
	writeCMD(CMD_WAKEUP);
	Delay_us(1); // min delayus: t11 = 4 * 1 / 7,68 Mhz = 0,52 micro sec
	read = readData();
	return read;
}
/*
*********************************************************************************************************
*	name: scanSEChannels
*	Get one single-ended analog input value from input channels you set (min 1, max 8).
* 	It reads a value from previous conversion!
*	DRDY needs to be low!
*	parameter: 
*	The return value: val
*********************************************************************************************************
*/
void scanSEChannels(uint8_t channels[], uint8_t numOfChannels, uint32_t *values)
{
	for (int i = 0; i < numOfChannels; ++i){
		waitDRDY();
		values[i] = getValSEChannel(channels[i]);
	}
}
/*
*********************************************************************************************************
*	name: scanDIFFChannels
*	Get one differential analog input value from input channels you set (min 1, max 4).
* 	It reads a value from previous conversion!
*	DRDY needs to be low!
*	parameter: 
*	The return value: val
*********************************************************************************************************
*/

void scanDIFFChannels(uint8_t positiveChs[], uint8_t negativeChs[], uint8_t numOfChannels, int32_t *values, int32_t dataLength)
{
	for (int j = 0; j < dataLength; j++){
		for (int i = 0; i < numOfChannels; ++i){
			waitDRDY();
			values[j] = getValDIFFChannel(positiveChs[i], negativeChs[i]);
		}
	}
}
/*
*********************************************************************************************************
*	name: scanSEChannelContinuous
*	Continuously acquire analog data from one single-ended analog input.
* 	Allows sampling of one single-ended input channel up to 30,000 SPS.
*	parameter: 
*	The return value: val
*********************************************************************************************************
*/
void scanSEChannelContinuous(uint8_t channel, uint32_t numOfMeasure, uint32_t *values, uint32_t *currentTime)
{
	uint8_t buffer[3];
	uint32_t read = 0;
	uint8_t del = 0;
	
	// Set single-ended analog input channel.
	setSEChannel(channel);
	Delay_us(del);
	
	// Set continuous mode.
	CS_0();
	waitDRDY();
	send8bit(CMD_RDATAC);
	Delay_us(del); // min delay: t6 = 50 * 1/7.68 MHz = 6.5 microseconds
	
	// Start reading data
	currentTime [numOfMeasure];
	clock_t startTime = clock();
	for (int i = 0; i < numOfMeasure; ++i)
	{
		waitDRDY();
		buffer[0] = receive8bit();
		buffer[1] = receive8bit();
		buffer[2] = receive8bit();

		// construct 24 bit value
		read  = ((uint32_t)buffer[0] << 16) & 0x00FF0000;
		read |= ((uint32_t)buffer[1] << 8);
		read |= buffer[2];
		if (read & 0x800000){
			read |= 0xFF000000;
		}
		values[i] = read;
		currentTime[i] = clock() - startTime;
		//printf("%f %i\n", (float)read/1670000, clock() - startTime); // TESTING
		Delay_us(del);
	}
	
	// Stop continuous mode.
	waitDRDY();
	send8bit(CMD_SDATAC); // Stop read data continuous.
	CS_1();
}

/*
*********************************************************************************************************
*	name: scanDIFFChannelContinuous
*	Continuously acquire analog data from one differential analog input.
*   Allows sampling of one differential input channel up to 30,000 SPS.
*	parameter: 
*	The return value: val
*********************************************************************************************************
*/
void scanDIFFChannelContinuous(uint8_t positiveCh, uint8_t negativeCh, uint32_t numOfMeasure, int32_t *values, uint32_t *currentTime)
{
	uint8_t buffer[3];
	int32_t read = 0;
	uint8_t del = 10;

	// Set differential analog input channel.
	setDIFFChannel(positiveCh, negativeCh);
	Delay_us(del);

	// Set continuous mode.
	CS_0();
	waitDRDY();
	send8bit(CMD_RDATAC);
	Delay_us(del); // min delay: t6 = 50 * 1/7.68 MHz = 6.5 microseconds

	// Start reading data.
	currentTime [numOfMeasure];	
	clock_t startTime = clock();
	for (int i = 0; i < numOfMeasure; ++i)
	{
		waitDRDY();
		buffer[0] = receive8bit();
		buffer[1] = receive8bit();
		buffer[2] = receive8bit();

		// construct 24 bit value
		read  = ((uint32_t)buffer[0] << 16) & 0x00FF0000;
		read |= ((uint32_t)buffer[1] << 8); 
		read |= buffer[2];
		if (read & 0x800000){
			read |= 0xFF000000;
			
		}
		values[i] = read;
		currentTime[i] = clock() - startTime;
		//printf("%f %i\n", (float)read/1670000, clock() - startTime); // TESTING
		Delay_us(del);
	}

	// Stop continuous mode.
	waitDRDY();
	send8bit(CMD_SDATAC); // Stop read data continuous.
	CS_1();
}


/*
*********************************************************************************************************
*	name: saveCsvFile
*	Continuously save data to csv file
*	parameter: Data
*	The return value: val
*********************************************************************************************************
*/

void saveCsvFile(int32_t* values_DIFF_CONT,int32_t dataLength)
{
	std::fstream outFile;
	int32_t trueValue = 0;
	outFile.open("dataset.csv");
	for(size_t i=0; i< dataLength;i++)
	{
		outFile << values_DIFF_CONT[i] << std::endl;
	}
		
	outFile.close();
	printf("Save file done\n");
	
}
