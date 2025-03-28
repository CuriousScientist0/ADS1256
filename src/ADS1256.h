//ADS1256 header file
/*
 Name:		ADS1256.h
 Created:	2022/07/14
 Author:	Curious Scientist
 Editor:	Notepad++
 Comment: Visit https://curiousscientist.tech/blog/ADS1256-custom-library
 Special thanks to 
 Abraão Queiroz for spending time on the code and suggesting corrections for ESP32 microcontrollers
 Benjamin Pelletier for pointing out and fixing an issue around the handling of the DRDY signal
*/

#ifndef _ADS1256_h
#define _ADS1256_h

//Differential inputs
#define DIFF_0_1 0b00000001 //A0 + A1 as differential input
#define DIFF_2_3 0b00100011 //A2 + A3 as differential input
#define DIFF_4_5 0b01000101 //A4 + A5 as differential input
#define DIFF_6_7 0b01100111 //A6 + A7 as differential input

//Single-ended inputs
#define SING_0 0b00001111 //A0 + GND (common) as single-ended input
#define SING_1 0b00011111 //A1 + GND (common) as single-ended input
#define SING_2 0b00101111 //A2 + GND (common) as single-ended input
#define SING_3 0b00111111 //A3 + GND (common) as single-ended input
#define SING_4 0b01001111 //A4 + GND (common) as single-ended input
#define SING_5 0b01011111 //A5 + GND (common) as single-ended input
#define SING_6 0b01101111 //A6 + GND (common) as single-ended input
#define SING_7 0b01111111 //A7 + GND (common) as single-ended input

//PGA settings			  //Input voltage range
#define PGA_1 0b00000000  //± 5 V
#define PGA_2 0b00000001  //± 2.5 V
#define PGA_4 0b00000010  //± 1.25 V
#define PGA_8 0b00000011  //± 625 mV
#define PGA_16 0b00000100 //± 312.5 mV
#define PGA_32 0b00000101 //+ 156.25 mV
#define PGA_64 0b00000110 //± 78.125 mV

//Datarate						  //DEC
#define DRATE_30000SPS 0b11110000 //240
#define DRATE_15000SPS 0b11100000 //224
#define DRATE_7500SPS 0b11010000  //208
#define DRATE_3750SPS 0b11000000  //192
#define DRATE_2000SPS 0b10110000  //176
#define DRATE_1000SPS 0b10100001  //161
#define DRATE_500SPS 0b10010010   //146
#define DRATE_100SPS 0b10000010   //130
#define DRATE_60SPS 0b01110010    //114
#define DRATE_50SPS 0b01100011    //99
#define DRATE_30SPS 0b01010011    //83
#define DRATE_25SPS 0b01000011    //67
#define DRATE_15SPS 0b00110011    //51
#define DRATE_10SPS 0b00100011    //35
#define DRATE_5SPS 0b00010011     //19
#define DRATE_2SPS 0b00000011     //3

//Status register
#define BITORDER_MSB 0
#define BITORDER_LSB 1
#define ACAL_DISABLED 0
#define ACAL_ENABLED 1
#define BUFFER_DISABLED 0
#define BUFFER_ENABLED 1

//Register addresses
#define STATUS_REG 0x00
#define MUX_REG 0x01
#define ADCON_REG 0x02
#define DRATE_REG 0x03
#define IO_REG 0x04
#define OFC0_REG 0x05
#define OFC1_REG 0x06
#define OFC2_REG 0x07
#define FSC0_REG 0x08
#define FSC1_REG 0x09
#define FSC2_REG 0x0A

//Command definitions
#define WAKEUP 0b00000000
#define RDATA 0b00000001
#define RDATAC 0b00000011
#define SDATAC 0b00001111
#define RREG 0b00010000
#define WREG 0b01010000
#define SELFCAL 0b11110000
#define SELFOCAL 0b11110001
#define SELFGCAL 0b11110010
#define SYSOCAL 0b11110011
#define SYSGCAL 0b11110100
#define SYNC 0b11111100
#define STANDBY 0b11111101
#define RESET 0b11111110
//----------------------------------------------------------------

class ADS1256
{
	
public:

	//Constructor
	ADS1256(const byte DRDY_pin, const byte RESET_pin, const byte SYNC_pin, const byte CS_pin, float VREF);
	
	//Initializing function
	void InitializeADC();	
	//ADS1256(int drate, int pga, int byteOrder, bool bufen);
	
	//Read a register
	long readRegister(uint8_t registerAddress);
	
	//Write a register
	void writeRegister(uint8_t registerAddress, uint8_t registerValueToWrite);	

	//Individual methods
	void setDRATE(uint8_t drate);
	void setPGA(uint8_t pga);
	uint8_t getPGA();
	void setMUX(uint8_t mux);
	void setByteOrder(uint8_t byteOrder);
	uint8_t getByteOrder();
	void setBuffer(uint8_t bufen);
	uint8_t getBuffer();
	void setAutoCal(uint8_t acal);
	uint8_t getAutoCal();
	void setGPIO(uint8_t dir0, uint8_t dir1, uint8_t dir2, uint8_t dir3);
	void writeGPIO(uint8_t dir0value, uint8_t dir1value, uint8_t dir2value, uint8_t dir3value);
	uint8_t readGPIO(uint8_t gpioPin);	
	void setCLKOUT(uint8_t clkout);
	void setSDCS(uint8_t sdcs);	
	void sendDirectCommand(uint8_t directCommand);	

	//Get a single conversion
	long readSingle();
	
	//Single input continuous reading
	long readSingleContinuous();
	
	//Cycling through the single-ended inputs
	long cycleSingle(); //Ax + COM
	
	//Cycling through the differential inputs
	long cycleDifferential(); //Ax + Ay
		
	//Converts the reading into a voltage value
	float convertToVoltage(int32_t rawData);
		
	//Stop AD
	void stopConversion();
	
private:

void waitForLowDRDY(); // Block until DRDY is low
void waitForHighDRDY(); // Block until DRDY is high
void updateMUX(uint8_t muxValue);

void updateConversionParameter(); //Refresh the conversion parameter based on the PGA

float _VREF = 0; //Value of the reference voltage
float conversionParameter = 0; //PGA-dependent multiplier
//Pins
byte _DRDY_pin; //Pin assigned for DRDY
byte _RESET_pin; //Pin assigned for RESET
byte _SYNC_pin; //Pin assigned for SYNC
byte _CS_pin; //Pin assigned for CS

//Register values
byte _DRATE; //Value of the DRATE register
byte _ADCON; //Value of the ADCON register
byte _MUX; //Value of the MUX register
byte _PGA; //Value of the PGA (within ADCON)
byte _GPIO; //Value of the GPIO register
byte _STATUS; //Value of the status register
byte _GPIOvalue; //GPIO value
byte _ByteOrder; //Byte order

byte _outputBuffer[3]; //3-byte (24-bit) buffer for the fast acquisition - Single-channel, continuous
long _outputValue; //Combined value of the _outputBuffer[3]
bool _isAcquisitionRunning; //bool that keeps track of the acquisition (running or not)
uint8_t _cycle; //Tracks the cycles as the MUX is cycling through the input channels
};




#endif
