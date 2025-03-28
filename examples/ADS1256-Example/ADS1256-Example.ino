//This code belongs to the ADS1256 library developed by Curious Scientist
//A very detailed documentation can be found at: https://curiousscientist.tech/ads1256-custom-library

#include <ADS1256.h>

//Below a few examples of pin descriptions for different microcontrollers I used:
//ADS1256 A(2, 0, 8, 10, 2.500); //DRDY, RESET, SYNC(PDWN), CS, VREF(float).    //Arduino Nano
//ADS1256 A(7, 10, 5, 9, 2.500); //DRDY, RESET, SYNC(PDWN), CS, VREF(float).      //ATmega32U4
//ADS1256 A(PA2, 0, 0, PA4, 2.500); //DRDY, RESET, SYNC(PDWN), CS, VREF(float). //STM32 blue pill
//ADS1256 A(16, 17, 0, 5, 2.500); //DRDY, RESET, SYNC(PDWN), CS, VREF(float).   //ESP32 WROOM 32
//ADS1256 A(7, 0, 8, 10, 2.500); //DRDY, RESET, SYNC(PDWN), CS, VREF(float).    //Teensy 4.0
//ADS1256 A(7, 0, 6, 5, 2.500); //DRDY, RESET, SYNC(PDWN), CS, VREF(float).    //RP2040 Waveshare Mini
ADS1256 A(18, 20, 21, 19, 2.500); //DRDY, RESET, SYNC(PDWN), CS, VREF(float).  //RP2040 Zero

long rawConversion = 0; //24-bit raw value
float voltageValue = 0; //human-readable floating point value

int singleEndedChannels[8] = {SING_0, SING_1, SING_2, SING_3, SING_4, SING_5, SING_6, SING_7}; //Array to store the single-ended channels
int differentialChannels[4] = {DIFF_0_1, DIFF_2_3, DIFF_4_5, DIFF_6_7}; //Array to store the differential channels
int inputChannel = 0; //Number used to pick the channel from the above two arrays
char inputMode = ' '; //can be 's' and 'd': single-ended and differential

int pgaValues[7] = {PGA_1, PGA_2, PGA_4, PGA_8, PGA_16, PGA_32, PGA_64}; //Array to store the PGA settings
int pgaSelection = 0; //Number used to pick the PGA value from the above array

int drateValues[16] =
{
  DRATE_30000SPS,
  DRATE_15000SPS,
  DRATE_7500SPS,
  DRATE_3750SPS,
  DRATE_2000SPS,
  DRATE_1000SPS,
  DRATE_500SPS,
  DRATE_100SPS,
  DRATE_60SPS,
  DRATE_50SPS,
  DRATE_30SPS,
  DRATE_25SPS,
  DRATE_15SPS,
  DRATE_10SPS,
  DRATE_5SPS,
  DRATE_2SPS
}; //Array to store the sampling rates

int drateSelection = 0; //Number used to pick the sampling rate from the above array

String registers[11] =
{
  "STATUS",
  "MUX",
  "ADCON",
  "DRATE",
  "IO",
  "OFC0",
  "OFC1",
  "OFC2",
  "FSC0",
  "FSC1",
  "FSC2"
};//Array to store the registers

int registerToRead = 0; //Register number to be read
int registerToWrite = 0; //Register number to be written
int registerValueToWrite = 0; //Value to be written in the selected register

void setup()
{
  Serial.begin(115200); //The value does not matter if you use an MCU with native USB

  while (!Serial)
  {
    ; //Wait until the serial becomes available
  }

  Serial.println("ADS1256 - Custom Library Demo File by Curious Scientist - 2025-03-28");

  A.InitializeADC(); //See the documentation for every details
  //Setting up CS, RESET, SYNC and SPI
  //Assigning default values to: STATUS, MUX, ADCON, DRATE
  //Performing a SYSCAL

  //Below is a demonstration to change the values through the built-on functions of the library
  //Set a PGA value
  A.setPGA(PGA_1);  //0b00000000 - DEC: 0
  //--------------------------------------------

  //Set input channels
  A.setMUX(DIFF_6_7); //0b01100111 - DEC: 103
  //--------------------------------------------

  //Set DRATE
  A.setDRATE(DRATE_5SPS); //0b00010011 - DEC: 19
  //--------------------------------------------

  //Read back the above 3 values to check if the writing was succesful
  Serial.print("PGA: ");
  Serial.println(A.getPGA());
  delay(100);
  //--
  Serial.print("MUX: ");
  Serial.println(A.readRegister(MUX_REG));
  delay(100);
  //--
  Serial.print("DRATE: ");
  Serial.println(A.readRegister(DRATE_REG));
  delay(100);

  //Freeze the display for 3 sec
  delay(3000);
}

void loop()
{
  /* Here I implemented some typical functions that can be useful for using the ADS1256
      Changing the registers by the built-in functions or by directly writing the registers
      Reading the registers to check the value of it
      Read a single conversion
      Read a single channel continuously
      Read multiple channels continuously
      Stop the conversion
      ...etc.
      All the above things are done through the serial port, the use only has to send certain commands
  */


  if (Serial.available() > 0)
  {
    char commandCharacter = Serial.read(); //we use characters (letters) for controlling the switch-case

    switch (commandCharacter) //based on the command character, we decide what to do
    {
      case 's': //SDATAC - Stop Reading Data Continously
        A.stopConversion();
        break;
      //--------------------------------------------------------------------------------------------------------
      case 'L': //Perform a self calibration
        A.sendDirectCommand(SELFCAL);
        break;
      //--------------------------------------------------------------------------------------------------------
      case 'G': //Read a single input continuously
        while (Serial.read() != 's') //The conversion is stopped by a character received from the serial port
        {
          Serial.println(A.convertToVoltage(A.readSingleContinuous()), 6);
          //The conversion is printed in Volts with 6 decimal digits
          //Note: Certain serial terminals cannot keep up with high speed datastream!
        }
        A.stopConversion();
        break;
      //--------------------------------------------------------------------------------------------------------
      case 'C': //Cycle single ended inputs (A0+GND, A1+GND ... A7+GND)
        while (Serial.read() != 's')//The conversion is stopped by a character received from the serial port
        {
          float channels[8]; //Buffer that holds 8 conversions (8 single-ended channels)
          for (int j = 0; j < 8; j++)
          {
            channels[j] = A.convertToVoltage(A.cycleSingle()); //store the converted single-ended results in the buffer
          }
          for (int i = 0; i < 8; i++)
          {
            Serial.print(channels[i], 4); //print the converted single-ended results with 4 digits

            if (i < 7) //Only printing tab between the first 7 conversions
            {
              Serial.print("\t"); //tab separator to separate the 8 conversions shown in the same line
            }
          }
          Serial.println();//Printing a linebreak - this will put the next 8 conversions in a new line
        }
        A.stopConversion();
        break;
      //--------------------------------------------------------------------------------------------------------
      case 'D': //Cycle differential inputs (A0+A1, A2+A3, A4+A5, A6+A7)
        while (Serial.read() != 's') //The conversion is stopped by a character received from the serial port
        {
          float channels[4]; //Buffer that holds 4 conversions (4 differential channels)
          for (int j = 0; j < 4; j++)
          {
            channels[j] = A.convertToVoltage(A.cycleDifferential()); //store the converted differential results in the buffer
          }

          //After the 4 conversions are in the buffer, the contents are printed on the serial terminal
          for (int i = 0; i < 4; i++)
          {
            Serial.print(channels[i], 4);//print the converted differential results from the buffer

            if (i < 3) //Only printing tab between the first 3 conversions
            {
              Serial.print("\t"); //tab separator to separate the 4 conversions shown in the same line
            }
          }
          Serial.println();//Printing a linebreak - this will put the next 4 conversions in a new line
        }
        A.stopConversion();
        break;
      //--------------------------------------------------------------------------------------------------------
      case 'B': //Speed test
        {
          //Variables to store and measure elapsed time and define the number of conversions
          long numberOfSamples = 30000; //Number of conversions
          long finishTime = 0;
          long startTime = micros();

          for (long i = 0; i < numberOfSamples; i++)
          {
            A.readSingleContinuous();
            //Note: here we just perform the readings and we don't print the results
          }

          finishTime = micros() - startTime; //Calculate the elapsed time

          A.stopConversion();

          //Printing the results
          Serial.print("Total conversion time for 150k samples: ");
          Serial.print(finishTime);
          Serial.println(" us");

          Serial.print("Sampling rate: ");
          Serial.print(numberOfSamples * (1000000.0 / finishTime), 3);
          Serial.println(" SPS");
        }
        break;
      //--------------------------------------------------------------------------------------------------------
      case 'T': //Testing the serial connection
        Serial.println("The serial connection is OK!");
        break;
      //--------------------------------------------------------------------------------------------------------
      case 'a': //Testing a single conversion - Only one single result is returned

        rawConversion = A.readSingle(); //Reading the raw value from a previously selected input, passing it to a variable
        voltageValue = A.convertToVoltage(rawConversion); //Converting the above conversion into a floating point number

        //Printing the results
        Serial.print("Single-ended conversion result: ");
        Serial.println(voltageValue, 8); //Print the floating point number with 8 digits.
        break;
      //--------------------------------------------------------------------------------------------------------
      case 'M': //set MUX
        {
          while (!Serial.available());
          inputMode = Serial.read(); //Read the input mode

          if (inputMode == 's') //single-ended
          {
            while (!Serial.available());
            inputChannel = Serial.parseInt();
            A.setMUX(singleEndedChannels[inputChannel]);
            //Example: "Ms1" selects the SING_1 as input channel
          }

          if (inputMode == 'd') //differential
          {
            while (!Serial.available());
            inputChannel = Serial.parseInt();
            A.setMUX(differentialChannels[inputChannel]);
            //Example: "Md0" selects the DIFF_0_1 as input channel
          }
        }
        break;
      //--------------------------------------------------------------------------------------------------------
      case 'P': //Set PGA
        {
          while (!Serial.available());
          pgaSelection = Serial.parseInt();
          A.setPGA(pgaValues[pgaSelection]);
          //Example: P4 will select the PGA = 16
          Serial.print("PGA value: ");
          Serial.println(A.getPGA()); //Read the PGA from the register and print it
        }
        break;
      //--------------------------------------------------------------------------------------------------------
      case 'F': //Set sampling frequency
        {
          while (!Serial.available());
          drateSelection = Serial.parseInt(); //Parse the number (item number in the array)
          delay(100);
          Serial.print("DRATE is selected as: ");
          Serial.println(drateValues[drateSelection]); //Print the value from the array
          delay(100);
          A.setDRATE(drateValues[drateSelection]); //Pass the value to the register on the ADS1256
          delay(100);
          Serial.print("DRATE is set to ");
          Serial.println(A.readRegister(DRATE_REG)); //Read the register to see if the value was updated correctly
          //Example: F3 will make the DRATE = 3750 SPS
        }
        break;
      //--------------------------------------------------------------------------------------------------------
      case 'R': //read register
        {
          while (!Serial.available());
          registerToRead = Serial.parseInt(); //This part reads the number of the register from the serial port
          Serial.print("Value of ");
          Serial.print(registers[registerToRead]);
          Serial.print(" register is: ");
          Serial.println(A.readRegister(registerToRead));
          //Example: "R2" will read the register at address 2 which is the ADCON register
          //Note: The value is printed as a decimal number
        }
        break;
      //--------------------------------------------------------------------------------------------------------
      case 'W': //Write register
        {
          while (!Serial.available());
          registerToWrite = Serial.parseInt(); //This part reads the number of the register from the serial port
          while (!Serial.available());
          registerValueToWrite = Serial.parseInt(); //This part reads the value of the register from the serial port

          A.writeRegister(registerToWrite, registerValueToWrite);
          //Example: "W1 35" will write 35 ("00100011") on register 1 which is the MUX register.
          //This will make the input as DIFF_2_3 (A2(+) & A1(-))
        }
        break;
        //--------------------------------------------------------------------------------------------------------
    }
  }
}
