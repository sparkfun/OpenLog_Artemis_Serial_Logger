/*
  OpenLog Artemis Serial Logging
  By: Paul Clark

  Based on: OpenLog Artemis
  By: Nathan Seidle
  SparkFun Electronics
  Date: September 18th 2021
  
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/16832

  This firmware runs the OpenLog Artemis abd is dedicated to logging serial data via the RX pin.
  A large variety of system settings can be adjusted by connecting at 115200bps.

  The Board should be set to SparkFun Apollo3 \ RedBoard Artemis ATP.

  Please note: this version of the firmware compiles on v2.1.0 of the Apollo3 boards.

*/

const int FIRMWARE_VERSION_MAJOR = 1;
const int FIRMWARE_VERSION_MINOR = 0;

//Define the OLA board identifier:
//  This is an int which is unique to this variant of the OLA and which allows us
//  to make sure that the settings in EEPROM are correct for this version of the OLA
//  (sizeOfSettings is not necessarily unique and we want to avoid problems when swapping from one variant to another)
//  It is the sum of:
//    the variant * 0x100 (OLA = 1; GNSS_LOGGER = 2; GEOPHONE_LOGGER = 3; SERIAL_LOGGER = 4)
//    the major firmware version * 0x10
//    the minor firmware version
#define OLA_IDENTIFIER 0x410 // Stored as 1040 decimal in OLA_Serial_settings.txt

#include "settings.h"

//Define the pin functions
//Depends on hardware version. This can be found as a marking on the PCB.
//x04 was the SparkX 'black' version.
//v10 was the first red version.
#define HARDWARE_VERSION_MAJOR 1
#define HARDWARE_VERSION_MINOR 0

#if(HARDWARE_VERSION_MAJOR == 0 && HARDWARE_VERSION_MINOR == 4)
const byte PIN_MICROSD_CHIP_SELECT = 10;
const byte PIN_IMU_POWER = 22;
#elif(HARDWARE_VERSION_MAJOR == 1 && HARDWARE_VERSION_MINOR == 0)
const byte PIN_MICROSD_CHIP_SELECT = 23;
const byte PIN_IMU_POWER = 27;
const byte PIN_PWR_LED = 29;
const byte PIN_VREG_ENABLE = 25;
const byte PIN_VIN_MONITOR = 34; // VIN/3 (1M/2M - will require a correction factor)
#endif

const byte PIN_POWER_LOSS = 3;
//const byte PIN_LOGIC_DEBUG = 11; // Useful for debugging issues like the slippery mux bug
const byte PIN_MICROSD_POWER = 15;
const byte PIN_QWIIC_POWER = 18;
const byte PIN_STAT_LED = 19;
const byte PIN_IMU_INT = 37;
const byte PIN_IMU_CHIP_SELECT = 44;
const byte PIN_STOP_LOGGING = 32;
const byte BREAKOUT_PIN_32 = 32;
const byte BREAKOUT_PIN_TX = 12;
const byte BREAKOUT_PIN_RX = 13;
const byte BREAKOUT_PIN_11 = 11;
const byte PIN_TRIGGER = 11;
const byte PIN_QWIIC_SCL = 8;
const byte PIN_QWIIC_SDA = 9;

const byte PIN_SPI_SCK = 5;
const byte PIN_SPI_CIPO = 6;
const byte PIN_SPI_COPI = 7;

enum returnStatus {
  STATUS_GETBYTE_TIMEOUT = 255,
  STATUS_GETNUMBER_TIMEOUT = -123455555,
  STATUS_PRESSED_X,
};

//Setup Qwiic Port
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include <Wire.h>
TwoWire qwiic(PIN_QWIIC_SDA,PIN_QWIIC_SCL); //Will use pads 8/9
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//EEPROM for storing settings
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include <EEPROM.h>
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//microSD Interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include <SPI.h>

#include <SdFat.h> //SdFat v2.0.7 by Bill Greiman: http://librarymanager/All#SdFat_exFAT

#define SD_FAT_TYPE 3 // SD_FAT_TYPE = 0 for SdFat/File, 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_CONFIG SdSpiConfig(PIN_MICROSD_CHIP_SELECT, SHARED_SPI, SD_SCK_MHZ(24)) // 24MHz

#if SD_FAT_TYPE == 1
SdFat32 sd;
File32 serialDataFile; //File that all incoming serial data is written to
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile serialDataFile; //File that all incoming serial data is written to
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile serialDataFile; //File that all incoming serial data is written to
#else // SD_FAT_TYPE == 0
SdFat sd;
File serialDataFile; //File that all incoming serial data is written to
#endif  // SD_FAT_TYPE

static char serialDataFileName[30] = ""; //We keep a record of this file name so that we can re-open it upon wakeup from sleep
const int sdPowerDownDelay = 100; //Delay for this many ms before turning off the SD card power
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Add RTC interface for Artemis
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include "RTC.h" //Include RTC library included with the Arduino_Apollo3 core
Apollo3RTC myRTC; //Create instance of RTC class
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

volatile unsigned long lastSeriaLogSyncTime = 0;
const unsigned long MAX_IDLE_TIME_MSEC = 1000;
#define incomingBufferSize (512*64)
static char incomingBuffer[incomingBufferSize]; //This size of this buffer is sensitive
volatile int incomingBufferSpot = 0;
volatile unsigned long charsReceived = 0; //Used for verifying/debugging serial reception
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
volatile unsigned long lastSDFileNameChangeTime; //Used to calculate the interval since the last SD filename change
const byte menuTimeout = 15; //Menus will exit/timeout after this number of seconds
volatile static bool stopLoggingSeen = false; //Flag to indicate if we should stop logging
volatile int lowBatteryReadings = 0; // Count how many times the battery voltage has read low
const int lowBatteryReadingsLimit = 10; // Don't declare the battery voltage low until we have had this many consecutive low readings (to reject sampling noise)
volatile static bool triggerEdgeSeen = false; //Flag to indicate if a trigger interrupt has been seen
static char serialTimestamp[40]; //Buffer to store serial timestamp, if needed
volatile static bool powerLossSeen = false; //Flag to indicate if a power loss event has been seen
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// gfvalvo's flash string helper code: https://forum.arduino.cc/index.php?topic=533118.msg3634809#msg3634809
void SerialPrint(const char *);
void SerialPrint(const __FlashStringHelper *);
void SerialPrintln(const char *);
void SerialPrintln(const __FlashStringHelper *);
void DoSerialPrint(char (*)(const char *), const char *, bool newLine = false);

#define DUMP( varname ) {Serial.printf("%s: %d\r\n", #varname, varname);}
#define SerialPrintf1( var ) {Serial.printf( var );}
#define SerialPrintf2( var1, var2 ) {Serial.printf( var1, var2 );}
#define SerialPrintf3( var1, var2, var3 ) {Serial.printf( var1, var2, var3 );}
#define SerialPrintf4( var1, var2, var3, var4 ) {Serial.printf( var1, var2, var3, var4 );}
#define SerialPrintf5( var1, var2, var3, var4, var5 ) {Serial.printf( var1, var2, var3, var4, var5 );}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// create a Thread object (from the rtos namespace)
// https://os.mbed.com/docs/mbed-os/v6.2/apis/thread.html

rtos::Thread thread1;  
rtos::Thread thread2;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

volatile bool threadSerialRXrun = true;
volatile bool muteTerminalOutput = false;
volatile bool threadSDrun = true;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// This thread reads the incoming serial data and stores it in incomingBuffer
void thread_serial_rx(void)
{
  while (1)
  {
    if (threadSerialRXrun)
    {
      size_t timestampCharsLeftToWrite = strlen(serialTimestamp);
    
      if (Serial1.available() || (timestampCharsLeftToWrite > 0))
      {
        while (Serial1.available() || (timestampCharsLeftToWrite > 0))
        {
          if (timestampCharsLeftToWrite > 0) // Based on code written by @DennisMelamed in PR #70
          {
            incomingBuffer[incomingBufferSpot] = serialTimestamp[0]; // Add a timestamp character to incomingBuffer
    
            for (size_t i = 0; i < timestampCharsLeftToWrite; i++)
            {
              serialTimestamp[i] = serialTimestamp[i+1]; // Shuffle the remaining chars along by one
            }
    
            timestampCharsLeftToWrite -= 1;
          }
          else
          {
            incomingBuffer[incomingBufferSpot] = Serial1.read();
    
            //Get the RTC timestamp if we just received the timestamp token
            if (settings.timestampSerial && (incomingBuffer[incomingBufferSpot] == settings.timeStampToken))
            {
              getTimeString(&serialTimestamp[3]);
              serialTimestamp[0] = '\r'; // Add Carriage Return and Line Feed at the start of the timestamp
              serialTimestamp[1] = '\n';
              serialTimestamp[2] = '^'; // Add an up-arrow to indicate the timestamp relates to the preceeding data
            }
          }
    
          //Print to terminal
          if ((settings.enableTerminalOutput == true) && (muteTerminalOutput == false))
            Serial.write(incomingBuffer[incomingBufferSpot]); //Print to terminal
    
          //Increment the buffer pointer - unless the buffer is full
          if (incomingBufferSpot < (incomingBufferSize - 1))
            incomingBufferSpot++;
            
          charsReceived++; // Update the serial character count
        }
      }
    }
  }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// This thread writes the data in incomingBuffer to SD card
void thread_sd_write(void)
{
  while (1)
  {
    if (threadSDrun)
    {
      if (settings.logSerial == true && online.serialLogging == true)
      {
        while (incomingBufferSpot >= 512) // Is there enough data in the buffer for a complete write?
        {
          digitalWrite(PIN_STAT_LED, HIGH); //Toggle stat LED to indicating log recording
          serialDataFile.write(incomingBuffer, 512); //Record the buffer to the card
          digitalWrite(PIN_STAT_LED, LOW);
          memmove(&incomingBuffer[0], &incomingBuffer[512], incomingBufferSize - 512);
          incomingBufferSpot -= 512;
    
          if ((millis() - lastSeriaLogSyncTime) > MAX_IDLE_TIME_MSEC) //Should we sync the log file?
          {
            serialDataFile.sync();
            lastSeriaLogSyncTime = millis(); //Reset the last sync time to now
          }
        }
    
        if ((millis() - lastSeriaLogSyncTime) > MAX_IDLE_TIME_MSEC) //If we haven't received any characters recently then sync log file
        {
          if (incomingBufferSpot > 0)
          {
            //Write the remainder of the buffer
            digitalWrite(PIN_STAT_LED, HIGH); //Toggle stat LED to indicating log recording
            serialDataFile.write(incomingBuffer, incomingBufferSpot); //Record the buffer to the card
            serialDataFile.sync();
            updateDataFileAccess(&serialDataFile); // Update the file access time & date
            digitalWrite(PIN_STAT_LED, LOW);
            incomingBufferSpot = 0;
          }
          lastSeriaLogSyncTime = millis(); //Reset the last sync time to now
          if (muteTerminalOutput == false)
            printDebug("\r\nTotal chars received: " + (String)charsReceived + "\r\n");
        }
      }
    }
  }
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void setup() {
  //If 3.3V rail drops below 3V, system will power down and maintain RTC
  pinMode(PIN_POWER_LOSS, INPUT); // BD49K30G-TL has CMOS output and does not need a pull-up

  delay(1); // Let PIN_POWER_LOSS stabilize

  if (digitalRead(PIN_POWER_LOSS) == LOW) powerDownOLA(); //Check PIN_POWER_LOSS just in case we missed the falling edge
  //attachInterrupt(PIN_POWER_LOSS, powerDownOLA, FALLING); // We can't do this with v2.1.0 as attachInterrupt causes a spontaneous interrupt
  attachInterrupt(PIN_POWER_LOSS, powerLossISR, FALLING);
  powerLossSeen = false; // Make sure the flag is clear

  powerLEDOn(); // Turn the power LED on - if the hardware supports it

  pinMode(PIN_STAT_LED, OUTPUT);
  digitalWrite(PIN_STAT_LED, HIGH); // Turn the STAT LED on while we configure everything

  SPI.begin(); //Needed if SD is disabled

  Serial.begin(115200); //Default for initial debug messages if necessary

  //pinMode(PIN_LOGIC_DEBUG, OUTPUT); // Debug pin to assist tracking down slippery mux bugs
  //digitalWrite(PIN_LOGIC_DEBUG, HIGH);

  EEPROM.init();

  beginSD(); //285 - 293ms

  enableCIPOpullUp(); // Enable CIPO pull-up _after_ beginSD

  loadSettings(); //50 - 250ms

  Serial.flush(); //Complete any previous prints
  Serial.begin(settings.serialTerminalBaudRate);

  SerialPrintf3("Artemis OpenLog Serial Logger v%d.%d\r\n", FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR);

  if (settings.useGPIO32ForStopLogging == true)
  {
    SerialPrintln(F("Stop Logging is enabled. Pull GPIO pin 32 to GND to stop logging."));
    pinMode(PIN_STOP_LOGGING, INPUT_PULLUP);
    delay(1); // Let the pin stabilize
    attachInterrupt(PIN_STOP_LOGGING, stopLoggingISR, FALLING); // Enable the interrupt
    am_hal_gpio_pincfg_t intPinConfig = g_AM_HAL_GPIO_INPUT_PULLUP;
    intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_HI2LO;
    pin_config(PinName(PIN_STOP_LOGGING), intPinConfig); // Make sure the pull-up does actually stay enabled
    stopLoggingSeen = false; // Make sure the flag is clear
  }

  if (settings.useGPIO11ForTrigger == true)
  {
    pinMode(PIN_TRIGGER, INPUT_PULLUP);
    delay(1); // Let the pin stabilize
    am_hal_gpio_pincfg_t intPinConfig = g_AM_HAL_GPIO_INPUT_PULLUP;
    if (settings.fallingEdgeTrigger == true)
    {
      SerialPrintln(F("New File Interrupt is enabled. A new serial log file will be opened on a falling edge on GPIO pin 11."));
      attachInterrupt(PIN_TRIGGER, triggerPinISR, FALLING); // Enable the interrupt
      intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_HI2LO;
    }
    else
    {
      SerialPrintln(F("New File Interrupt is enabled. A new serial log file will be opened on a rising edge on GPIO pin 11."));
      attachInterrupt(PIN_TRIGGER, triggerPinISR, RISING); // Enable the interrupt
      intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI;
    }
    pin_config(PinName(PIN_TRIGGER), intPinConfig); // Make sure the pull-up does actually stay enabled
    triggerEdgeSeen = false; // Make sure the flag is clear
  }

  lastSDFileNameChangeTime = millis(); // Record the time of the file name change

  serialTimestamp[0] = '\0'; // Empty the serial timestamp buffer

  beginSerialLogging(); //20 - 99ms

  if (online.microSD == true) SerialPrintln(F("SD card online"));
  else SerialPrintln(F("SD card offline"));

  if (online.serialLogging == true) SerialPrintln(F("Serial logging online"));
  else SerialPrintln(F("Serial logging offline"));

  if (settings.enableTerminalOutput == false && online.serialLogging == true)
    SerialPrintln(F("Logging to microSD card with no terminal output"));

  //Start the threads
  thread1.start(thread_serial_rx);
  thread2.start(thread_sd_write);

  digitalWrite(PIN_STAT_LED, LOW); // Turn the STAT LED off now that everything is configured
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void loop() {

  //Pause the threads
  threadSerialRXrun = false;
  threadSDrun = false;

  checkBattery(); // Check for low battery

  //Unpause the serial rx thread
  threadSerialRXrun = true;

  //Check for a New File trigger event
  if (((settings.useGPIO11ForTrigger == true) && (triggerEdgeSeen == true))
   || ((settings.openNewLogFilesAfter > 0) && ((millis() - lastSDFileNameChangeTime) > (settings.openNewLogFilesAfter * 1000))))
  {
    openNewLogFile();
    lastSDFileNameChangeTime = millis(); // Record the time of the file name change
  }

  if ((settings.useGPIO32ForStopLogging == true) && (stopLoggingSeen == true)) // Has the user pressed the stop logging button?
  {
    stopLogging();
  }

  //Unpause the SD write thread
  threadSDrun = true;

  if (Serial.available())
  {
    muteTerminalOutput = true;
    menuMain(); //Present user menu
    muteTerminalOutput = false;
  }

  //Let the threads do their thing
  delay(100);

}

void beginQwiic()
{
  pinMode(PIN_QWIIC_POWER, OUTPUT);
  pin_config(PinName(PIN_QWIIC_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  qwiicPowerOn();
  qwiic.begin();
  setQwiicPullups(1); //Just to make it really clear what pull-ups are being used, set pullups here.
}

void setQwiicPullups(uint32_t qwiicBusPullUps)
{
  //Change SCL and SDA pull-ups manually using pin_config
  am_hal_gpio_pincfg_t sclPinCfg = g_AM_BSP_GPIO_IOM1_SCL;
  am_hal_gpio_pincfg_t sdaPinCfg = g_AM_BSP_GPIO_IOM1_SDA;

  if (qwiicBusPullUps == 0)
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE; // No pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE;
  }
  else if (qwiicBusPullUps == 1)
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K; // Use 1K5 pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K;
  }
  else if (qwiicBusPullUps == 6)
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_6K; // Use 6K pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_6K;
  }
  else if (qwiicBusPullUps == 12)
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_12K; // Use 12K pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_12K;
  }
  else
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_24K; // Use 24K pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_24K;
  }

  pin_config(PinName(PIN_QWIIC_SCL), sclPinCfg);
  pin_config(PinName(PIN_QWIIC_SDA), sdaPinCfg);
}

void beginSD()
{
  pinMode(PIN_MICROSD_POWER, OUTPUT);
  pin_config(PinName(PIN_MICROSD_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  pinMode(PIN_MICROSD_CHIP_SELECT, OUTPUT);
  pin_config(PinName(PIN_MICROSD_CHIP_SELECT), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_MICROSD_CHIP_SELECT, HIGH); //Be sure SD is deselected

  if (settings.enableSD == true)
  {
    // For reasons I don't understand, we seem to have to wait for at least 1ms after SPI.begin before we call microSDPowerOn.
    // If you comment the next line, the Artemis resets at microSDPowerOn when beginSD is called from wakeFromSleep...
    // But only on one of my V10 red boards. The second one I have doesn't seem to need the delay!?
    delay(5);

    microSDPowerOn();

    //Max power up time is 250ms: https://www.kingston.com/datasheets/SDCIT-specsheet-64gb_en.pdf
    //Max current is 200mA average across 1s, peak 300mA
    for (int i = 0; i < 10; i++) //Wait
    {
      checkBattery();
      delay(1);
    }

    if (sd.begin(SD_CONFIG) == false) // Try to begin the SD card using the correct chip select
    {
      printDebug(F("SD init failed (first attempt). Trying again...\r\n"));
      for (int i = 0; i < 250; i++) //Give SD more time to power up, then try again
      {
        checkBattery();
        delay(1);
      }
      if (sd.begin(SD_CONFIG) == false) // Try to begin the SD card using the correct chip select
      {
        SerialPrintln(F("SD init failed (second attempt). Is card present? Formatted?"));
        SerialPrintln(F("Please ensure the SD card is formatted correctly using https://www.sdcard.org/downloads/formatter/"));
        digitalWrite(PIN_MICROSD_CHIP_SELECT, HIGH); //Be sure SD is deselected
        online.microSD = false;
        return;
      }
    }

    //Change to root directory. All new file creation will be in root.
    if (sd.chdir() == false)
    {
      SerialPrintln(F("SD change directory failed"));
      online.microSD = false;
      return;
    }

    online.microSD = true;
  }
  else
  {
    microSDPowerOff();
    online.microSD = false;
  }
}

void enableCIPOpullUp() // updated for v2.1.0 of the Apollo3 core
{
  //Add 1K5 pull-up on CIPO
  am_hal_gpio_pincfg_t cipoPinCfg = g_AM_BSP_GPIO_IOM0_MISO;
  cipoPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K;
  pin_config(PinName(PIN_SPI_CIPO), cipoPinCfg);
}

void disableCIPOpullUp() // updated for v2.1.0 of the Apollo3 core
{
  am_hal_gpio_pincfg_t cipoPinCfg = g_AM_BSP_GPIO_IOM0_MISO;
  pin_config(PinName(PIN_SPI_CIPO), cipoPinCfg);
}

void configureSerial1TxRx(void) // Configure pins 12 and 13 for UART1 TX and RX
{
  am_hal_gpio_pincfg_t pinConfigTx = g_AM_BSP_GPIO_COM_UART_TX;
  pinConfigTx.uFuncSel = AM_HAL_PIN_12_UART1TX;
  pin_config(PinName(BREAKOUT_PIN_TX), pinConfigTx);
  am_hal_gpio_pincfg_t pinConfigRx = g_AM_BSP_GPIO_COM_UART_RX;
  pinConfigRx.uFuncSel = AM_HAL_PIN_13_UART1RX;
  pinConfigRx.ePullup = AM_HAL_GPIO_PIN_PULLUP_WEAK; // Put a weak pull-up on the Rx pin
  pin_config(PinName(BREAKOUT_PIN_RX), pinConfigRx);
}

void beginSerialLogging()
{
  //We may need to manually restore the Serial1 TX and RX pins
  configureSerial1TxRx();

  Serial1.begin(settings.serialLogBaudRate);

  if (online.microSD == true && settings.logSerial == true)
  {
    //If we don't have a file yet, create one. Otherwise, re-open the last used file
    if (strlen(serialDataFileName) == 0)
      strcpy(serialDataFileName, findNextAvailableLog(settings.nextSerialLogNumber, "serialLog"));

    if (serialDataFile.open(serialDataFileName, O_CREAT | O_APPEND | O_WRITE) == false)
    {
      SerialPrintln(F("Failed to create serial log file"));
      online.serialLogging = false;
      return;
    }

    updateDataFileCreate(&serialDataFile); // Update the file create time & date

    online.serialLogging = true;
  }
  else
    online.serialLogging = false;
}

void openNewLogFile()
{
  if (online.microSD == true && online.serialLogging == true)
  {
    if (incomingBufferSpot > 0)
    {
      //Write the remainder of the buffer
      digitalWrite(PIN_STAT_LED, HIGH); //Toggle stat LED to indicating log recording
      serialDataFile.write(incomingBuffer, incomingBufferSpot); //Record the buffer to the card
      serialDataFile.sync();
      updateDataFileAccess(&serialDataFile); // Update the file access time & date
      digitalWrite(PIN_STAT_LED, LOW);
      incomingBufferSpot = 0;
    }
    lastSeriaLogSyncTime = millis(); //Reset the last sync time to now
    printDebug("Total chars received: " + (String)charsReceived + "\r\n");

    SerialPrint(F("Closing: "));
    SerialPrintln(serialDataFileName);

    serialDataFile.close();
      
    strcpy(serialDataFileName, findNextAvailableLog(settings.nextSerialLogNumber, "serialLog"));

    if (serialDataFile.open(serialDataFileName, O_CREAT | O_APPEND | O_WRITE) == false)
    {
      SerialPrintln(F("Failed to create new serial log file"));
      online.serialLogging = false;
      return;
    }

    updateDataFileCreate(&serialDataFile); // Update the file create time & date
  }
}

#if SD_FAT_TYPE == 1
void updateDataFileCreate(File32 *dataFile)
#elif SD_FAT_TYPE == 2
void updateDataFileCreate(ExFile *dataFile)
#elif SD_FAT_TYPE == 3
void updateDataFileCreate(FsFile *dataFile)
#else // SD_FAT_TYPE == 0
void updateDataFileCreate(File *dataFile)
#endif  // SD_FAT_TYPE
{
  myRTC.getTime(); //Get the RTC time so we can use it to update the create time
  //Update the file create time
  dataFile->timestamp(T_CREATE, (myRTC.year + 2000), myRTC.month, myRTC.dayOfMonth, myRTC.hour, myRTC.minute, myRTC.seconds);
}

#if SD_FAT_TYPE == 1
void updateDataFileAccess(File32 *dataFile)
#elif SD_FAT_TYPE == 2
void updateDataFileAccess(ExFile *dataFile)
#elif SD_FAT_TYPE == 3
void updateDataFileAccess(FsFile *dataFile)
#else // SD_FAT_TYPE == 0
void updateDataFileAccess(File *dataFile)
#endif  // SD_FAT_TYPE
{
  myRTC.getTime(); //Get the RTC time so we can use it to update the last modified time
  //Update the file access time
  dataFile->timestamp(T_ACCESS, (myRTC.year + 2000), myRTC.month, myRTC.dayOfMonth, myRTC.hour, myRTC.minute, myRTC.seconds);
  //Update the file write time
  dataFile->timestamp(T_WRITE, (myRTC.year + 2000), myRTC.month, myRTC.dayOfMonth, myRTC.hour, myRTC.minute, myRTC.seconds);
}

//Power Loss ISR
void powerLossISR(void)
{
  powerLossSeen = true;
}

//Stop Logging ISR
void stopLoggingISR(void)
{
  stopLoggingSeen = true;
}

//Trigger Pin ISR
void triggerPinISR(void)
{
  triggerEdgeSeen = true;
}

void SerialFlush(void)
{
  Serial.flush();
}

// gfvalvo's flash string helper code: https://forum.arduino.cc/index.php?topic=533118.msg3634809#msg3634809

void SerialPrint(const char *line)
{
  DoSerialPrint([](const char *ptr) {return *ptr;}, line);
}

void SerialPrint(const __FlashStringHelper *line)
{
  DoSerialPrint([](const char *ptr) {return (char) pgm_read_byte_near(ptr);}, (const char*) line);
}

void SerialPrintln(const char *line)
{
  DoSerialPrint([](const char *ptr) {return *ptr;}, line, true);
}

void SerialPrintln(const __FlashStringHelper *line)
{
  DoSerialPrint([](const char *ptr) {return (char) pgm_read_byte_near(ptr);}, (const char*) line, true);
}

void DoSerialPrint(char (*funct)(const char *), const char *string, bool newLine)
{
  char ch;

  while ((ch = funct(string++)))
  {
    Serial.print(ch);
  }

  if (newLine)
  {
    Serial.print(F("\r\n"));
  }
}
