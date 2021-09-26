/*
  OpenLog Artemis Serial Logging
  By: Paul Clark

  Based on: OpenLog Artemis
  By: Nathan Seidle
  SparkFun Electronics
  Date: September 26th 2021
  
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/16832

  This firmware runs the OpenLog Artemis and is dedicated to logging serial data via the RX pin.
  The code uses two threads to buffer the incoming serial data and write it to SD card.
  A large variety of system settings can be adjusted by connecting at 115200bps.

  Limitations:
    This code works well with baud rates up to 230400.
    Near-continuous (80% duty) logging of NMEA data at 230400 (close to 20KB/s) produces clean log files.
    The wheels come off at 460800 baud, for reasons I don't yet understand.

  The Board should be set to SparkFun Apollo3 \ RedBoard Artemis ATP.

  Please note: this version of the firmware was compiled with v2.1.0 of the Apollo3 boards.
  v2.2.0 of Apollo3 should work nicely too.

  The code uses v2.0.7 of SdFat by Bill Greiman (_not_ v2.1.0).

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
#include "RingBufferNPlus.h"

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

//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
volatile unsigned long lastSeriaLogSyncTime = 0;
volatile unsigned long lastSerialCharTime = 0;
const unsigned long SYNC_FILE_AFTER_MSEC = 1000;
RingBufferNPlus incomingBuffer; //Serial RX buffer. This size of this buffer is sensitive
#define sdWriteSize (512)
volatile char sdBuffer[sdWriteSize]; // SD-Write buffer
volatile int sdBufferLen = 0;
volatile bool sdBufferDoSync = false;
volatile unsigned long charsReceived = 0; //Used for verifying/debugging serial reception
volatile unsigned long lastSDFileNameChangeTime; //Used to calculate the interval since the last SD filename change
const byte menuTimeout = 15; //Menus will exit/timeout after this number of seconds
volatile static bool stopLoggingSeen = false; //Flag to indicate if we should stop logging
volatile int lowBatteryReadings = 0; // Count how many times the battery voltage has read low
const int lowBatteryReadingsLimit = 10; // Don't declare the battery voltage low until we have had this many consecutive low readings (to reject sampling noise)
volatile static bool triggerEdgeSeen = false; //Flag to indicate if a trigger interrupt has been seen
volatile char serialTimestamp[40]; //Buffer to store serial timestamp, if needed
volatile size_t timestampCharsLeftToWrite = 0; // The length of serialTimestamp
volatile static bool powerLossSeen = false; //Flag to indicate if a power loss event has been seen
volatile bool muteTerminalOutput = false; //Simple (non-mutex) flag to mute the terminal output when the menus are open
volatile bool printCharsReceivedNow = false; //Flag to indicate when is a good time to print charsReceive
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
volatile bool thread1Run = true;
volatile bool thread1IsRunning = false;
rtos::Thread thread2;
volatile bool thread2Run = true;
volatile bool thread2IsRunning = false;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// UBX and NMEA Parse State
#define looking_for_B5_dollar   0
#define looking_for_62          1
#define looking_for_class       2
#define looking_for_ID          3
#define looking_for_length_LSB  4
#define looking_for_length_MSB  5
#define processing_payload      6
#define looking_for_checksum_A  7
#define looking_for_checksum_B  8
#define sync_lost               9
#define looking_for_asterix     10
#define looking_for_csum1       11
#define looking_for_csum2       12
#define looking_for_term1       13
#define looking_for_term2       14
volatile int ubx_nmea_state = looking_for_B5_dollar;
volatile int ubx_length = 0;
volatile int ubx_class = 0;
volatile int ubx_ID = 0;
volatile int ubx_checksum_A = 0;
volatile int ubx_checksum_B = 0;
volatile int ubx_expected_checksum_A = 0;
volatile int ubx_expected_checksum_B = 0;
volatile int nmea_char_1 = '0'; // e.g. G
volatile int nmea_char_2 = '0'; // e.g. P
volatile int nmea_char_3 = '0'; // e.g. G
volatile int nmea_char_4 = '0'; // e.g. G
volatile int nmea_char_5 = '0'; // e.g. A
volatile int nmea_csum = 0;
volatile int nmea_csum1 = '0';
volatile int nmea_csum2 = '0';
volatile int nmea_expected_csum1 = '0';
volatile int nmea_expected_csum2 = '0';
#define max_nmea_len 128 // Maximum length for an NMEA message: use this to detect if we have lost sync while receiving an NMEA message

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-


// This thread reads the incoming serial data and stores it in incomingBuffer
void thread_serial_rx(void)
{
  while (1)
  {
    thread1IsRunning = true; // Flag that this thread is running
      
    while (thread1Run)
    {
      if (Serial1.available() || (timestampCharsLeftToWrite > 0))
      {
        while (Serial1.available() || (timestampCharsLeftToWrite > 0))
        {
          uint8_t c;
          if (timestampCharsLeftToWrite > 0) // Based on code written by @DennisMelamed in PR #70
          {
            c = serialTimestamp[0];
            incomingBuffer.store_char(c); // Add a timestamp character to incomingBuffer
    
            // Shuffle the remaining chars along by one
            memmove((char *)&serialTimestamp[0], (const char *)&serialTimestamp[1], (size_t)timestampCharsLeftToWrite);
    
            timestampCharsLeftToWrite -= 1;
          }
          else
          {
            c = Serial1.read();
            incomingBuffer.store_char(c);
    
            charsReceived++; // Update the serial character count
  
            //Get the RTC timestamp if we just received the timestamp token
            if (settings.timestampSerial && (c == settings.timeStampToken))
            {
              getTimeString(&serialTimestamp[3]);
              serialTimestamp[0] = '\r'; // Add Carriage Return and Line Feed at the start of the timestamp
              serialTimestamp[1] = '\n';
              serialTimestamp[2] = '^'; // Add an up-arrow to indicate the timestamp relates to the preceeding data
              timestampCharsLeftToWrite = strlen((const char *)&serialTimestamp[0]);
            }

            if (settings.printUBXDebugMessages && !muteTerminalOutput) 
            {
              // NMEA and UBX parsing - process data bytes according to ubx_nmea_state
              // For UBX messages:
              // Sync Char 1: 0xB5
              // Sync Char 2: 0x62
              // Class byte
              // ID byte
              // Length: two bytes, little endian
              // Payload: length bytes
              // Checksum: two bytes
              // For NMEA messages:
              // Starts with a '$'
              // The next five characters indicate the message type (stored in nmea_char_1 to nmea_char_5)
              // Message fields are comma-separated
              // Followed by an '*'
              // Then a two character checksum (the logical exclusive-OR of all characters between the $ and the * as ASCII hex)
              // Ends with CR LF
              // Only allow a new file to be opened when a complete packet has been processed and ubx_nmea_state has returned to "looking_for_B5_dollar"
              // Or when a data error is detected (sync_lost)
              switch (ubx_nmea_state) {
                case (sync_lost):
                case (looking_for_B5_dollar): {
                  if (c == 0xB5) { // Have we found Sync Char 1 (0xB5) if we were expecting one?
                    ubx_nmea_state = looking_for_62; // Now look for Sync Char 2 (0x62)
                  }
                  else if (c == '$') { // Have we found an NMEA '$' if we were expecting one?
                    ubx_nmea_state = looking_for_asterix; // Now keep going until we receive an asterix
                    ubx_length = 0; // Reset ubx_length then use it to track which character has arrived
                    nmea_csum = 0; // Reset the nmea_csum. Update it as each character arrives
                    nmea_char_1 = '0'; // Reset the first five NMEA chars to something invalid
                    nmea_char_2 = '0';
                    nmea_char_3 = '0';
                    nmea_char_4 = '0';
                    nmea_char_5 = '0';
                  }
                  else {
                    printDebug("\r\nPanic!! Was expecting Sync Char 0xB5 or an NMEA $ but did not receive one!\r\n");
                    ubx_nmea_state = sync_lost;
                  }
                }
                break;
                case (looking_for_62): {
                  if (c == 0x62) { // Have we found Sync Char 2 (0x62) when we were expecting one?
                    ubx_expected_checksum_A = 0; // Reset the expected checksum
                    ubx_expected_checksum_B = 0;
                    ubx_nmea_state = looking_for_class; // Now look for Class byte
                  }
                  else {
                    printDebug("\r\nPanic!! Was expecting Sync Char 0x62 but did not receive one!\r\n");
                    ubx_nmea_state = sync_lost;
                  }
                }
                break;
                case (looking_for_class): {
                  ubx_class = c;
                  ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
                  ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
                  ubx_nmea_state = looking_for_ID; // Now look for ID byte
                }
                break;
                case (looking_for_ID): {
                  ubx_ID = c;
                  ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
                  ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
                  ubx_nmea_state = looking_for_length_LSB; // Now look for length LSB
                }
                break;
                case (looking_for_length_LSB): {
                  ubx_length = c; // Store the length LSB
                  ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
                  ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
                  ubx_nmea_state = looking_for_length_MSB; // Now look for length MSB
                }
                break;
                case (looking_for_length_MSB): {
                  ubx_length = ubx_length + (c * 256); // Add the length MSB
                  ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
                  ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
                  ubx_nmea_state = processing_payload; // Now look for payload bytes (length: ubx_length)
                }
                break;
                case (processing_payload): {
                  ubx_length = ubx_length - 1; // Decrement length by one
                  ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
                  ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
                  if (ubx_length == 0) {
                    ubx_expected_checksum_A = ubx_expected_checksum_A & 0xff; // Limit checksums to 8-bits
                    ubx_expected_checksum_B = ubx_expected_checksum_B & 0xff;
                    ubx_nmea_state = looking_for_checksum_A; // If we have received length payload bytes, look for checksum bytes
                  }
                }
                break;
                case (looking_for_checksum_A): {
                  ubx_checksum_A = c;
                  ubx_nmea_state = looking_for_checksum_B;
                }
                break;
                case (looking_for_checksum_B): {
                  ubx_checksum_B = c;
                  ubx_nmea_state = looking_for_B5_dollar; // All bytes received so go back to looking for a new Sync Char 1 unless there is a checksum error
                  if ((ubx_expected_checksum_A != ubx_checksum_A) or (ubx_expected_checksum_B != ubx_checksum_B)) {
                    printDebug("\r\nPanic!! UBX checksum error!\r\n");
                    ubx_nmea_state = sync_lost;
                  }
                }
                break;
                // NMEA messages
                case (looking_for_asterix): {
                  ubx_length++; // Increase the message length count
                  if (ubx_length > max_nmea_len) { // If the length is greater than max_nmea_len, something bad must have happened (sync_lost)
                    printDebug("\r\nPanic!! Excessive NMEA message length!\r\n");
                    ubx_nmea_state = sync_lost;
                    break;
                  }
                  // If this is one of the first five characters, store it
                  // May be useful for on-the-fly message parsing or DEBUG
                  if (ubx_length <= 5) {
                    if (ubx_length == 1) {
                      nmea_char_1 = c;
                    }
                    else if (ubx_length == 2) {
                      nmea_char_2 = c;
                    }
                    else if (ubx_length == 3) {
                      nmea_char_3 = c;
                    }
                    else if (ubx_length == 4) {
                      nmea_char_4 = c;
                    }
                    else { // ubx_length == 5
                      nmea_char_5 = c;
                    }
                  }
                  // Now check if this is an '*'
                  if (c == '*') {
                    // Asterix received
                    // Don't exOR it into the checksum
                    // Instead calculate what the expected checksum should be (nmea_csum in ASCII hex)
                    nmea_expected_csum1 = ((nmea_csum & 0xf0) >> 4) + '0'; // Convert MS nibble to ASCII hex
                    if (nmea_expected_csum1 >= ':') { nmea_expected_csum1 += 7; } // : follows 9 so add 7 to convert to A-F
                    nmea_expected_csum2 = (nmea_csum & 0x0f) + '0'; // Convert LS nibble to ASCII hex
                    if (nmea_expected_csum2 >= ':') { nmea_expected_csum2 += 7; } // : follows 9 so add 7 to convert to A-F
                    // Next, look for the first csum character
                    ubx_nmea_state = looking_for_csum1;
                    break; // Don't include the * in the checksum
                  }
                  // Now update the checksum
                  // The checksum is the exclusive-OR of all characters between the $ and the *
                  nmea_csum = nmea_csum ^ c;
                }
                break;
                case (looking_for_csum1): {
                  // Store the first NMEA checksum character
                  nmea_csum1 = c;
                  ubx_nmea_state = looking_for_csum2;
                }
                break;
                case (looking_for_csum2): {
                  // Store the second NMEA checksum character
                  nmea_csum2 = c;
                  // Now check if the checksum is correct
                  if ((nmea_csum1 != nmea_expected_csum1) or (nmea_csum2 != nmea_expected_csum2)) {
                    // The checksum does not match so sync_lost
                    printDebug("\r\nPanic!! NMEA checksum error!\r\n");
                    ubx_nmea_state = sync_lost;
                  }
                  else {
                    // Checksum was valid so wait for the terminators
                    ubx_nmea_state = looking_for_term1;
                  }
                }
                break;
                case (looking_for_term1): {
                  // Check if this is CR
                  if (c != '\r') {
                    printDebug("\r\nPanic!! NMEA CR not found!\r\n");
                    ubx_nmea_state = sync_lost;
                  }
                  else {
                    ubx_nmea_state = looking_for_term2;
                  }
                }
                break;
                case (looking_for_term2): {
                  // Check if this is LF
                  if (c != '\n') {
                    printDebug("\r\nPanic!! NMEA LF not found!\r\n");
                    ubx_nmea_state = sync_lost;
                  }
                  else {
                    // LF was received so go back to looking for B5 or a $
                    ubx_nmea_state = looking_for_B5_dollar;
                  }
                }
                break;
              }
            }
          }
    
          //Print to terminal
          if ((settings.enableTerminalOutput == true) && (muteTerminalOutput == false))
          {
            if (Serial.write(c) != (size_t)1) //Print to terminal
            {
              delay(1);
              Serial.write(c); //Try again if TX buffer was full
            }
          }
          //Echo the data on Serial1 TX
          if ((settings.echoSerial == true))
          {
            if (Serial1.write(c) != (size_t)1) //Print to terminal
            {
              delay(1);
              Serial1.write(c); //Try again if TX buffer was full
            }
          }

          lastSerialCharTime = millis(); // Update lastSerialCharTime
        }
      }
  
      //Check if we have enough data for a full SD write and if the SD-Write buffer is empty
      if ((incomingBuffer.available() >= sdWriteSize) && (sdBufferLen == 0))
      {
        //Copy sdWriteSize bytes from incomingBuffer into sdBuffer
        incomingBuffer.read_chars((volatile char *)&sdBuffer[0], sdWriteSize);
        //Clear sdBufferDoSync
        sdBufferDoSync = false;
        //Update sdBufferLen
        sdBufferLen = sdWriteSize;
      }    
      //Periodically sync the log file: write any remaining data if the SD-Write buffer is empty
      else if (((millis() - lastSeriaLogSyncTime) > SYNC_FILE_AFTER_MSEC) && (incomingBuffer.available() > 0) && (sdBufferLen == 0))
      {
        //Now is a good time to print charsReceived if required
        printCharsReceivedNow = true;
        //Copy available bytes from incomingBuffer into sdBuffer
        int incomingBufferAvailable = incomingBuffer.available();
        incomingBuffer.read_chars((volatile char *)&sdBuffer[0], incomingBufferAvailable);
        //Set sdBufferDoSync
        sdBufferDoSync = true;
        //Update sdBufferLen
        sdBufferLen = incomingBufferAvailable;
        //Update lastSeriaLogSyncTime
        lastSeriaLogSyncTime = millis();
      }

      if (sdBufferLen > 0)
      {
        delay(1); // Let the other threads get a foot in the door
      }
      else if ((millis() - lastSerialCharTime) > 10)
      {
        delay(1); // Let the other threads get a foot in the door
        lastSerialCharTime = millis();
      }
    }

    thread1IsRunning = false; // Flag that this thread is not running

    while (!thread1Run)
    {
      // Spin the wheels...
      delay(10);
    }
  }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// This thread writes the data in sdBuffer to SD card
void thread_sd_write(void)
{
  while (1)
  {
    thread2IsRunning = true; // Flag that this thread is running
      
    while (thread2Run)
    {
      if (sdBufferLen > 0) // Is there any data waiting to be written?
      {
        if ((settings.logSerial == true) && (online.serialLogging == true))
        {
          digitalWrite(PIN_STAT_LED, HIGH); //Toggle stat LED to indicating log recording
    
          serialDataFile.write((char *)sdBuffer, sdBufferLen); //Record the buffer to the card
    
          if (sdBufferDoSync) //Should we sync the log file?
          {
            serialDataFile.sync();
            updateDataFileAccess(&serialDataFile); // Update the file access time & date
          }
    
          sdBufferLen = 0;
          
          digitalWrite(PIN_STAT_LED, LOW);
        }
        else
        {
          sdBufferLen = 0; // Clean up if we are not writing to SD card
        }
      }
      delay(10); // Let the other threads get a foot in the door
    }

    thread2IsRunning = false; // Flag that this thread is not running

    while (!thread2Run)
    {
      // Spin the wheels...
      delay(10);
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

  analogReadResolution(14); //Increase from default of 10

  lastSDFileNameChangeTime = millis(); // Record the time of the file name change

  serialTimestamp[0] = '\0'; // Empty the serial timestamp buffer

  beginSerialLogging();

  if (online.microSD == true) SerialPrintln(F("SD card online"));
  else SerialPrintln(F("SD card offline"));

  if (online.serialLogging == true) SerialPrintln(F("Serial logging online"));
  else SerialPrintln(F("Serial logging offline"));

  if (settings.enableTerminalOutput == false && online.serialLogging == true)
    SerialPrintln(F("Logging to microSD card with no terminal output"));
  else
    SerialPrintln(F("You may need to disable \"Display serial data in terminal\" via the Serial Logging Menu when logging at high data/baud rates."));

  //Clear any spurious serial data
  while (Serial1.available())
    Serial1.read();

  digitalWrite(PIN_STAT_LED, LOW); // Turn the STAT LED off now that everything is configured

  //Start the threads
  startThreads();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void startThreads(void)
{
  thread1Run = true;
  thread1.start(thread_serial_rx);
  thread1.set_priority(osPriorityNormal1);
  thread2Run = true;
  thread2.start(thread_sd_write);
}

void pauseThreads(void)
{
  thread1Run = false;
  thread2Run = false;
  while (thread1IsRunning)
    delay(1);
  while (thread2IsRunning)
    delay(1);
}

void pauseThread2(void)
{
  thread2Run = false;
  while (thread2IsRunning)
    delay(1);
}

void restartThreads(void)
{
  thread1Run = true;
  thread2Run = true;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void loop() {

  checkBattery(); // Check for low battery

  //Check for a New File trigger event
  if (((settings.useGPIO11ForTrigger == true) && (triggerEdgeSeen == true))
   || ((settings.openNewLogFilesAfter > 0) && ((millis() - lastSDFileNameChangeTime) > (settings.openNewLogFilesAfter * 1000))))
  {
    pauseThreads();
    triggerEdgeSeen = false;
    openNewLogFile();
    lastSDFileNameChangeTime = millis(); // Record the time of the file name change
    restartThreads();
  }

  if ((settings.useGPIO32ForStopLogging == true) && (stopLoggingSeen == true)) // Has the user pressed the stop logging button?
  {
    stopLogging();
  }

  if (Serial.available())
  {
    muteTerminalOutput = true;
    menuMain(); //Present user menu
    muteTerminalOutput = false;
  }

  if (printCharsReceivedNow)
  {
     printDebug("\r\nTotal chars received: " + (String)charsReceived + "\r\n");
     printCharsReceivedNow = false;
  }

  //Let the threads do their thing
  delay(100);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

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
    int incomingBufferAvailable = incomingBuffer.available();
    if (incomingBufferAvailable > 0)
    {
      //Write the remainder of the buffer
      digitalWrite(PIN_STAT_LED, HIGH); //Toggle stat LED to indicating log recording

      //Copy available bytes from incomingBuffer into sdBuffer
      incomingBuffer.read_chars((char *)&sdBuffer[0], incomingBufferAvailable);
      serialDataFile.write((char *)&sdBuffer[0], incomingBufferAvailable); //Record the buffer to the card      
      
      serialDataFile.sync();
      updateDataFileAccess(&serialDataFile); // Update the file access time & date
      digitalWrite(PIN_STAT_LED, LOW);
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

//Returns next available log file name
//Checks the spots in EEPROM for the next available LOG# file name
//Updates EEPROM and then appends to the new log file.
char* findNextAvailableLog(int &newFileNumber, const char *fileLeader)
{
  SdFile newFile; //This will contain the file for SD writing

  if (newFileNumber < 2) //If the settings have been reset, let's warn the user that this could take a while!
  {
    SerialPrintln(F("Finding the next available log file."));
    SerialPrintln(F("This could take a long time if the SD card contains many existing log files."));
  }

  if (newFileNumber > 0)
    newFileNumber--; //Check if last log file was empty. Reuse it if it is.

  //Search for next available log spot
  static char newFileName[40];
  while (1)
  {
    char newFileNumberStr[6];
    if (newFileNumber < 10)
      sprintf(newFileNumberStr, "0000%d", newFileNumber);
    else if (newFileNumber < 100)
      sprintf(newFileNumberStr, "000%d", newFileNumber);
    else if (newFileNumber < 1000)
      sprintf(newFileNumberStr, "00%d", newFileNumber);
    else if (newFileNumber < 10000)
      sprintf(newFileNumberStr, "0%d", newFileNumber);
    else
      sprintf(newFileNumberStr, "%d", newFileNumber);
    sprintf(newFileName, "%s%s.TXT", fileLeader, newFileNumberStr); //Splice the new file number into this file name. Max no. is 99999.

    if (sd.exists(newFileName) == false) break; //File name not found so we will use it.

    //File exists so open and see if it is empty. If so, use it.
    newFile.open(newFileName, O_READ);
    if (newFile.fileSize() == 0) break; // File is empty so we will use it. Note: we need to make the user aware that this can happen!

    newFile.close(); // Close this existing file we just opened.

    newFileNumber++; //Try the next number
    if (newFileNumber >= 100000) break; // Have we hit the maximum number of files?
  }
  
  newFile.close(); //Close this new file we just opened

  newFileNumber++; //Increment so the next power up uses the next file #

  if (newFileNumber >= 100000) // Have we hit the maximum number of files?
  {
    SerialPrint(F("***** WARNING! File number limit reached! (Overwriting "));
    SerialPrint(newFileName);
    SerialPrintln(F(") *****"));
    newFileNumber = 100000; // This will overwrite Log99999.TXT next time thanks to the newFileNumber-- above
  }
  else
  {
    SerialPrint(F("Logging to: "));
    SerialPrintln(newFileName);    
  }

  //Record new file number to EEPROM and to config file
  //This works because newFileNumber is a pointer to settings.newFileNumber
  recordSystemSettings();

  return (newFileName);
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
