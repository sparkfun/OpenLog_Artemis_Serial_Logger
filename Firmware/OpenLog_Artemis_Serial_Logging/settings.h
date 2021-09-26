//This is all the settings that can be set on OpenLog. It's recorded to NVM and the config file.
struct struct_settings {
  int sizeOfSettings = 0; //sizeOfSettings **must** be the first entry and must be int
  int olaIdentifier = OLA_IDENTIFIER; // olaIdentifier **must** be the second entry
  int nextSerialLogNumber = 1;
  bool enableSD = true;
  bool enableTerminalOutput = true;
  bool logSerial = true;
  bool americanDateStyle = true;
  bool hour24Style = true;
  int  serialTerminalBaudRate = 115200;
  int  serialLogBaudRate = 9600;
  bool printDebugMessages = false;
  bool printUBXDebugMessages = false;
  unsigned long openNewLogFilesAfter = 0; //Default to 0 (Never) seconds
  float vinCorrectionFactor = 1.47; //Correction factor for the VIN measurement; to compensate for the divider impedance
  bool useGPIO32ForStopLogging = false; //If true, use GPIO as a stop logging button
  bool enableLowBatteryDetection = false; // Low battery detection
  float lowBatteryThreshold = 3.4; // Low battery voltage threshold (Volts)
  bool useGPIO11ForTrigger = false; // If true, use GPIO to trigger sensor logging
  bool fallingEdgeTrigger = true; // Default to falling-edge triggering (If false, triggering will be rising-edge)
  bool timestampSerial = false; // If true, the RTC time will be added to the serial log file when timeStampToken is received
  uint8_t timeStampToken = 0x0A; // Add RTC time to the serial log when this token is received. Default to Line Feed (0x0A). Suggested by @DennisMelamed in Issue #63
  bool echoSerial = false; // If true, the incoming serial data is echoed on the TX pin
} settings;

//These are the devices on board OpenLog that may be on or offline.
struct struct_online {
  bool microSD = false;
  bool serialLogging = false;
} online;
