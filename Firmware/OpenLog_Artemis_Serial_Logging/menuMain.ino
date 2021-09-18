//Display the options
//If user doesn't respond within a few seconds, return to main loop
void menuMain()
{
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Main Menu"));

    SerialPrintln(F("1) Configure Terminal Output"));

    SerialPrintln(F("2) Configure Time"));

    SerialPrintln(F("3) Configure Serial Logging"));

    SerialPrintln(F("4) Configure Power Options"));

    SerialPrintln(F("r) Reset all settings to default"));

    SerialPrintln(F("q) Quit: Close log files and power down"));

    SerialPrintln(F("d) Debug Menu"));

    SerialPrintln(F("x) Return to logging"));

    byte incoming = getByteChoice(menuTimeout);

    if (incoming == '1')
      menuLogRate();
    else if (incoming == '2')
      menuTimeStamp();
    else if (incoming == '3')
      menuSerialLogging();
    else if (incoming == '4')
      menuPower();
    else if (incoming == 'd')
      menuDebug();
    else if (incoming == 'r')
    {
      SerialPrintln(F("\r\nResetting to factory defaults. Press 'y' to confirm: "));
      byte bContinue = getByteChoice(menuTimeout);
      if (bContinue == 'y')
      {
        //Pause the threads
        threadSerialRXrun = false;
        threadSDrun = false;

        //Save files before going to sleep
        if (online.serialLogging == true)
        {
          serialDataFile.sync();
          updateDataFileAccess(&serialDataFile); // Update the file access time & date
          serialDataFile.close();
        }
        
        EEPROM.erase();
        if (sd.exists("OLA_Serial_settings.txt"))
          sd.remove("OLA_Serial_settings.txt");

        SerialPrint(F("Settings erased. Please reset OpenLog Artemis and open a terminal at "));
        Serial.print((String)settings.serialTerminalBaudRate);
        SerialPrintln(F("bps..."));
        while (1);
      }
      else
        SerialPrintln(F("Reset aborted"));
    }
    else if (incoming == 'q')
    {
      SerialPrintln(F("\r\nQuit? Press 'y' to confirm:"));
      byte bContinue = getByteChoice(menuTimeout);
      if (bContinue == 'y')
      {
        //Pause the threads
        threadSerialRXrun = false;
        threadSDrun = false;
        
        //Save files before going to sleep
        if (online.serialLogging == true)
        {
          serialDataFile.sync();
          updateDataFileAccess(&serialDataFile); // Update the file access time & date
          serialDataFile.close();
        }
        
        SerialPrint(F("Log files are closed. Please reset OpenLog Artemis and open a terminal at "));
        Serial.print((String)settings.serialTerminalBaudRate);
        SerialPrintln(F("bps..."));
        delay(sdPowerDownDelay); // Give the SD card time to shut down
        powerDownOLA();
      }
      else
        SerialPrintln(F("Quit aborted"));
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }

  //Pause the SD thread
  threadSDrun = false;

  recordSystemSettings(); //Once all menus have exited, record the new settings to EEPROM and config file

  //Unpause the SD thread
  threadSDrun = true;

  while (Serial.available()) Serial.read(); //Empty buffer of any newline chars
}
