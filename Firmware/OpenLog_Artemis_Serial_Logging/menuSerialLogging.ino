//TODO Add time stamp option after a certain timeout

void menuSerialLogging()
{
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure Serial Logging"));

    SerialPrint(F("1) Log serial data to SD: "));
    if (settings.logSerial == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("2) Display serial data in terminal: "));
    if (settings.enableTerminalOutput == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("3) Set serial baud rate: "));
    Serial.print(settings.serialLogBaudRate);
    SerialPrintln(F(" bps"));

    SerialPrint(F("4) Add RTC timestamp when token is received: "));
    if (settings.timestampSerial == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("5) Timestamp token: "));
    Serial.print(settings.timeStampToken);
    SerialPrint(F(" (Decimal)"));
    switch (settings.timeStampToken)
    {
      case 0x00:
        SerialPrintln(F(" = NULL"));
        break;
      case 0x03:
        SerialPrintln(F(" = End of Text"));
        break;
      case 0x0A:
        SerialPrintln(F(" = Line Feed"));
        break;
      case 0x0D:
        SerialPrintln(F(" = Carriage Return"));
        break;
      case 0x1B:
        SerialPrintln(F(" = Escape"));
        break;
      default:
        SerialPrintln(F(""));
        break;
    }

    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
    {
      if (settings.logSerial == false)
      {
        settings.logSerial = true;

        beginSerialLogging(); //Start up port and log file (this will set online.serialLogging to true if successful)
      }
      else
      {
        if (online.serialLogging)
        {
          //Shut it all down
          updateDataFileAccess(&serialDataFile); // Update the file access time & date
          serialDataFile.close();

          online.serialLogging = false;
        }
        settings.logSerial = false;
      }
    }
    else if (incoming == '2')
      settings.enableTerminalOutput ^= 1;
    else if (incoming == '3')
    {
      SerialPrint(F("Enter baud rate (1200 to 921600): "));
      int newBaud = getNumber(menuTimeout); //Timeout after x seconds
      if (newBaud < 1200 || newBaud > 921600)
      {
        SerialPrintln(F("Error: baud rate out of range"));
      }
      else
      {
        configureSerial1TxRx();
        settings.serialLogBaudRate = newBaud;
        Serial1.begin(settings.serialLogBaudRate);
      }
    }
    else if (incoming == '4')
      settings.timestampSerial ^= 1;
    else if (incoming == '5')
    {
      SerialPrint(F("Enter the timestamp token in decimal (0 to 255): "));
      int newToken = getNumber(menuTimeout); //Timeout after x seconds
      if (newToken < 0 || newToken > 255)
      {
        SerialPrintln(F("Error: token out of range"));
      }
      else
      {
        settings.timeStampToken = (uint8_t)newToken;
      }
    }
    else if (incoming == 'x')
      return;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      return;
    else
      printUnknown(incoming);
  }
}
