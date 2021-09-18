void menuPower()
{
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure Power Options"));

    SerialPrint(F("1) Use pin 32 to Stop Logging: "));
    if (settings.useGPIO32ForStopLogging == true) SerialPrintln(F("Yes"));
    else SerialPrintln(F("No"));

#if(HARDWARE_VERSION_MAJOR >= 1)
    SerialPrint(F("2) Low Battery Voltage Detection: "));
    if (settings.enableLowBatteryDetection == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("3) Low Battery Threshold (V): "));
    char tempStr[16];
    olaftoa(settings.lowBatteryThreshold, tempStr, 2, sizeof(tempStr) / sizeof(char));
    SerialPrintf2("%s\r\n", tempStr);
#endif

    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
    {
      settings.useGPIO32ForStopLogging ^= 1;
    }
#if(HARDWARE_VERSION_MAJOR >= 1)
    else if (incoming == '2')
    {
      settings.enableLowBatteryDetection ^= 1;
    }
    else if (incoming == '3')
    {
      SerialPrintln(F("Please enter the new low battery threshold:"));
      float tempBT = (float)getDouble(menuTimeout); //Timeout after x seconds
      if ((tempBT < 3.0) || (tempBT > 6.0))
        SerialPrintln(F("Error: Threshold out of range"));
      else
        settings.lowBatteryThreshold = tempBT;
    }
#endif
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}
