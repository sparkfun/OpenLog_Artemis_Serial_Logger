void menuDebug()
{
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure Debug Settings"));

    SerialPrint(F("1) Debug Messages: "));
    if (settings.printDebugMessages == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrint(F("2) Debug UBX / NMEA Messages: "));
    if (settings.printUBXDebugMessages == true) SerialPrintln(F("Enabled"));
    else SerialPrintln(F("Disabled"));

    SerialPrintln(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout); //Timeout after x seconds

    if (incoming == '1')
    {
      settings.printDebugMessages ^= 1;
    }
    if (incoming == '2')
    {
      settings.printUBXDebugMessages ^= 1;
      if ((settings.printUBXDebugMessages == true) && (settings.printDebugMessages == false))
        SerialPrintln(F("You need to enable Debug Messages to see the UBX / NMEA Debug Messages"));
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}
