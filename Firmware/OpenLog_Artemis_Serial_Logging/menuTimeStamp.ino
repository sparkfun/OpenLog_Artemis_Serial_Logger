void menuTimeStamp()
{
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure Time Stamp"));
    SerialPrint(F("Current date/time: "));
    myRTC.getTime();

    char rtcDate[11]; // 10/12/2019
    char rtcDay[3];
    char rtcMonth[3];
    char rtcYear[5];
    if (myRTC.dayOfMonth < 10)
      sprintf(rtcDay, "0%d", myRTC.dayOfMonth);
    else
      sprintf(rtcDay, "%d", myRTC.dayOfMonth);
    if (myRTC.month < 10)
      sprintf(rtcMonth, "0%d", myRTC.month);
    else
      sprintf(rtcMonth, "%d", myRTC.month);
    if (myRTC.year < 10)
      sprintf(rtcYear, "200%d", myRTC.year);
    else
      sprintf(rtcYear, "20%d", myRTC.year);
    if (settings.americanDateStyle == true)
      sprintf(rtcDate, "%s/%s/%s,", rtcMonth, rtcDay, rtcYear);
    else
      sprintf(rtcDate, "%s/%s/%s,", rtcDay, rtcMonth, rtcYear);

    SerialPrint(rtcDate);
    SerialPrint(F(" "));

    char rtcTime[13]; //09:14:37.41,
    int adjustedHour = myRTC.hour;
    if (settings.hour24Style == false)
    {
      if (adjustedHour > 12) adjustedHour -= 12;
    }
    char rtcHour[3];
    char rtcMin[3];
    char rtcSec[3];
    char rtcHundredths[3];
    if (adjustedHour < 10)
      sprintf(rtcHour, "0%d", adjustedHour);
    else
      sprintf(rtcHour, "%d", adjustedHour);
    if (myRTC.minute < 10)
      sprintf(rtcMin, "0%d", myRTC.minute);
    else
      sprintf(rtcMin, "%d", myRTC.minute);
    if (myRTC.seconds < 10)
      sprintf(rtcSec, "0%d", myRTC.seconds);
    else
      sprintf(rtcSec, "%d", myRTC.seconds);
    if (myRTC.hundredths < 10)
      sprintf(rtcHundredths, "0%d", myRTC.hundredths);
    else
      sprintf(rtcHundredths, "%d", myRTC.hundredths);
    sprintf(rtcTime, "%s:%s:%s.%s", rtcHour, rtcMin, rtcSec, rtcHundredths);

    SerialPrintln(rtcTime);

    SerialPrintln(F("1) Set RTC to compiler macro time"));

    SerialPrintln(F("2) Manually set RTC date"));

    SerialPrint(F("3) Toggle date style: "));
    if (settings.americanDateStyle == true) SerialPrintln(F("mm/dd/yyyy"));
    else SerialPrintln(F("dd/mm/yyyy"));

    SerialPrintln(F("4) Manually set RTC time"));

    SerialPrint(F("5) Toggle time style: "));
    if (settings.hour24Style == true) SerialPrintln(F("24 hour"));
    else SerialPrintln(F("12 hour"));

    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after x seconds

    if (incoming == 1)
    {
      myRTC.setToCompilerTime(); //Set RTC using the system __DATE__ and __TIME__ macros from compiler
      SerialPrintln(F("RTC set to compiler time"));
    }
    else if (incoming == 2) //Manually set RTC date
    {
      myRTC.getTime();
      int dd = myRTC.dayOfMonth, mm = myRTC.month, yy = myRTC.year, h = myRTC.hour, m = myRTC.minute, s = myRTC.seconds;

      SerialPrint(F("Enter current two digit year: "));
      yy = getNumber(menuTimeout); //Timeout after x seconds
      if (yy > 2000 && yy < 2100) yy -= 2000;

      SerialPrint(F("Enter current month (1 to 12): "));
      mm = getNumber(menuTimeout); //Timeout after x seconds

      SerialPrint(F("Enter current day (1 to 31): "));
      dd = getNumber(menuTimeout); //Timeout after x seconds

      myRTC.getTime();
      h = myRTC.hour; m = myRTC.minute; s = myRTC.seconds;
      myRTC.setTime(0, s, m, h, dd, mm, yy); //Manually set RTC
      lastSDFileNameChangeTime = millis(); // Record the time of the file name change
    }
    else if (incoming == 3)
    {
      settings.americanDateStyle ^= 1;
    }
    else if (incoming == 4) //Manually set time
    {
      myRTC.getTime();
      int dd = myRTC.dayOfMonth, mm = myRTC.month, yy = myRTC.year, h = myRTC.hour, m = myRTC.minute, s = myRTC.seconds;

      SerialPrint(F("Enter current hour (0 to 23): "));
      h = getNumber(menuTimeout); //Timeout after x seconds

      SerialPrint(F("Enter current minute (0 to 59): "));
      m = getNumber(menuTimeout); //Timeout after x seconds

      SerialPrint(F("Enter current second (0 to 59): "));
      s = getNumber(menuTimeout); //Timeout after x seconds

      myRTC.setTime(0, s, m, h, dd, mm, yy); //Manually set RTC
      lastSDFileNameChangeTime = millis(); // Record the time of the file name change
    }
    else if (incoming == 5)
    {
      settings.hour24Style ^= 1;
    }
    else if (incoming == STATUS_PRESSED_X)
      return;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      return;
  }
}
