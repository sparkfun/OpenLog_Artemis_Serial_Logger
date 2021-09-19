//Query the RTC and put the appropriately formatted (according to settings) 
//string into the passed buffer.
//Code modified by @DennisMelamed in PR #70
void getTimeString(volatile char timeStringBuffer[])
{
  //reset the buffer
  timeStringBuffer[0] = '\0';

  myRTC.getTime();

  char rtcDate[12]; // 10/12/2019,
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
  strcat((char *)&timeStringBuffer[0], rtcDate);

  char rtcTime[14]; //09:14:37.41\r\n
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
  sprintf(rtcTime, "%s:%s:%s.%s\r\n", rtcHour, rtcMin, rtcSec, rtcHundredths);
  strcat((char *)&timeStringBuffer[0], rtcTime);
}
