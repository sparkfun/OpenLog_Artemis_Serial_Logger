// Read the battery voltage
// If it is low, increment lowBatteryReadings
// If lowBatteryReadings exceeds lowBatteryReadingsLimit then powerDownOLA
void checkBattery(void)
{
#if(HARDWARE_VERSION_MAJOR >= 1)
  if (settings.enableLowBatteryDetection == true)
  {
    float voltage = readVIN(); // Read the battery voltage
    if (voltage < settings.lowBatteryThreshold) // Is the voltage low?
    {
      lowBatteryReadings++; // Increment the low battery count
      if (lowBatteryReadings > lowBatteryReadingsLimit) // Have we exceeded the low battery count limit?
      {
        // Gracefully powerDownOLA

        //Save files before powerDownOLA
        if (online.serialLogging == true)
        {
          serialDataFile.sync();
          updateDataFileAccess(&serialDataFile); // Update the file access time & date
          serialDataFile.close();
        }
      
        delay(sdPowerDownDelay); // Give the SD card time to finish writing ***** THIS IS CRITICAL *****

        SerialPrintln(F("***      LOW BATTERY VOLTAGE DETECTED! GOING INTO POWERDOWN      ***"));
        SerialPrintln(F("*** PLEASE CHANGE THE POWER SOURCE AND RESET THE OLA TO CONTINUE ***"));
      
        SerialFlush(); //Finish any prints

        powerDownOLA(); // power down and wait for reset
      }
    }
    else
    {
      lowBatteryReadings = 0; // Reset the low battery count (to reject noise)
    }    
  }
#endif

  if (powerLossSeen)
    powerDownOLA(); // power down and wait for reset
}

//Power down the entire system but maintain running of RTC
//This function takes 100us to run including GPIO setting
//This puts the Apollo3 into 2.36uA to 2.6uA consumption mode
//With leakage across the 3.3V protection diode, it's approx 3.00uA.
void powerDownOLA(void)
{
  //Prevent voltage supervisor from waking us from sleep
  detachInterrupt(PIN_POWER_LOSS);

  //Prevent stop logging button from waking us from sleep
  if (settings.useGPIO32ForStopLogging == true)
  {
    detachInterrupt(PIN_STOP_LOGGING); // Disable the interrupt
    pinMode(PIN_STOP_LOGGING, INPUT); // Remove the pull-up
  }

  //Prevent trigger from waking us from sleep
  if (settings.useGPIO11ForTrigger == true)
  {
    detachInterrupt(PIN_TRIGGER); // Disable the interrupt
    pinMode(PIN_TRIGGER, INPUT); // Remove the pull-up
  }

  //WE NEED TO POWER DOWN ASAP - we don't have time to close the SD files

  //SerialFlush(); //Don't waste time waiting for prints to finish

  //  Wire.end(); //Power down I2C
  qwiic.end(); //Power down I2C

  SPI.end(); //Power down SPI

  powerControlADC(false); // power_adc_disable(); //Power down ADC. It it started by default before setup().

  Serial.end(); //Power down UART
  Serial1.end();

  //Force the peripherals off
  //This will cause badness with v2.1 of the core but we don't care as we are waiting for a reset
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM0);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM1);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM2);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM3);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM4);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM5);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_ADC);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART0);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART1);

  //Disable pads (this disables the LEDs too)
  for (int x = 0; x < 50; x++)
  {
    if ((x != PIN_POWER_LOSS) &&
        //(x != PIN_LOGIC_DEBUG) &&
        (x != PIN_MICROSD_POWER) &&
        (x != PIN_QWIIC_POWER) &&
        (x != PIN_IMU_POWER))
    {
      am_hal_gpio_pinconfig(x, g_AM_HAL_GPIO_DISABLE);
    }
  }

  //Make sure PIN_POWER_LOSS is configured as an input for the WDT
  pinMode(PIN_POWER_LOSS, INPUT); // BD49K30G-TL has CMOS output and does not need a pull-up

  //We can't leave these power control pins floating
  imuPowerOff();
  microSDPowerOff();

  //Keep Qwiic bus powered on if user desires it - but only for X04 to avoid a brown-out
#if(HARDWARE_VERSION_MAJOR == 0)
  if (settings.powerDownQwiicBusBetweenReads == true)
    qwiicPowerOff();
  else
    qwiicPowerOn(); //Make sure pins stays as output
#else
  qwiicPowerOff();
#endif

  //Power down cache, flash, SRAM
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_ALL); // Power down all flash and cache
  am_hal_pwrctrl_memory_deepsleep_retain(AM_HAL_PWRCTRL_MEM_SRAM_384K); // Retain all SRAM

  //Keep the 32kHz clock running for RTC
  am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
  am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ);

  while (1) // Stay in deep sleep until we get reset
  {
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP); //Sleep
  }
}

//Reset the Artemis
void resetArtemis(void)
{
  //Save files before resetting
  if (online.serialLogging == true)
  {
    serialDataFile.sync();
    updateDataFileAccess(&serialDataFile); // Update the file access time & date
    serialDataFile.close();
  }

  delay(sdPowerDownDelay); // Give the SD card time to finish writing ***** THIS IS CRITICAL *****

  SerialFlush(); //Finish any prints

  //  Wire.end(); //Power down I2C
  qwiic.end(); //Power down I2C

  SPI.end(); //Power down SPI

  powerControlADC(false); // power_adc_disable(); //Power down ADC. It it started by default before setup().

  Serial.end(); //Power down UART
  Serial1.end();

  //Force the peripherals off
  //This will cause badness with v2.1 of the core but we don't care as we are going to force a WDT reset
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM0);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM1);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM2);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM3);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM4);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM5);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_ADC);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART0);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART1);

  //Disable pads
  for (int x = 0; x < 50; x++)
  {
    if ((x != PIN_POWER_LOSS) &&
        //(x != PIN_LOGIC_DEBUG) &&
        (x != PIN_MICROSD_POWER) &&
        (x != PIN_QWIIC_POWER) &&
        (x != PIN_IMU_POWER))
    {
      am_hal_gpio_pinconfig(x, g_AM_HAL_GPIO_DISABLE);
    }
  }

  //We can't leave these power control pins floating
  imuPowerOff();
  microSDPowerOff();

  //Disable power for the Qwiic bus
  qwiicPowerOff();

  //Disable the power LED
  powerLEDOff();

  //Enable the Watchdog so it can reset the Artemis
  startWatchdog();
  while (1) // That's all folks! Artemis will reset in 1.25 seconds
    ;
}

void stopLogging(void)
{
  detachInterrupt(PIN_STOP_LOGGING); // Disable the interrupt

  //Save file before going to sleep
  if (online.serialLogging == true)
  {
    serialDataFile.sync();
    updateDataFileAccess(&serialDataFile); // Update the file access time & date
    serialDataFile.close();
  }

  SerialPrint(F("Logging is stopped. Please reset OpenLog Artemis and open a terminal at "));
  Serial.print((String)settings.serialTerminalBaudRate);
  SerialPrintln(F("bps..."));
  delay(sdPowerDownDelay); // Give the SD card time to shut down
  powerDownOLA();
}

void qwiicPowerOn()
{
  pinMode(PIN_QWIIC_POWER, OUTPUT);
  pin_config(PinName(PIN_QWIIC_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
#if(HARDWARE_VERSION_MAJOR == 0)
  digitalWrite(PIN_QWIIC_POWER, LOW);
#else
  digitalWrite(PIN_QWIIC_POWER, HIGH);
#endif
}
void qwiicPowerOff()
{
  pinMode(PIN_QWIIC_POWER, OUTPUT);
  pin_config(PinName(PIN_QWIIC_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
#if(HARDWARE_VERSION_MAJOR == 0)
  digitalWrite(PIN_QWIIC_POWER, HIGH);
#else
  digitalWrite(PIN_QWIIC_POWER, LOW);
#endif
}

void microSDPowerOn()
{
  pinMode(PIN_MICROSD_POWER, OUTPUT);
  pin_config(PinName(PIN_MICROSD_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_MICROSD_POWER, LOW);
}
void microSDPowerOff()
{
  pinMode(PIN_MICROSD_POWER, OUTPUT);
  pin_config(PinName(PIN_MICROSD_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_MICROSD_POWER, HIGH);
}

void imuPowerOn()
{
  pinMode(PIN_IMU_POWER, OUTPUT);
  pin_config(PinName(PIN_IMU_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_IMU_POWER, HIGH);
}
void imuPowerOff()
{
  pinMode(PIN_IMU_POWER, OUTPUT);
  pin_config(PinName(PIN_IMU_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_IMU_POWER, LOW);
}

void powerLEDOn()
{
#if(HARDWARE_VERSION_MAJOR >= 1)
  pinMode(PIN_PWR_LED, OUTPUT);
  pin_config(PinName(PIN_PWR_LED), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_PWR_LED, HIGH); // Turn the Power LED on
#endif
}
void powerLEDOff()
{
#if(HARDWARE_VERSION_MAJOR >= 1)
  pinMode(PIN_PWR_LED, OUTPUT);
  pin_config(PinName(PIN_PWR_LED), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
  digitalWrite(PIN_PWR_LED, LOW); // Turn the Power LED off
#endif
}

//Returns the number of milliseconds according to the RTC
//(In increments of 10ms)
//Watch out for the year roll-over!
uint64_t rtcMillis()
{
  myRTC.getTime();
  uint64_t millisToday = 0;
  int dayOfYear = calculateDayOfYear(myRTC.dayOfMonth, myRTC.month, myRTC.year + 2000);
  millisToday += ((uint64_t)dayOfYear * 86400000ULL);
  millisToday += ((uint64_t)myRTC.hour * 3600000ULL);
  millisToday += ((uint64_t)myRTC.minute * 60000ULL);
  millisToday += ((uint64_t)myRTC.seconds * 1000ULL);
  millisToday += ((uint64_t)myRTC.hundredths * 10ULL);

  return (millisToday);
}

//Returns the day of year
//https://gist.github.com/jrleeman/3b7c10712112e49d8607
int calculateDayOfYear(int day, int month, int year)
{
  // Given a day, month, and year (4 digit), returns
  // the day of year. Errors return 999.

  int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  // Verify we got a 4-digit year
  if (year < 1000) {
    return 999;
  }

  // Check if it is a leap year, this is confusing business
  // See: https://support.microsoft.com/en-us/kb/214019
  if (year % 4  == 0) {
    if (year % 100 != 0) {
      daysInMonth[1] = 29;
    }
    else {
      if (year % 400 == 0) {
        daysInMonth[1] = 29;
      }
    }
  }

  // Make sure we are on a valid day of the month
  if (day < 1)
  {
    return 999;
  } else if (day > daysInMonth[month - 1]) {
    return 999;
  }

  int doy = 0;
  for (int i = 0; i < month - 1; i++) {
    doy += daysInMonth[i];
  }

  doy += day;
  return doy;
}

//WatchDog Timer code by Adam Garbo:
//https://forum.sparkfun.com/viewtopic.php?f=169&t=52431&p=213296#p213296

// Watchdog timer configuration structure.
am_hal_wdt_config_t g_sWatchdogConfig = {

  // Configuration values for generated watchdog timer event.
  .ui32Config = AM_HAL_WDT_LFRC_CLK_16HZ | AM_HAL_WDT_ENABLE_RESET | AM_HAL_WDT_ENABLE_INTERRUPT,

  // Number of watchdog timer ticks allowed before a watchdog interrupt event is generated.
  .ui16InterruptCount = 16, // Set WDT interrupt timeout for 1 second.

  // Number of watchdog timer ticks allowed before the watchdog will issue a system reset.
  .ui16ResetCount = 20 // Set WDT reset timeout for 1.25 seconds.
};

void startWatchdog()
{
  // LFRC must be turned on for this example as the watchdog only runs off of the LFRC.
  am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_LFRC_START, 0);

  // Configure the watchdog.
  am_hal_wdt_init(&g_sWatchdogConfig);

  // Enable the interrupt for the watchdog in the NVIC.
  NVIC_EnableIRQ(WDT_IRQn);
  //NVIC_SetPriority(WDT_IRQn, 0); // Set the interrupt priority to 0 = highest (255 = lowest)
  //am_hal_interrupt_master_enable(); // ap3_initialization.cpp does this - no need to do it here

  // Enable the watchdog.
  am_hal_wdt_start();
}

// Interrupt handler for the watchdog.
extern "C++" void am_watchdog_isr(void)
{
  // Clear the watchdog interrupt.
  am_hal_wdt_int_clear();

  // DON'T Restart the watchdog.
  //am_hal_wdt_restart(); // "Pet" the dog.
}
