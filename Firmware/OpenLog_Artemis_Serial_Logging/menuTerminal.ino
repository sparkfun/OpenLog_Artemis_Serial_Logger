void menuLogRate()
{
  while (1)
  {
    SerialPrintln(F(""));
    SerialPrintln(F("Menu: Configure Terminal Output"));

    SerialPrint(F("1) Set Serial Terminal Baud Rate: "));
    Serial.print(settings.serialTerminalBaudRate);
    SerialPrintln(F(" bps"));

    SerialPrint(F("2) Open New Log File After (s): "));
    SerialPrintf2("%d", settings.openNewLogFilesAfter);
    if (settings.openNewLogFilesAfter == 0) SerialPrintln(F(" (Never)"));
    else SerialPrintln(F(""));

    SerialPrint(F("3) Use pin 11 to open a new log file: "));
    if (settings.useGPIO11ForTrigger == true) SerialPrintln(F("Yes"));
    else SerialPrintln(F("No"));

    SerialPrint(F("4) New log file is opened when the signal on pin 11 is: "));
    if (settings.fallingEdgeTrigger == true) SerialPrintln(F("Falling"));
    else SerialPrintln(F("Rising"));

    SerialPrintln(F("x) Exit"));

    int incoming = getNumber(menuTimeout); //Timeout after x seconds

    if (incoming == 1)
    {
      SerialPrint(F("Enter baud rate (1200 to 500000): "));
      int newBaud = getNumber(menuTimeout); //Timeout after x seconds
      if (newBaud < 1200 || newBaud > 500000)
      {
        SerialPrintln(F("Error: baud rate out of range"));
      }
      else
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
        
        settings.serialTerminalBaudRate = newBaud;
        recordSystemSettings(); //Normally recorded upon all menu exits
        SerialPrintf2("Terminal now set at %dbps. Please reset device and open terminal at new baud rate. Freezing...\r\n", settings.serialTerminalBaudRate);
        while (1);
      }
    }
    else if (incoming == 2)
    {
      SerialPrintln(F("Open new log file after this many seconds (0 or 10 to 129,600) (0 = Never):"));
      int64_t tempSeconds = getNumber(menuTimeout); //Timeout after x seconds
      if ((tempSeconds < 0) || ((tempSeconds > 0) && (tempSeconds < 10)) || (tempSeconds > 129600ULL))
        SerialPrintln(F("Error: Invalid interval"));
      else
        settings.openNewLogFilesAfter = tempSeconds;
    }
    else if (incoming == 3)
    {
      if (settings.useGPIO11ForTrigger == true)
      {
        // Disable triggering
        settings.useGPIO11ForTrigger = false;
        detachInterrupt(PIN_TRIGGER); // Disable the interrupt
        pinMode(PIN_TRIGGER, INPUT); // Remove the pull-up
        pin_config(PinName(PIN_TRIGGER), g_AM_HAL_GPIO_INPUT); // Make sure the pin does actually get re-configured
        triggerEdgeSeen = false; // Make sure the flag is clear
      }
      else
      {
        // Enable triggering
        settings.useGPIO11ForTrigger = true;
        pinMode(PIN_TRIGGER, INPUT_PULLUP);
        pin_config(PinName(PIN_TRIGGER), g_AM_HAL_GPIO_INPUT_PULLUP); // Make sure the pin does actually get re-configured
        delay(1); // Let the pin stabilize
        am_hal_gpio_pincfg_t intPinConfig = g_AM_HAL_GPIO_INPUT_PULLUP;
        if (settings.fallingEdgeTrigger == true)
        {
          attachInterrupt(PIN_TRIGGER, triggerPinISR, FALLING); // Enable the interrupt
          intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_HI2LO;
        }
        else
        {
          attachInterrupt(PIN_TRIGGER, triggerPinISR, RISING); // Enable the interrupt
          intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI;
        }
        pin_config(PinName(PIN_TRIGGER), intPinConfig); // Make sure the pull-up does actually stay enabled
        triggerEdgeSeen = false; // Make sure the flag is clear
      }
    }
    else if (incoming == 4)
    {
      if (settings.useGPIO11ForTrigger == true) // If interrupts are enabled, we need to disable and then re-enable
      {
        detachInterrupt(PIN_TRIGGER); // Disable the interrupt
        settings.fallingEdgeTrigger ^= 1; // Invert the flag
        am_hal_gpio_pincfg_t intPinConfig = g_AM_HAL_GPIO_INPUT_PULLUP;
        if (settings.fallingEdgeTrigger == true)
        {
          attachInterrupt(PIN_TRIGGER, triggerPinISR, FALLING); // Enable the interrupt
          intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_HI2LO;
        }
        else
        {
          attachInterrupt(PIN_TRIGGER, triggerPinISR, RISING); // Enable the interrupt
          intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI;
        }
        pin_config(PinName(PIN_TRIGGER), intPinConfig); // Make sure the pull-up does actually stay enabled
        triggerEdgeSeen = false; // Make sure the flag is clear
      }
      else
        settings.fallingEdgeTrigger ^= 1; // Interrupt is not currently enabled so simply invert the flag
    }
    else if (incoming == STATUS_PRESSED_X)
      return;
    else if (incoming == STATUS_GETNUMBER_TIMEOUT)
      return;
    else
      printUnknown(incoming);
  }
}
