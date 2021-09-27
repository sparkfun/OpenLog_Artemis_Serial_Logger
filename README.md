SparkFun OpenLog Artemis : Serial Logger
===========================================================

[![SparkFun OpenLog Artemis](https://cdn.sparkfun.com//assets/parts/1/5/7/5/3/16832-SparkFun_OpenLog_Artemis-01.jpg)](https://www.sparkfun.com/products/16832)

[*SparkFun OpenLog Artemis (SPX-16832)*](https://www.sparkfun.com/products/16832)

The OpenLog Artemis is an open source datalogger that comes preprogrammed to automatically log IMU, GPS, serial data, and various pressure, humidity, and distance sensors. All without writing a single line of code! OLA automatically detects, configures, and logs Qwiic sensors. OLA is designed for users who just need to capture a bunch of data to SD and get back to their larger project.

The firmware in this repo is dedicated to logging high rate serial data. [You can find the main OpenLog Artemis repo here](https://github.com/sparkfun/OpenLog_Artemis).

OpenLog Artemis is highly configurable over an easy to use serial interface. Simply plug in a USB C cable and open a terminal at 115200kbps. The logging output is automatically streamed to both the terminal and the microSD. Pressing any key will open the configuration menu.

New features are constantly being added so we’ve released an easy to use firmware upgrade tool. No need to install Arduino or a bunch of libraries, simply open the [Artemis Firmware Upload GUI](https://github.com/sparkfun/Artemis-Firmware-Upload-GUI), load the latest OLA firmware, and add features to OpenLog Artemis as they come out! Full instructions are available in [UPGRADE.md](./UPGRADE.md).

Limitations
-------------------

This firmware works well with baud rates up to 230400. Near-continuous (80% duty, ~20KB/s) logging of NMEA data at 230400 produces clean log files.
The wheels come off at 460800 baud, for reasons we don't yet understand.

Repository Contents
-------------------

* **/Binaries** - The binary files for the different versions of the OLA firmware.
* **/Firmware** - The main sketch that runs on the OpenLog Artemis.

Documentation
--------------

* **[UPGRADE.md](./UPGRADE.md)** - contains full instructions on how to upgrade the firmware on the OLA using the [Artemis Firmware Upload GUI](https://github.com/sparkfun/Artemis-Firmware-Upload-GUI).
* **[CONTRIBUTING.md](./CONTRIBUTING.md)** - guidance on how to contribute to this library.
* **[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)** - OLA includes a large number of libraries that will need to be installed before compiling will work.
* **Hookup Guide** - You can find the OpenLog Artemis Hookup Guide [here](https://learn.sparkfun.com/tutorials/openlog-artemis-hookup-guide).

License Information
-------------------

This product is _**open source**_!

Please see [LICENSE.md](./LICENSE.md) for full details.

- Your friends at SparkFun.
