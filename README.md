# Current/Voltage DataLogger
![Image](./PowerDataLogger540.png)

## Open source data logger for voltage and current using Teensy 3.5 microcontroller<br>
Reads current and/or voltage and records data on SD card<br>
Based on DFRobot INA219-based board<br>
Supports voltage from 0 to 26 V and current for -8 to +8 A (bidirectionnal)<br>
Sampling frequency of the recordings adjustable from 5 to 1000 Hz<br>
Sampling buffer size of either 2,000 or 20,000 measurements<br>
Date and time of the recording is stored using Teensy RTC<br>
Optional zero-phase filtering of the data<br>

## Bill of materials:
Teensy 3.5 https://www.pjrc.com/store/teensy35.html<br>
DFRobot INA219 board: https://www.dfrobot.com/product-1827.html<br>
protoboard (surplus)<br>
6-row dipswitch https://www.digikey.ca/en/products/detail/te-connectivity-alcoswitch-switches/3-5435640-7/969216<br>
toggle switch (surplus)<br>
common anode bicolor led https://www.digikey.ca/en/products/detail/lumex-opto-components-inc/SSL-LX3059IGW-CA/<br>
2x24 female 0.1" header (1x4 optional for the sensor) <br>
2x24 + 1x2 + 1x5 male 0.1" header<br>
coin cell holder https://www.digikey.ca/en/products/detail/te-connectivity-amp-connectors/1775485-2/5272886<br>
CR2032 battery https://www.digikey.ca/en/products/detail/panasonic-bsg/CR2032/31939<br>
