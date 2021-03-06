# Bluetooth Low Energy (BLE) Eddystone beacon using STM8 and nRF24L01+

This demo code implements one of the cheapest possible ways to implement a BLE beacon.

It uses low-cost nRF24L01-module for BLE compatible radio and STM8 development module as a microcontroller.

Idea for abusing NRFL01 originates from http://dmitry.gr/?r=05.Projects&proj=11.%20Bluetooth%20LE%20fakery (including related code).

Idea to use that for Eddystone URL-beacon, and some related code, I found from https://github.com/kasparsd/tinystone .

This implementation adds related STM8 code and also support for interleaved Eddystone telemetry (TLM) frames containing temperature data, which can be read from attached DS18B20 sensor.

Git branch 'temperature-beacon' contains version that broadcasts only temperature data without any Eddystone data.

[A blog post about all this...](https://mcuer.sikamo.net/blog/stm8-nrf24-ble-beacon)

## Hardware used
This example assumes following hardware configuration:
- STM8 development board based on **STM8S103F3**.

  I am using the really cheap Chinese STM8S103F3P6 based minimal development
  board available from Ebay/Aliexpress ("STM8 Minimum System Development Board Module").

- **ST-LINK/V2** programmer
- **nRF24L01+** module

  Connect PINs:
  ```
  CSN <-> A3
  CE <-> D2
  SCK <-> C5
  MOSI <-> C6
  MISO <-> C7
  ```

- Optionally for TLM temperature: **DS18B20**-sensor. 

  See https://github.com/jukkas/stm8-sdcc-examples/tree/master/ds18b20 . Except DS18B20 data pin is `B4`.

## Software/development environment
- Linux
- SDCC compiler
- stm8flash for flashing the board. https://github.com/vdudouyt/stm8flash


## Notes
BLE address that this implementation broadcasts is staticly defined in code. So if you have multiple devices, change the address in code to be unique for each device.

## Copyrights

Some of the NRF24 BLE code is copied/adapted from https://github.com/kasparsd/tinystone/blob/master/main.c ,
which in turn has copied from http://dmitry.gr/?r=05.Projects&proj=11.%20Bluetooth%20LE%20fakery ,
which says:
> "All the code as well as the research that went into this and is published here is under this license:
> you may use it in any way you please if and only if it is for non-commercial purposes,
> you must provide a link to this page as well. Any commercial use must be discussed with me." (http://dmitry.gr/)

All other written by me is public domain. No guarantees etc. Use at your own risk.