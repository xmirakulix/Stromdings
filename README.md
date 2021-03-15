# Stromdings - 16-port smart power meter

Stromdings uses an ATMEGA328 microcontroller and four MCP3304 analog-digital converters to measure the power on up to 16 circuits using current transformers.
Measurements are transmitted via WiFi to a home assistant server using mqtt.

Features:
- Measure up to 16 ports 
- Up to 16A (3.6kW @ 230V) per port
- Non-invasive installation using 'clip on' current transformer clamps
- +/- 2 Watt accuracy
- Transmit measurements via WiFi using MQTT

## Software

Stromdings is an Arduino project, developed in VSCode using the [PlatrormIO extension](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide). It runs on an 16MHz ATMega328P microcontroller.

Requirements to the microcontroller:
- Analog port with hardware pinchange interrupt
- Two serial ports or support for software serial interface
- Serial peripheral interface (SPI)
- i2c bus

Used third party libraries:
- Software serial: [AltSoftSerial](https://github.com/xmirakulix/AltSoftSerial) (forked)
- WiFi (ESP8266): [WiFiEspAT](https://github.com/xmirakulix/WiFiEspAT) (forked)
- i2c LCD (16x2): [hd44780](https://github.com/duinoWitchery/hd44780)
- MQTT: [pubsubclient](https://github.com/xmirakulix/pubsubclient) (forked)

## Hardware

- ATMega328P microcontroller
- MCP3304 analog-digital converters
- [YHDC SCT-013](http://en.yhdc.com/product/SCT013-401.html) 100A current transformers (1:2000 turns)
- 220 Ohm shunt resistors for measurements up to 16A (3.6kW @ 230V) per port
- ESP-01 WiFi module using ESP8266 chip
- LCD Display with 16x2 characters via i2c
- Rotary encoder to select displayed info on the LCD

## Schematics / PCB design

Schematics and PCB was designed using the [EasyEDA](https://easyeda.com/) online editor. Schematics, PCB Design and final Gerber-Files are available in the 'doc' folder.

Schematics: https://github.com/xmirakulix/Stromdings/raw/master/doc/Schematics.pdf

PCB: 

![PCB](https://github.com/xmirakulix/Stromdings/raw/master/doc/PCB.svg "PCB design")


## Ideas for the future

- PCB: D1 orientation wrong
- PCB: add reset button
- PCB: Power cycle possibility for ESP-01
- PCB: Optimize 2.5V ref voltage track routing
- PCB: Check debounce cap necessity for rotary
