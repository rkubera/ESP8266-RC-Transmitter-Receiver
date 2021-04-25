# ESP8266 RC transmitter (TX) and receiver (RX) with RoboRemo support

## License
- For non-commercial use only. You cannot remove author header from source code.
Modification and redistribution allowed with the same license agreement.
For commerial use please contact with author.

## Technology
- Arduino sketch for ESP8266 processors.

## RC Receiver:
- ESP8266 as RX Module

## Supported RC transmitters:
- RoboRemo http://www.roboremo.com (mobile phone application) as transmitter
- ESP8266 TX Module connected to standard RC transmitter
- PC connected to RX Module via wifi (with heartbeat feature disabled on RX)

## Features:
- Up to 16 RC Channels
- UART 2-way transmitter/receiver for telemetry
- 1 PPM Input(TX)/Output(RX) - GPIO2
- 7 or 8 (if PPM is disabled) PWM Input(TX)/Output(RX) - GPIO14, GPIO12, GPIO13, GPIO15, GPIO5, GPIO4, GPIO0 , GPIO2 (if PPM is disabled)
- RSSI support (last RC channel as RSSI PWM signal: 1000ms=0%, 2000ms=100%)
- FailSafe support (throttle RC channel (default 3) = 900ms if communication losts between TX and RX)
- Heartbeat

## Warning:
- only 1 transmitter for RC Channels can be connected to the receiver at the same time 
- only 1 transmitter for UART communication can be connected to the receiver at the same time
- if you use esp8266 as TX Module and you need to 2 separate transmitters: 1 for RC Channels and 1 for UART, disable one of protocols in esp8266 TX Module
- sometimes you will need 3.3V/5V signal converter for communication with RC tansmitter or RC controller

## ESP8266 RX Module compilation (quick steps)
- open code in Arduino IDE
- change wifi settings (wifi, password) in ESP8266_tx_rx.ino
- comment #define MODULE_TX
- uncomment #define MODULE_RX
- check enabled features and other settings
- compile and upload to ESP8266

## ESP8266 TX Module compilation (quick steps)
- open code in Arduino IDE
- change wifi settings (wifi, password) in ESP8266_tx_rx.ino (the same as in RX)
- uncomment #define MODULE_TX
- comment #define MODULE_RX
- check enabled features and other settings
- compile and upload to ESP8266

## Information for RoboRemo users (quick steps)
- power on RX and connect mobile phone to RX wifi network. Connect RoboRemo via UDP to host 192.168.0.1 port:9878
- add heartbeat control do RoboRemo UI with id="hbt" and refresh period=100 (important!)
- for all controls (except heartbeat and field control type) set id as RC Channel number, e.g.for RC Channel 1 set id=1
- for field control type set 2 RC Channels using "_" as separator, e.g. for RC Channels 1,3 set id=1_3.
- all controls values must set a minimal value=1000
- all controls values must set a maximal value=2000
- if RX receives signal, generates the same callback signal to RoboRemo with "ret" prefix id.
  For example if TX sends id=5 (RC channel 5) with value=1500, RX will return to RoboRemo id=ret5 with value=1500.
  Using this mechanism You can create a led control in RoboRemo with id=ret5 for visualisaton of id=5 state.

## Information for PC connection
- compile RX Module with heartbeat feature disabled. Upload code to ESP8266
- power on RX and connect PC to RX wifi network. 
- connect PC via UDP (using UDP client) to host 192.168.0.1 port:9878 for RC commands
  - send command: 1 1500\n for set RC Channel 1 to 1500ms
- connect PC via UDP (using UDP client) to host 192.168.0.1 port:9879 for UART transmission

## Range tests:
- 110 meters in urban area: ESP01 as RX and Wemos D1 Mini Pro with external antenna (2dbi) as TX
- 160 meters in urban area: Wemos D1 Mini Pro with external antenna (2dbi) and Wemos D1 Mini Pro with external antenna (2dbi) as TX
- 220 meters in urban area: Wemos D1 Mini Pro with external antenna (2dbi) and Wemos D1 Mini Pro with external antenna (12dbi) as TX
- in rural or open area range can be match more, but not tested.

## Tested with hardware
- ESP01 as TX and RX
- Wemos D1 Mini Pro as TX and RX

## Tested with software
- QGroundControl/MissionPlanner connected to Micro Pix/Pixhawk controller via UART transmitter/receiver (ESP01)
