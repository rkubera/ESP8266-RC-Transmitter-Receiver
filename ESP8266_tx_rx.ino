/* 
 *  RC Receiver and Transmitter based on esp8266 by Radek Kubera (rkubera)
 *  https://github.com/rkubera/ESP8266-RC-Transmitter-Receiver
 *  Licence:  For non-commercial use only.
 *            You cannot remove author header from the source code.
 *            Modification and redistribution allowed with the same license agreement.
 *            For commerial use please contact with author.
 */
 
#include <ESP8266WiFi.h>
#include <Servo.h>

/************************************************************
 * Module type and Wifi settings
 ************************************************************/
//MODULE TYPE
#define MODULE_RX                             //Compile software as RX Module
//#define MODULE_TX                             //Compile software as TX Module

//WIFI
const char *ssid = "rcreceiver";              //Wifi SSID
const char *pw = "qwerty123";                 //Wifi password

/************************************************************
 * Set configuration
 ************************************************************/

//ENABLED FEATURES
#define ENABLE_PWM                            //For TX: enable PWM outputs; for RX: enable PWM inputs
#define ENABLE_PPM                            //For TX: enable PPM output; for RX: enable PPM input
#define ENABLE_RSSI                           //Enable RSSI measuring. Works good with ESP8266 TX Module. if RemoRobo is used as TX Module, only 2 states will be generated: 50% (connected), 0% (no signal)
#define ENABLE_HEARTBEAT                      //Enable Heartbeat. Most recommedned!. If you use RoboRemo as TX Module, add Heartbeat control to UI with id=hbt and repeat period=100
#define ENABLE_FAIL_SAFE                      //Enable FailSafe. Working only if ENABLE_HEARTBEAT is enabled. If communication lost between TX and RX, Throttle channel (normally channel 3) will be set to low.
#define ENABLE_RX_CALLBACK                    //Enable callbacks. Most recommended for RoboRemo. If RX receives signal, generates the same callback signal to RoboRemo with "ret" prefix id. 
                                              //For example if TX sends id=5 (RC channel 5) with value=1500, RX will return to RoboRemo id=ret5 with value=1500.
                                              //You can create a led control in RoboRemo with id=ret5 for visualisaton of id=5 state.  

//TRANSMISSION PROTOCOLS
#define PROTOCOL_UDP_RC                       //Enable Channels transmission via udp. UDP is recommended.
#define PROTOCOL_UDP_UART                     //Enable UART transmission via udp. Can be used for telemetry (e.g. MavLink). UDP is recommended.
//#define PROTOCOL_TCP_RC                     //Enable Channels transmission via tcp
//#define PROTOCOL_TCP_UART                   //Enable UART transmission via tcp. Can be used for telemetry (e.g. MavLink).

const int tcp_port_rc = 9876;                 //Port for RC transmission via tcp
const int tcp_port_uart = 9877;               //Port for UART transmission via tcp
const int udp_port_rc = 9878;                 //Port for RC transmission via udp
const int udp_port_uart = 9879;               //Port for UART transmission via udp

//UART
#define UART_BAUD 57600                       //UART baud rate. MAvlink uses 57600 by default.

//HEARTBEAT
#define HEARTBEAT_TIMEOUT_MS 500              //Maximum Hearbeat period. After this period FailSafe event will be triggered. Default=500.

//PWM range
int usMin = 1000;                             // Minimal PWM value
int usMax = 2000;                             // Maximal PWM value
int usMiddle = usMin+((usMax-usMin)/2);       // Average PWM value

//THROTTLE
int throttle_rc_channel = 3;                  //Throttle RC Channel. Default=3
int throttle_initial_value = usMin;           //Initial Throttle value. Default=1000.

//RX FAIL SAFE
#define FAIL_SAFE_PWM_VALUE 900               //PWM Value level if FailSave event is generated. Default=900

//PPM
#define PPM_PIN 2                             //GPIO2 (D4)
#define CHANNELS_COUNT 12                     //1 - 16. Recommended=8 (Standard). Not all controllers work fine with more than 8 channels im PPM modulation.
                                              //INFO: 1-12 channels: 22500us PPM Frame Length. 13-16 channels: 38500us PPM Frame Length
#define PPM_POLARITY 1                        //Set polarity of the pulses: 1=Negative, 0=Positive. Default=1.

//RSSI
#define RSSI_RC_CHANNEL CHANNELS_COUNT        //-1=Disabled
#define RSSI_PWM_MIN_VALUE 1000               //PWM value if RSSI is the lowest. Default=1000.
#define RSSI_PWM_MAX_VALUE 2000               //PWM value if RSSI is the highest. Default=2000.

//PWM  
#define PWM1_PIN 14                           //GPIO14 (D5)
#define PWM2_PIN 12                           //GPIO12 (D6)
#define PWM3_PIN 13                           //GPIO13 (D7)
#define PWM4_PIN 15                           //GPIO15 (D8)
#define PWM5_PIN 5                            //GPIO5  (D1)
#define PWM6_PIN 4                            //GPIO4  (D2)
#define PWM7_PIN 0                            //GPIO0  (D3)
#if defined ENABLE_PPM
#define PWM8_PIN -1                           //-1=Disabled. If PPM is disabeld you can use PWM8_PIN 2: GPIO2 (D4)
#else
#define PWM8_PIN 2                            //GPIO2  (D4)
#endif
#define PWM9_PIN -1                           //-1=Disabled.
#define PWM10_PIN -1                          //-1=Disabled.
#define PWM11_PIN -1                          //-1=Disabled.
#define PWM12_PIN -1                          //-1=Disabled.
#define PWM13_PIN -1                          //-1=Disabled.
#define PWM14_PIN -1                          //-1=Disabled.
#define PWM15_PIN -1                          //-1=Disabled.
#define PWM16_PIN -1                          //-1=Disabled.

//LED
#define LED_PIN -1                            //-1=Disabled.

/************************************************************
 * End configuration
 ************************************************************/
#if defined (MODULE_TX) && defined (MODULE_RX)
#error You cannot define MODULE_TX and MODULE_RX together. Select only one option.
#endif

Servo servoCh[CHANNELS_COUNT];
int chPin[] = {PWM1_PIN, PWM2_PIN, PWM3_PIN, PWM4_PIN, PWM5_PIN, PWM6_PIN, PWM7_PIN, PWM8_PIN, PWM9_PIN, PWM10_PIN, PWM11_PIN, PWM12_PIN, PWM13_PIN, PWM14_PIN, PWM15_PIN, PWM16_PIN,};
volatile uint16_t chVal[CHANNELS_COUNT];

IPAddress rx_ip(192, 168, 0, 1);
IPAddress rx_netmask(255, 255, 255, 0);

//BUFFERS SIZE
#define bufferSize 1024

uint8_t myBuffer[bufferSize];
int bufIdx=0;

uint8_t uartBuffer[bufferSize];
int uartBufIdx=0;



unsigned long rcTcpLastAliveTimer = 0;
unsigned long uartTcpLastAliveTimer = 0;
unsigned long rcUdpLastAliveTimer = 0;
unsigned long uartUdpLastAliveTimer = 0;

unsigned long lastRetSentTimer = 0;

volatile int rcClientConnected = 0;
volatile int uartClientConnected = 0;

#if defined (PROTOCOL_UDP_RC) || defined (PROTOCOL_UDP_UART)
#include <WiFiUdp.h>
WiFiUDP udpSocket;
#endif

#if defined (PROTOCOL_TCP_RC) || defined (PROTOCOL_TCP_UART)
#include <WiFiClient.h>
WiFiClient tcpSocket;
#endif


void setup() {
  delay(500);
  Serial.begin(UART_BAUD);
  
  #ifdef MODULE_RX
  rx_setup();
  #endif

  #ifdef MODULE_TX
  tx_setup();
  #endif
}

void loop() {
  #ifdef MODULE_RX
  rx_loop();
  #endif

  #ifdef MODULE_TX
  tx_loop();
  #endif
}
