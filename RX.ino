/* 
 *  RC Receiver and Transmitter based on esp8266 by Radek Kubera (rkubera)
 *  https://github.com/rkubera/ESP8266-RC-Transmitter-Receiver
 *  Licence:  For non-commercial use only.
 *            You cannot remove author header from the source code.
 *            Modification and redistribution allowed with the same license agreement.
 *            For commerial use please contact with author.
 */
 
#ifdef MODULE_RX
#include <String.h>

#ifdef PROTOCOL_TCP_RC
#include <WiFiClient.h>
WiFiServer rx_server_tcp_rc(tcp_port_rc);
WiFiClient rx_client_tcp_rc;
#endif

#ifdef PROTOCOL_TCP_UART
#include <WiFiClient.h>
WiFiServer rx_server_tcp_uart(tcp_port_uart);
WiFiClient rx_client_tcp_uart;
#endif

#ifdef PROTOCOL_UDP_RC
#include <WiFiUdp.h>
WiFiUDP rx_udp_rc;
IPAddress rx_remoteIp_udp_rc;
int rx_remotePort_udp_rc;
#endif

#ifdef PROTOCOL_UDP_UART
#include <WiFiUdp.h>
WiFiUDP rx_udp_uart;
IPAddress rx_remoteIp_udp_uart;
int rx_remotePort_udp_uart;
#endif

char rx_message[255];

void rx_send_uart_message(char* my_message, int my_size) {
  if (my_size<=0) return;
  #ifdef PROTOCOL_UDP_UART
  rx_udp_uart.beginPacket(rx_remoteIp_udp_uart, rx_remotePort_udp_uart);
  rx_udp_uart.write(my_message, my_size);
  rx_udp_uart.endPacket();
  #endif
  
  #ifdef PROTOCOL_TCP_UART
  if (rx_client_tcp_uart.connected()) {
    rx_client_tcp_uart.write(my_message, my_size);
  }
  #endif
}

void rx_send_rc_message(char* my_message, int my_size) {
  if (my_size<=0) return;
  #ifdef PROTOCOL_TCP_RC
  rx_client_tcp_rc.write(my_message, my_size);
  #endif
        
  #ifdef PROTOCOL_UDP_RC
  rx_udp_rc.beginPacket(rx_remoteIp_udp_rc, rx_remotePort_udp_rc);
  rx_udp_rc.write(my_message, my_size);
  rx_udp_rc.endPacket();
  #endif
}

#ifdef ENABLE_RX_CALLBACK
void rx_sendCommand (int channelIdx, int value) {  
  sprintf(rx_message, "ret%i %i\n"+0,channelIdx,value);
  rx_send_rc_message(rx_message, strlen (rx_message));
  //Hack for RoboRemo
  #ifdef PROTOCOL_UDP_RC
  int tmp = rx_remotePort_udp_rc;
  rx_remotePort_udp_rc = udp_port_rc;
  rx_send_rc_message(rx_message, strlen (rx_message));
  rx_remotePort_udp_rc = tmp;
  #endif
}
#endif

void rx_parseCommand(uint8_t buf[bufferSize]) {
  static String myBuff = "";
  myBuff = myBuff+(char*) buf;
  int start = 0;
  String command;
  String line;
  String value;
  String value1, value2;
  int lineIdx;
  String message = "";
  do {
    lineIdx = myBuff.indexOf('\n', start);
    if (lineIdx>-1) {
      line = myBuff.substring(start, lineIdx);
      start = lineIdx+1;
      line.trim();
    }
    else {
      line = "";
    }
    
    if (line.length()>0) {
      command = line.substring(0, 2);
      int actCh = -1;
      //BIN format
      if (command=="cr" || command=="ch") {
        for (unsigned int i=2; i<line.length(); i=i+2) {
          //uint16_t value = (byte)line[i]+(((byte)line[i+1])*256);
          uint16_t value = (byte)line[i]+(((byte)line[i+1]) << 8);
          if (actCh==-1) {
            actCh=0;
            if (command=="cr") {
              #ifdef ENABLE_RSSI
              if (rcClientConnected==0) {
                setRSSI(value);
              }
              else {
                setRSSI(value);
              }
              #endif
              continue;
            }
          }
          if (actCh<CHANNELS_COUNT) {
            chVal[actCh] = value;
          }
          actCh++;
        }
      }
      //TXT format
      else {
        //Serial.println(line);
        int valIdx = line.indexOf(' ');
        if (valIdx>-1) {
          command = line.substring(0, valIdx);
        }
        else {
          command = line;
        }
  
        int channelIdx1 = command.toInt();
        int channelIdx2 = -1;
  
        #ifdef ENABLE_RSSI
        if (command == "rssi") {
          channelIdx1 = -1;
          int valIdx = line.indexOf(' ');
          int rssi = line.substring(valIdx+1).toInt();
          if (rcClientConnected==0) {
            setRSSI(usMin);
          }
          else {
            setRSSI(rssi);
          }
        }
        #endif
        
        valIdx = command.indexOf('_');
        if (valIdx>-1) {
          String command1 = command.substring(0, valIdx);
          channelIdx1 = command1.toInt();
          command1 = command.substring(valIdx+1);
          channelIdx2 = command1.toInt();
        }
        
        if (channelIdx1>0 && channelIdx1<=CHANNELS_COUNT) {
          value = chVal[channelIdx1-1];
          valIdx = line.indexOf(' ');
          if (valIdx>-1) {
            value = line.substring(valIdx+1);
            valIdx = value.indexOf(' ');
            if (valIdx>-1) {
              //2 values
              value1 = value.substring(0,valIdx+1);
              chVal[channelIdx1-1] = value1.toInt();
              
              value2 = value.substring(valIdx+1);
              if (channelIdx2>0 && channelIdx2<=CHANNELS_COUNT) {
                chVal[channelIdx2-1] = value2.toInt();
              }
              #ifdef ENABLE_RX_CALLBACK
              sprintf(rx_message, "ret%i %i\n",channelIdx1, chVal[channelIdx1-1]);
              message = message+(char*)rx_message;
              if (channelIdx2>0 && channelIdx1<=CHANNELS_COUNT) {
                rx_sendCommand(channelIdx2, chVal[channelIdx2-1]);
              }
              #endif
            }
            else {
              //1 value
              chVal[channelIdx1-1] = value.toInt();
              #ifdef ENABLE_RX_CALLBACK
              sprintf(rx_message, "ret%i %i\n",channelIdx1, chVal[channelIdx1-1]);
              message = message+(char*)rx_message;
              #endif
            }
          }
          else {
            // no value
            if (value.toInt()==usMiddle) {
              chVal[channelIdx1-1] = usMax;
            }
            else {
              chVal[channelIdx1-1] = usMiddle;
            }
            #ifdef ENABLE_RX_CALLBACK
            sprintf(rx_message, "ret%i %i\n",channelIdx1, chVal[channelIdx1-1]);
            message = message+(char*)rx_message;
            #endif
          }
        } 
      }
    }
  }
  while (lineIdx>-1);
  if (start>0) {
    myBuff = myBuff.substring(start);
  }
  if (message.length()>0) {
    sprintf(rx_message,"%s", message.c_str());
    rx_send_rc_message(rx_message, strlen (rx_message));
    //Hack for RoboRemo
    #ifdef PROTOCOL_UDP_RC
    int tmp = rx_remotePort_udp_rc;
    rx_remotePort_udp_rc = udp_port_rc;
    rx_send_rc_message(rx_message, strlen (rx_message));
    rx_remotePort_udp_rc = tmp;
    #endif
  }
}

void rx_setup() {

  for(int i=0; i<CHANNELS_COUNT; i++){
    chVal[i]= usMiddle;
  }
  
  if (throttle_rc_channel>=0 && throttle_rc_channel<CHANNELS_COUNT) {
    chVal[throttle_rc_channel-1]=throttle_initial_value;
  }

  wifi_set_phy_mode(PHY_MODE_11B);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(rx_ip, rx_ip, rx_netmask);
  WiFi.softAP(ssid, pw);
  
  Serial.println("");
  Serial.println("------------------");
  Serial.println("Starting RX Module");
  Serial.println("------------------");

  #ifdef PROTOCOL_TCP_RC
  Serial.print("Starting TCP Server for RC transmission on port ");
  Serial.println(tcp_port_rc);
  rx_server_tcp_rc.begin();
  #endif

  #ifdef PROTOCOL_UDP_RC
  Serial.print("Starting UDP Server for RC transmission on port ");
  Serial.println(udp_port_rc);
  rx_udp_rc.begin(udp_port_rc);
  #endif

  #ifdef PROTOCOL_TCP_UART
  Serial.print("Starting TCP Server for UART transmission on port ");
  Serial.println(tcp_port_uart);
  rx_server_tcp_uart.begin();
  #endif

  #ifdef PROTOCOL_UDP_UART
  Serial.print("Starting UDP Server for UART transmission on port ");
  Serial.println(udp_port_uart);
  rx_udp_uart.begin(udp_port_uart);
  #endif
  
   //PPM
  #ifdef ENABLE_PPM
  pinMode(PPM_PIN,OUTPUT);
  digitalWrite(PPM_PIN, PPM_POLARITY); //set the PPM signal pin to the default state (off)
  rx_ppmInit();
  #endif

  //RSSI
  #ifdef ENABLE_RSSI
  rssi_setup();
  #endif

  //PWM
  #ifdef ENABLE_PWM
  for(int i=0; i<CHANNELS_COUNT; i++) {
    if (chPin[i]>-1) {
      pinMode(chPin[i], OUTPUT);
      servoCh[i].attach(chPin[i], usMin, usMax);
      #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
      if (i!=(RSSI_RC_CHANNEL-1))
      #endif
      {
        servoCh[i].writeMicroseconds( chVal[i] );
      }
    }
  }
  #endif

  //LED
  if (LED_PIN>-1) {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
  }
}

void rx_loop() {

  #ifdef ENABLE_HEARTBEAT
  //Check heardbeat
  if (HEARTBEAT_TIMEOUT_MS<(millis()-rcTcpLastAliveTimer) && HEARTBEAT_TIMEOUT_MS<(millis()-rcUdpLastAliveTimer)) {
    rcClientConnected = 0;
  }
  else {
    rcClientConnected = 1;
  }
  if (HEARTBEAT_TIMEOUT_MS<(millis()-uartTcpLastAliveTimer) && HEARTBEAT_TIMEOUT_MS<(millis()-uartUdpLastAliveTimer)) {
    uartClientConnected = 0;
  }
  else {
    uartClientConnected = 1;
  }
  #else
  uartClientConnected = 1;
  rcClientConnected = 1;

  if (LED_PIN>-1) {
    if (rx_rcClientConnected==1) {
      digitalWrite(LED_PIN, LOW);
    }
    else {
      digitalWrite(LED_PIN, HIGH);
    }
  }
  #endif
  #ifdef ENABLE_PWM
  for(int i=0; i<CHANNELS_COUNT; i++) {
    if (chPin[i]>-1) {
      #ifdef ENABLE_FAIL_SAFE
      if (rcClientConnected==0 && (i+1)==throttle_rc_channel && chPin[i+1]>-1) {
        servoCh[i].writeMicroseconds(FAIL_SAFE_PWM_VALUE);
      }
      else
      #endif
      {
        #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
        if (i!=(RSSI_RC_CHANNEL-1))
        #endif
        {
          servoCh[i].writeMicroseconds(chVal[i]);
        }
      }
    }
  }
  #endif

  #ifdef ENABLE_RX_CALLBACK
  static int act_Channel = 0;
  if (100<(millis()-lastRetSentTimer) && rcClientConnected==1) {
    rx_sendCommand(act_Channel+1, chVal[act_Channel]);
    act_Channel ++;
    if (act_Channel>=CHANNELS_COUNT) {
      act_Channel = 0;
    }
    lastRetSentTimer = millis();
  }
  #endif

  //Sockets and UART loop
  #ifdef PROTOCOL_TCP_RC
  if(!rx_client_tcp_rc.connected()) {
    rx_client_tcp_rc.stop();
    rx_client_tcp_rc = rx_server_tcp_rc.available();
  }
  else {
    tcpSocket = rx_client_tcp_rc;
    if (getTcpBuffer((char*)myBuffer, bufIdx)) {
      rx_parseCommand(myBuffer);
      rcTcpLastAliveTimer = millis();
    }
  }
  #endif

  #ifdef PROTOCOL_TCP_UART
  if(!rx_client_tcp_uart.connected()) {
    rx_client_tcp_uart.stop();
    rx_client_tcp_uart = rx_server_tcp_uart.available();
  }
  else {
    tcpSocket = rx_client_tcp_uart;
    if (getTcpBuffer((char*)myBuffer, bufIdx)) {
      Serial.write(myBuffer, bufIdx);
      uartTcpLastAliveTimer = millis();
    }
  }
  #endif

  #ifdef PROTOCOL_UDP_RC
  udpSocket = rx_udp_rc;
  if (getUdpBuffer((char*)myBuffer, bufIdx, rx_remoteIp_udp_rc, rx_remotePort_udp_rc)) {
    rx_parseCommand(myBuffer);
    rcUdpLastAliveTimer = millis();
  } 
  #endif

  #ifdef PROTOCOL_UDP_UART
  udpSocket = rx_udp_uart;
  if (getUdpBuffer((char*)myBuffer, bufIdx, rx_remoteIp_udp_uart, rx_remotePort_udp_uart)) {
    Serial.write(myBuffer, bufIdx);
    uartUdpLastAliveTimer = millis();
  }
  #endif
  #if defined (PROTOCOL_UDP_UART) || defined (PROTOCOL_TCP_UART)
  if (getSerialBuffer((char*)myBuffer, bufIdx)) {
    rx_send_uart_message((char*)myBuffer, bufIdx);
  }
  #endif
  
  yield();
  delay(10);
}
#endif
