/* 
 *  RC Receiver and Transmitter based on esp8266 by Radek Kubera (rkubera)
 *  https://github.com/rkubera/ESP8266-RC-Transmitter-Receiver
 *  Licence:  For non-commercial use only.
 *            You cannot remove author header from the source code.
 *            Modification and redistribution allowed with the same license agreement.
 *            For commerial use please contact with author.
 */
 
#include <String.h>
#ifdef MODULE_TX

#ifdef PROTOCOL_TCP_RC
#include <WiFiClient.h>
WiFiServer tx_server_tcp_rc(tcp_port_rc);
WiFiClient tx_client_tcp_rc;
#endif

#ifdef PROTOCOL_TCP_UART
#include <WiFiClient.h>
WiFiServer tx_server_tcp_uart(tcp_port_uart);
WiFiClient tx_client_tcp_uart;
#endif

#ifdef PROTOCOL_UDP_RC
#include <WiFiUdp.h>
WiFiUDP tx_udp_rc;
IPAddress tx_remoteIp_udp_rc;
int tx_remotePort_udp_rc;
#endif

#ifdef PROTOCOL_UDP_UART
#include <WiFiUdp.h>
WiFiUDP tx_udp_uart;
IPAddress tx_remoteIp_udp_uart;
int tx_remotePort_udp_uart;
#endif

char tx_message[255];

volatile int chLastVal[CHANNELS_COUNT];

uint16_t rssiValue = 0;
void tx_parseCommand(uint8_t buf[bufferSize]) {
  
}

void tx_send_uart_message(char* my_message, int my_size) {
  if (my_size<=0) return;
  #ifdef PROTOCOL_UDP_UART
  tx_udp_uart.beginPacket(rx_ip, udp_port_uart);
  tx_udp_uart.write(my_message, my_size);
  tx_udp_uart.endPacket();
  #endif
  
  #ifdef PROTOCOL_TCP_UART
  if (tx_client_tcp_uart.connected()) {
    tx_client_tcp_uart.write(my_message, my_size);
  }
  #endif
}

void tx_send_rc_message(char* my_message, int my_size) {
  if (my_size<=0) return;
  #ifdef PROTOCOL_UDP_RC
  tx_udp_rc.beginPacket(rx_ip, udp_port_rc);
  tx_udp_rc.write(my_message, my_size);
  tx_udp_rc.endPacket();
  #endif

  #ifdef PROTOCOL_TCP_RC
  if (tx_client_tcp_rc.connected()) {
    tx_client_tcp_rc.write(my_message, my_size);
  }
  #endif
}

void tx_setup() {
  for (int i=0; i<CHANNELS_COUNT;i++) {
    chLastVal[i] = 0;
    chVal[i] = usMiddle;
  }
  
  wifi_set_phy_mode(PHY_MODE_11B);
  WiFi.mode(WIFI_STA);
  
  //RSSI
  #ifdef ENABLE_RSSI
  rssi_setup();
  #endif
  
  //PWM
  #ifdef ENABLE_PWM
  tx_pwm_setup();
  #endif
  
  //PPM
  #ifdef ENABLE_PPM
  tx_ppm_setup();
  #endif

  Serial.println("");
  Serial.println("------------------");
  Serial.println("Starting TX Module");
  Serial.println("------------------");
   
  #ifdef PROTOCOL_TCP_RC
  Serial.print("Starting TCP Server for RC transmission on port ");
  Serial.println(tcp_port_rc);
  tx_server_tcp_rc.begin();
  #endif

  #ifdef PROTOCOL_UDP_RC
  Serial.print("Starting UDP Server for RC transmission on port ");
  Serial.println(udp_port_rc);
  tx_udp_rc.begin(udp_port_rc);
  #endif

  #ifdef PROTOCOL_TCP_UART
  Serial.print("Starting TCP Server for UART transmission on port ");
  Serial.println(tcp_port_uart);
  tx_server_tcp_uart.begin();
  #endif

  #ifdef PROTOCOL_UDP_UART
  Serial.print("Starting UDP Server for UART transmission on port ");
  Serial.println(udp_port_uart);
  tx_udp_uart.begin(udp_port_uart);
  #endif
}

void tx_loop() {
  //Reconnect to AP if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pw);
    for (int i = 0; i < 1000; i++) {
      if ( WiFi.status() != WL_CONNECTED ) {
        delay(10); 
      }
      else {
        break;
      }
    }
  }
  //AP Connected
  else {
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
      if (rcClientConnected==1) {
        digitalWrite(LED_PIN, LOW);
      }
      else {
        digitalWrite(LED_PIN, HIGH);
      }
    }
    #endif
  
    if (20<(millis()-lastRetSentTimer)) {
      //BIN Format
      byte lowbyte, hibyte;
      String message = "";
      #ifdef ENABLE_RSSI
      message = "cr";
      rssiValue = WiFi.RSSI();
      rssiValue = map (rssiValue, -100, -50, 1000, 2000);
      if (rssiValue<1000) rssiValue = 1000;
      if (rssiValue>2000) rssiValue = 2000;
      setRSSI(rssiValue);
      lowbyte = (byte) (rssiValue & 0xFF);
      hibyte = (byte) ((rssiValue >> 8) & 0xFF);
      message = message + (char)lowbyte + (char)hibyte;
      #else
      message = "ch";
      #endif
      for (int i=0; i<CHANNELS_COUNT; i++) {
        lowbyte = (byte) (chVal[i] & 0xFF);
        hibyte = (byte) ((chVal[i] >> 8) & 0xFF);
        message = message + (char)lowbyte + (char)hibyte;
      }
        
      /*
       * //TXT Format
      for (int i=0; i<CHANNELS_COUNT; i++) {
        sprintf(tx_message, "%i %i\n"+0,i+1,chVal[i]);
        message = message+(char*)tx_message;
      }

      #ifdef ENABLE_RSSI
      rssiValue = WiFi.RSSI();
      rssiValue = map (rssiValue, -100, -50, 1000, 2000);
      if (rssiValue<1000) rssiValue = 1000;
      if (rssiValue>2000) rssiValue = 2000;
      setRSSI(rssiValue);
      sprintf(tx_message, "rssi %i\n"+0,(int) rssiValue);
      message = message+(char*)tx_message;
      #endif
      */
      sprintf(tx_message,"%s\n"+0, message.c_str());
      tx_send_rc_message(tx_message, strlen(tx_message));
      lastRetSentTimer = millis();
    }

    #ifdef PROTOCOL_TCP_RC
    if(!tx_client_tcp_rc.connected()) {
      tx_client_tcp_rc.stop();
      tx_client_tcp_rc.connect(rx_ip, tcp_port_rc);
    }
    else {
      tcpSocket = tx_client_tcp_rc;
      if (getTcpBuffer((char*)myBuffer, bufIdx)) {
        tx_parseCommand(myBuffer);
        rcTcpLastAliveTimer = millis();
      }
    }
    #endif
  
    #ifdef PROTOCOL_TCP_UART
    if(!tx_client_tcp_uart.connected()) {
      tx_client_tcp_uart.stop();
      tx_client_tcp_uart.connect(rx_ip, tcp_port_uart);
    }
    else {
      tcpSocket = tx_client_tcp_uart;
      if (getTcpBuffer((char*)myBuffer, bufIdx)) {
        Serial.write(myBuffer, bufIdx);
        uartTcpLastAliveTimer = millis();
      }
    }
    #endif

    #ifdef PROTOCOL_UDP_RC
    udpSocket = tx_udp_rc;
    if (getUdpBuffer((char*)myBuffer, bufIdx, tx_remoteIp_udp_rc, tx_remotePort_udp_rc)) {
      tx_parseCommand(myBuffer);
      rcUdpLastAliveTimer = millis();
    }
    #endif
  
    #ifdef PROTOCOL_UDP_UART
    udpSocket = tx_udp_uart;
    if (getUdpBuffer((char*)myBuffer, bufIdx, tx_remoteIp_udp_uart, tx_remotePort_udp_uart)) {
      Serial.write(myBuffer, bufIdx);
      uartUdpLastAliveTimer = millis();
    }
    #endif
  }
  
  #if defined (PROTOCOL_UDP_UART) || defined (PROTOCOL_TCP_UART)
  if (getSerialBuffer((char*)myBuffer, bufIdx)) {
    tx_send_uart_message((char*)myBuffer, bufIdx);
  }
  #endif
  yield();
  delay(10);
}
#endif

