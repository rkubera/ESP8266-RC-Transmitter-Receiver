/* 
 *  RC Receiver and Transmitter based on esp8266 by Radek Kubera (rkubera)
 *  https://github.com/rkubera/ESP8266-RC-Transmitter-Receiver
 *  Licence:  For non-commercial use only.
 *            You cannot remove author header from the source code.
 *            Modification and redistribution allowed with the same license agreement.
 *            For commerial use please contact with author.
 */

bool getSerialBuffer (char* buf, int &idx) {
  idx = 0;
  bool ret = false;
  while(true) {
    if(Serial.available()) {
      ret = true;
      buf[idx] = (char)Serial.read();
      buf[idx+1] = 0;
      idx++;
      if((idx+1)>=bufferSize) break;
    } else break;
  }
  return ret;
}

#if defined (PROTOCOL_UDP_RC) || defined (PROTOCOL_UDP_UART)
#include <WiFiUdp.h>
bool getUdpBuffer (char* buf, int &idx, IPAddress &ipAddr, int &port) {
  idx = 0;
  bool ret = false;
  if ((idx = udpSocket.parsePacket())>0) {
    ret = true;
    ipAddr = udpSocket.remoteIP();
    port = udpSocket.remotePort();
    udpSocket.read(buf, bufferSize-1);
    if (idx>(bufferSize-1)) {
      idx = bufferSize-1;
    }
    myBuffer[idx]=0;
  }
  return ret;
}
#endif

#if defined (PROTOCOL_TCP_RC) || defined (PROTOCOL_TCP_UART)
#include <WiFiClient.h>
bool getTcpBuffer (char* buf, int &idx) {
  idx = 0;
  bool ret = false;
  while(true) {
    if(tcpSocket.available()) {
      ret = true;
      buf[idx] = (uint8_t)tcpSocket.read();
      buf[idx+1] = 0;
      idx++;
      if((idx+1)>=bufferSize) break;
    } else break;
  }
  return ret;
}
#endif

