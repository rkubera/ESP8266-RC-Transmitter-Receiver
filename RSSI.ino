/* 
 *  RC Receiver and Transmitter based on esp8266 by Radek Kubera (rkubera)
 *  https://github.com/rkubera/ESP8266-RC-Transmitter-Receiver
 *  Licence:  For non-commercial use only.
 *            You cannot remove author header from the source code.
 *            Modification and redistribution allowed with the same license agreement.
 *            For commerial use please contact with author.
 */
 
#ifdef ENABLE_RSSI
static int rx_rssiValue;

void rssi_setup () {
  rx_rssiValue = usMiddle;
  #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
    if (RSSI_RC_CHANNEL>0 && RSSI_RC_CHANNEL<=CHANNELS_COUNT && chPin[RSSI_RC_CHANNEL-1]>-1) {
      pinMode(chPin[RSSI_RC_CHANNEL-1], OUTPUT);
      servoCh[chPin[RSSI_RC_CHANNEL-1]].attach(chPin[RSSI_RC_CHANNEL-1], usMin, usMax);
      servoCh[chPin[RSSI_RC_CHANNEL-1]].writeMicroseconds(rx_rssiValue);
    }
  #endif
}

void setRSSI(int rssi) {
  rx_rssiValue = rssi;
  #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
  if (RSSI_RC_CHANNEL>0 && RSSI_RC_CHANNEL<=CHANNELS_COUNT && chPin[RSSI_RC_CHANNEL-1]>-1) {
    servoCh[chPin[RSSI_RC_CHANNEL-1]].writeMicroseconds(rssi);
  }
  #endif
}
#endif
