/* 
 *  RC Receiver and Transmitter based on esp8266 by Radek Kubera (rkubera)
 *  https://github.com/rkubera/ESP8266-RC-Transmitter-Receiver
 *  Licence:  For non-commercial use only.
 *            You cannot remove author header from the source code.
 *            Modification and redistribution allowed with the same license agreement.
 *            For commerial use please contact with author.
 */
 
#ifdef MODULE_TX
#ifdef ENABLE_PPM

void tx_ppm_setup() {
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), PPM_read_channel, CHANGE);  
}

static unsigned long PPMDuration;

void PPM_read_channel(){
  static unsigned int pulse;

  static byte i;
  static unsigned long last_micros;

  PPMDuration = micros() - last_micros;
  last_micros = micros();

  if(PPMDuration < 510){
    pulse = PPMDuration;
  }
  else if(PPMDuration > 1910){
    i = 0;
  }
  else{
    if (i<CHANNELS_COUNT) {
      chVal[i] = PPMDuration + pulse;
    }
    i++;
  }
}

#endif
#endif
