/* 
 *  RC Receiver and Transmitter based on esp8266 by Radek Kubera (rkubera)
 *  https://github.com/rkubera/ESP8266-RC-Transmitter-Receiver
 *  Licence:  For non-commercial use only.
 *            You cannot remove author header from the source code.
 *            Modification and redistribution allowed with the same license agreement.
 *            For commerial use please contact with author.
 */
 
#ifdef MODULE_RX

#if defined ENABLE_PPM
volatile unsigned long rx_next_timer_int;

#define RX_PPM_PULSE_LENGTH 300  //set the pulse length
#if CHANNELS_COUNT>12
#define RX_PPM_FRAME_LENGTH 38500  //set the PPM frame length in microseconds (1ms = 1000µs)
#else
#define RX_PPM_FRAME_LENGTH 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#endif

void inline rx_ppmISR(void){
  static boolean rx_ppm_state = true;
  if (rx_ppm_state) {
    digitalWrite(PPM_PIN, !PPM_POLARITY);
    rx_next_timer_int = rx_next_timer_int + (RX_PPM_PULSE_LENGTH * (F_CPU/1000000));
    rx_ppm_state = false;
  } 
  else{
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(PPM_PIN, PPM_POLARITY);
    rx_ppm_state = true;

    if(cur_chan_numb >= CHANNELS_COUNT){
      cur_chan_numb = 0;
      calc_rest = calc_rest + RX_PPM_PULSE_LENGTH;// 
      rx_next_timer_int = rx_next_timer_int + ((RX_PPM_FRAME_LENGTH - calc_rest) * (F_CPU/1000000));
      calc_rest = 0;
    }
    else{
      #ifdef ENABLE_FAIL_SAFE
      if (rcClientConnected==0 && (cur_chan_numb+1)==throttle_rc_channel) {
        rx_next_timer_int = rx_next_timer_int + ((FAIL_SAFE_PWM_VALUE - RX_PPM_PULSE_LENGTH) * (F_CPU/1000000));
        calc_rest = calc_rest + FAIL_SAFE_PWM_VALUE;
      }
      else 
      #endif
      {
        int val = chVal[cur_chan_numb];
        #ifdef ENABLE_RSSI
        if((cur_chan_numb+1) == RSSI_RC_CHANNEL) {
          if (rcClientConnected==0) {
            val = RSSI_PWM_MIN_VALUE;
          }
          else {
            val = rx_rssiValue;
          }
        }
        #endif
        rx_next_timer_int = rx_next_timer_int + ((val - RX_PPM_PULSE_LENGTH) * (F_CPU/1000000));
        calc_rest = calc_rest + chVal[cur_chan_numb];
      }
      cur_chan_numb++;
    }     
  }
  timer0_write(rx_next_timer_int);
}

void rx_ppmInit() {
  timer0_isr_init();
  timer0_attachInterrupt(rx_ppmISR);
  rx_next_timer_int=ESP.getCycleCount()+1000;
  timer0_write(rx_next_timer_int);
}
#endif
#endif
