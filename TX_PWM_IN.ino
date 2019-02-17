/* 
 *  RC Receiver and Transmitter based on esp8266 by Radek Kubera (rkubera)
 *  https://github.com/rkubera/ESP8266-RC-Transmitter-Receiver
 *  Licence:  For non-commercial use only.
 *            You cannot remove author header from the source code.
 *            Modification and redistribution allowed with the same license agreement.
 *            For commerial use please contact with author.
 */
 
#ifdef MODULE_TX
#ifdef ENABLE_PWM

void tx_pwm_setup() {
  #if PWM1_PIN >= 0
    #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
    if (RSSI_RC_CHANNEL!=1)
    #endif
    {
      pinMode(PWM1_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(PWM1_PIN), PWM1_PIN_Change, CHANGE);
    } 
  #endif
  #if PWM2_PIN >= 0
    #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
    if (RSSI_RC_CHANNEL!=2)
    #endif
    {
      pinMode(PWM2_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(PWM2_PIN), PWM2_PIN_Change, CHANGE);
    }
  #endif
  #if PWM3_PIN >= 0
    #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
    if (RSSI_RC_CHANNEL!=3)
    #endif
    {
      pinMode(PWM3_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(PWM3_PIN), PWM3_PIN_Change, CHANGE);
    }
  #endif
  #if PWM4_PIN >= 0
    #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
    if (RSSI_RC_CHANNEL!=4)
    #endif
    {
      pinMode(PWM4_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(PWM4_PIN), PWM4_PIN_Change, CHANGE);
    }
  #endif
  #if PWM5_PIN >= 0
    #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
    if (RSSI_RC_CHANNEL!=5)
    #endif
    {
      pinMode(PWM5_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(PWM5_PIN), PWM5_PIN_Change, CHANGE);
    }
  #endif
  #if PWM6_PIN >= 0
    #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
    if (RSSI_RC_CHANNEL!=6)
    #endif
    {
      pinMode(PWM6_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(PWM6_PIN), PWM6_PIN_Change, CHANGE);
    }
  #endif
  #if PWM7_PIN >= 0
    #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
    if (RSSI_RC_CHANNEL!=7)
    #endif
    {
      pinMode(PWM7_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(PWM7_PIN), PWM7_PIN_Change, CHANGE);
    } 
  #endif
  #if PWM8_PIN >= 0
    #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
    if (RSSI_RC_CHANNEL!=8)
    #endif
    {
      pinMode(PWM8_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(PWM8_PIN), PWM8_PIN_Change, CHANGE);
    }
  #endif
  #if PWM9_PIN >= 0
    #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
    if (RSSI_RC_CHANNEL!=9)
    #endif
    {
      pinMode(PWM9_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(PWM9_PIN), PWM9_PIN_Change, CHANGE);
    }
  #endif
  #if PWM10_PIN >= 0
    #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
    if (RSSI_RC_CHANNEL!=10)
    #endif
    {
      pinMode(PWM10_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(PWM10_PIN), PWM10_PIN_Change, CHANGE);
    }
  #endif
  #if PWM11_PIN >= 0
    #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
    if (RSSI_RC_CHANNEL!=11)
    #endif
    {
      pinMode(PWM11_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(PWM11_PIN), PWM11_PIN_Change, CHANGE);
    }
  #endif
  #if PWM12_PIN >= 0
    #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
    if (RSSI_RC_CHANNEL!=12)
    #endif
    {
      pinMode(PWM12_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(PWM12_PIN), PWM12_PIN_Change, CHANGE);
    }
  #endif
  #if PWM13_PIN >= 0
    #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
    if (RSSI_RC_CHANNEL!=13)
    #endif
    {
      pinMode(PWM13_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(PWM13_PIN), PWM13_PIN_Change, CHANGE);
    }
  #endif
  #if PWM14_PIN >= 0
    #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
    if (RSSI_RC_CHANNEL!=14)
    #endif
    {
      pinMode(PWM14_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(PWM14_PIN), PWM14_PIN_Change, CHANGE);
    }
  #endif
  #if PWM15_PIN >= 0
    #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
    if (RSSI_RC_CHANNEL!=15)
    #endif
    {
      pinMode(PWM15_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(PWM15_PIN), PWM15_PIN_Change, CHANGE);
    }
  #endif
  #if PWM16_PIN >= 0
    #if defined (ENABLE_RSSI) && defined (RSSI_RC_CHANNEL)
    if (RSSI_RC_CHANNEL!=16)
    #endif
    {
      pinMode(PWM16_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(PWM16_PIN), PWM16_PIN_Change, CHANGE);
    }
  #endif
}

#if PWM1_PIN >= 0
volatile unsigned long intCh1Rise;
void PWM1_PIN_Change(){
  //noInterrupts();
  if(digitalRead(PWM1_PIN) == HIGH){
    intCh1Rise = micros();
  }
  else{
    unsigned long PulseLength = micros() - intCh1Rise;
    if (PulseLength>=1000 && PulseLength<=2000) {
      chVal[0] = PulseLength;
    }
  }
  //interrupts();
}
#endif

#if PWM2_PIN >= 0
volatile unsigned long intCh2Rise;
void PWM2_PIN_Change(){
  //noInterrupts();
  if(digitalRead(PWM2_PIN) == HIGH){
    intCh2Rise = micros();
  }
  else{
    unsigned long PulseLength = micros() - intCh2Rise;
    if (PulseLength>=1000 && PulseLength<=2000) {
      chVal[1] = PulseLength;
    }
  }
  //interrupts();
}
#endif

#if PWM3_PIN >= 0
volatile unsigned long intCh3Rise;
void PWM3_PIN_Change(){
  //noInterrupts();
  if(digitalRead(PWM3_PIN) == HIGH){
    intCh3Rise = micros();
  }
  else{
    unsigned long PulseLength = micros() - intCh3Rise;
    if (PulseLength>=1000 && PulseLength<=2000) {
      chVal[2] = PulseLength;
    }
  }
  //interrupts();
}
#endif

#if PWM4_PIN >= 0
volatile unsigned long intCh4Rise;
void PWM4_PIN_Change(){
  //noInterrupts();
  if(digitalRead(PWM4_PIN) == HIGH){
    intCh4Rise = micros();
  }
  else{
    unsigned long PulseLength = micros() - intCh4Rise;
    if (PulseLength>=1000 && PulseLength<=2000) {
      chVal[3] = PulseLength;
    }
  }
  //interrupts();
}
#endif

#if PWM5_PIN >= 0
volatile unsigned long intCh5Rise;
void PWM5_PIN_Change(){
  //noInterrupts();
  if(digitalRead(PWM5_PIN) == HIGH){
    intCh5Rise = micros();
  }
  else{
    unsigned long PulseLength = micros() - intCh5Rise;
    if (PulseLength>=1000 && PulseLength<=2000) {
      chVal[4] = PulseLength;
    }
  }
  //interrupts();
}
#endif

#if PWM6_PIN >= 0
volatile unsigned long intCh6Rise;
void PWM6_PIN_Change(){
  //noInterrupts();
  if(digitalRead(PWM6_PIN) == HIGH){
    intCh6Rise = micros();
  }
  else{
    unsigned long PulseLength = micros() - intCh6Rise;
    if (PulseLength>=1000 && PulseLength<=2000) {
      chVal[5] = PulseLength;
    }
  }
  //interrupts();
}
#endif

#if PWM7_PIN >= 0
volatile unsigned long intCh7Rise;
void PWM7_PIN_Change(){
  //noInterrupts();
  if(digitalRead(PWM7_PIN) == HIGH){
    intCh7Rise = micros();
  }
  else{
    unsigned long PulseLength = micros() - intCh7Rise;
    if (PulseLength>=1000 && PulseLength<=2000) {
      chVal[6] = PulseLength;
    }
  }
  //interrupts();
}
#endif

#if PWM8_PIN >= 0
volatile unsigned long intCh8Rise;
void PWM8_PIN_Change(){
  //noInterrupts();
  if(digitalRead(PWM8_PIN) == HIGH){
    intCh8Rise = micros();
  }
  else{
    unsigned long PulseLength = micros() - intCh8Rise;
    if (PulseLength>=1000 && PulseLength<=2000) {
      chVal[7] = PulseLength;
    }
  }
  //interrupts();
}
#endif

#if PWM9_PIN >= 0
volatile unsigned long intCh9Rise;
void PWM9_PIN_Change(){
  //noInterrupts();
  if(digitalRead(PWM9_PIN) == HIGH){
    intCh9Rise = micros();
  }
  else{
    unsigned long PulseLength = micros() - intCh9Rise;
    if (PulseLength>=1000 && PulseLength<=2000) {
      chVal[8] = PulseLength;
    }
  }
  //interrupts();
}
#endif

#if PWM10_PIN >= 0
volatile unsigned long intCh10Rise;
void PWM10_PIN_Change(){
  //noInterrupts();
  if(digitalRead(PWM10_PIN) == HIGH){
    intCh10Rise = micros();
  }
  else{
    unsigned long PulseLength = micros() - intCh10Rise;
    if (PulseLength>=1000 && PulseLength<=2000) {
      chVal[9] = PulseLength;
    }
  }
  //interrupts();
}
#endif

#if PWM11_PIN >= 0
volatile unsigned long intCh11Rise;
void PWM11_PIN_Change(){
  //noInterrupts();
  if(digitalRead(PWM11_PIN) == HIGH){
    intCh11Rise = micros();
  }
  else{
    unsigned long PulseLength = micros() - intCh11Rise;
    if (PulseLength>=1000 && PulseLength<=2000) {
      chVal[10] = PulseLength;
    }
  }
  //interrupts();
}
#endif

#if PWM12_PIN >= 0
volatile unsigned long intCh12Rise;
void PWM12_PIN_Change(){
  //noInterrupts();
  if(digitalRead(PWM12_PIN) == HIGH){
    intCh12Rise = micros();
  }
  else{
    unsigned long PulseLength = micros() - intCh12Rise;
    if (PulseLength>=1000 && PulseLength<=2000) {
      chVal[11] = PulseLength;
    }
  }
  //interrupts();
}
#endif

#if PWM13_PIN >= 0
volatile unsigned long intCh13Rise;
void PWM13_PIN_Change(){
  //noInterrupts();
  if(digitalRead(PWM13_PIN) == HIGH){
    intCh13Rise = micros();
  }
  else{
    unsigned long PulseLength = micros() - intCh13Rise;
    if (PulseLength>=1000 && PulseLength<=2000) {
      chVal[12] = PulseLength;
    }
  }
  //interrupts();
}
#endif

#if PWM14_PIN >= 0
volatile unsigned long intCh14Rise;
void PWM14_PIN_Change(){
  //noInterrupts();
  if(digitalRead(PWM14_PIN) == HIGH){
    intCh14Rise = micros();
  }
  else{
    unsigned long PulseLength = micros() - intCh14Rise;
    if (PulseLength>=1000 && PulseLength<=2000) {
      chVal[13] = PulseLength;
    }
  }
  //interrupts();
}
#endif

#if PWM15_PIN >= 0
volatile unsigned long intCh15Rise;
void PWM15_PIN_Change(){
  //noInterrupts();
  if(digitalRead(PWM15_PIN) == HIGH){
    intCh15Rise = micros();
  }
  else{
    unsigned long PulseLength = micros() - intCh15Rise;
    if (PulseLength>=1000 && PulseLength<=2000) {
      chVal[14] = PulseLength;
    }
  }
  //interrupts();
}
#endif

#if PWM16_PIN >= 0
volatile unsigned long intCh16Rise;
void PWM16_PIN_Change(){
  //noInterrupts();
  if(digitalRead(PWM16_PIN) == HIGH){
    intCh16Rise = micros();
  }
  else{
    unsigned long PulseLength = micros() - intCh16Rise;
    if (PulseLength>=1000 && PulseLength<=2000) {
      chVal[15] = PulseLength;
    }
  }
  //interrupts();
}
#endif

#endif
#endif
