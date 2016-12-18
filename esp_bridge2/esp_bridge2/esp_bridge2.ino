unsigned long timer0_count = 0; //gobal 32bit timer
///////////////////////////////////////////////////////////////////////
void timer0_ISR (void) {

  timer0_count = timer0_count + 1; //incr. count

  if (timer0_count == 100) {
    timer0_count = 0;
  }
  timer0_write(ESP.getCycleCount() + 2000);
}
////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  ///////////Setup Timer0 for Sampling////////////////////////////////
  timer0_isr_init();
  timer0_attachInterrupt(timer0_ISR);
  timer0_write(ESP.getCycleCount() + 160000000); // Adds a small delay  to the first timer0 call.
  /////////////////////////////////////////////////////////////////
}

void loop() {
  delay(1000);
  Serial.println("hello");
}
