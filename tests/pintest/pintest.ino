#define redPin D1
#define bluPin D2
#define grnPin D3
#define irPin D4

#define RFsend D6





void setup() {
  
///////////Set Variables//////

//Index=0;
//bitIndex=0;

  
//////////Setup Pins//////////
pinMode(redPin, OUTPUT);
pinMode(bluPin, OUTPUT);
pinMode(grnPin, OUTPUT);
pinMode(RFsend, OUTPUT);

pinMode(irPin, OUTPUT);
//pinMode(sendPin, OUTPUT);

//pinMode(recvPin, INPUT);
//pinMode(irRecv, INPUT_PULLUP);
//////////////////////////////
Serial.begin(115200);
//////////////////////////////

///////////Setup Timer for Sampling//////////////////////////////
noInterrupts();
//timer0_isr_init();
//timer0_attachInterrupt(timer0_ISR);
//timer0_write(ESP.getCycleCount() + 80000000L); // 80MHz == 1sec
//interrupts();
/////////////////////////////////////////////////////////////////

}

void loop() {
delay(50);
digitalWrite(redPin,HIGH);
delay(50);
digitalWrite(redPin,LOW);
delay(50);
digitalWrite(bluPin,HIGH);
delay(50);
digitalWrite(bluPin,LOW);
delay(50);
digitalWrite(grnPin,HIGH);
delay(50);
digitalWrite(grnPin,LOW);
delay(200);
digitalWrite(irPin,HIGH);
delay(200);
digitalWrite(irPin,LOW);
delay(200);
digitalWrite(RFsend,HIGH);
delay(200);
digitalWrite(RFsend,LOW);
}
