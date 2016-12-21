//Pin defs

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>

#define redPin D1 //pin of red led
#define grnPin D2 //pin of blue led
#define bluPin D3 //pin of green led
#define IRsendPin D4 //pin of IR led
#define IRrecvPin D5 //pin of IR reciever

#define RFsendPin D6 //Pin for 433 transmit, active high.
#define RFrecvPin D7 //Pin for 433 recv, active low.

#define RFrecvPinBuffSize 128 //size of RF buffer to hold sampled data 
#define IRrecvPinBuffSize 400 //size of IR buffer to hold sampled data

#define RFdataDecodedSize 256 //size of RF buffer to hold data to send
#define IRdataDecodedSize 256 //size of IR buffer to hold data to send

#define RFsendBuffSize 256 //size of RF buffer to hold data to send
#define IRsendBuffSize 256 //size of IR buffer to hold data to send

//////////////////Telnet//////////////////////
#define MAX_SRV_CLIENTS 1 //max telnet clients
byte TelnetBuff[1024];    //telnet buff

/////////////////////MQTT///////////////////
const char* mqtt_server = "192.168.1.103";
const char* mqtt_clientid = "ESP433IR";
const char* mqtt_username = "openhab";
const char* mqtt_password = "h1A8iap@123";
long lastReconnectAttempt = 0;

String mqttbuff;
char mqttmsg[1024];
//////////////////////////////////////////

/////////Non Blocking Delay Func/////////


///////////////Led States////////////////
int redVal = 0;
int grnVal = 0;
int bluVal = 0;

bool FlashonSend = true; //pulses LED pin when packet is being sent
bool FlashonRecv = true; //pulses Led pin when packet is being recv

long minDelta = 500000;
int minDeltaCycles = 152;

byte rfDataIn[RFrecvPinBuffSize];  //define sample arrays and allocate memory, this is RAW pin data, 0/1 stored per bit format
byte irDataIn[IRrecvPinBuffSize];  //define sample arrays and allocate memory, this is RAW pin data, 0/1 stored per bit format

byte rfDataDecoded[RFdataDecodedSize];  //Array to hold decoded RF data,  first byte defines length of data
byte irDataDecoded[IRdataDecodedSize];  //Array to hold decoded IR data,  first byte defines length of data

byte rfDataOut[RFsendBuffSize];  //define sample arrays and allocate memory, first byte defines length of data to send
byte irDataOut[IRsendBuffSize];  //define sample arrays and allocate memory, first byte defines length of data to send

bool RFaddtoQue = false;

unsigned long lastRFrecv = 0;
#define AntiCollisionDelay 100 //won't transmit a packet within x number of milliseconds of recieving a packet, basic packet collision avoidance.

//byte irDataOut[] ={48,24,8,23,8,8,24,23,8,24,8,8,24,7,24,8,24,23,8,8,24,8,24,7,151,23,8,24,8,8,23,24,8,23,9,7,24,8,24,8,23,24,8,8,24,7,24,8};
//byte rfDataOut[] = {25,13,12,7,13,6,13,7,12,13,6,14,6,13,6,13,6,14,5,14,6,13,6,7,12};
//byte test1[]={51,8,8,15,7,5,16,15,6,15,7,15,6,6,15,16,6,6,15,6,16,5,16,15,7,5,16,15,7,15,6,5,16,15,7,15,6,15,7,15,6,6,15,15,7,15,6,15,7,15,6};
//byte test2[]={68,85,86,9,33,9,32,10,33,9,12,9,12,9,11,9,12,9,12,9,31,10,33,9,33,9,12,9,12,9,12,9,12,9,12,9,12,10,32,10,12,9,12,9,12,9,12,9,12,11,10,9,33,10,11,10,33,9,33,9,33,9,33,9,33,10,33,10};

int IRsendIndex; //used in send function to track current index
int IRsendSubIndex; //used in send function to track current index

int RFsendIndex; //used in send function to track current index
int RFsendSubIndex; //used in send function to track current index

byte IRpacketRepeats = 0; //number of times to repeat transmitted IR packet.
byte IRpacketCounter = 0; //Counter for number of times to repeat IR packet

byte RFpacketRepeats = 10; //number of times to repeat transmitted IR packet.
byte RFpacketCounter = 0; //Counter for number of times to repeat IR packet
bool rfDelayTime = true;
int rfDelayCounter = 0;
int rfDelayCycles = 50; //Number of cycles of delay
bool firstpacket = true;

int IRindex; //current byte of irDataIn array, used in data sampling function
byte bitIRindex; //current bit of irDataIn array, used in data sampling function

int RFindex; //current byte of rfDataIn array, used in data sampling function
byte bitRFindex; //current bit of rfDataIn array, used in data sampling function

bool samplePins = true; //if true this function will sample the pins in the timer0 loop, during IR sending or important functions we turn this off.

bool IRsendPin_state = false; //holds IR pin state for bitbanging

bool IR_on = false; //true if there needs to be an RF carrier active

bool RF_on = false;

bool IR_NeedtoSend = false; //true where there needs to be a IR packet sent, this will run the subroutine to bitbang the packet out.
bool RF_NeedtoSend = false; //true where there needs to be a IR packet sent, this will run the subroutine to bitbang the packet out.

bool IRin_detected = false; //has a falling pulse been detected on IR reciever pin?, this optimisation stops unnessesary sampling of the pins where there no data.

unsigned long timer0_count = 0; //gobal 32bit timer

////Variables to detect the start of RF packets//////
boolean rfTriggerCondition = false;
unsigned long lastLowRFsampleTime = 0;
boolean RFtrigger1 = false;
boolean RFtrigger2 = false;

/////////////////////////////////////////////////////////////////////
void clipData(byte* inputArray) {
  int inputSize = sizeof(inputArray);
  Serial.println(inputSize);
}
//////////////////////////////////////////////////////////////////////
void LedAllOff() {
  digitalWrite(redPin, LOW);
  digitalWrite(bluPin, LOW);
  digitalWrite(grnPin, LOW);
}

void LedSet() {
  analogWrite(redPin, redVal);
  analogWrite(grnPin, grnVal);
  analogWrite(bluPin, bluVal);
}


///////////////////////////////////////////////////////////////////////
void timer0_ISR (void) {

  //main system timer that bitbangs the RF and IR pins.
  //This timer ticks at 38khz, these fast ticks generate the intertupts to pulse the IR led at 38khz.
  //The ticks are further devided by 4, this runs the routines to sample the RF and IR data pins, sample rate is around 21.5khz.

  timer0_count = timer0_count + 1; //incr. count

  if (timer0_count == 4) { //run this every 4 ticks of the timer function
    timer0_count = 0; //reset timer
    if (samplePins == true) { //if its time to sample the data pins run the subroutines
      IR_sample();
      RF_sample();
    }
    if (IR_NeedtoSend == true) { //if there is an IR packet waiting in the buffer
      IR_SendFuncPWM();
    }
    if (RF_NeedtoSend == true) { //if there is an RF packet waiting in the buffer
      RF_SendPacket();
    }
  }

  IRsend_ISR(); // this is called to generate the RF carrier pulses

  timer0_write(ESP.getCycleCount() + 1860); // = ~76khz @ 160mhz system clock, needs to be twice carrier freq of 38khz.
  //ESP.wdtFeed(); //reset watchdog                                          // Sets the next tick of the timer callback function.
}
////////////////////////////////////////////////////////////////////////

void sendIRpuls() {
  //(10);
  LedAllOff();
  samplePins = false;
  IR_NeedtoSend = true;
  while (IR_NeedtoSend == true) {
    //digitalWrite(D4, LOW);
    digitalWrite(redPin, HIGH);
    //delay(10);
  }
  digitalWrite(redPin, LOW);
  //digitalWrite(D4, HIGH);
  LedSet();
  samplePins = true;
}

void sendRFpuls() {
  //(10);
  LedAllOff();
  samplePins = false;
  RF_NeedtoSend = true;
  while (RF_NeedtoSend == true) {
    //digitalWrite(D4, LOW);
    digitalWrite(redPin, HIGH);
    // delay(10);
  }
  digitalWrite(redPin, LOW);
 // digitalWrite(D4, HIGH);
  LedSet();
  samplePins = true;
}

WiFiClient espClient;
PubSubClient client(espClient);

WiFiServer server(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];

boolean reconnect() {
  if (client.connect(mqtt_clientid, mqtt_username, mqtt_password)) {
    client.publish("ESP/status", "ESP8266 Network Bridge reconnected....");
    client.subscribe("ESP/RFtoSend");
    client.subscribe("ESP/IRtoSend");
    client.subscribe("ESP/RLED");
    client.subscribe("ESP/GLED");
    client.subscribe("ESP/BLED");

  }
  return client.connected();
}

void setup() {
  ///////////Set Variables////////////////////////////////////////////////

  IRindex = 0;
  bitIRindex = 0;

  IRsendIndex = 0;
  IRsendSubIndex = 0;

  IRpacketCounter = 0;

  //////////Setup Pins//////////////////////////////////////////////////
  pinMode(redPin, OUTPUT);
  pinMode(bluPin, OUTPUT);
  pinMode(grnPin, OUTPUT);
  pinMode(IRsendPin, OUTPUT);
  pinMode(RFsendPin, OUTPUT);

  digitalWrite(redPin, LOW);
  digitalWrite(bluPin, LOW);
  digitalWrite(grnPin, LOW);

  pinMode(RFrecvPin, INPUT);
  pinMode(IRrecvPin, INPUT);
  ////////////////////////////////////////////////////////////////////
  Serial.begin(115200);
  ////////////////////////////////////////////////////////////////////

  ///////////Setup Timer0 for Sampling////////////////////////////////
  WiFiManager wifiManager;
  //wifiManager.resetSettings();
  wifiManager.autoConnect("ESP Network Bridge");
  client.setServer(mqtt_server, 1883);
  client.setCallback(MQTTcallback);
  client.connect(mqtt_clientid, mqtt_username, mqtt_password);
  client.subscribe("ESP/RFtoSend");
  client.subscribe("ESP/IRtoSend");
  client.subscribe("ESP/RLED");
  client.subscribe("ESP/GLED");
  client.subscribe("ESP/BLED");
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(timer0_ISR);
  interrupts();
  //ESP.wdtDisable();
  attachInterrupt(D5, IRin_ISR, FALLING);
  Serial.println("Connected to Wifi and ready....");
  timer0_write(ESP.getCycleCount() + 160000000L); // Adds a small delay  to the first timer0 call.
  /////////////////////////////////////////////////////////////////
  mqttbuff.reserve(1000);


}

void HandleRFtransmit() {
  if (RFaddtoQue == true) {
    if ((millis() - lastRFrecv) < AntiCollisionDelay) {
      return;
    } else {
      sendRFpuls();
      RFaddtoQue = false;
    }

  }

}

void loop() {
  //delay(1000);
  //sendIRpuls();
  //sendRFpuls();
  //HandleTelnet();
  if (RFaddtoQue == false) {
    HandleMQTT();
  }
  HandleRFtransmit();
}



