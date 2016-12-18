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

const char* mqtt_server = "192.168.1.103";

String mqttbuff;
char mqttmsg[1024];

long minDelta = 640000;
int minDeltaCycles = 152;

byte rfDataIn[RFrecvPinBuffSize];  //define sample arrays and allocate memory, this is RAW pin data, 0/1 stored per bit format
byte irDataIn[IRrecvPinBuffSize];  //define sample arrays and allocate memory, this is RAW pin data, 0/1 stored per bit format

byte rfDataDecoded[RFdataDecodedSize];  //Array to hold decoded RF data,  first byte defines length of data
byte irDataDecoded[IRdataDecodedSize];  //Array to hold decoded IR data,  first byte defines length of data

byte rfDataOut[RFsendBuffSize];  //define sample arrays and allocate memory, first byte defines length of data to send
byte irDataOut[IRsendBuffSize];  //define sample arrays and allocate memory, first byte defines length of data to send

//byte irDataOut[] ={48,24,8,23,8,8,24,23,8,24,8,8,24,7,24,8,24,23,8,8,24,8,24,7,151,23,8,24,8,8,23,24,8,23,9,7,24,8,24,8,23,24,8,8,24,7,24,8};
//byte rfDataOut[] = {25,13,12,7,13,6,13,7,12,13,6,14,6,13,6,13,6,14,5,14,6,13,6,7,12};
//byte test1[]={51,8,8,15,7,5,16,15,6,15,7,15,6,6,15,16,6,6,15,6,16,5,16,15,7,5,16,15,7,15,6,5,16,15,7,15,6,15,7,15,6,6,15,15,7,15,6,15,7,15,6};
//byte test2[]={68,85,86,9,33,9,32,10,33,9,12,9,12,9,11,9,12,9,12,9,31,10,33,9,33,9,12,9,12,9,12,9,12,9,12,9,12,10,32,10,12,9,12,9,12,9,12,9,12,11,10,9,33,10,11,10,33,9,33,9,33,9,33,9,33,10,33,10};

int IRsendIndex; //used in send function to track current index
int IRsendSubIndex; //used in send function to track current index

int RFsendIndex; //used in send function to track current index
int RFsendSubIndex; //used in send function to track current index

byte IRpacketRepeats = 1; //number of times to repeat transmitted IR packet.
byte IRpacketCounter = 0; //Counter for number of times to repeat IR packet

byte RFpacketRepeats = 5; //number of times to repeat transmitted IR packet.
byte RFpacketCounter = 0; //Counter for number of times to repeat IR packet
bool rfDelayTime=true;
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


///////////////////////////////////////////////////////////////
bool checkForData(byte* inputArray, byte arraysize, bool printArray) {
  int i = 0;
  byte lastValue = 0xFF;
  byte currValue = 0xFF;

  for (i = 0; i < arraysize - 1; i++) {
    currValue = lastValue & inputArray[i];
    lastValue = currValue;
  }

  //Serial.println(currValue);
  //if(currValue<255){
  if (printArray == true) {
    //printDataBits();
  }
  //byte nextStep[recvBuff];
  //nextStep==irDataIn;
  //clipData(nextStep);
  //}
  return true;
}
/////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
void clipData(byte* inputArray) {
  int inputSize = sizeof(inputArray);
  Serial.println(inputSize);
}
//////////////////////////////////////////////////////////////////////

/////////////////////Prints Input Array bit-wise/////////////////////
void printDataBits(byte* inputArray, int arrayLength) {
  Serial.print("Recieved Packet:");
  for (int i = 0; i < arrayLength; i++) {
    for (int j = 0; j <= 7; j++) {
      byte currByte = inputArray[i];
      Serial.print(bitRead(currByte, j));
      //Serial.print("0x");
      //Serial.print(inputArray[i],HEX);
      //Serial.print(",");

    }
  }
  Serial.println("");
}
///////////////////////////////////////////////////////////////////////

/////////////////////Prints an Array Byte wise/////////////////////
void printDataByte(byte* inputArray, int arraysize) {
  //Serial.print("Recieved IR Packet:");
  Serial.print("{");
  for (int i = 0; i < arraysize; i++) {
    Serial.print(inputArray[i]);
    if (i < arraysize - 1) {
      Serial.print(",");
    } else {
      Serial.println("}");
    }
  }
  //Serial.println("");
}
///////////////////////////////////////////////////////////////////////

/////////////////////Print in Human Readable Format////////////////////
void printDataHuman_old(byte* inputArray) {
  int arrayLength = inputArray[0];

  byte byte1 = 0;
  byte byte2 = 0;

  for (int i = 1; i < (arrayLength - 2); i++) {
    byte1 = inputArray[i];
    byte2 = inputArray[i + 1];
    if (byte1 == 0 && byte2 == 0) {
      Serial.print("_");
    } else {
      if (byte1 == 1 && byte2 == 1) {
        Serial.print("¯");
      } else {
        if ((byte1 == 0 && byte2 == 1) || (byte1 == 1 && byte2 == 0))
          Serial.print("|");
      }
    }

  }
}
////////////////////////////////////////////////////////////////////////

/////////////////////Print in Human Readable Format/////////////////////
void printDataHuman(byte* inputArray) {
  byte char1 = 45;
  int arrayLength = inputArray[0];

  boolean state = true;

  for (int i = 1; i < arrayLength - 1; i++) {
    byte counter = inputArray[i];
    for (int j = 0; j < counter; j++) {
      if (state == true) {
        Serial.print("`");
      } else {
        Serial.print(".");
      }
    }
    state = !state;
    Serial.print("|");

  }
  Serial.println("");
}
////////////////////////////////////////////////////////////////////////

bool compare_arrays(byte* array1, byte* array2) {
  //Serial.println("compare array");
  int length1 = array1[0];
  int length2 = array2[0];
  int count = 0;

  //byte currVal = 0xFF;
  //byte lastVal = 0xFF;

  //if (length1 == length2) {
  for (int i = 1; i < length1; i++) {
    if (array1[i] == array2[i]) {
    } else {
      if (array1[i] == array2[i] + 1) {
      } else {
        if (array1[i] == array2[i] + 2) {
          //stop
        } else {
          if (array1[i] == array2[i] - 1) {
          } else {
            if (array1[i] == array2[i] - 2) {
              //Serial.println("Checking byte" + String(i));
            } else {
              count = count + 1;
            }
          }
        }
      }
    }
  }
  //    return true;
  //    Serial.println("true");
  // }
  // return false;
  //Serial.println("false");
  Serial.print("Count is:");
  Serial.println(count);
  if (count > 1) {
    return false;
  } else {
    return true;
  }
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
    if (RF_NeedtoSend == true) { //if there is an IR packet waiting in the buffer
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
  samplePins = false;
  IR_NeedtoSend = true;
  while (IR_NeedtoSend == true) {
    digitalWrite(redPin, HIGH);
    //delay(10);
  }
  digitalWrite(redPin, LOW);
  samplePins = true;
}

void sendRFpuls() {
  //(10);
  samplePins = false;
  RF_NeedtoSend = true;
  while (RF_NeedtoSend == true) {
    digitalWrite(redPin, HIGH);
   // delay(10);
  }
  digitalWrite(redPin, LOW);
  samplePins = true;
}

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[512];
int value = 0;

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
  client.connect("ESP8266Client");
  client.subscribe("RFtoSend");
  client.subscribe("IRtoSend");
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(timer0_ISR);
  interrupts();
  //ESP.wdtDisable();
  attachInterrupt(D5, IRin_ISR, FALLING);
  Serial.println("Connected and ready....");
  timer0_write(ESP.getCycleCount() + 160000000L); // Adds a small delay  to the first timer0 call.
  /////////////////////////////////////////////////////////////////
  mqttbuff.reserve(1000);


}
//  byte irDataOut[]={48,24,8,23,8,8,24,23,8,24,8,8,24,7,24,8,24,23,8,8,24,8,24,7,151,23,8,24,8,8,23,24,8,23,9,7,24,8,24,8,23,24,8,8,24,7,24,8,1,1,1,1,1,1};
void loop() {
  //delay(1000);
  //sendIRpuls();
  //sendRFpuls();
//  if (!client.connected()) {
//    reconnect();
//  }

  client.loop();


//
  long now = millis();
  if (now - lastMsg > 2000) {
    //Serial.printf("loop heap size: %u\n", ESP.getFreeHeap());
    //client.loop();
 //Serial.println("pulsing");
 //sendIRpuls();
 
 lastMsg= now;
    }
//    //String thisString = String(rfDataOut[0],BIN);
//    mqttbuff.toCharArray(mqttmsg, mqttbuff.length()+1);
//    client.publish("outTopic",mqttmsg);
//    Serial.println(mqttbuff);
//    mqttbuff="";
  }
  


