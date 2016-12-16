///////////////////////////////////////////////////
//Project 9 Fade RGB colors using sin wave
int redLED = D1;
int greenLED = D2;
int blueLED = D3;
int redLevel = 0;
int greenLevel = 0;
int blueLevel = 0;
float counter = 0;
float pi = 3.14159;
void setup(){
pinMode(redLED,OUTPUT);
pinMode(greenLED,OUTPUT);
pinMode(blueLED,OUTPUT);



delay(1000);
delay(500);
digitalWrite(redLED,HIGH);
delay(500);
digitalWrite(redLED,LOW);
delay(500);
digitalWrite(greenLED,HIGH);
delay(500);
digitalWrite(greenLED,LOW);
delay(500);
digitalWrite(blueLED,HIGH);
delay(500);
digitalWrite(blueLED,LOW);
delay(500);
digitalWrite(redLED,HIGH);
delay(500);
digitalWrite(redLED,LOW);
delay(500);
digitalWrite(greenLED,HIGH);
delay(500);
digitalWrite(greenLED,LOW);
delay(500);
digitalWrite(blueLED,HIGH);
delay(500);
digitalWrite(blueLED,LOW);
delay(1000);
//digitalWrit
}

void loop(){
counter = counter + 1;
redLevel = sin(counter/100)*1000;
greenLevel = sin(counter/100 + pi*2/3)*1000;
blueLevel = sin(counter/100 + pi*4/3)*1000;
redLevel = map(redLevel,-1000,1000,0,1024);
greenLevel = map(greenLevel,-1000,1000,0,1024);
blueLevel = map(blueLevel,-1000,1000,0,1024);
analogWrite(redLED,redLevel);
analogWrite(greenLED,greenLevel);
analogWrite(blueLED,blueLevel);
delay(3); 
}
///////////////////////////////////////////////////
