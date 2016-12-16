


void RF_sample() {
  if (rfTriggerCondition == false){
    if (digitalRead(RFrecvPin) == true) {
      RFtrigger1 = false;
      RFtrigger2 = false;
      return;
    }

    if (digitalRead(RFrecvPin) == false) {
      if (RFtrigger1 == false) {
        lastLowRFsampleTime = ESP.getCycleCount();
        RFtrigger1 = true;
        return;
      } else {
        unsigned long delta = ESP.getCycleCount() - lastLowRFsampleTime;
        if ((delta > 640000)) {
          //if(digitalRead(RFrecvPin) == true){
          //if//640000 cycles @ 160mhz = 4ms
          //Serial.println(delta);
          RFtrigger1 = false;
          //RFtrigger2 = false;
          rfTriggerCondition = true;
          RFclearArray();
          digitalWrite(redPin, HIGH);
          
          //lastLowRFsampleTime=ESP.getCycleCount();
          //}
        }
      }
    }
  } else {
    if(RFtrigger2==false){
   if(digitalRead(RFrecvPin) == true){
    RFtrigger2=true;
     return;
    }
   }else{

    ////////////////Top Half of function waits for the RF signal to stay low for 4ms, this indicates that the reciever is about to get a packet of data.////////////////
    ////////////////Bottom half of function waits until this is true and then starts sampling the data//////////////////////////////////////////////////////////////////

    if (bitRFindex == 8) {
      bitRFindex = 0;
      RFindex += 1;
      if (RFindex == RFrecvPinBuffSize) {
        RFindex = 0;
        //printData();
        //checkForData(irDataIn);
        //RF_cleanPacket();
        //printDataHuman(rfDataIn);
        rfTriggerCondition = false;
        irDataIn[0] = 0; //first bit is always a 1, clear this.
        //printDataBits(rfDataIn, RFrecvPinBuffSize);
        RF_cleanPacket();
        ////This branch is called when the function fills the buffer, the data is then passed on for analysis.
      }
    }
    if (digitalRead(RFrecvPin)) {
      bitSet(rfDataIn[RFindex], bitRFindex);
    } else {
      bitClear(rfDataIn[RFindex], bitRFindex);
    }
    bitRFindex = bitRFindex + 1;

  }
}
}

void RFclearArray(){
  for(int i=0;i<RFrecvPinBuffSize-1;i++){
    rfDataIn[i]=0xFF;
  }
}

void RF_cleanPacket() { //convert raw data into pulse length format
  bool currBit = false;
  bool prevBit = false;
  int absIndex = 0;
  int prevAbsIndex = 0;
  int rfOutputIndex = 1;
  rfDataIn[0] = 0;
  rfDataDecoded[0] = 0;

  for (int i = 0; i < RFrecvPinBuffSize; i++) {
    for (int j = 0; j < 8; j++) {
      byte currByte = rfDataIn[i];
      currBit = bitRead(currByte, j);
      //Serial.println(j);

      if (currBit == !prevBit) {
        //Serial.println(absIndex,DEC);

        rfDataDecoded[rfOutputIndex] = absIndex - prevAbsIndex;

        rfOutputIndex = rfOutputIndex + 1;
        prevAbsIndex = absIndex;
      }
      prevBit = currBit;
      absIndex = absIndex + 1;
      //Serial.println(absIndex);

      if ((absIndex - prevAbsIndex) > 90 and (rfOutputIndex > 10)) {
        goto exitloop;
      }
    }

  }
exitloop:
  rfDataDecoded[0] = rfOutputIndex;
  
  if(rfDataDecoded[0]>1){
  Serial.print("Raw RF Packet Recieved:");
  printDataByte(rfDataDecoded, rfOutputIndex);
  //printDataHuman(rfDataDecoded);
  digitalWrite(redPin, LOW);
  //printDataBits();
  }
}
