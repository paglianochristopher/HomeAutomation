
void RFin_ISR() { //trigger to detect when there is a falling edge on IR input pin
  //digitalWrite(redPin, HIGH);
  detachInterrupt(RFrecvPin);
  //IRin_detected = true;
  
  //Serial.println("detect");
  //rfTriggerCondition=true;
  RFisr=true;
}

void RF_sample() {
  if(RFisr==true){
  if (rfTriggerCondition == false) {
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
        if ((delta > 500000)) {
          //if(digitalRead(RFrecvPin) == true){
          //if//640000 cycles @ 160mhz = 4ms
          //Serial.println(delta);
          RFtrigger1 = false;
          //RFtrigger2 = false;
          rfTriggerCondition = true;
          RFclearArray();
          digitalWrite(redPin, HIGH);
          //digitalWrite(D4, LOW);

          //lastLowRFsampleTime=ESP.getCycleCount();
          //}
        }
      }
    }
  } else {
    //else {
    //    if (RFtrigger2 == false) {
    //      if (digitalRead(RFrecvPin) == true) {
    //        RFtrigger2 = true;
    //        return;
    //      }
    //    } else {

    ////////////////Top Half of function waits for the RF signal to stay low for 4ms, this indicates that the reciever is about to get a packet of data.////////////////
    ////////////////Bottom half of function waits until this is true and then starts sampling the data//////////////////////////////////////////////////////////////////

    if (bitRFindex == 8) {
      bitRFindex = 0;
      RFindex += 1;
      if (RFindex == RFrecvPinBuffSize) {
        RFindex = 0;
        //printData();
        //RF_cleanPacket();
        //printDataHuman(rfDataIn);
        rfTriggerCondition = false;
        rfDataIn[0] = 0; //first bit is always a 1, clear this.
        //printDataBits(rfDataIn, RFrecvPinBuffSize);

        RF_cleanPacket();

        //boolean rc = compare_arrays(test1,rfDataDecoded);
        //Serial.println("Result is:");
        //Serial.println(rc);
        //        if(rc){
        //          Serial.print("WE GOT ITTTTT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        //        }
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
//}

void RFclearArray() {
  for (int i = 0; i < RFrecvPinBuffSize - 1; i++) {
    rfDataIn[i] = 0xFF;
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

      if ((absIndex - prevAbsIndex) > 500 and (rfOutputIndex > 255)) {
        goto exitloop;
      }
    }

  }
exitloop:
  rfDataDecoded[0] = rfOutputIndex;
  int newval = rfDataDecoded[1] + 60; //magic const 60
  if (newval < 255) {
    rfDataDecoded[1] = newval;
  } else {
    rfDataDecoded[1] = 255;
  }
  

  if (rfDataDecoded[0] > 1 && CheckforValidity(rfDataDecoded)) {
    Serial.print("Raw RF Packet Recieved:");
    printDataByte(rfDataDecoded, rfOutputIndex);
    pubMQTT(rfDataDecoded, "ESP/RFrecv");
    //printDataHuman(rfDataDecoded);
    digitalWrite(redPin, LOW);
    //digitalWrite(D4, HIGH);
    //printDataBits();
    lastRFrecv = millis();
  }
    attachInterrupt(RFrecvPin, RFin_ISR, RISING);
    RFisr=false;
}


void RF_SendPacket() {

  if (RFsendSubIndex == rfDataOut[RFsendIndex + 1]) {
    RF_on = !RF_on;
    digitalWrite(RFsendPin, RF_on);
    RFsendSubIndex = 0;
    RFsendIndex = RFsendIndex + 1;
    //Serial.print("1");
  }
  RFsendSubIndex = RFsendSubIndex + 1;
  if (RFsendIndex == rfDataOut[0] - 1) {
    digitalWrite(RFsendPin, LOW);
    if (RFpacketCounter < RFpacketRepeats) {
      //digitalWrite(RFsendPin, LOW);
      RF_on = false;
      RFsendIndex = 0;
      RFsendSubIndex = 0;
      RFpacketCounter = RFpacketCounter + 1;
      //Serial.println("sent part data");
      //RF_on = true;
      //digitalWrite(RFsendPin, LOW);
      rfDelayTime = true;
      firstpacket = true;

    } else {
      RF_on = false;
      RFsendIndex = 0;
      RFsendSubIndex = 0;
      Serial.print("Sent Packet over RF (repeats," + String(RFpacketCounter));
      Serial.print("): ");
      printDataByte(rfDataOut, rfDataOut[0]);
      Serial.println("");
      RFpacketCounter = 0;
      digitalWrite(RFsendPin, LOW);
      RF_NeedtoSend = false;
      rfDelayTime = true;
      firstpacket = true;
      //rfDelayCounter = 0;
    }
  }
}
//}
//}
