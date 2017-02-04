void MQTTcallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT arrived [");
  Serial.print(topic);
  Serial.print("] ");
  //noInterrupts();

  String input;
  input.reserve(5);
  byte val = 0;
  int Index = 0;

  if (topic[5] == 'L') { //LED state change req, process subroutine
    MQTTleds(topic, payload, length);
  } else {

    if ((char)payload[0] == 123) { // { bracket

      for (int i = 1; i < length; i++) { //decode comma delimited byte array... hopefully...
        if ( (char)payload[i] == 44 || (char)payload[i] == 125) { //comma char.
          //Serial.print("comma");
          //Serial.println(input);
          if (topic[5] == 'F') {
            //Serial.print("I made it");
            rfDataOut[Index] = input.toInt();
            //Serial.println(rfDataOut[Index]);
          }

          if (topic[5] == 'R') {

            irDataOut[Index] = input.toInt();

          }
          //Serial.println(val);
          //Serial.println(input);
          input = "";
          Index = Index + 1;
          //        //Serial.println(Index);
        } else {
          input.concat(String((char)payload[i]));

        }
        if ((char)payload[i] == 125) {
          //Serial.println("Complete Packet Recev IR");
          if (topic[5] == 'R') {
            sendIRpuls();
            //Serial.println("Sending IR");
            return;
            //Serial.println("Sending IR");

          }
          if (topic[5] == 'F') {
            if (RFaddtoQue == false) {
              RFaddtoQue = true;
            }
              //sendRFpuls();
              //delay(5);
              //sendRFpuls();
              //delay(5);
              //sendRFpuls();
              //Serial.println("Sending RF");
              return;
            }
          }
        }

      } else {
        Serial.println("INVALID MQTT MSG RECV");
      }
     // interrupts();
    }
  }






void pubMQTT(byte* inputArray, char* topic) {
  mqttbuff.concat("{");
  for (int i = 0; i < inputArray[0]; i++) {
    mqttbuff.concat(String((inputArray[i])));
    if (i < inputArray[0] - 1) {
      mqttbuff.concat(",");
    }
  }
  mqttbuff.concat("}");
  mqttbuff.toCharArray(mqttmsg, mqttbuff.length() + 1);
  client.publish(topic, mqttmsg);
  Serial.print("pub MQTT, topic:" + String(topic) + " msg: ");
  Serial.println(mqttbuff);
  mqttbuff = "";
}

void HandleMQTT() {
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 2500) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected

    client.loop();
  }
}


