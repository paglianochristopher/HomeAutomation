import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, rc):
 print("Connected with result code "+str(rc))
  # Subscribing in on_connect() means that if we lose the connection and
  # reconnect then subscriptions will be renewed.
 client.subscribe("ESP/RFrecv")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
 messagestring = str(msg.payload)
 print("Python Recieved Packet - " + msg.topic+" - " + messagestring)
 print("   Checking Packet")

 messagestring = messagestring[1:-1]
 messagearray = messagestring.split(",")
 messagearray = map(int, messagearray)
 [success, message, topic] = checkpacket(messagearray)
 if success == 1:
  print("   Packet Successful")
  print("   Published to MQTT ")
  publish.single("RFswitch/Inputs/"+topic, message, hostname="127.0.0.1")
  print topic + " " + message
 else:
  print("   Packet not recognised - Discarding")

def packetarray():
 arrayname = []
 arraypacket = []
 arraystate = []

 arrayname.append("RFSwitch1")
 arraypacket.append([51,154,5,15,5,15,5,16,15,5,14,6,15,5,5,16,4,16,15,5,5,16,15,5,15,6,4,16,15,5,5,16,5,15,5,15,15,6,5,15,5,15,15,6,5,15,5,15,5,16,5])
 arraystate.append("ON")

 arrayname.append("RFSwitch1")
 arraypacket.append([51,149,5,14,16,4,16,4,16,5,15,5,15,5,5,15,5,14,6,14,15,5,5,15,5,15,5,14,15,5,15,6,14,6,14,6,4,16,4,15,5,15,15,5,5,15,4,16,4,15,5])
 arraystate.append("OFF")

 arrayname.append("RFSwitch2")
 arraypacket.append([4,4,1,3])
 arraystate.append("ON")

 arrayname.append("RFSwitch2")
 arraypacket.append([5,6,4,1,1])
 arraystate.append("OFF")

 return [arrayname, arraypacket, arraystate]

def checkpacket(messagearray):
 [arrayname, arraypacket, arraystate] = packetarray()

 Numberofpackets = messagearray[0]
 i = -1
 for x in arraypacket:
  i = i + 1
  if (Numberofpackets == x[0]):
    if (messagearray == x):
      return [1, arraystate[i], arrayname[i]]
    else:
      z = -1
      sum = 0
      passed = True
      for y in x:
        z = z + 1
        if y == messagearray[z]:
            passed = True
        elif y == messagearray[z]+1 or y == messagearray[z]-1 or y == messagearray[z]+2 or y == messagearray[z]-2 or y == messagearray[z]+3 or y == messagearray[z]-3 or y == messagearray[z]+4 or y == messagearray[z]-4 or y == messagearray[z]+5 or y == messagearray[z]-5:
            sum = sum + 1
        else:
            passed = False
            break
      if passed == True and sum<= Numberofpackets:                    
          return [1, arraystate[i], arrayname[i]]
 return [0, "", ""]



client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("192.168.1.103", 1883, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_forever()
