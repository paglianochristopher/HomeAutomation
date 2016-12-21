import paho.mqtt.client as mqtt

def on_connect(client, userdata, rc):
 print("Connected with result code "+str(rc))
	# Subscribing in on_connect() means that if we lose the connection and
	# reconnect then subscriptions will be renewed.
 client.subscribe("IRrecv")

 
 
 def on_connect(client, userdata, rc):
 print("Connected with result code "+str(rc))
	# Subscribing in on_connect() means that if we lose the connection and
	# reconnect then subscriptions will be renewed.
 client.subscribe("RFrecv")
 

 
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
	print topic + " " + message
 else:
  print("   Packet not recognised - Discarding")
 
 
 
 
 
 
 
 client.loop_forever()