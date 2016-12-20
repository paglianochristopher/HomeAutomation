#!/bin/sh
#### NETWORK SPEED ####
iperf3 -sD

#### PRESENCE DETECTION ####

###############################################################
# Proximity detection
#
# A script designed to run on a router running DD-WRT to detect certain devices connected to the router.
# It runs at startup and runs continually, checking for a specific list of devices (phones/laptop, etc)
# that connect wirelessly to the router.  Once a device is connected, the OpenHAB status will
# be updated with either an ON or OFF.  Make sure you set up a switch item in OpenHAB for each device
# you want to track.
#
# The searching frequency can be adjusted to be slower/faster depending on your requirements. Searching too fast
# could burden your router.  Too slow might not update the status as necessary for the application.
#


# Make changes below
# MAC address of each device to watch. Don't leave blank.
# For security purposes, if your router requires a password, even if someone could clone the MAC of your
# phone, they would still require the password of your network to link to your router.
# arp command is case sensitive! letters need to be lower case 
# run "arp -i br0" from CLI to check MAC Addresses
macdevice1="60:f1:89:5d:5b:10"    #Joe Nexus 6
macdevice2="00:00:00:47:f8:0a"    #Wife Nexus 5
macdevice3="00:00:00:00:c0:0b"    #Samsung TV Hard Wired
macdevice4="f0:00:47:00:00:0c"    #Kid iPad Wi-Fi Radio

#OpenHAB username, password, and IP Address
username="pi"
password="raspberry"
IPAddr="192.168.1.103"
port="8080"

# OpenHAB switch items to be updated for each tracked MAC
item1="JoePhone"
item2="WifePhone"
item3="SamsungTV"
item4="KidIPAD"

#String Item in openHAB to display total devices connected 
item99="DDWRTDeviceCount" 

# Occupied and unoccupied delay in seconds to check status
# Adjust for shorter/longer wait times.  For instance, when one device is already
# connected, you might want to check less frequently.  This could also delay the
# notification of a disconnect.
delay_occupied=4
delay_unoccupied=2

# initial testing loop count - uncomment the counter near the bottom of the script for testing only. 
limit=120

###############################################
# do not change below here
###############################################

sleep
#initialize internal variables

# status of each MAC. 0=disconnected. 1=connected.  -1 initially forces openHAB update first loop
macconnected1=-1
macconnected2=-1
macconnected3=-1
macconnected4=-1
connected=-1
# total number of currently conencted devices.
currentconnected=0
counter=1

# Initial testing loop.  Will run continually after testing is complete
while [ $counter -lt $limit ]; do

	#reset current status. Two variables are used for each device.  The past known status and the current
	# status.  Only a change is reported to openHAB.  Otherwise, it would constantly be updating openHAB with
	# the current status creating unnecessary traffic for both the router and openHAB
	maccurrent1=0;
	maccurrent2=0;
	maccurrent3=0;
	maccurrent4=0;


	## Old Case Section Replaced
	# compare each device that is currently connected to the MAC devices we want to watch.
	# changed the following to chek for each MAC
	arpout=$(arp -i br0)

	maccurrent1=$(echo $arpout | grep -c $macdevice1)
	if [ $maccurrent1 -gt 0 ]; then
	   maccurrent1=1
	fi

	maccurrent2=$(echo $arpout | grep -c $macdevice2)
	if [ $maccurrent2 -gt 0 ]; then
	   maccurrent2=1
	fi

	maccurrent3=$(echo $arpout | grep -c $macdevice3)
	if [ $maccurrent3 -gt 0 ]; then
	   maccurrent3=1
	fi

	maccurrent4=$(echo $arpout | grep -c $macdevice4)
	if [ $maccurrent4 -gt 0 ]; then
	   maccurrent4=1
	fi


	# Look for a change in status from the old known to the current status.
	# If it changed, update openHAB. Otherwise it leaves it as is. 
	if [ $macconnected1 -ne $maccurrent1 ]; then
		 if [ $maccurrent1 -eq 1 ]; then
			 macstatus1="ON";
		 else
			 macstatus1="OFF";
		 fi
		 curl -X POST -d $macstatus1 -H "Content-Type: text/plain" -i http://$username:$password@$IPAddr:$port/rest/items/$item1
	fi

	if [ $macconnected2 -ne $maccurrent2 ]; then
		 if [ $maccurrent2 -eq 1 ]; then
			 macstatus2="ON";
		 else
			 macstatus2="OFF";
		 fi
		 curl -X POST -d $macstatus2 -H "Content-Type: text/plain" -i http://$username:$password@$IPAddr:$port/rest/items/$item2
	fi

	if [ $macconnected3 -ne $maccurrent3 ]; then
		 if [ $maccurrent3 -eq 1 ]; then
			 macstatus3="ON";
		 else
			 macstatus3="OFF";
		 fi
		 curl -X POST -d $macstatus3 -H "Content-Type: text/plain" -i http://$username:$password@$IPAddr:$port/rest/items/$item3
	fi

	if [ $macconnected4 -ne $maccurrent4 ]; then
		 if [ $maccurrent4 -eq 1 ]; then
			 macstatus4="ON";
		 else
			 macstatus4="OFF";
		 fi
		 curl -X POST -d $macstatus4 -H "Content-Type: text/plain" -i http://$username:$password@$IPAddr:$port/rest/items/$item4
	fi

	# Update the known status from the current.  Ready for the next loop. 
	macconnected1=$maccurrent1;
	macconnected2=$maccurrent2;
	macconnected3=$maccurrent3;
	macconnected4=$maccurrent4;

	# Total up the number of devices connected.
	let currentconnected=$macconnected1+$macconnected2+$macconnected3+$macconnected4

	# Look for a change, and update openHAB.
	if [ $connected -ne $currentconnected ]; then
	   curl -X POST -d $currentconnected -H "Content-Type: text/plain" -i http://$username:$password@$IPAddr:$port/rest/items/$item99
	fi
	connected=$currentconnected

	# Delay (sleep) depending on the connection status.
	# No devices connected could delay less.  Once a device is connected, it could delay longer.
	if [ $connected -gt 0 ]; then
		sleep $delay_occupied
	else
		sleep $delay_occupied
	fi
done