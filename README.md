
# M5STACK_UnitLoRaWAN_Lib
Python library for the M5STACK LoRaWAN module based on the ASR6501 module shown below.


![image](https://user-images.githubusercontent.com/15849181/181081701-dac20552-7a27-42bb-9530-ce337279bdee.png)

I bought one of these to play with then went looking for a library, which would give me full control over the device, but was unable to find one. There are examples to connect to TTN alright but they were all very basic and I didn't see any downlink handling (I could be wrong). Most were written in C but, since the device handles the LoRaWAN stack for you via a UART, I chose to use Python. I may consider converting the code to C in the distant future.

The device does all the LoRaWAN management for you. As far as I know, it doesn't do LoRa so can't be used that way unless the firmware is changed. You can reboot the device into the boot loader. 

There are many other devices like the Heltecs and Node MCUs that can handle LoRa using libraries like RadioHead.


# Examples

Well, just one for now a simple TTN_test.py which joins to TTN using OTAA and sends periodic uplinks conforming to the EU duty cycle but not the TTN fair use so as not to slow down testing.

You can add fair use control by accumulating the air time for all your messages and then stop when 30s is reached/exceeded.

The example also shows how to access downlinks sent to the module.

# Module Logging

The TTN_Test.py example shows how to set various parameters prior to a join.

NOTE: The library turns off device logging which the device outputs to the serial port. It can be quite verbose. I turn it off to simplify parsing the messages I want to see from the mdoule.

# Joining

If your device has, in the past, stored the MAC parameters after a successful join you should call restoreMacConfiguration() before calling join().

The call to join() to TTN has a number of default parameters:-

```
def join(self,start:int=1,autojoin:int=0,interval:int=8,retries:int=8)->bool:
```

If autojoin is set to 1 the device will try to rejoin TTN after a power cycle. If start=0 the device will abort joining. I assume this only works if the retries haven't expired.

Prior to calling join() you should set up your data rate, ttn keys and the join mode etc. See TTN_test.py - generalSetup() and configureOTAA()

After a successful join your code should call saveMACConfiguration() so that it will rejoin after a reboot/power cycle.

# API Doc

The HTML document was generated using pydoc. 

# Downlinks

The LoRaWAN device supports all three classes of devices - although I have only used class A for testing. 

Class A downlinks occur, if any, in the RX1 window following an uplink.  
Class B opens ping slots at set times  
Class C can receive downlinks at any time 

Which ever mode you are using all the downlinks are returned on the serial port using the format OK+RECV:mtype,port,len,msg. These can be captured by calling checkForDownlink() and will be passed to your downlink handler which you must setup, preferably, before joining.

The TTN_test.py example shows how to do that. The example just logs and prints the downlink information like so..

```
def downlinkCallback(mType,port,msgLen,msg):
    logger.info(f"downlink received type: {mType} port:{port} msglen:{msgLen} msg:{msg}")
    print(f"DownlinkCallback {mType},port {port}, len {msgLen},msg {msg}")

LoRaWAN.setDownlinkCallback(downlinkCallback)
```

# Managing Duty cycle

The API provides a method calcAirTime for computing a message airtime using the formula here. 
```
https://avbentem.github.io/airtime-calculator/ttn/au915
```
It has been tested amd matches the online values I found.

# Error handling

The M5STACK module generates a number of error messages like CME ERROR, ERR+SEND, ERR+SENT. The API handles these by raising python exceptions.

CME ERROR:1 appears to mean that a syntx error was encountered with the AT command sent. I have never seen any other number with this.

In addition, there are code parameter assertions and other exceptions raised by the code to prevent sending silly numbers to the device.

After an exception you can access the last error with a call to getLastATError() which returns a dict like this:
```
{ "AtCmd":"CJOIN","Type":"CME ERROR","Num":1}
```




