"""

TTN_test.py

Test program to check ASR6501 library functionality with TTN

    Operates as a class A device
    Joins TTN
    Sends periodic uplinks conforming to duty cycle
    Reads any downlinks

Downlinks must be scheduled via the TTN console and are only
transmitted after an uplink so you will have to wait for the
duty cycle to expire first.

Note, you will need to modify the dataRates list if not in EU868

A TOML config file is used

"""
from ASR6501 import asr6501,STATUS
import logging
import time
import toml
import serial
import sys


# these are for EU868 only - you should modify this for different regions
# only used to calculate the message airtime
# ASR6501.toml specified  join data rate ["network"]["freqplan"]["joinDataRate"]

dataRates=[(12,125),(11,125),(10,125),(9,125),(8,125),(7,127),(7,250)]

LOG_LEVEL=logging.DEBUG
LOG_FILE= "TTN_test.log"
CONFIG_FILE= "../ASR6501.toml"
OTAA=0
ABP=1


CLASS=0                 # class A
THIS_JOIN_MODE=OTAA
DUTY_CYCLE=0.01         # EU 1%
UNCONFIRMED_UPLINK=0    # this is preferred otherwise duty_cycle and fair use may be exceeded
CONFIRMED_UPLINK=1
LOOP_DELAY=10           # remember RX1delay is normally 5s so this should be bigger to catch any downlink
                        # but be shorter than the DUTY_CYCLE period.

#######################################
#
# setup logging
#
#######################################

logging.basicConfig(filename=LOG_FILE, format='%(asctime)s - %(name)s - %(lineno)d - %(levelname)s - %(message)s')
logger = logging.getLogger() # main logger
logger.setLevel(LOG_LEVEL)
msg=f"\n#####\nASR6501 TTN Test Starting\n#####"
logger.info(msg)
print(msg)

#######################################
#
# try to load config info
#
#######################################
try:
    config = toml.load(CONFIG_FILE)
except Exception as e:
    logger.exception(e)
    raise e

########################################
#
# try to create serial port
#
########################################
try:
    # create a serial port

    serialPort=serial.Serial(port=config["serial"]["port"],
        baudrate=config["serial"]["baud"],
        bytesize=config["serial"]["bytesize"],
        timeout=config["serial"]["timeout"],
        stopbits=config["serial"]["stopbits"])

except Exception as e:
    logger.exception(e)
    sys.exit(e)

# get values for this frequency plan
bandwidths=config["network"]["freqplan"]["bandwidths"]
dataRates=config["network"]["freqplan"]["data_rates"]    #[(12,125),(11,125),(10,125),(9,125),(8,125),(7,127),(7,250)]

########################################
#
# create an instance of the library
#
########################################

LoRaWAN=asr6501(serialPort,LOG_LEVEL)


# capturing downlink messages
def downlinkCallback(mType,port,msgLen,msg):
    logger.info(f"downlink received type: {mType} port:{port} msglen:{msgLen} msg:{msg}")
    print(f"DownlinkCallback {mType},port {port}, len {msgLen},msg {msg}")

LoRaWAN.setDownlinkCallback(downlinkCallback)

########################################
#
# device configuration
#
########################################

def generalSetup():
    """
    setup general operating conditions
    :return:
    """
    print("General setup()")
    logger.info("performing general setup")

    # restore saved MAC parameters
    # these will be overwritten using the config file
    # settings later (and resaved)
    LoRaWAN.restoreMacConfiguration()

    # the LoRaWAN module is pre-programmed, in my case, for 868Mhz Region frequencies
    # the ASR AT command documentation says the following command is not supported
    # so I don't know if it can be changed programmatically
    # LoRaWAN.setFrequencyTable()

    # set the RX1 delay. This is always 5s after the Tx
    LoRaWAN.setRX1Delay(config["network"]["freqplan"]["RX1Delay"])

    # set new data rate if required.
    # note that ADR must be turned off to be able to change the DR
    # this code turns it back on afterwards if it was on
    oldDR=LoRaWAN.getDataRate()
    newDR = int(config["network"]["freqplan"]["joinDataRate"])

    if oldDR!=newDR:
        print(f"Device datarate was {oldDR} trying to set to {newDR}")
        ADR = LoRaWAN.getADR()
        if ADR:
            LoRaWAN.disableADR()
        LoRaWAN.setDataRate(newDR)  # set to middle of range to allow ADR to adjust up/down
        if ADR:
            LoRaWAN.enableADR() # re-enable if was enabled
    else:
        print(f"Device datarate already {newDR}")

    # set message type (UNCONFIRMED or CONFIRMED and number of trials to join (retries)
    LoRaWAN.setNbTrials(UNCONFIRMED_UPLINK,config["network"]["nbTrials"])

    # Select the TX power.
    # it is a legal requirement that antenna-gain+txPower is within
    # the limits set for each region. The module is pre-programmed
    # for 17dbm so this must be reduced to 13dbm for a 3db antenna in EU
    # The server may send downlinks with MAC commands requesting
    # the device alter it's power setting

    oldTxp= LoRaWAN.getTxPower()
    newTxp=int(config["network"]["txPower"])

    if oldTxp!=newTxp:
        LoRaWAN.setTxPower(newTxp)
    else:
        print(f"Device txPower already set to {newTxp}")

    # ADR is recommended to enable the server to request DR changes etc
    # to optimise comms
    ADR = LoRaWAN.getADR()
    if ADR==0:
        LoRaWAN.enableADR(1)

    # in EU the RX1 frequency is the same as the uplink frequency
    # other regions will/may be different
    # In EU the RX2 window is DR3 and at a fixed frequency
    # the device is already pre-programmed for EU868 so this could be skipped
    LoRaWAN.setReceiveWindowParameters( config["network"]["freqplan"]["RX1DRoffset"],
            config["network"]["freqplan"]["RX2DataRate"],
            config["network"]["freqplan"]["RX2Frequency"])


    # you can request a linkcheck - the following downlink contains information
    # about the demod margin, RSSI, SNR and number of gateways that
    # received the transmission
    # do not enable this for every transmission since you will exceed the
    # TTN fair use policy and legal duty cycle very quickly
    #LoRaWAN.enableLinkCheck(1) requests a linkcheck once only

    # the module defaults to port 10 until changed and saved
    # if you are sending uplinks on different ports you need
    # to change the port prior to sending
    LoRaWAN.setApplicationPort(1)

    print("General setup finished")

######################################################
#
# configuration methods for OTAA or ABP
#
# TTN recommends OTAA. After a join the session keys
# are used as if using ABP. After a join the session
# keys are saved in eeprom within the device by the call
# to LoRaWAN.saveMacConfiguration()
#
######################################################
def configureOTAA():
    """
    Sets the join keys. Call before join()
    :return: Nothing
    """
    global config,logger,LoRaWAN
    if THIS_JOIN_MODE!=OTAA:
        print("program is not configured for OTAA")
        return

    logger.info("Configuring for OTAA")
    print("Configuring OTAA")
    try:
        LoRaWAN.setJoinMode(OTAA)
        LoRaWAN.setDevEui(config["network"]["otaa"]["devEui"])
        LoRaWAN.setAppEui(config["network"]["otaa"]["appEui"])
        LoRaWAN.setAppKey(config["network"]["otaa"]["appKey"])
        return True
    except Exception as e:
        logger.error(f"configure OTAA failed with error {e}")
        return False

def configureABP():
    """
    sets the ABP keys. Call before sending a message

    :return: True or False on error
    """
    global config,logger,LoRaWAN

    if THIS_JOIN_MODE != ABP:
        print("program is not configured for ABP")
        return

    print("Configuring ABP")
    logger.info("Configuring for ABP")
    try:
        LoRaWAN.setJoinMode(ABP)
        LoRaWAN.setDevAddr(config["network"]["abp"]["devaddr"])
        LoRaWAN.setNwkSKey(config["network"]["abp"]["nwkSKey"])
        LoRaWAN.setAppSKey(config["network"]["abp"]["appSKey"])
        return True
    except Exception as e:
        logger.error(f"configure ABP failed with error {e}")
        return False

#########################################
#
# Join the network
#
#########################################

def joinTTN():
    """
    ABP keys are already programmed so joining is not required

    :return:  Nothing
    """
    if LoRaWAN.joined:
        # already joined - nothing to do
        print("joinTTN() - already joined")
        return

    print("Trying to join TTN")

    # use default join parameters
    # does not return till joined or timeout or Exception
    LoRaWAN.join()

    print("Join finished")


##########################################
#
# uplinks
#
##########################################

msgNum=1                # dummy counter for uplink message
lastSend=None           # used to conform to duty cycle legal limits
nextSend=time.time()    # enables first uplink to be sent


def getAirTime(msgLen):
    """
    calculate the expected airtime for this message

    :param msgLen: int
    :return: float: expected air time
    """

    DR=LoRaWAN.getDataRate() # possibly changed by ADR
    if DR is None:
        print("Error getting current data rate")
        return None

    sf,bwIdx=dataRates[DR]
    bw=bandwidths[bwIdx]

    return LoRaWAN.calcAirTime(msgLen,sf,bw)

def sendUplink():
    """
    send an uplink if the duty cycle has expired otherwise
    just return so as not to block
    :return: True if message sent otherwise False
    """
    global lastSend,msgNum,nextSend,logger

    logger.info("sending an uplink")

    # duty cycle control
    if time.time()<nextSend:
        return False

    lastSend=time.time()

    # ASR6501 default fPort is 10
    # this must be set before sending the payload if changing it
    # to use ports efficiently and save 1 byte of payload
    LoRaWAN.setApplicationPort(2)

    # don't use CONFIRMED_UPLINK as that requires a downlink
    # and the TTN limit is 10 per day

    message=f"HELLO {msgNum}" # you would normally send some sensor data here

    print(f"Sending uplink {message} at {time.time()}")

    LoRaWAN.sendPayload(message,UNCONFIRMED_UPLINK, config["network"]["nbTrials"])

    airTime=getAirTime(len(message))

    while LoRaWAN.getStatus() ==  STATUS.SENDING:
        # serial comms to the ASR6501 are at 115200 baud.
        # no point hammering it with queries
        # don't use 'pass' as it doesn't absorb time and pushes CPU use up
        time.sleep(airTime)

    if  LoRaWAN.getStatus() ==  STATUS.SEND_OK:
        # duty cycle
        nextSend=lastSend+(1/DUTY_CYCLE)*airTime
        msgNum+=1
        return True

    return False

###########################################
#
# main loop sending uplinks and checking for
# downlinks
#
############################################


generalSetup()  # set up common things like ADR,nbtrials, datarate

if THIS_JOIN_MODE==OTAA:
    configureOTAA()
else:
    configureABP()

joinTTN()


"""
The remainder of the code is a continuous loop which
sends periodic messages via uplinks.
Any downlinks will cause the downlinkCallback() method
to be called - downlinks usually occur in the receive windows RX1Delay
and RX2Delay after the uplink was sent

"""

print("Continuous loop..")

while True:
    loop_start=time.time()

    # this method call is only necessary for class B and Class C
    # devices
    LoRaWAN.checkForDownlink()

    # this only succeeds when the duty cycle has expired
    # otherwise is an immediate return
    sendUplink()

    # todo think about class B and ping slots

    # small delay to reduce serial traffic
    while time.time()-loop_start<LOOP_DELAY:
        time.sleep(0.1)

