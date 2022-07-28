"""
ASR6501.py

Author: Brian Norman
Date: 28/7/2022

Python library API for an M5STACK LoRaWAN module, using an ASR6501, to talk to TTN.

The module is driven using serial AT commands.

If inquiry commands result in an error an Exception is raised. The lastATCmd,lastErrorType and lastErrorNum
are stored for caller access and can be retrieved in a dict using getLastATError()

Based on the document 'ASR650X AT Command Summary.pdf' found here:-

https://github.com/r3cn3p5/lorawan/blob/main/ASR650X%20AT%20Command%20Introduction-20190605.pdf

"""
import re
import time
import logging
import math

OTAA=0
ABP=1
DEFAULT_LOG_LEVEL = logging.DEBUG

# exceptions

""" 
Exceptions which may be raised
"""

class ASRUnsupported(Exception):
    """ ASR the chosen method is not supported in this library"""
    pass

class ASRNoPayload(Exception):
    """ ASR DTRX no payload specified """
    pass

class ASRReceiveTimeout(Exception):
    """ ASR AT command response timeout """
    pass

class ASRTomlConfigKeyError(Exception):
    """ ASR error accessing TOML config file entry """
    pass

class ASRPatternMatchFailed(Exception):
    """ ASR unable to extract value (pattern match failed) """
    pass

class ASRCMEError(Exception):
    """ ASR the last AT command failed """
    pass

class ASRERRSend(Exception):
    """ ASR DTRX error sending the last uplink  """
    pass

class ASRERRSent(Exception):
    """ ASR DTRX TX_CNT error sending the last uplink"""
    pass

class ASRErrorUnexpected(Exception):
    """ ASR an unexpected error occurred"""
    pass

class ASRUnsupportedManuf(Exception):
    """ Expected 'ASR' as manufacturer ID"""
    pass

class _ASRShortReply(Exception):
    """ Expected at least 3 lines in ASR response"""
    pass

class ASRInvalidDevEui(Exception):
    """ DevEui should be a 8 char hex string"""
    pass

class ASRInvalidAppEui(Exception):
    """ AppEui should be a 8 char hex string"""
    pass

class ASRInvalidAppKey(Exception):
    """ AppKey should be a 16 char hex string"""
    pass

class ASRInvalidDevAddr(Exception):
    """ Devaddr should be a 4 char hex string"""
    pass

class ASRInvalidNwkSKey(Exception):
    """ nwkSKey should be a 16 char hex string"""
    pass

class ASRInvalidAppSKey(Exception):
    """ appSKey should be a 16 char hex string"""
    pass

class ASRUnknownClassCombination(Exception):
    """ setClass() Unknown class/branch combination"""
    pass

class ASRSerialError(Exception):
    """ Unable to get manuf id. Check serial connection."""
    pass

class ASRInvalidBootMode(Exception):
    """ An invalid reboot mode was given Should be 0,1 or 7"""
    pass


"""
re patterns used to match AT responses
"""
reply_pat   = re.compile("(.*)[:=](.*)")    # some responses use '=' but most use ':'
kv_pat      = re.compile("([0-9]+):(.*)")   # key value pairs as in rssi list
"""
re patterns used to validate keys
"""
hex8char =  re.compile("^([0-9A-F][0-9A-F]){4}$")   # 8 char hex string e.g. DevAddr (64 bits)
hex16char =  re.compile("^([0-9A-F][0-9A-F]){8}$")  # 16 character hex string e.g. AppEui,DevEui (128 bits)
hex32char = re.compile("^([0-9A-F][0-9A-F]){16}$")  # 32 character hex string e.g AppKey, NwkSKey,AppSKey

# possible status responses to getStatus()
class STATUS:
    """
    LoRaWAN module status values.
    """
    #                           ASR manual explanation.
    NOOP = 0                    # 00 – there is no data operation
    SENDING = 1                 # 01 – there is data in sending
    SEND_FAILED = 2             # 02 – there is data sent but failed
    SEND_OK = 3                 # 03 – there is data sent and success
    JOIN_OK = 4                 # 04 – JOIN success(only appear in first join procedure)
    JOIN_FAILED = 5             # 05 – JOIN fail(only appear in first join procedure)
    LINKCHECK = 6               # 06 – Network may abnormal（result from Link Check）
    NO_DOWNLOAD = 7             # 07 – there is data sent and success but no download
    DOWNLOAD_AVAILABLE = 8      # 08 – there is data sent and success, there is download too.

class asr6501:
    """
    The ASR6501 API class

    Create a serial object and a logger in your main code before instantiating the class

    """
    def __init__(self,serial,logging_level:int=DEFAULT_LOG_LEVEL):
        """
        initialises the ASR6501 class

        :param serial: serial stream object
        :param logging_level: such as logging.INFO, logging.DEBUG etc
        :param debug:
        """

        self.serial=serial

        # logging should be set up by caller
        self.logger = logging.getLogger("ASR")
        self.logger.setLevel(logging_level)

        self.logger.info("__init__ starting")

        self.downlinkCallback=None # downlink callback
        self.downlinkRecvd = False

        # cleared to None when new command is sent
        self.lastErrorNum = None
        self.lastErrorType = None
        self.lastATCmd = None

        # check if modem is available on the serial port
        # this command requests a single string reply
        try:
            reply = self.getManufId()

            #self.logger.debug(f"getManufId() reply was {reply}")

            if reply is None:
                self.logger.error(f"Unable to communicate with the ASR module. Reply to AT+CGMI was None")
                raise ASRSerialError

            if reply !="ASR":
                self.logger.error(f"The device manufacturer id was not ASR")
                raise ASRUnsupportedManuf

        except Exception as e:
            self.logger.exception(f"getting manufacturer Id raised exception {e}")
            exit(e)

        self.setLogLevel(0) # turn of the extra output it confuses parsing of some commands like CJOINMODE?
        self.joined=False
        self.joinMode=self.getJoinMode()
        self.logger.info(f"Current join mode is {self.joinMode}. 0=OTAA,1=ABP")

    ################################################
    #
    # methods not intended for public use
    #
    ################################################
    def setDownlinkCallback(self, func=None)->bool:
        """
        Configure the callback function which will receive 4 parameters
        Mtype,Port,msgLen,msg

        NOTE: when sending uplinks this code automatically checks for any downlinks
        in the RX1/RX2 windows. For class B/C operation your code needs to frequently call
        checkForDownlink() which will invoke the callback if a downlink has been received.

        :param func: the function to call when a downlink message is received
        :return: True if set, False if not
        """
        if hasattr(func, '__call__'):
            self.logger.info("Setting downlinkCallback to %s", func)
            self.downlinkCallback = func
            return True
        else:
            self.logger.info("downlinkCallback is not callable")
            return False

    def checkForDownlink(self):
        """
        check if a downlink has been received.

        Primarily for class B & C operation since , with class A
        downlinks occur after an uplink in the RX1/RX2 windows.

        If a message has been received invokes the downlinkCallback
        method if set up before hand otherwise does nothing
        with the downlink.

        Should be called repeatedly from a loop in the control
        program


        :return: Nothing
        """
        if self.serial.in_waiting==0:
            return

        rx = self.serial.readline().rstrip().decode("utf-8")

        if rx[:8] == "OK+RECV:":
            self.logger.debug(f"{rx}")
            # OK+RECV: TYPE, PORT, LEN, DATA
            if self.downlinkCallback is not None:
                type, port, msglen, msg = rx[8:].split(",", 4)
                self.downlinkCallback(type, port, msglen, msg)

    def _getDelayedReplies(self):
        """
        Some commands like JOIN,TRX,RXD return OK or CME ERROR immediately.
        Status messages occur a short while later.

        For example JOIN returns an immediate OK if the syntax is correct. It can
        then take a few seconds before receiving JOIN:OK etc

        If a downlink is received 'OK+RECV:...' we invoke a callback, if configured

        Caller should wait till there is information to be read from the ASR6501 module
        or timeout.

        :return: Nothing
        """

        self._clearLastError()

        # analyse any module messages
        while self.serial.in_waiting>0:

            rx = self.serial.readline().rstrip()
            if rx != b'':
                self.logger.info(f"Received {rx}")
                rx = rx.decode("utf-8")
                if rx =="+CJOIN:OK":
                    self.logger.info("Join OK")
                    self.joined=True
                elif rx == "+CJOIN:FAIL":
                    self.logger.error("Join failed")
                    self.joined=False
                elif rx[:8] == "OK+SENT:":
                    self.logger.info(f"Msg sent OK {rx}")
                    pass
                elif rx[:8] == "OK+RECV:":
                    # OK+RECV: TYPE, PORT, LEN, DATA
                    if self.downlinkCallback is not None:
                        self.downlinkRecvd = True
                        type, port, msglen, msg = rx[8:].split(",", 4)
                        self.downlinkCallback(type, port, msglen, msg)
                    else:
                        self.logger.info("Downlink received but no callback has been setup")
                elif rx[:8] == "OK+SEND:":
                    self.logger.info(f"Msg send OK {rx}")
                    pass
                elif rx[:9] == "ERR+SEND:":
                    self.lastErrorType="ERR+SEND"
                    self.lastErrorNum=int(rx[9:])
                    self.logger.error("Send failed : {rx}")
                    raise ASRERRSend
                elif rx[:9] == "ERR+SENT:":
                    self.lastErrorType = "ERR+SENT"
                    self.lastErrorNum = int(rx[9:])
                    self.logger.error("Sent failed : {rx}")
                    raise ASRERRSent
                else:
                    self.logger.error("Untrapped rx msg {rx}")

    def _inquire(self,cmd:str,rxTimeout:int=2) -> list or str:
        """
        called by all inquiry (get) commands which either return the data
        requested or raise an exception if a CME ERROR occurs

        AT+ is prefixed to the cmd and "?\r\n" is postfixed

        :param cmd: str: the AT command like "CRSSI 1"
        :return: list (RSSI) or str (single value)
        """
        self._clearLastError()
        self.lastATCmd = cmd
        self.serial.reset_input_buffer()  # clear out any residual log stuff produced if loglvl>0

        self.logger.info(f"=>{cmd}")

        ATcmd = f"AT+{cmd}?\r\n"
        self.serial.write(ATcmd.encode())

        # inquire commands either raise CME ERROR or return valid data
        rssiData=[]             # RSSI returns a list terminated by OK
        kvp=None                # key value pair for single value returns
        doingRssiList=False     # set if we see +CRSSI in the response

        start=time.time()
        while True:

            if time.time()-start>=rxTimeout: # prevent an infinite loop
                raise ASRReceiveTimeout

            if self.serial.in_waiting > 0:
                rxData = self.serial.readline().rstrip()
                self.logger.debug(f"rxData {rxData}")
                if rxData != b'':
                    rxData = rxData.decode("utf-8")
                    if rxData[:3]=="AT+":
                        # first line returned is always a command echo
                        pass
                    elif rxData=="+CRSSI:":
                        # after seeing this we get a list of "Channel: RSSI" key-value pairs terminated by OK
                        doingRssiList=True
                    elif rxData == "OK":
                        # OK signifies a command reply end
                        if doingRssiList:
                            # end of rssi list, return it
                            return rssiData
                        else:
                            # get the value part of the key-value pair (kvp)
                            # value replies pattern is (.*)[:=](.*)
                            # = is used by CGMI, CGMR and CGSN
                            # all other return values use a colon
                            m = re.match(reply_pat, kvp)
                            if m:
                                # we only need the value part. It could be a string, a csv or simple number
                                # called will know what to do with it.
                                return m.group(2)
                            raise ASRPatternMatchFailed

                    elif rxData[:11] == "+CME ERROR:":
                        # this ALWAYS seems to be CME ERROR:1 regardless
                        self.lastErrorType, self.lastErrorNum = rxData.split(":", 2)
                        raise ASRCMEError
                    else:
                        if doingRssiList:
                            rssiData.append(rxData)
                        else:
                            kvp=rxData

    def _setCmd(self, cmd: str,rxTimeout:int=2) -> bool:
            """
            called by all set commands which either return OK
            or raise an exception if a CME ERROR occurs

            :param cmd: str: the full command with parameters (if any)
            :return: True/False or raise an Exception
            """
            self._clearLastError()
            self.lastATCmd = cmd
            self.serial.reset_input_buffer()  # clear out any residual log stuff produced if loglvl>0

            ATcmd = f"AT+{cmd}\r\n"

            self.logger.info(f"=>{cmd}")

            self.serial.write(ATcmd.encode())

            # looking for OK or CME ERROR as a response to a command
            start=time.time()
            while True:
                if time.time()-start>=rxTimeout:
                    raise ASRReceiveTimeout

                if self.serial.in_waiting > 0:
                    rxData = self.serial.readline().rstrip()
                    self.logger.debug(f"<= {rxData}")
                    if rxData != b'':
                        rxData = rxData.decode("utf-8")

                        if rxData == "OK":
                            # this response just shows the command was accepted
                            return True
                        elif rxData[:8] == "OK+SEND:":
                            # this response shows the message was transmitted
                            m=re.match(reply_pat,rxData)
                            if m:
                                self.logger.info(f"Sent {m.group(2)} bytes")
                            return True

                        elif rxData[:11] == "+CME ERROR:":
                            self.lastErrorType, self.lastErrorNum = rxData.split(":", 2)
                            raise ASRCMEError
                        else:
                            # no other possibilities matter
                            pass

    def _noneOrInt(self,value:None or str) -> int or None:
        """
        If an AT command returns an integer in string form
        convert it to int

        just reduces the amount of repetition in coding

        :param value: str: hopefully an int
        :return: int or None
        """
        if value is None:
            return None
        return int(value)

    def _clearLastError(self):
        """
        any errors are cached till the next AT command
        
        :return: nothing
        """
        self.LastErrorType = None
        self.LastErrorNum = None

    ##########################################################
    #
    # calcAirTime
    #
    ##########################################################
    def calcAirTime(self,PL: int = 0, SF: int = 9, BW: int = 125, NP: int = 8, HE: int = 0, DE: int = 0, CR: int = 1):
        """
        The LoRaWAN module AT commands do not provide airtime feedback (AFAIK) therefore this
        method is provided to assist the user to stay within the legal duty cycle limits
        and TTN fair access policy

        This method, by default,calculates the airtime of a packet at DR3 - you only need to
        provide the  message length if that is true.

        To use it for other regions and data rates pass in the SF and BW eg

            Tair=getAirTime(myMsgLen,8,500) - AU915 DR6

        The output matches that of online airtime calculators

        see:-
            https://avbentem.github.io/airtime-calculator/ttn/au915

        :param PL:   payload length
        :param SF:   Spreading factor   7-12
        :param BW:   bandwidth (125,250,500)
        :param NP:   number of preamble symbols
        :param HE:   header 0=enabled/explicit, 1=disabled/implicit
        :param DE:   Low data Rate 1=enabled, 0=disabled
        :param CR:   Coding rate 1..4 default is 1
        :return:     float: air time in ms
        """

        # assuming the uplink contains an fPort
        overhead = 13  # MHDR(1)+DevAddr(4)+FCtrl(1)+FCnt(2)+MIC(4)+Fport(1)

        Tsym = math.pow(2, SF) / (BW)

        Tpreamble = (NP + 4.25) * Tsym

        payloadSymbNb = 8 + max(math.ceil((8 * PL - 4 * SF + 28 + 16 - 20 * HE) / (4 * (SF - 2 * DE))) * (CR + 4), 0)

        Tpayload = payloadSymbNb * Tsym

        return Tpayload + Tpreamble
   
    #####################################################
    # General commands
    #####################################################

    def getLastATError(self) -> dict or None:
        """
        last error info is updated per transaction.

        provided for exception handling

        last error is a string like
            "+CME ERROR:<num>"
            "+ERR SEND:<num>
            "+ERR SENT:<num>

        These values are rcleared to None at the start of a command. If there was no error
        then Type and Num will be None

        Error Type is the string preceeding the colon (:) and the error number is the part after
        the colon.

        :return: dict:  { "AtCmd":lastATCmd,"Type":lastErrorType,"Num":lastErrorNum} or None if no error.
        """
        if self.lastErrorType is None:
            return None

        return { "AtCmd":self.lastATCmd,"Type":self.lastErrorType,"Num":self.lastErrorNum}

    def getManufId(self) -> str:
        """
        get the manufacturer ID which should always be ASR for this device

        :return: str:<manufacturer> e.g. 'ASR'
        """
        return self._inquire("CGMI")

    def getModelRevision(self) -> str:
        """
        return the LoRaWAN module revision

        :return: str: v4.3 softversion=V1.2.0 or similar
        """
        return self._inquire("CGMR")

    def getSerialNumber(self) -> str:
        """
        get the device serial number

        :return: str: '0D39FFCD00031E43' or similar
        """
        return self._inquire("CGSN")

    def setBaud(self,baud:int)-> bool:
        """
        Return True if the AT command succeeds otherwise False

        :param baud: int e.g. 115200 (default)
        :return: True/False
        """
        return self._setCmd(f"CGBR={baud}")

    def getBaud(self)->int:
        """
        returns the UART current serial baud rate setting, normally 115200

        :return: int: <baud>
        """
        return self._noneOrInt(self._inquire("CGBR"))

    #####################################################
    # Network related
    #####################################################
    def setJoinMode(self,mode:int=0) -> bool:
        """
        use this command before sending any data

        :param mode: 0=OTAA, 1=ABP
        :return: Nothing
        """
        assert 0<=mode<=1,"Join mode must be 0=OTAA or 1=ABP"

        if self._setCmd(f"CJOINMODE={mode}"):
            self.joinMode = mode
            return True
        return False

    def getJoinMode(self) -> int:
        """
        Return the current join mode

        :return: int:<joinmode> 0=OTAA, 1=ABP
        """
        mode=self._noneOrInt(self._inquire("CJOINMODE"))
        self.joinMode=mode
        return mode

    def setDevEui(self,deveui:str) -> bool:
        """
        set the DevEui string

        :param deveui: str: 8 hex chars
        :return: True/False
        """
        assert self.joinMode is OTAA,"setDevEui is only allowed for OTAA"
        m=re.match(hex16char,deveui)
        if m:
            return self._setCmd(f"CDEVEUI={deveui}")
        else:
            raise ASRInvalidDevEui

    def getDevEui(self) -> str:
        """
        The device comes with a deveui already programmed. This will need
        changing for your app.

        Expected AT response

        ['AT+CDEVEUI?', '+CDEVEUI:00BB9DA5B97ADDF6', 'OK', 'ASR6501:~#']

        :return: str: <deveui>
        """
        return self._inquire("CDEVEUI")

    def setAppEui(self, appeui:str) -> bool:
        """
        sets the devEui key
        
        :param appeui: 32 char hex string
        :return: True/False
        """
        assert self.joinMode is OTAA, "setAppEui is only allowed for OTAA"
        m=re.match(hex16char,appeui)
        if m:
            return self._setCmd(f"CAPPEUI={appeui}")
        else:
            raise ASRInvalidAppEui

    def getAppEui(self) -> str:
        """
        The device already comes with an appEui programmed. It needs changing.

        :return: str:<appeui>
        """
        return self._inquire("CAPPEUI")

    def setAppKey(self, appkey:str) -> bool:
        """
        set the appKey for OTAA join
        
        :param appkey: 32 char hex string
        :return: True/False
        """
        assert self.joinMode is OTAA, "setAppKey is only allowed for OTAA"
        m=re.match(hex32char,appkey)
        if m:
            return self._setCmd(f"CAPPKEY={appkey}")
        else:
            raise ASRInvalidAppKey

    def getAppKey(self) -> str:
        """
        The device comes programmed with an appkey - it will need changing for your app.

        :return: str: <appkey>
        """
        return self._inquire("CAPPKEY")

    def setDevAddr(self, devaddr:str) -> bool:
        """
        Set the DevAddr string

        :param devaddr: 16 char hex string
        :return: True/False
        """
        m=re.match(hex16char,devaddr)
        if m:
            return self._setCmd(f"CDEVADDR={devaddr}")
        else:
            raise ASRInvalidDevAddr

    def getDevAddr(self) -> str:
        """
        return the current DevAddr

        :return: str: <devaddr>
        """
        assert self.joinMode is ABP, "getDevAddr is only allowed for ABP"
        return self._inquire("CDEVADDR")

    def setAppSKey(self, appskey:str) -> bool:
        """
        set the AppSKey string. Only used for ABP

        :param appskey: str: 32 hex chars
        :return: True/False
        """
        assert self.joinMode is ABP, "setAppsKey is only allowed for ABP"
        m=re.match(hex32char,appskey)
        if m:
            return self._setCmd(f"CAPPSKEY={appskey}")
        else:
            raise ASRInvalidAppSKey

    def getAppSKey(self) -> str :
        """
        return the current AppSKey

        :return: str: <APPSKEY>
        """
        return self._inquire("CAPPSKEY")

    def setNwkSKey(self, nwkskey:str)-> bool:
        """
        Should be set before sending a message

        :param nwkskey: 32 char hex string (16 bytes)
        :return: Nothing
        """
        m=re.match(hex32char,nwkskey)
        if m:
            return self._setCmd(f"CNWKSKEY={nwkskey}")
        else:
            raise ASRInvalidNwkSKey

    def getNwkSKey(self) -> str:
        """
        return the current nwkSkey

        :return: str: <nwkskey>
        """
        return self._inquire("CNWKSKEY")

    def setFreqBandMask(self,mask:int=1)-> bool:
        """
        sets the bit mask used to enable/disable channels

        1 - ch 0-7
        2 - ch 8-15
        3 - ch 16 -23

        Should be set before JOIN

        :param mask: int: default 1
        :return: True/False
        """
        self._setCmd(f"CFREQBANDMASK=000{mask}")

    def getFreqBandMask(self) -> int:
        """
        get the Current frequency band mask

        ch0-7 mask is 1
        ch8-15 mask is 2

        :return: int: frequency band mask
        """
        return self._noneOrInt(self._inquire("CFREQBANDMASK"))

    def setULDLmode(self,mode) -> bool:
        """
        Sets how uplink/downlink frequencies are configured

        In EU the downlink freq is the same as the uplink

        :param mode: 1: same frequency mode, 2: different freq mode
        :return: True/False
        """
        assert 1<=mode<=2,"setULDLmode invalid mode should be 1 or 2"
        return self._setCmd(f"CULDLMODE={mode}")

    def getULDLmode(self) -> int:
        """
        get the current uplink/downlink mode

        :return: int
        """
        return self._noneOrInt(self._inquire("CULDLMODE"))

    #################################################
    #
    # Optional multicast addressing
    #
    # see https://lora-alliance.org/resource_hub/lorawan-remote-multicast-setup-specification-v1-0-0/
    #
    #################################################

    def addMulticastAddr(self,addr:str) -> bool:
        """
        Add a multicast address (32 hex characters expected)

        :param addr: str: 32 hex characters
        :return: Nothing
        """
        assert len(addr)==32,"Multicast address must be 32 hex characters"
        return self._setCmd(f"CADDMUTICAST={addr}")

    def delMulticastAddr(self,addr:str) -> bool:
        """
        delete a multicast address from the list.

        :param addr: str: Hex address 32 characters
        :return: Nothing
        """
        assert len(addr) == 32, "Multicast address must be 32 characters"
        return self._setCmd(f"CDELMUTICAST={addr}")

    def getNumberOfMulticastAddr(self) -> int:
        """
        Gets a count of the number of multi-cast addresses known by the device
        
        :return: int: count
        """
        return self._noneOrInt(self._inquire("CNUMMUTICAST"))

    ##################################################
    # Control and Status
    ##################################################

    def setWorkMode(self, mode:int=2) -> bool:
        """
        Set work mode. The default is 2 (normal)
        
        Should be set before join. Spec does not indicate other
        values that can be used.
        
        :param mode: int: default 2
        :return: Nothing
        """
        return self._setCmd(f"CWORKMODE={mode}")

    def getWorkMode(self) -> int:
        """
        returned mode will be 2 (normal work mode)

        :return: int: mode
        """
        return self._noneOrInt(self._inquire("CWORKMODE"))

    def setClass(self, loraClass=0,branch=0,param1=None,param2=None,param3=None,param4=None) -> bool:
        """
        set the device class 0=A,1=B,2=C
        
        If class is 1 and branch is 0, then only para1 parameter is used to set the ping slot periodicity,
        whose value range is 0~7, the related period time is 0.96*2^periodicity seconds；

        If class is 1 and branch is 1, then 
            param1 is used to set the frequency of beacon, its unit is Hz;
            param2 is used to set the data rate of the beacon;
            param3 is used to set the frequency of ping slot, its unit is Hz; 
            param4 is used to set the data rate of ping slot.

        Raises an exception if the class/branch combination is unknown

        :param loraClass: 0-A, 1=B, 2=C
        :param branch: 0 or 1
        :param para1: ping slot periodicity if class=1 and branch=0 else beacon frequency
        :param para2: datarate if class=1 and branch=1 else not used
        :param para3: ping slot frequency
        :param para4: ping slot data rate
        :return: True/false
        """
        if branch==0 and loraClass==0:
            return self._setCmd(f"CCLASS={loraClass}")

        if branch == 0 and loraClass == 1:
            return self._setCmd(f"CCLASS={loraClass},{param1}")

        if branch==1 and loraClass==1:
            return self._setCmd(f"CCLASS={loraClass},{param1},{param2},{param3},{param4}")

        raise ASRUnknownClassCombination

    def getClass(self) -> int:
        """
        Returns the current class (0..2) of the device

        0 - class A -   listens during RX1 and RX2
        1 - class B -   preconfigured listening slots
        2 - class C -   always listening

        :return: int: class number
        """
        return self._noneOrInt(self._inquire("CCLASS"))

    def getBatteryLevel(self) -> int:
        """
        get the current battery level 0..100.

         :return: int: 0..100
        """
        return self._noneOrInt(self._inquire("CBL"))

    def getStatus(self) -> int:
        """
        Returns current status of the device:-

        00 – there is no data operation
        01 – there is data in sending
        02 – there is data sent but failed
        03 – there is data sent and success
        04 – JOIN success (only appear in first join procedure)
        05 – JOIN fail (only appear in first join procedure)
        06 – Network may abnormal（result from Link Check）
        07 – there is data sent and success but no download
        08 – there is data sent and success, there is download too.

        :return: int: status
        """
        return self._noneOrInt(self._inquire("CSTATUS"))

    def join(self,start:int=1,autojoin:int=0,interval:int=8,retries:int=8)->bool:
        """
        Start or stop the join process.
        Set the OTAA keys and other parameters first.

        :param start: int: 0: stop join,1: start join
        :param autojoin: int: 0: disable auto join, 1:enable auto join (factory)
                if enabled the device will re-join after a power cycle
        :param interval: int: 7..255 (factory is 8 seconds) period between retries
        :param retries: int: 1..256 max number of join retries - default is 8

        responses from the module
        OK                  command syntax ok, then one of
        +CJOIN:OK           Authentication Success
        OK+SENT:nn          presumably TTN keys
        OK+RECV:aa,bb,cc    downlink received in RX1/RX2 window

        +CJOIN:FAIL         Authentication Fail

        :return: True/False or raise ASRCMEError
        """
        assert 0 <= start <= 1,"start parameter must be 0 (stop) or 1 (start)"
        assert 0 <= autojoin <= 1, "autojoin parameter must be 0 (disable) or 1 (enable)"
        assert 1 <= interval <= 255, "interval parameter must be in range 1..255 seconds"
        assert 1 <= retries <= 256, "retries parameter must be in range 1..256"

        if self.joined and start!=0:
            self.logger.warning("Attempt to join when already joined")
            return False

        self._clearLastError()

        self.serial.reset_input_buffer()    # clear out any residual stuff

        cmd=f"CJOIN={start},{autojoin},{interval},{retries}"

        if not self._setCmd(cmd):
            # probably an Exception would prevent getting here
            return False

        # each join retry could take RX1Delay (5s)+RX2Delay (1s) before the attempt has
        # timed out (no response from server)
        RX1Delay=self.getRX1Delay()
        rxTimeout=retries*(RX1Delay+1)

        start=time.time()
        while self.serial.in_waiting==0:
            if time.time()-start>=rxTimeout:
                raise ASRReceiveTimeout

        self.logger.info("Waiting for 'JOIN:OK' status")
        self._getDelayedReplies()   # soak up the replies and check them
        return self.joined

    def getJoinInfo(self) -> dict:
        """
        get the current join parameters as a dict

        Keys        Values
        start       0=stop join, 1- start join
        autojoin    0=disable auto join, 1=enable auto-join
        interval    retry interval - seconds
        retires     max retries

        :return: dict: containing named parameters
        """
        res=self._inquire("CJOIN")
        params=res.split(",",4)
        return { "start":params[0],"autojoin":params[1],"interval":params[3],"retries":params[3]}

    ####################################################
    # send and receive data
    ####################################################

    def sendPayload(self,payload:str=None,confirm:int=0,nbtrials:int=8) -> bool:
        """
        possible responses

            OK+SEND:TX_LEN              # confirm length
            OK+SENT:TX_CNT              # nbtrials
            OK+RECV:TYPE,PORT,LEN,DATA  # received data
            or
            ERR+SEND:ERR_NUM            # error sending
            ERR+SENT:TX_CNT             # nbtrials expired?
            or
            +CME ERROR:<err>            # command error

        Note OK+RECV information is passed to a downlink callback method
        if it has been setup using setDownlinkCallback(func)

        :param payload: str: msg to send
        :param confirm: int: 0/1 1=confirmed message 0=unconfirmed
        :param nbtrials: int: default is 8

        :return: True/false
        """
        assert payload is not None, "A payload string is required"

        # the payload needs converting to hex bytes

        hexPayload=payload.encode("utf-8").hex()
        lenHexPayload=len(hexPayload)

        try:
            self.downlinkRecvd = False # downlinks always come after uplinks with class A

            # this will raise an exception on error and not return
            self._setCmd(f"DTRX={confirm},{nbtrials},{lenHexPayload},{hexPayload}")

            # after sending a message wait for a possible reply which should occur within
            # RX1Delay and timeout after RX1Delay+RX2Delay (1s)
            delay=self.getRX1Delay()+2 # give it more than enough time
            start=time.time()
            while time.time()-start<=delay:
                # if a downlink is recvd this method will
                # invoke the downlinkCallback then
                # continue to absorb all other serial input
                # till it is exhausted
                self._getDelayedReplies()

            return True

        except Exception as e:
            self.logger.exception(f"Uplink error: {e}")
        finally:
            return False

    def receivePayload(self) -> dict:
        """
        get downlink messages if any.

        This is separate from the downlinkCallback which includes the message type,
        port,msglen and msg information

        RX buffer is emptied.

        :return: dict: containing length and payload
        """

        reply = self._inquire("DRX")  # returns string "length,payload" or "0"
        if reply!="0":
            # do we have a payload or not?
            res= reply.split(",",2)
            return {"length": res[0], "payload": res[1]}
        else:
            return {}
    ####################################################
    # MAC setup command
    ####################################################

    def setSendMessageConfirm(self,confirm:int=0)->bool:
        """
        Set the default message type as confirmed(1) or unconfirmed (0)

        Using confirmed messages will eat into your TTN fair use and legal
        duty cycle limits

        :param confirm: int: 1 (confirmed) or 0 (unconfirmed)
        :return: True/False
        """
        assert 0<=confirm<=1,"Invalid value for confirm. Should be 0 or 1."
        return self._setCmd(f"CCONFIRM={confirm}")

    def getMessageType(self) -> int :
        """
        returns the default message type as 1=CONFIRMED or 0=UNCONFIRMED

        0 for unconfirmed, 1 for confirmed

        :return: int: 0 for unconfirmed, 1 for confirmed
        """
        return self._noneOrInt(self._inquire("CCONFIRM")) # returns 1 or None

    def setApplicationPort(self, port:int=10) -> bool:
        """
        Set the port to sending messages.

        If using the port to group messages you need to call this
        before sending a message.

        :param port: int: default 10 (factory)
        :return: True/False
        """
        assert 1<=port<=223,"Application port should be 1..223 other values are reserved"
        return self._setCmd(f"CAPPPORT={port}")

    def getApplicationPort(self) -> int:
        """
        Returns the current port number in range 1..223 other values
        are reserved.

        :return: int: application port (fPort)
        """
        return self._noneOrInt(self._inquire("CAPPPORT"))

    def setDataRate(self,DR:int=3) ->bool:
        """
        The EU868 factory default value is DR3, its value range is:

        0 - SF12，BW125
        1 - SF11，BW125
        2 - SF10，BW125
        3 - SF9，BW125
        4 - SF8，BW125
        5 - SF7，BW125

        :param DR: int: 0..5
        :return: True/False
        """
        assert 0<=DR<=5,"setDataRate: Invalid value. range 0..5"
        return self._setCmd(f"CDATARATE={DR}")

    def getDataRate(self) -> int:
        """
        returns the current datarate 0..5

        :return: int: data rate
        """
        return self._noneOrInt(self._inquire("CDATARATE"))

    def getRSSI(self,freqBandIdx:int=1) -> dict:
        """
        return a list of RSSI key-value pairs for each frequency band

            0:<Channel 0 rssi>
            1:<Channel 1 rssi>
            …
            15:<Channel 8 rssi>

        :freqBandIdx: int: frequency band index
        :return: dict
        """
        rssiList=self._inquire(f"CRSSI 000{freqBandIdx}")
        # list contains "key: value" pairs
        d={}
        #
        for entry in rssiList:
            m=re.match(kv_pat,entry)
            if m:
                d[m.group(1)]=int(m.group(2))
        return d

    def getNbTrials(self) -> dict:
        """
        returns the message type and current nbtrials setting, which controls join retries

        expected AT response
        ['AT+CNBTRIALS?', '+CNBTRIALS:1,8', 'OK', 'ASR6501:~#']

        First digit is confirmed/unconfirned message type
        0: unconfirm package, 1: confirm
        Second value nbtrials= max send times, range 1..15

        :return: dict: {"MType":<type>, "NbTrials":<value>} or None if error
        """

        res = self._inquire("CNBTRIALS") # returns str "<MType>,<NbTrials>"
        MType,NbTrials=res.split(",",2)
        return {"MType":int(MType),"NbTrials":int(NbTrials)}

    def setNbTrials(self,Mtype:int,nbTrials:int) -> bool:
        """
        This parameter affects how the device attempts to join by reducing
        the data rate till a join succeeds (if it ever does). This only works if
        ADR is enabled.

        ASR documentation places a limit of 15 on this despite what is said here:-

        ref: https://stackforce.github.io/LoRaMac-doc/LoRaMac-doc-v4.4.5/group___l_o_r_a_m_a_c.html

        NbTrials should be set prior to a join

        :param: MType: int 0=unconfirmed, 1=confirmed
        :param nbtrials: int: range 1-15
        :return: True/False
        """
        assert 0<=Mtype<=1,"The message type must be 0 (unconfirmed) or 1 (confirmed)"
        assert 1<=nbTrials<=15,"The ASR6501 AT command document sets a range of 1..15 for nbtrials"
        return self._setCmd(f"CNBTRIALS={Mtype},{nbTrials}")

    def getReportMode(self) -> dict:
        """
        The command is mainly used for test purposes and should be issued before a send

        Expected AT response
            +CRM:<reportMode>,[reportInterval]
            OK

        :return: dict or None
        """
        t=self._inquire("CRM")
        mode,interval=t.split(",",2)
        return {"mode": int(mode),"interval":int(interval)}

    def setReportMode(self,reportmode:int,interval:int) -> bool:
        """
        Sets the device reporting mode and, if periodic, the interval between uploads.

        I'm not sure what this actually is used for.

        This needs to be issued before sending data.

             period
        DR   LV1 LV2
        DR0 150 300
        DR1  75 150
        DR2  35 70
        DR3  15 30
        DR4  10 20
        DR5   5 10

        :param reportmode: int: 0=non-periodic, 1=periodic
        :param interval: int: if periodic (seconds 5..300, see table)
        :return: bool: True/False
        """
        if reportmode==0:
            return self._setCmd(f"CRM={reportmode}")
        else:
            return self._setCmd(f"CRM={reportmode},{interval}")

    def setTxPower(self,power:int) -> bool:
        """
        Set the transmitter power

        The EU868 max is 16dbm therefore, with a 3dbm antenna, the power should be set to 13dbm

        0 - 17dBm factory setting
        1 - 15dBm
        2 - 13dBm
        3 - 11dBm
        4 - 9dBm
        5 - 7dBm
        6 - 5dBm
        7 - 3dBm

        NOTE: If ADR is enabled the server may adjust this using MAC commands.

        :param power: int: 0..7
        :return: bool: True/False
        """
        assert 0<=power<=7,"setTxPower invalid value range is 0..7"
        return self._setCmd(f"CTXP={power}")

    def getTxPower(self) -> int:
        """
        returns the index into the power table (see setTxPower)

        :return: int: current power setting
        """
        return self._noneOrInt(self._inquire("CTXP"))

    def enableLinkCheck(self,mode:int=0)-> bool:
        """
        The command must be used before an uplink transmission

        :param mode: int 0=disable,1=link check once only, 2=after every uplink
        :return: True/False
        """
        return self._setCmd(f"CLINKCHECK={mode}")

    def getLinkCheck(self) -> dict:
        """
        This command throws an error - presumably until setLinkCheck has been called
        and needs to be sent before sending data.

        After a time (presumably, asynchronously, after an uplink) it will return
            +CLINKCHECK:Y0，Y1，Y2，Y3，Y4

            Y0 0=success, non-zero failed
            Y1 represent the DemodMargin
            Y2 represent the NbGateways
            Y3 represent the RSSI of the command’s download
            Y4 represent the SNR of the command’s download

        :return: dict: {"status":int(Y0),"demod margin": int(Y1), "num gateways": int(Y2), "RSSI": int(Y3), "SNR": int(Y4)}
        """
        res=self._inquire("CLINKCHECK")
        Y0, Y1, Y2, Y3, Y4 = res.split(",", 5)
        # turn the results into human readable form
        return {"status":{int(Y0)},"demod margin": int(Y1), "num gateways": int(Y2), "RSSI": int(Y3), "SNR": int(Y4)}

    def enableADR(self,enable:int=1) -> bool:
        """
        enable or disable Adaptive Data Rate adjustment.

        :param enable: 1 to enable, 0 to disable
        :return: Nothing
        """
        assert 0<=enable<=1,"ADR enable must be 1 or 0"
        return self._setCmd(f"CADR={enable}")

    def disableADR(self) -> bool:
        """
        disable Adaptive Data Rate adjustment.

        :return: True/False
        """
        return self.enableADR(0)

    def getADR(self) -> int:
        """
        Return adaptive data rate setting

        1=enabled, 0=disabled

        :return: int: <setting>
        """
        return self._noneOrInt(self._inquire("CADR"))

    def setReceiveWindowParameters(self,RX1DRoffset:int,RX2DataRate:int,RX2Frequency:int)-> bool:
        """
        Must allow transmitter to settle after changing frequency

        :param RX1DRoffset: int: 0..5
        :param RX2DataRate: int: 0..5
        :param RX2Frequency: int: nnnmmmooo like 868000000
        :return: bool: True/False
        """
        assert 0<=RX1DRoffset<=5,"RX1DRoffset range error, should be 0..5"
        assert 0<=RX2DataRate<=5,"RX2DataRate range error, should be 0..5"
        #TBD the following range depends on Frequency Plan
        assert 433000000<=RX2Frequency<=999000000,"RX2Frequency range error"
        # TBD warning, need to check frequency is in the table
        return self._setCmd(f"CRXP={RX1DRoffset},{RX2DataRate},{RX2Frequency}")

    def getReceiveWindowParameters(self) -> dict:
        """
        returns the current datarate settings for RX1 and RX2 plus RX2 frequency

        :return: dict {"RX1Datarate":int(p[0]),"RX2Datarate":int(p[1]),"RX2Frequency":int(p[2])}
        """
        res=self._inquire("CRXP")
        p=res.split(",",3)
        return {"RX1Datarate":int(p[0]),"RX2Datarate":int(p[1]),"RX2Frequency":int(p[2])}

    def setFrequencyTable(self,ULDL:int,method:int,number:int,freqlist:str) -> None:
        """
        optional and not supported anyway, according to the AT Command introduction document

        :param ULDL:   int: 1=UL, 2=DL
        :param method: int: 1 autogenerated according to start freq & num channels
                            2 set logical channel frequency explicitly
        :param number: int: number of channels 1..16
        :param freqlist: str: if method is 1 this is the start frequency otherwise a comma separated list of frequencies
        :return: Nothing
        """
        raise ASRUnsupported

    def setRX1Delay(self, delay:int=5) -> bool:
        """
        set the RX1Delay. By default this is 5s with EU868

        Call before sending messages.

        :param delay: int: delay after tx to catch RX1 window (normally 5s)
        :return: True/False
        """
        assert delay<=5,"RX1DELAY should be 5 seconds to catch the RX1 window which starts at 5s."
        return self._setCmd(f"CRX1DELAY={delay}")

    def getRX1Delay(self) -> int:
        """
        get the current RX1Delay (seconds)

        :return: int:<RX1 delay>
        """
        return self._noneOrInt(self._inquire("CRX1DELAY"))

    def saveMacConfiguration(self) -> bool:
        """
        save current MAC settings to eeprom.

        :return: True/False
        """
        return self._setCmd(f"CSAVE")

    def restoreMacConfiguration(self) -> bool:
        """
        restore the last saved MAC settings.

        Normally restore at program start before sending

        :return: True/False
        """
        return self._setCmd(f"CRESTORE")

    def getPingSlotInfo(self)-> int:
        """
        Only valid for Class B but returned regardless

        Ping slot info is the periodicity: See :-
        https://lora-developers.semtech.com/documentation/tech-papers-and-guides/lorawan-class-b-devices

        :return: int: ping slot periodicity
        """
        return self._noneOrInt(self._inquire("CPINGSLOTINFOREQ"))

    def setPingSlotPeriod(self,period) -> bool:
        """
        only used in class B mode

        :param period: int:
        :return: None
        """
        deviceClass=self.getClass()

        assert deviceClass==2,"setPingSlotPeriod() error device should be class B"

        return self._setCmd(f"CPINGSLOTINFOREQ={period}")


    #####################################################
    # Misc other commands
    #####################################################

    def reboot(self,mode:int=0):
        """
        reboot the LoRaWAN unit

        supported modes
            0   immediate reboot
            1   wait till current transmission has finished
            7   reboot and enter boot loader

        :return: Nothing
        """
        if mode in [0,1,7]:
            # the unit returns a lot of messages when loglvl >0
            # also CJOIN:OK/OK+SENT,OK+RECV
            self._setCmd(f"IREBOOT={mode}")
            self._getDelayedReplies()
        else:
            self.logger.error("Invalid boot mode {mode}.")
            raise ASRInvalidBootMode

    def setLogLevel(self,level:int) -> bool:
        """
        set current log level.

        This library turns logging off when instantiated to
        make it easier to analyse messages received following
        AT commands.

        0 - disable log information
        1..5 increasing verbosity

        :param level: int 0..5
        :return: nothing
        """
        assert 0<=level<=5,"Invalid LOGLVL velue. Range 0..5"
        return self._setCmd(f"ILOGLVL={level}")

    def getLogLevel(self) -> int:
        """
        Get current log level which ranges 0..5 where 0 means off and higher
        values mean higher verbosity

        :return: int:<LOGLVL> 0..5
        """
        return self._noneOrInt(self._inquire("ILOGLVL"))
    #####################################################
    # Private commands
    #####################################################

    def enableLowPower(self,enable:int=1) -> bool:
        """
        Puts the ASR6501 into low power mode.

        :param enable: int 1=enable,
        :return: bool: True/False
        """
        return self._setCmd(f"CLPM={enable}")

    def setKeysProtect(self,protect:int=1)->bool:
        """
        protect current keys

        :param protect: int: 1 (enable) or 0 (disable)
        :return: bool: True/False
        """
        assert 0<=protect<=1,"protect parameter can only be 1 or 0"
        return self._setCmd(f"CKEYSPROTECT={protect}")

    def getKeysProtect(self) -> int:
        """
        Return current state of key protection.

        I assume, in the lack of manuf detail, that protected keys
        cannot be changed (accidentally)

        0 = not protected
        1 = protected

        :return: int:<keysprotect>
        """
        return self._noneOrInt(self._inquire("CKEYSPROTECT"))

    def lowPowerTest(self,which:str)-> bool:
        """
        lowPowerTest

        :param which: str: "sleep","MCU" or "standby"
        :return: bool: True/False
        """
        assert which in ["sleep","MCU","standby"], "Invalid low power test option. Should be one of 'sleep','MCU','standby'"
        if which=="sleep":
            return self._setCmd(f"CSLEEP")
        elif which=="MCU":
            return self._setCmd(f"CMCU")
        elif which=="standby":
            return self._setCmd(f"CSTDBY")

    def loraRxTest(self,freq:int,datarate:int) -> bool:
        """
        Continuous receive mode (testing only)

        :param freq: int: hz e.g. 865000000
        :param datarate: int:0..5
        :return: bool: True/False
        """
        #TBD need to limit to current frequency plan?
        assert 150000000<=freq<=960000000,"Freq must be in range 150000000..960000000"
        assert 0<=datarate<=5,"datarate must be in range 0..5"
        return self._setCmd(f"CRX={freq},{datarate}")

    def loraTxTest(self,freq:int,datarate:int,power:int) -> bool:
        """
        transmit test, loops once per second (testing only)

        :param freq: int: 150000000..960000000
        :param datarate: int: range 0..5
        :param power: int: range 0..22
        :return: bool: True/false
        """
        assert 150000000<=freq<=960000000,"Freq must be in range 150000000..960000000"
        assert 0<=datarate<=5,"datarate must be in range 0..5"
        assert 0<=power<=22,"Power must be in range 0..22"
        return self._setCmd(f"CTX={freq},{datarate},{power}")

    def loraTxContinuousTest(self,freq:int,power:int,paOpt:int=0) -> bool:
        """
        Continuous transmit test

        :param freq: int: 150000000..960000000
        :param power: int: range 0..22
        :param paOpt: int: default 0 range 0..3 Power amp optimal setting
        :return: bool: True/False
        """
        assert 150000000<=freq<=960000000,"Freq must be in range 150000000..960000000"
        assert 0<=power<=22,"Power must be in range 0..22"
        assert 0 <= paOpt <= 3, "Power amp opt must be in range 0..3"
        return self._setCmd(f"CTXCW={freq},{power},{paOpt}")

if __name__ == "__main__":
    import serial
    BAUD=115200
    LOG_LEVEL=logging.DEBUG

    logging.basicConfig(filename="ASR6501.log", format='%(asctime)s - %(name)s - %(lineno)d - %(levelname)s - %(message)s')

    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    logger.info("#########################")
    logger.info("Loading ASR6501")
    logger.info("#########################")

    ser=serial.Serial(port="COM5" ,baudrate=BAUD ,parity=serial.PARITY_NONE,bytesize=serial.EIGHTBITS ,timeout=2 ,stopbits=serial.STOPBITS_ONE)

    LoRaWAN=asr6501(ser,LOG_LEVEL)