#!/usr/bin/env python3
import math
import time
import board
import busio
try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus
import adafruit_ina260
from gpiozero import DigitalOutputDevice
from MOSParameters import *
#################################################################
#################################################################
# PARAMETERS
# Set Poll interval to at least 5 minutes (300s) to prevent
# excessive on/off of charging circuits
VERSION="0.01b"
MONITORPOLLINTERVAL=5 #seconds
MIN_VOLTAGE=12.5 #minimum Pb battery voltage to allow charging
MIN_CAM_PC=70 #minimum percentage of camera charge before charging
MIN_MULTI_CHARGE=13.0 #minimum Pb battery voltage to charge two cams
CAM_CHARGED=100
CONTROLROOT="climateMonitor/"
################################################################
################################################################
# Measurements made 
# List of tuples (key, factor, unit)
MEASUREMENTS = \
[('Current',0.001,'A'), \
 ('Voltage',1,'V'), \
 ('Power',0.001,'W')]
################################################################
################################################################
# Camera batteries to charge
# List of tuples (camera_entity_id, control_relay_index)
#CAMERABATTS=[('winter_field', 1), ('summer_field', 0)]
BATTROOT='/battPerCent'

################################################################
################################################################
# Class to represent a camera
# - manages reading camera state from MQTT
# class Camera(intMosParameter):

    # def __init__(self,entityID,chargePC,mosClient,relay):
        # intMosParameter.__init__(self,entityID,mosClient,chargePC)
        # self._relay = relay

    # #define the comparison operators so that can be sorted in list
    # def __lt__(self, other):
        # return self.chargePC() <other.chargePC()

    # def __eq__(self, other):
        # return self.chargePC() == other.chargePC()

    # def __le__(self, other):
        # return self.chargePC() <= other.chargePC()

    # def __ge__(self, other):
        # return self.chargePC() >= other.chargePC()

    # def __ne__(self, other):
        # return self.chargePC() != other.chargePC()

    # def chargePC(self):
        # return intMosParameter.value(self)

    # def relay(self):
        # return self._relay
################################################################
################################################################
# set up the parameter objects
def getParameters(handle):
    parameters = {}
    # parameters['PbVMin']=floatMosParameter('PbVMin',handle,MIN_VOLTAGE)
    # parameters['MinCamPC']=intMosParameter('MinCamPC',handle,MIN_CAM_PC)
    # parameters['PbMultiV']=floatMosParameter('PbMultiV',handle,MIN_MULTI_CHARGE)
    parameters['MonPoll']=intMosParameter('MonPoll',handle,MONITORPOLLINTERVAL,BATTROOT)
    return parameters
################################################################
################################################################
# set up the camera objects
# - handle is the MQTT client object
# def getCameras(cams,handle):
    # cameras = []
    # for (id,relay) in cams:
       # cameras.append(Camera(id,100,handle,relay))
    # return cameras

################################################################
################################################################
# def setupRelays():
    # pins = [ 23, 24]
    # relays = []
    # for pin in pins:
        # relays.append(DigitalOutputDevice(pin,initial_value=False))
    # return relays
#################################################################
#################################################################
# Setup for single conversions
def setupINA260(logger):
    i2c = busio.I2C(1, 0)
    devices = i2c.scan()
    if not devices:
        logger.log ("Fatal Error: No devices found on i2C bus.",error=True)
        logger.log ("Terminating.",error=True)
        exit(1)
    ina260 = adafruit_ina260.INA260(i2c)
    return ina260
##############################################################
##############################################################
#Easy access to desired property
def ina260Get(ina260,property):
    if property == 'Voltage':
        value = ina260.voltage
    if property == 'Current':
        value = ina260.current
    if property == 'Power':
        value = ina260.power
    return value
##############################################################
##############################################################
# MAIN PROGRAM
logger = doLog()
logger.log("Monitor Climate v"+VERSION,error=True)
doExit = sigExit(logger)
logger.log("Setting up hardware: ", end="")
logger.log(" INA260 ",end="")
ina260 = setupINA260(logger)
#last_values are values after scaling
last_values={}
for (key,factor,unit) in MEASUREMENTS:
    last_values[key]=0.0
#logger.log(" relays ", end="")
#relays = setupRelays()
logger.log(" done.")
logger.log("Setting up MQTT ...", end="")
#setup MQTT
mosClient = setupMQTT(BATTROOT)
if mosClient.connected_flag:
    #cameras = getCameras(CAMERABATTS,mosClient)
    parameters = getParameters(mosClient)
    logger.log(" done.")
    while not doExit.isSet():
        for (key,factor,unit) in MEASUREMENTS:
            value = ina260Get(ina260,key) * factor
            #only publish changes
            if value != last_values[key]:
                last_values[key]= value
                publishMQTT(mosClient,key,value,CONTROLROOT)
        #If Pb battery charged enough to support cameras?
        pbBattVolts = last_values['Voltage']
        logger.log("pbBatt : {:.3f} V".format(pbBattVolts))
        #if pbBattVolts>=parameters['PbVMin'].value():
            #does anything need charging? 
            #v 1.07 include those already charging (relay.value > 1) so that
            #the charging focus is always on the least charged camera
            #it also allows multiple charging to be stopped if the voltage drops
            #below the multiple charging level            
            # toCharge = []
            # turnOff=[]
            # for c in cameras:
                # if (c.chargePC() < parameters['MinCamPC'].value()) or (relays[c.relay()].value > 0 and c.chargePC() < CAM_CHARGED ):
                    # toCharge.append(c)
                # elif c.chargePC() == CAM_CHARGED:
                    # turnOff.append(c)
            # if toCharge:
                # toCharge.sort()
                # relays[toCharge[0].relay()].on()
                # toCharge.pop(0)
                # if toCharge and pbBattVolts >= parameters['PbMultiV'].value():
                    # while toCharge:
                        # relays[toCharge[0].relay()].on()
                        # toCharge.pop(0)
                # elif toCharge:
                    # #turn off more than one
                    # while toCharge:
                        # relays[toCharge[0].relay()].off()
                        # toCharge.pop(0)
            # #turn off anything that is on and charged
            # while turnOff:
                # relays[turnOff[0].relay()].off()
                # turnOff.pop(0)
        #else: #need to turn off camera charging
            # for r in relays:
                # r.off()
        # for c in cameras:
            # logger.log("{:12} {:3}%".format(c.entityID(),c.chargePC()))
        doExit.wait(parameters['MonPoll'].value())
else:
    logger.log("Connection failed rc=",str(mosClient.connected_rc),error=True)
    syslog.closelog()
    exit(mosClient.connected_rc)
logger.log("Exiting")
finishMQTT(mosClient)
logger.close()
exit(0)
