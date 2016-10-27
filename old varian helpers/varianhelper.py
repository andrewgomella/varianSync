#!/usr/bin/env python
"""
CHANGELOG:
10/01/2013  (AP) added beep sounds which turn on when the x-ray is firing. This is run in a seperate thread.
01/14/2016  (AP) added devIocStats related records:STARTTOD, TOD, KERNEL_VERS, UPTIME, HOSTNAME, RECORD_CNT
                 EPICS_VERS, APP_DIR1, ENGINEER, LOCATION for iocAdmin

"""

from pcaspy import Driver, SimpleServer, cas
import time, threading
from PyDAQmx import *
from epics import *
import numpy as np
import winsound
import datetime, os, sys, socket, platform

# Global variables
EXPERIMENT       = 'RAD'
XRAY_IOC         = EXPERIMENT + ':' + 'OXFORD:xray:'
DET_IOC          = EXPERIMENT + ':' + 'VARIAN:cam1:'
VARIAN_DAQ   = 'paxscanSync'
# NIDAQ line constants
EXP_OK          = VARIAN_DAQ + "/port0/line1"
EXP_REQ         = VARIAN_DAQ + "/port0/line0"

# Varian PaxScan 3024M process variables
VARIAN_PV       = PV('RAD:VARIAN:cam1:Acquire', callback = True)
VARIAN_FILENAME = PV('RAD:VARIAN:TIFF1:FileName', callback = True)
VARIAN_RAD      = PV('RAD:VARIAN:cam1:VarianMode', callback = True)
VARIAN_CONFIG   = PV('RAD:VARIAN:cam1:VarianConfig', callback = True)
XRAY_ON_MONITOR = PV(XRAY_IOC + 'WATT_RBV', callback = True)

# Additional PVs for x-ray sync and save motor/ps/xray PVs
prefix = 'RAD:VarianSync:'
pvdb = {
    'PaxscanShutter'     : { 'asyn' : True }, # switches to 1 only when finished
    'ExpOk'      : {},
    'XSYNC'      : {'type'  : 'enum',
                    'enums' : ['OXFORD-96004']},
    'OxfordSyncMode': {'type'  : 'enum',
                    'enums' : ['Pulse', 'Continuous']},
    'DarkMode': {},
    ##############################################################
    # values relating to IOC admin/restart
    # these record names are the same as DevIocStats record names
    ##############################################################
    'STARTTOD'           : {'type': 'string'},
    'TOD'                : {'type': 'string',
                            'scan': 1 },
    'KERNEL_VERS'        : {'type': 'string'},
    'HOSTNAME'           : {'type': 'string'},
    'RECORD_CNT'         : {'type': 'int'},
    'UPTIME'             : {'type': 'string',
                            'scan': 1 },
    'EPICS_VERS'         : {'type': 'string'},
    'APP_DIR1'           : {'type': 'string'},
    'SysReset'           : {'asyn': True},
    'ENGINEER'           : {'type': 'string'},
    'LOCATION'           : {'type': 'string'},
}

class myDriver(Driver):
    def  __init__(self):
        super(myDriver, self).__init__()
#       initialize static pv's 
        self.start_time = datetime.datetime.now()
        self.up_time_mm = 0
        self.up_time_hh = 0
        self.setParam('STARTTOD', str(datetime.datetime.now().strftime("%m/%d/%Y %H:%M:%S")))
        self.setParam('ENGINEER', 'Andrew Gomella')
        self.setParam('LOCATION', 'B1D521C DT-DELLWIN100')
        self.setParam('HOSTNAME', str(socket.gethostname()))
        self.setParam('RECORD_CNT', len(pvdb.keys()))
        self.setParam('EPICS_VERS', str(cas.EPICS_VERSION_STRING))
        self.setParam('APP_DIR1', str(os.getcwd()))
        self.setParam('UPTIME', str(self.start_time))
        self.setParam('KERNEL_VERS', str(platform.system() + " " + platform.release()))

        self.shutter=0 #signal that the ADShutter Open
        self.zero = numpy.zeros((1,), dtype=numpy.uint8)
        self.one = numpy.ones((1,), dtype=numpy.uint8)
        self.written = int32() #needed for daqmx writes
        self.configValue = VARIAN_CONFIG.get() #initialize configValue
        self.darkMode = 0 #init darkmode to off
        
        #Set up the DI task to check the ExposeOK output from the Varian
        self.ExpOkInHandle = TaskHandle()
        DAQmxCreateTask("",byref(self.ExpOkInHandle))
        DAQmxCreateDIChan(self.ExpOkInHandle, EXP_OK, "", DAQmx_Val_ChanForAllLines)
        DAQmxStartTask(self.ExpOkInHandle)

        #Set up the DO task to send the ExposeRequest signal to the Varian. 
        # Must be "active drive" or it will not work.
        self.ExpReqOutTask = TaskHandle()
        DAQmxCreateTask("",byref(self.ExpReqOutTask))
        DAQmxCreateDOChan(self.ExpReqOutTask, EXP_REQ, "", DAQmx_Val_ChanForAllLines)
        DAQmxSetDOOutputDriveType(self.ExpReqOutTask, EXP_REQ, DAQmx_Val_ActiveDrive);
        DAQmxStartTask(self.ExpReqOutTask)
        #make sure expreq is low
        self.setExpReqOutputLow()  

        # reset filenum to zero (feature requested by users)
        #VARIAN_FILENAME.add_callback(self.resetFileNum)
        # monitor x-ray firing pv 
        XRAY_ON_MONITOR.add_callback(self.monitorXrayFlux)

        # keep track of whether we are in rad or fluoro mode
        VARIAN_CONFIG.add_callback(self.configChange)

    # Waits for ExpOk Output from Paxscan to be 1, return when true
    def waitForExpOkOn(self):
        while True:
            newVal = np.array([0], dtype=np.uint8)
            DAQmxReadDigitalLines(self.ExpOkInHandle, 1, 1, 0,  newVal, 1, None, None, None)
            if newVal == self.one:
                break
            time.sleep(.001)
        self.timeOn = time.time()

    # Waits for ExpOk Output from Paxscan to be 0, return when true
    def waitForExpOkOff(self):
        while True:
            newVal = np.array([0], dtype=np.uint8)
            DAQmxReadDigitalLines(self.ExpOkInHandle, 1, 1, 0,  newVal, 1, None, None, None)
            if newVal == self.zero:
                break
            time.sleep(.001)
        print "ExpOk was on for ", time.time()- self.timeOn

    def waitForShutterOff(self):
        while self.shutter == 1:
            time.sleep(.001)

    # Set ExpReq high, this basically tells the panel we are ready to X-ray on demand
    def setExpReqOutputHigh(self):
        try:
            DAQmxWriteDigitalLines(self.ExpReqOutTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.one,self.written, None)
        except DAQError as err:
            print "DAQmx Error: %s"%err

    # Set ExpReq low, timing of this is not important 
    def setExpReqOutputLow(self):
        try:
            DAQmxWriteDigitalLines(self.ExpReqOutTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.zero,self.written, None)
        except DAQError as err:
            print "DAQmx Error: %s"%err
   
    # Returns when oxford is on AND outputting x-ray at set values
    def startupOxford(self):
        if self.darkMode==0:
            #if x-ray was just cold-booted, wait for warm up to finish
            while caget(XRAY_IOC + 'STATUS_RBV') == 0:
                time.sleep(.01)
            if caget(XRAY_IOC + 'STATUS_RBV') == 1:
                caput(XRAY_IOC + 'ON', '1')         
                #wait for output from x-ray
                while caget(XRAY_IOC + 'STATUS_RBV') != 2:
                    time.sleep(.01)
            elif caget(XRAY_IOC + 'STATUS_RBV') == 3:
                caput(XRAY_IOC + 'PULSE_MODE', '0')         
            # wait for kvp and watts to hit setpoints
            while (caget(XRAY_IOC + 'KVP_RBV') != caget(XRAY_IOC + 'KVP')) and (caget(XRAY_IOC + 'WATT_RBV') != caget(XRAY_IOC + 'WATT')):
                time.sleep(.01)

    # Immediate stop x-ray flux
    def stopXrayFlux(self):
        if self.darkMode==0:
            if self.getParam('OxfordSyncMode') == 1:
                 #if in continuous mode completely shut off x-ray
                 caput('RAD:OXFORD:xray:OFF', '1')
            else:
                #if in pulse mode turn pulse mode on so x-ray output stops
                caput(XRAY_IOC + 'PULSE_MODE', '1')      

    # Determine if it is time to shut off x-ray (cool-down)
    def resumeXray(self):
        # TODO:
        #are we scanning? are there more images left in epics sequence?
        #if not, shut x-ray off (kv to zero), otherwise continue without turning
        # x-ray off
        pass

    # Signal sent from ADShutter when it requests x-ray output (ASAP)
    def paxscanShutterOpen(self):
        # checks if x-ray is ready, returns when it is ready
        self.startupOxford()  
        print "xray ready and outputting at setpoints" 
        self.setExpReqOutputHigh() 
        print "Sent Expose Request, waiting for Expose Ok signal from Paxscan"
        self.waitForExpOkOn()      
        #the following has problems when switching from cal to acquisitions
        #if VARIAN_PV.get() == 1: #normal acquire in progress, wait for expok
        #    self.waitForExpOkOn()      
        #    print "Expose Ok received"        
        if caget('RAD:VARIAN:cam1:DoubleMode') == 1:
            print "DoubleMode: waiting for setShutter(0) from ADVarian to shut off x-ray"
            #(or if doing gain fluoro cal?)
            self.waitForShutterOff() 
        else:
            print "Normal exposure: waiting for Expose Ok from Paxscan to go to 0"
            self.waitForExpOkOff()
        self.stopXrayFlux()        # turns off x-ray output 
        print "x-ray off"
        self.setExpReqOutputLow()  
        print "expreq now low"
        #self.resumeXray()          # decides whether to leave x-ray "enabled"

    def read(self, reason):
        """
        pcaspy native read method
        """
        format_time = ""
        if reason == 'UPTIME':
            format_time = datetime.datetime.now() - self.start_time
            value  = str(format_time).split(".")[0] 
        elif reason == 'TOD':
            value = str(datetime.datetime.now().strftime("%m/%d/%Y %H:%M:%S"))    
        else: 
            value = self.getParam(reason)
        return value

    def write(self, reason, value):
        global XRAY_IOC
        if reason == 'PaxscanShutter': 
            self.shutter = value
            # only call function if in rad mode
            if self.configValue == 0 : 
                print "Radmode Shutter Signal"
                if value == 1:
                    # start rad mode sequence
                    self.tid = threading.Thread(target = self.paxscanShutterOpen, args = ())
                    self.tid.start()
            else:
                print "Fluoro Shutter Signal"
                if value == 1:
                    #xray on
                    self.startupOxford()
                    self.setParam(reason, value)
                elif value == 0:
                    #xray off
                    self.stopXrayFlux()
                    self.setParam(reason, value)
        elif reason == "XSYNC":
           XRAY_IOC = EXPERIMENT + ':' + 'OXFORD:xray:'
           print XRAY_IOC
        elif reason == "DarkMode":
           self.darkMode=value
        self.setParam(reason, value)
        self.updatePVs()

    # This function will be called back when user switches Varian Config to either
    #  0- Radiography mode, or 1- Fluoroscopy Mode
    # Syncing is very different for these modes so we need to keep track of this
    def configChange(self, value=None, **kw):
        self.configValue = value

    def monitorXrayFlux(self, **kw):
        self.mid = threading.Thread(target = self.warning_sound, args=())
        self.mid.start()

    def warning_sound(self):
        while (caget(XRAY_IOC + 'WATT_RBV') > 0):
            winsound.Beep(1318, 500)
        self.mid = None

#    def resetFileNum(self, **kw):
#        self.rid = threading.Thread(target = self.reset, args=())
#        self.rid.start()

#    def reset(self):
#        caput(EXPERIMENT + ':VARIAN:TIFF1:FileNumber', 0)
#        self.rid = None

if __name__ == '__main__':
    server = SimpleServer()
    server.createPV(prefix, pvdb)
    driver = myDriver()
    print "VarianSync now online"
    # process CA transactions
    while True:
        server.process(0.1)
