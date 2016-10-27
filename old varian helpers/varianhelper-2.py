#!/usr/bin/env python

from pcaspy import Driver, SimpleServer
import time
from PyDAQmx import *
import numpy as np
from epics import *
import threading

# Global variables
EXPERIMENT       = 'IPL'
XRAY_IOC         = EXPERIMENT + ':' + 'SRI:xray:'
PS_IOC           = EXPERIMENT + ':' + 'AGILENT:dcps:'
DET_IOC          = EXPERIMENT + ':' + 'VARIAN:cam1:'
PV_LIST          = [ XRAY_IOC + 'UAS',                        \
                     XRAY_IOC + 'KVPS',                       \
                     EXPERIMENT + ':' + 'NEWPORT:m3.VAL',     \
                   ]
                    
# ADVarian Specific PVs
VARIAN_PV       = PV('IPL:VARIAN:cam1:Acquire', callback = True)
VARIAN_FILENAME = PV('IPL:VARIAN:TIFF1:FileName', callback = True)
VARIAN_RAD      = PV('IPL:VARIAN:cam1:VarianMode', callback = True)

# NIDAQ lines
EXP_OK          = "paxscanSync/port0/line1"
EXP_REQ         = "paxscanSync/port0/line0"
USER_IN         = "paxscanSync/port0/line2"

# PV database for x-ray sync, selecting x-ray, and saving motor/ps/xray PVs
#prefix = EXPERIMENT + ':' + 'VARIAN:' 
prefix = 'VarianSync:'
pvdb = {
#   PV base names           VALUES
    'ExpReq'       : { 'asyn' : True },
    'ExpOk'        : {},
    'DOC'           : { 'type' : 'enum',
                        'enums': ['ON', 'OFF'] },
    'EXP_TIME'      : { 'type' : 'int' },
    'XSYNC'         : { 'type'  : 'enum',
                        'enums' : ['SRI-SB-80-1K', 'OXFORD-96004'] },
    }

class myDriver(Driver):
    def  __init__(self):
        super(myDriver, self).__init__()
        self.time1 = 0
        self.time2 = 0
        self.zero = numpy.zeros((1,), dtype=numpy.uint8)
        self.one = numpy.ones((1,), dtype=numpy.uint8)
        self.written = int32()
        
        # Set up the DI task to check the ExposeOK output from the Varian
        self.taskHandle = TaskHandle()
        DAQmxCreateTask("", byref(self.taskHandle))
        DAQmxCreateDIChan(self.taskHandle, EXP_OK, "", DAQmx_Val_ChanForAllLines)
        DAQmxStartTask(self.taskHandle)

        # Set up the DO task to send the ExposeRequest signal to the Varian. 
        # This has to be "active drive" or it will not work.
        self.taskHandle2 = TaskHandle()
        DAQmxCreateTask("", byref(self.taskHandle2))
        DAQmxCreateDOChan(self.taskHandle2, USER_IN, "", DAQmx_Val_ChanForAllLines)
        DAQmxSetDOOutputDriveType(self.taskHandle2, USER_IN, DAQmx_Val_ActiveDrive);
        DAQmxStartTask(self.taskHandle2)

        # Start thread to continuously poll the ExposeOK signal
        self.xid = threading.Thread(target=self.pollexpok,args=())
        self.xid.start()
        
        # Start a thread when the camera is fired to save PV values to .txt file
        VARIAN_PV.add_callback(self.checkDoc)
        VARIAN_FILENAME.add_callback(self.resetFileNum)
        VARIAN_RAD.add_callback(self.updateExp)
        
        # This works only if the main IOC is started first.
        if VARIAN_RAD.get() == 1:
            self.setParam('EXP_TIME', 2)
        elif VARIAN_RAD.get() == 2:
            self.setParam('EXP_TIME', 4)
        elif VARIAN_RAD.get() == 3:
            self.setParam('EXP_TIME', 6)
    
    def pollexpok(self):
        """
        Since the NI DAQ 6501 doesnt support change detection, 
        we have to poll and do change detection ourselves
        """
        x = np.array([0], dtype=np.uint8)
        oldval = np.array([0], dtype=np.uint8)
        DAQmxReadDigitalLines(self.taskHandle, 1, 1, 0, oldval, 1, None, None, None)
        while True:
            DAQmxReadDigitalLines(self.taskHandle, 1, 1, 0,  x, 1, None, None, None)   
            #print oldval,x
            if oldval != x:
                if x == 1:
                    self.setParam('ExpOk', 1)
                    self.time1 = time.time()
                elif x == 0:
                    self.setParam('ExpOk', 0)
                    print "Frame time: ", time.time() - self.time1 , " seconds"
                DAQmxReadDigitalLines(self.taskHandle, 1, 1, 0, oldval, 1, None, None, None)
                self.updatePVs()
            time.sleep(.001)

    def write(self, reason, value):
        # take proper actions
        global XRAY_IOC
        if reason == 'EXP_REQ' and value == 1:
            self.tid = threading.Thread(target = self.writeone, args = ())
            self.tid.start()
        elif reason == 'EXP_REQ' and value == 0:
            self.bid = threading.Thread(target = self.writezero, args = ())
            self.bid.start()
        elif reason == 'DOC':
            self.setParam(reason, value)
        elif reason == 'EXP_TIME':
            self.setParam(reason, value)
            self.eid = threading.Thread(target = self.calcExp, args = ())
            self.eid.start()
        elif reason == "XSYNC" and value == 0:
           XRAY_IOC = EXPERIMENT + ':' + 'SRI:xray:'
           self.setParam(reason, value)
           print XRAY_IOC
        elif reason == "XSYNC" and value == 1:
           XRAY_IOC = EXPERIMENT + ':' + 'OXFORD:xray:'
           self.setParam(reason, value)
           print XRAY_IOC
        self.setParam(reason, value)
        self.updatePVs()
        
    def writeone(self):
#        time.sleep(.05)
        DAQmxWriteDigitalLines(self.taskHandle2, 1, 1, 10.0, \
                               DAQmx_Val_GroupByChannel, self.one, \
                               self.written, None)
    def writezero(self):
        DAQmxWriteDigitalLines(self.taskHandle2, 1, 1, 10.0, \
                               DAQmx_Val_GroupByChannel, self.zero, \
                               self.written, None)

    def calcExp(self):
        radMode = VARIAN_RAD.get()
        if radMode == 1:
            noExp =  int(self.getParam('EXP_TIME')/2)
            caput('IPL:VARIAN:cam1:NumImages', noExp)
            caput('IPL:VARIAN:Proc1:NumFilter', noExp)
        elif radMode == 2:
            noExp =  int(self.getParam('EXP_TIME')/4)
            caput('IPL:VARIAN:cam1:NumImages', noExp)
            caput('IPL:VARIAN:Proc1:NumFilter', noExp)
        elif radMode == 3:
            noExp =  int(self.getParam('EXP_TIME')/6)
            caput('IPL:VARIAN:cam1:NumImages', noExp)
            caput('IPL:VARIAN:Proc1:NumFilter', noExp)
        else:
            self.eid = None
            self.ueid = None

    def updateExp(self, **kw):
        self.ueid = threading.Thread(target = self.calcExp, args = ())
        self.ueid.start()

    def saveParams(self):
            val = []
            path = caget('IPL:VARIAN:TIFF1:FilePath', as_string=True)
            fname = VARIAN_FILENAME.get(as_string=True)
            fnum = caget('IPL:VARIAN:TIFF1:FileNumber', as_string=False)
            trueFnum = str(fnum - 1)
            # Check if user entered a trailing \
            if path[-1] != '\\' and path != 'C:\\':
                f = open(path + '\\' + fname + '_' +trueFnum + str(time.strftime("_%H%M%S")) + '.txt', 'w')
            else:
                f = open(path + fname + '_' + trueFnum + str(time.strftime("_%H%M%S")) + '.txt', 'w')
            # Save relevant PVs in PV_LIST
            for pvs in PV_LIST:
                val.append(caget(pvs))
            for i,j in zip(PV_LIST, val):
                if('.' in i):
                    f.write(i.split('.')[0] + ':\t' + str(j) + ' ' + caget(i.split('.')[0] +'.EGU') + '\n')
                else:
                    f.write(i + ':\t' + str(j) + '\n')
            f.close()
            self.usid = None

    def checkDoc(self, **kw):
        # If DOC is ON then save PVs in the same folder where image is saved!
        if self.getParam('DOC') == 1 and VARIAN_PV.get() == 0:
            self.usid = threading.Thread(target = self.saveParams, args=())
            self.usid.start()

    def resetFileNum(self, **kw):
        self.rid = threading.Thread(target = self.reset, args=())
        self.rid.start()

    def reset(self):
        caput('IPL:VARIAN:TIFF1:FileNumber', 0)
        self.rid = None

if __name__ == '__main__':
    server = SimpleServer()
    server.createPV(prefix, pvdb)
    driver = myDriver()
    # process CA transactions
    while True:
        server.process(0.1)


#The following block of code is when we were using the USB-6221 daq board which supported change detection. 
# global time1
# global time2
# time1 =0
# time2 = 0
# def ChangeDetectionCallback_py(taskHandle, status, callbackData):
#     print "Change Detected"
#     global time1
#     global time2
#     DAQmxReadDigitalLines(taskHandle, 1, 1, 0,  x, 1, None, None, None)
#     print x
#     if x == 1:
#         print "X-ray ON"
#         time1 = time.time()
#         caput('srxr2:ON',1)
#         caput('VarianSync:ExpOk', 1)
#         print "Time since x-ray off ", time1-time2, " seconds"
#     elif x == 0:
#         print "X-ray OFF"
#         time2 = time.time()
#         print "X-ray was on for ",time2-time1, " seconds"
#         caput('srxr2:ON',0)
#         caput('VarianSync:ExpOk', 0)
#     return 0 # The function should return an integer
#ChangeDetectionCallback = DAQmxSignalEventCallbackPtr(ChangeDetectionCallback_py)
#DAQmxCfgChangeDetectionTiming(taskHandle,"Dev3/port1/line1","Dev3/port1/line1",DAQmx_Val_ContSamps,8)
#DAQmxRegisterSignalEvent(taskHandle,DAQmx_Val_ChangeDetectionEvent,0,ChangeDetectionCallback,None)