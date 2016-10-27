#!/usr/bin/env python

from pcaspy import Driver, SimpleServer
import time
from PyDAQmx import *
import numpy as np
from epics import *
import threading

# Global variables
EXPERIMENT = 'IPL'
PV_LIST =  ['IPL:SRI:xray:UAS']
XRAY_IOC = EXPERIMENT + ':' + 'SRI:xray:'
PS_IOC =   EXPERIMENT + ':' + 'AGILENT:dcps:'
# Varian 'FIRE' enum
VARIAN_PV = PV('IPL:VARIAN:cam1:Acquire', callback = True)
VARIAN_FILENAME = PV('IPL:VARIAN:TIFF1:FileName', callback = True)
VARIAN_RAD = PV('IPL:VARIAN:cam1:VarianMode', callback = True)
# Additional PVs for x-ray sync and save motor/ps/xray PVs
prefix = 'VarianSync:'
pvdb = {
    'ExpReq'     : { 'asyn' : True },
    'ExpOk'      : {},
    'DOC'        : { 'type' : 'enum',
                     'enums': ['ON', 'OFF'] },
    'ExpTime'    : { 'type' : 'int' }
}

class myDriver(Driver):
    def  __init__(self):
        super(myDriver, self).__init__()
        self.time1=0
        self.time2=0
        self.zero = numpy.zeros((1,), dtype=numpy.uint8)
        self.one = numpy.ones((1,), dtype=numpy.uint8)
        self.written = int32()
        
        #Set up the DI task to check the ExposeOK output from the Varian
        self.taskHandle = TaskHandle()
        DAQmxCreateTask("",byref(self.taskHandle))
        DAQmxCreateDIChan(self.taskHandle,"Dev2/port0/line1", "", DAQmx_Val_ChanForAllLines)
        DAQmxStartTask(self.taskHandle)

        #Set up the DO task to send the ExposeRequest signal to the Varian. This has to be "active drive" or it will not work.
        self.taskHandle2 = TaskHandle()
        DAQmxCreateTask("",byref(self.taskHandle2))
        DAQmxCreateDOChan(self.taskHandle2,"Dev2/port0/line0", "", DAQmx_Val_ChanForAllLines)
        DAQmxSetDOOutputDriveType(self.taskHandle2,"Dev2/port0/line0", DAQmx_Val_ActiveDrive);
        DAQmxStartTask(self.taskHandle2)

        #Start thread to continuously poll the ExposeOK signal
        self.xid = threading.Thread(target=self.pollexpok,args=())
        self.xid.start()
        #Start a thread when the camera is fired to save the PV values to .txt file
        VARIAN_PV.add_callback(self.checkDoc)
        VARIAN_FILENAME.add_callback(self.resetFileNum)
        VARIAN_RAD.add_callback(self.updateExp)
    #since the new ni daq 6501 doesnt support change detection, we have to poll and do change detection ourselves
    def pollexpok(self):
        x = np.array([0], dtype=np.uint8)
        oldval = np.array([0], dtype=np.uint8)
        DAQmxReadDigitalLines(self.taskHandle, 1, 1, 0, oldval, 1, None, None, None)
        while True:
            DAQmxReadDigitalLines(self.taskHandle, 1, 1, 0,  x, 1, None, None, None) 
            time.sleep(0.1)   
       #     print oldval,x
            if oldval != x:
                if VARIAN_PV.get == 1:
                    if x == 1:
                        print "X-ray ON"
                        self.time1 = time.time()
                        caput(XRAY_IOC + 'ON', 1)
                        caput('VarianSync:ExpOk', 1)
                        temp = self.time1 - self.time2
                        if temp > 1000000:
                            print "Time since x-ray off ", "IDKLOL", " seconds"
                        else:
                            print "Time since x-ray off ", temp , " seconds"
                    elif x == 0:
                        print oldval,x
                        print "X-ray OFF"
                        self.time2 = time.time()
                        print "X-ray was on for ", self.time2 - self.time1, " seconds"
                        caput(XRAY_IOC + 'ON', 0)
                        caput('VarianSync:ExpOk', 0)
                #refresh the oldval
                DAQmxReadDigitalLines(self.taskHandle, 1, 1, 0, oldval, 1, None, None, None)
            time.sleep(.001)

    def write(self, reason, value):
        #take proper actions
        if reason == 'ExpReq' and value == 1:
            self.tid = threading.Thread(target = self.writeone, args = ())
            self.tid.start()
        elif reason == 'ExpReq' and value == 0:
            self.bid = threading.Thread(target = self.writezero, args = ())
            self.bid.start()
        elif reason == 'DOC':
            self.setParam(reason, value)
        elif reason == 'ExpTime':
            self.setParam(reason, value)
            self.eid = threading.Thread(target = self.calcExp, args = ())
            self.eid.start()

        self.setParam(reason, value)
        self.updatePVs()
   
    def writeone(self):
        time.sleep(.1)
        DAQmxWriteDigitalLines(self.taskHandle2,1,1,10.0,DAQmx_Val_GroupByChannel,self.one,self.written, None)
    
    def writezero(self):
        DAQmxWriteDigitalLines(self.taskHandle2,1,1,10.0,DAQmx_Val_GroupByChannel,self.zero,self.written, None)

    def calcExp(self):
        radMode = caget('IPL:VARIAN:cam1:VarianMode')
        if radMode == 1:
            noExp =  int(self.getParam('ExpTime')/2)
            caput('IPL:VARIAN:cam1:NumImages', noExp)
            caput('IPL:VARIAN:Proc1:NumFilter', noExp)
        elif radMode == 2:
            noExp =  int(self.getParam('ExpTime')/4)
            caput('IPL:VARIAN:cam1:NumImages', noExp)
            caput('IPL:VARIAN:Proc1:NumFilter', noExp)
        elif radMode == 3:
            noExp =  int(self.getParam('ExpTime')/6)
            caput('IPL:VARIAN:cam1:NumImages', noExp)
            caput('IPL:VARIAN:Proc1:NumFilter', noExp)
        else:
            self.eid = None
            self.ueid = None

    def updateExp(self, **kw):
        self.ueid = threading.Thread(target = self.calcExp, args = ())
        self.ueid.start()

    
    def saveParams(self):

            val=[]
            path = caget('IPL:VARIAN:TIFF1:FilePath', as_string=True)
            fname = caget('IPL:VARIAN:TIFF1:FileName', as_string=True)
            fnum = caget('IPL:VARIAN:TIFF1:FileNumber', as_string=False)
            trueFnum = str(fnum - 1)
            # Check if user entered a trailing \
            if path[-1] != '\\' and path != 'C:\\':
                print "Talbot paramaters file is saved at the following location:\n", path + '\\'
                f = open(path + '\\' + fname + '_' +trueFnum + str(time.strftime("_%H%M%S")) + '.txt', 'w')
            else:
                print "Talbot paramaters file is saved at the following location:\n", path
                f = open(path + fname + '_' + trueFnum + str(time.strftime("_%H%M%S")) + '.txt', 'w')
            # Save mps motor positions, Xray kVP and uA, Supply current and Voltage
            for pvs in PV_LIST:
                val.append(caget(pvs))
            for i,j in zip(PV_LIST, val):
                if('.' in i):
                    f.write(i.split('.')[0] + ':\t' + str(j) + ' ' + caget(i.split('.')[0]+'.EGU') + '\n')
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