#!/usr/bin/env python

"""
PCAS driver to sync Varian PaxScan 3024M Detector with Oxford & SRI X-ray sources.
__author__   =   Andrew A. Gomella
__status__   =   Development R2.1
__to-do__    =   code clean up,
                 PV_LIST for param.txt file needs to be generalized somehow.
                 debug flags, Fix double mode sync
"""
"""
CHANGELOG:
01/09/2016  (AP) added beep sounds which turn on when the x-ray is firing. This is run in a seperate thread.
                 removed reset filenum on file name changes capability from here. This is now handled in 
                 a seperate sequencer thread and is part of the main camera ioc.
01/14/2016  (AP) added devIocStats related records:STARTTOD, TOD, KERNEL_VERS, UPTIME, HOSTNAME, RECORD_CNT
                 EPICS_VERS, APP_DIR1, ENGINEER, LOCATION for iocAdmin
02/05/2016  (AP) added code comments, sync capability for sri x-ray with varian
02/24/2016  (AP) added DOC record to save motor readback values required for imagej alignment scripts
                 removed DarkMode record and instead added "NONE" enum to XSYNC record. If XSYNC record
                 is set to "NONE" then we are effectively in DarkMode. Fixed fluoro mode xray sync issue. 
02/25/2016  (AP) added autosave restore functionality. runtime saving of VarianSync PV's (DOC, XSYNC) is 
                 performed every 30s and PV values are restored during ioc boot. Also fixed issue with setting
                 XRAY_IOC, before it would only set if the PV XSYNC was changed (i.e when the write method is 
                 called). Now we poll for the change (done in read method).
02/29/2016  (AP) When saving parameter file (DOC is ON) the parameter file number exactly matches the tiff image
                 file number now. Previously the file numbers of the tiff image and the parameter .txt file were 
                 different in the sense that the file numbers for the tiff image had zero's prepended. So for eg:
                 if the tiff filename is  test_005.tif then the parameter filename is test_005.txt and NOT test_5.txt
                 Fixed bug in oxford sync when going from standby to output mode during varian acquisition. 
                 Condition was checking for while kvp and watt are != kvp_rbv and watt_rbv. This should be kvp_min
                 and watt_min. To make it so that this works with nova source ray condition is changed to 
                 while (watt_min != watt_rbv - 1 and..):
                 Fixed bug during first acquisition of varian. X-ray wasn't firing for first acquisition. This is 
                 fixed.
03/01/2016  (AP) added function to set high priority for this python process.
03/03/2016  (AP) added some debug prints, fixed PV display for ExpOk.
04/12/2016  (AP) for oxford x-ray, single shots return the xray to standby mode. For scans the xray will go into pulse mode.
                 When the scan is finished, the xray returns to standby mode. This implementation is neccessitated by the 
                 new hardware alarm sound. Removed software based alarm sound.
04/15/2016  (AP) scan detector PV is set to Varian acquire PV during varianSync startup, moved iocstats records to its
                 own seperate function.
                 
"""

from pcaspy import Driver, SimpleServer, cas
from epics import *
from PyDAQmx import *
import numpy as np
import datetime, os, time, psutil
import threading

sys.path.append(os.path.realpath('../utils'))
import epicsApps

EXPERIMENT = 'RAD:'
VARIAN_DAQ                  = 'paxscanSync'               # NIUSB DAQ name (default is Dev0, Dev1 etc)
EXP_OK                      = VARIAN_DAQ + "/port0/line1" # NIDAQ input line which checks for expose ok signal from the Varian 
EXP_REQ                     = VARIAN_DAQ + "/port0/line0" # NIDAQ output line which sends an expose request to the Varian.
# Main IOC records
XRAY_IOC                    = EXPERIMENT + 'OXFORD:xray:'
SCAN_IOC                    = EXPERIMENT + 'SCAN:'
MOTOR_IOC                   = EXPERIMENT + 'NEWPORT:'
DET_IOC                     = EXPERIMENT + 'VARIAN:'
# Varian PaxScan 3024M callback PV's
VARIAN_PV                   = PV(DET_IOC + 'cam1:Acquire', callback = True)
VARIAN_FULL_FILENAME_RBV    = PV(DET_IOC + 'TIFF1:FullFileName_RBV', callback = True)
VARIAN_RAD                  = PV(DET_IOC + 'cam1:VarianMode', callback = True)
VARIAN_CONFIG               = PV(DET_IOC + 'cam1:VarianConfig', callback = True)
VARIAN_IMAGEMODE            = PV(DET_IOC + 'cam1:ImageMode', callback = True)
VARIAN_NUMFILTER            = PV(DET_IOC + 'Proc1:NumFilter', callback = False)

# Scan busy PV's
SCAN_BUSY_1                 = PV(SCAN_IOC + 'scan1.BUSY', callback = True)
SCAN_BUSY_2                 = PV(SCAN_IOC + 'scan2.BUSY', callback = True)
SCAN_BUSY_3                 = PV(SCAN_IOC + 'scan3.BUSY', callback = True)
SCAN_BUSY_4                 = PV(SCAN_IOC + 'scan4.BUSY', callback = True)
SCAN_DETECTOR_1             = PV(SCAN_IOC + 'scan1.T1PV', callback = False)
# If DOC is ON (1) save motor pv's.
MOTOR_IOC_LIST = [
                    MOTOR_IOC + 'm1',  MOTOR_IOC + 'm2',  MOTOR_IOC + 'm3', \
                    MOTOR_IOC + 'm4',  MOTOR_IOC + 'm5',  MOTOR_IOC + 'm6', \
                 ]
# Additional PVs for x-ray sync and save motor/ps/xray PVs
prefix = EXPERIMENT + 'VarianSync:'
pvdb = {
    'PaxscanShutter'        : {'asyn'  : False,},  # switches to 1 only when finished
    'ExpOk'                 : {'type'  : 'enum',
                               'enums' : ['OFF', 'ON'],
                               'value' : 0},
    'XSYNC'                 : {'type'  : 'enum',
                               'enums' : ['NONE', 'SRI', 'OXFORD', 'CPI']},
    'XSYNC_RBV'             : {'type'  : 'enum',
                               'enums' : ['NONE', 'SRI', 'OXFORD', 'CPI'],
                               'scan'  : 1},
    'DOC'                   : {'type'  : 'enum',
                               'enums' : ['OFF', 'ON'] },
    'SYNC_TRIGGER'          : {'asyn'  : True},
}
pvdb.update(epicsApps.pvdb)

class myDriver(Driver):
    def  __init__(self):
        super(myDriver, self).__init__()
        # set high priority for this process
        self.setProcessPriority()
        # load iocStats records
        self.iocStats()
        VARIAN_FULL_FILENAME_RBV.add_callback(self.checkDoc)  
        VARIAN_IMAGEMODE.add_callback(self.reset_num_filters)
        # keep track of whether we are in rad or fluoro mode
        VARIAN_CONFIG.add_callback(self.configChange)  
        self.shutter = 0                                # signal that the ADShutter Open
        self.zero = numpy.zeros((1,), dtype=numpy.uint8)
        self.one = numpy.ones((1,), dtype=numpy.uint8)
        self.written = int32()                          # needed for daqmx writes
        self.configValue = 0                            # initialize configValue
        VARIAN_CONFIG.get()
        #self.write('PaxscanShutter', 1)
        self.val = []
        self.x_twv = 0
        self.shutter_time = 0
        self.prior = 0
        # Set up the DI task to check the ExposeOK output from the Varian
        self.ExpOkInHandle = TaskHandle()
        DAQmxCreateTask("",byref(self.ExpOkInHandle))
        DAQmxCreateDIChan(self.ExpOkInHandle, EXP_OK, "", DAQmx_Val_ChanForAllLines)
        DAQmxStartTask(self.ExpOkInHandle)
        # Set up the DO task to send the ExposeRequest signal to the Varian. 
        # Must be "active drive" or it will not work.
        self.ExpReqOutTask = TaskHandle()
        DAQmxCreateTask("",byref(self.ExpReqOutTask))
        DAQmxCreateDOChan(self.ExpReqOutTask, EXP_REQ, "", DAQmx_Val_ChanForAllLines)
        DAQmxSetDOOutputDriveType(self.ExpReqOutTask, EXP_REQ, DAQmx_Val_ActiveDrive);
        DAQmxStartTask(self.ExpReqOutTask)
        # make sure expreq is low
        self.setExpReqOutputLow()   
        SCAN_DETECTOR_1.put(EXPERIMENT + 'VARIAN:cam1:Acquire')
        epicsApps.buildRequestFiles(prefix, pvdb.keys(), os.getcwd())
        epicsApps.makeAutosaveFiles()
        print '############################################################################'
        print '## ADVARIAN PCAS IOC Online $Date:' + str(datetime.datetime.now())[:-3]
        print '############################################################################'
    
    def setProcessPriority(self):
        try:
            sys.getwindowsversion()
        except:
            isWindows = False
        else:
            isWindows = True
        p = psutil.Process(os.getpid())
        if isWindows:
            try:
                p.set_nice(psutil.HIGH_PRIORITY_CLASS)
            except:
                print str(datetime.datetime.now())[:-3], "Failed setting high priority for this process. Need to run ioc as admin"
        else:
            try:
                os.nice(-10)
            except IOError:
                print str(datetime.datetime.now())[:-3], "Could not set high priority"
    
    def iocStats(self):
        """
        Sets application specific devIocStats records
        """
        self.start_time = datetime.datetime.now()
        self.setParam('ENGINEER', 'Andrew Gomella')
        self.setParam('LOCATION', 'B1D521D DT-ASUSWIN102')
        self.setParam('RECORD_CNT', len(pvdb.keys()))
        self.setParam('APP_DIR1', str(os.getcwd()))
        self.setParam('UPTIME', str(self.start_time))
        self.setParam('PARENT_ID', os.getpid())
        self.setParam('HEARTBEAT', 0)

    def read(self, reason):
        """
        pcaspy native read method used to display pcas ioc uptime, 
        and current date time
        """
        format_time = ""
        global XRAY_IOC
        if reason == 'UPTIME':
            format_time = datetime.datetime.now() - self.start_time
            value  = str(format_time).split(".")[0] 
        elif reason == 'TOD':
            value = str(datetime.datetime.now().strftime("%m/%d/%Y %H:%M:%S"))
        elif reason == 'HEARTBEAT':
            value = self.getParam('HEARTBEAT') + 1
            self.setParam('HEARTBEAT', value)
        elif reason == 'XSYNC_RBV':
            value = self.getParam('XSYNC')
            if value == 3:
              XRAY_IOC = EXPERIMENT + 'cpiSync:'  
            elif value == 2:
                XRAY_IOC = EXPERIMENT + 'OXFORD:xray:'
            elif value == 1:
                XRAY_IOC = EXPERIMENT + 'SRI:xray:'
            elif value == 0:
                XRAY_IOC = ''
        else: 
            value = self.getParam(reason)
        return value

    def write(self, reason, value):
        """
        pcaspy native write method
        """
        if reason == 'PaxscanShutter': 
            self.shutter = value
            # only call function if in rad mode
            if self.configValue == 0 : 
                if value == 1:
                    # start rad mode sequence
                    print str(datetime.datetime.now())[:-3], "Current Acquisiton Mode: Radiography"
                    self.tid = threading.Thread(target = self.paxscanShutterOpen, args = ())
                    self.tid.daemon = True
                    self.tid.start()
            elif self.configValue == 1:
                if value == 1:
                    # xray on
                    print str(datetime.datetime.now())[:-3], "Current Acquisition Mode: Fluoroscopy"
                    self.startupXray()
                    self.setParam(reason, value)
                elif value == 0:
                    # xray off
                    self.stopXrayFlux()
                    self.setParam(reason, value)
        elif reason == "XSYNC":
           self.setParam(reason, value)
        elif reason == 'SYNC_TRIGGER' and value == 1:
           self.hid = threading.Thread(target = self.liveXSync)
           self.hid.daemon = True
           self.hid.start()
        elif reason == "DOC":
            self.setParam(reason, value)
        self.setParam(reason, value)
        self.updatePVs()
            
    def waitForExpOkOn(self):
        """
        This function constantly polls the ExpOk signal from the Varian i.e. 
        the input line on the DAQ corresponding to EXP_OK. Returns true when the
        ExpOk Output from Varian is 1.
        """
        while True:
            newVal = np.array([0], dtype=np.uint8)
            DAQmxReadDigitalLines(self.ExpOkInHandle, 1, 1, 0,  newVal, 1, None, None, None)
            if newVal == self.one:
                break
            time.sleep(.001)
        self.timeOn = time.time()
        self.setParam('ExpOk', 1)
        self.updatePVs()

    def waitForExpOkOff(self):
        """
        This function constantly polls the ExpOk signal from the Varian i.e. 
        the input line on the DAQ corresponding to EXP_OK. Returns true when the
        ExpOk Output from Varian is 0.
        """
        while True:
            newVal = np.array([0], dtype=np.uint8)
            DAQmxReadDigitalLines(self.ExpOkInHandle, 1, 1, 0,  newVal, 1, None, None, None)
            if newVal == self.zero:
                break
            time.sleep(.001)
        self.setParam('ExpOk', 0)
        self.updatePVs()
        print str(datetime.datetime.now())[:-3], 'Expose Ok was on for', time.time() - self.timeOn, 's'

    def waitForShutterOff(self):
        """
        Wait here while the PaxScan shutter is open (Rad mode acquiring)
        """
        print self.shutter
        while self.shutter == 1:
            time.sleep(.001)

    # Set ExpReq high, this basically tells the panel we are ready to X-ray on demand
    def setExpReqOutputHigh(self):
        """
        This function sets the NIDAQ output line corresponding to EXP_REQ to 1, 
        to let the panel know that we are ready to expose.
        """
        try:
            DAQmxWriteDigitalLines(self.ExpReqOutTask,1,1,10.0, \
            DAQmx_Val_GroupByChannel,self.one,self.written, None)
        except DAQError as err:
            print "DAQmx Error: %s"%err

    # Set ExpReq low, timing of this is not important 
    def setExpReqOutputLow(self):
        """
        This function sets the NIDAQ output line corresponding to EXP_REQ to 0, 
        to let the panel know that we are done exposing.
        """
        try:
            DAQmxWriteDigitalLines(self.ExpReqOutTask,1,1,10.0, \
            DAQmx_Val_GroupByChannel,self.zero,self.written, None)
        except DAQError as err:
            print "DAQmx Error: %s"%err
   
    def startupXray(self):
        """
        Returns when the xray is on and outputting x-rays at set values
        """
        if self.getParam('XSYNC') == 3: # x-ray sync is set to cpi-cmp200
            self.myFlag = 0
            # check that the x-ray is not disconnected or in init phase or the emergency stop is on
            if caget(EXPERIMENT + 'CPI:xray:GeneratorStatus') == 0 or \
               caget(EXPERIMENT + 'CPI:xray:GeneratorStatus') == 1 or \
               caget(EXPERIMENT + 'CPI:xray:GeneratorStatus') == 9:
                   self.myFlag = 1
                   return
            else:
                # here we just get the generator ready to expose
                caput(XRAY_IOC + 'RAD_PREP' , 1)
                while (caget(EXPERIMENT + 'CPI:xray:' + 'RadPrep') != 2 and 
                       caget(EXPERIMENT + 'CPI:xray:ErrorLatching') != 22):
                    time.sleep(0.01)
                if caget(EXPERIMENT + 'CPI:xray:ErrorLatching') == 22:
                    self.myFlag = 1
                    caput(EXPERIMENT + 'CPI:xray:AcknowledgeError', 1)
                    
        elif self.getParam('XSYNC') == 2: # x-ray sync is set to oxford/nova
            if caget(XRAY_IOC + 'STATUS_RBV') == 5: # make sure x-ray is not in fault mode.
                print str(datetime.datetime.now())[:-3], 'X-ray is in fault mode!'
                return
            else:
                if caget(XRAY_IOC + 'STATUS_RBV') == 0: # xray is warming
                    print str(datetime.datetime.now())[:-3], 'Waiting for warm up to finish!'
                    return
                if caget(XRAY_IOC + 'STATUS_RBV') == 1: # if xray in standby mode 
                    # turn on x-ray 
                    caput(XRAY_IOC + 'ON', 1)
                    # wait for x-ray to reach set points
                    while (caget(XRAY_IOC + 'FIRING_RBV') != 1): # this record is sampled at 10Hz in the db
                        time.sleep(0.01)
                elif caget(XRAY_IOC + 'STATUS_RBV') == 3 or caget(XRAY_IOC + 'STATUS_RBV') == 2: # in pulse/output mode now.
                    caput(XRAY_IOC + 'PULSE_MODE', 0)   
                    while (caget(XRAY_IOC + 'FIRING_RBV') != 1):
                        time.sleep(0.01)
                print str(datetime.datetime.now())[:-3], 'X-ray is outputting at set points'
        elif self.getParam('XSYNC') == 1: # x-ray sync is set to sri
            caput(XRAY_IOC + 'ON', 1) 
            print str(datetime.datetime.now())[:-3], 'X-ray is outputting at set points'
        elif self.getParam('XSYNC') == 0:
            print str(datetime.datetime.now())[:-3], 'Acquiring dark image'
                
    def stopXrayFlux(self):
        """
        Stop x-ray flux
        """
        if self.getParam('XSYNC') == 0:
            return
        elif self.getParam('XSYNC') == 1: # x-ray sync is set to sri
            caput(XRAY_IOC + 'ON', 0)
        elif self.getParam('XSYNC') == 2: # x-ray sync is set to oxford
            caput(XRAY_IOC + 'ON', 0)
        elif self.getParam('XSYNC') == 3: # x-ray sync is set to cpi-cmp200
            caput(XRAY_IOC + 'EXPOSE', 0)
            time.sleep(.01)
            caput(XRAY_IOC + 'RAD_PREP', 0)
        print str(datetime.datetime.now())[:-3], 'X-ray is off'

    # Signal sent from ADShutter when it requests x-ray output (ASAP)
    def paxscanShutterOpen(self):
        while VARIAN_PV.get() != 1:
            time.sleep(0.01)
        self.startupXray()  
        self.setExpReqOutputHigh() 
        print str(datetime.datetime.now())[:-3], 'Expose Request sent to PaxScan'
        self.waitForExpOkOn()
        if self.getParam('XSYNC') == 3: # x-ray sync is set to cpi-cmp200
            caput(XRAY_IOC + 'EXPOSE', 1) 
        
        print str(datetime.datetime.now())[:-3], 'Waiting for Expose Ok from Paxscan to go to 0'
        self.waitForExpOkOff()
        if self.getParam('XSYNC') != 0:
            time.sleep(1)
            self.stopXrayFlux()        # turns off x-ray output 
        else:
            print str(datetime.datetime.now())[:-3], 'Dark Acquisition Finished'
        self.setExpReqOutputLow()  
        print str(datetime.datetime.now())[:-3], 'Expose Request now low'
        self.tid = None

    def configChange(self, **kw):
        """
        This function will be called back when user switches Varian Config to either
        0- Radiography mode, or 1- Fluoroscopy Mode. Syncing is very different 
        for these modes so we need to keep track of this
        """
        self.configValue = VARIAN_CONFIG.get()
    
    def checkDoc(self, **kw):
        """
        Callback function for Varian FullFileName_RBV PV. When image is saved,
        this function will start a thread to save parameter file if DOC is ON.
        """
        if self.getParam('DOC') == 1:
            self.usid = threading.Thread(target = self.saveParams, args=())
            self.usid.daemon = True
            self.usid.start()
        else:
            return
    
    def reset_num_filters(self, **kw):
        self.rid = threading.Thread(target = self.rnf, args=())
        self.rid.daemon = True
        self.rid.start()

    def rnf(self):
        if VARIAN_IMAGEMODE.get() == 0:
            VARIAN_NUMFILTER.put(1)
        self.rid = None

    def saveParams(self):
        """
        Daemon thread that saves a text file with the same name as 
        the image name. The file contains motor position readback values.
        """
        filePath = caget(DET_IOC + 'TIFF1:FilePath_RBV', as_string=True)
        fullFileName = caget(DET_IOC + 'TIFF1:FullFileName_RBV', as_string=True)
        if fullFileName == '':
            return 
        else:
            self.val = []
            for pvs in MOTOR_IOC_LIST:
                try:
                    self.val.append(caget(pvs + '.RBV'))
                except:
                    pass
            fileName = (fullFileName.split('\\')[-1]).split('.')[0]        
            f = open(filePath + fileName + '.txt', 'w')
            # Save relevant PVs
            for i, j in zip(MOTOR_IOC_LIST, self.val):
                f.write(i + " - " + str(j) + '\n')
            f.close()
            print str(datetime.datetime.now())[:-3], 'Document successful'
            self.usid = None
            
    def liveXSync(self):
        """
        Synchronizes the x-ray source exposure with the detector shutter
        """
        # initialize...
        print str(datetime.datetime.now())[:-3], 'Prepping for live scan'
        print VARIAN_RAD.get()
        if VARIAN_RAD.get() == 1:
            self.shutter_time = 2.0
        elif VARIAN_RAD.get() == 3:
            self.shutter_time = 6.0
        self.x_twv = caget(MOTOR_IOC + 'm2.TWV')
        caput(MOTOR_IOC + 'm2.VELO', np.abs(self.x_twv/self.shutter_time))
        VARIAN_PV.put(1)
        while(self.getParam('ExpOk') != 1):
            time.sleep(0.001)
        caput(MOTOR_IOC + 'm2.TWF', 1)
        while(caget(MOTOR_IOC + 'm2.DMOV', 0)):
            time.sleep(0.001)
        self.updatePVs()
        
if __name__ == '__main__':
    server = SimpleServer()
    server.createPV(prefix, pvdb)
    driver = myDriver()
    # process CA transactions
    while True:
        try:
            server.process(0.01)
        except KeyboardInterrupt:
            try:
                sys.exit(0)
            except SystemExit:
                os._exit(0)