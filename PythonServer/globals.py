from collections import namedtuple
import serial

DADContainerConstructor = namedtuple('DADContainerConstructor', ['SN', 'status', 'lastCommand', 'lastStatusByte', 'busy', 'error', 'lampStat', 'ISLT', 'statusTimestamp', 'noiseLow', 'noiseHigh'] )
DADSpectraConstructor = namedtuple('DADSpectraContainerConstructor', ['spectraTimestamp', 'Spectra','SpectraDetectorArray', 'injectionTimestamp',], )
pumpContainerConstruct = namedtuple('pumpContainerConstruct', ['pctA', 'pctB', 'pctC', 'pctD', 'pressure', 'flowRate', 'status',
                                             'timestamp', 'power', 'powerAckTimestamp', 'MNTE0', 'MNTE1', 'MNTE2', 'MNTE3', ], )
ovenContainerConstruct = namedtuple('ovenContainerConstruct', ['ovenTemp', 'timestamp', 'status', 'statusTimestamp', 'power',
                                             'powerTimestamp', 'tempAckTimestamp'])
ASContainerConstruct = namedtuple('ASContainerConstruct', ['stat1', 'stat3', 'stat4', 'stat1C', 'stat1B', 'stat14', 'timestamp', 'sampleValveStart', 'sampleValveTimestamp', 'wash', 'MNTE1', 'MNTE2', 'MNTE3', 'MNTE4', 'IJCK'])
ASTrayConstruct = namedtuple('ASTrayConstruct', ['washStrokes', 'injectionWashStrokes', 'needleWashSpeed', 'injectionWashSpeed',
                               'possibleRackCode', 'possibleNeedleVolA', 'possibleNeedleVolB', 'syringeSpeed', 'injectionMethod', 'cutLeadVol', 'cutRearVol', 'allFeedVol', 'loopWasteVol', 'unknown', 'needleDownSpeed', 'trayReady'])

DADContainer = DADContainerConstructor(SN=None, status=0, lastCommand=None, lastStatusByte=None, busy=-1, error=0, lampStat=0, ISLT=0, statusTimestamp=0, noiseLow=0, noiseHigh=0)
DADSpectraContainer = DADSpectraConstructor(spectraTimestamp=0, Spectra=0, SpectraDetectorArray=None, injectionTimestamp=0)
ovenContainer = ovenContainerConstruct(ovenTemp=0, timestamp=0, status=0, statusTimestamp=0, power=-1, powerTimestamp=0, tempAckTimestamp=0)
pumpContainer = pumpContainerConstruct(pctA=-1, pctB=-1, pctC=-1, pctD=-1, pressure=-1, flowRate=-1, status=0, timestamp=0, power=-1, powerAckTimestamp=0, MNTE0=-1, MNTE1=-1, MNTE2=-1, MNTE3=-1)
ASContainer = ASContainerConstruct(stat1=-1, stat3=-1, stat4=-1, stat1C=-1, stat1B=-1, stat14=-1, timestamp=0, sampleValveStart=-1, sampleValveTimestamp=0, wash=-1, MNTE1=-1, MNTE2=-1, MNTE3=-1, MNTE4=-1, IJCK=-1)
ASTray = ASTrayConstruct
ASTray.trayReady = 0



pumpPhase= namedtuple('pumpPhase', ['maxPres', 'minPres', 'time', 'pctA', 'pctB', 'pctC', 'pctD', 'flowRate', 'auxA', 'auxB', 'auxC', 'auxD'])
pumpMethod = []
def resetPumpMethod():
    for i in range(20):
        pumpMethod.append(
            pumpPhase(maxPres=0, minPres=0, time=-1, pctA=0, pctB=0, pctC=0, pctD=0, flowRate=0, auxA=b'\xFF',
                      auxB=b'\xFF', auxC=b'\xFF', auxD=b'\xFF'))
    #pumpBlank = pumpPhase(maxPres=0, minPres=0, time=-1, pctA=0, pctB=0, pctC=0, pctD=0, flowRate=0, auxA=b'\xFF', auxB=b'\xFF', auxC=b'\xFF', auxD=b'\xFF')
    #global pumpMethod
    #pumpMethod = pumpBlank * 20

resetPumpMethod()   #create empty method for the pump

pumpMethod[0] = pumpPhase(maxPres=100, minPres=10, time=0, pctA=500, pctB=500, pctC=0, pctD=0, flowRate=100, auxA=b'\xFF', auxB=b'\xFF', auxC=b'\xFF', auxD=b'\xFF')
pumpMethod[1] = pumpPhase(maxPres=100, minPres=10, time=60, pctA=500, pctB=500, pctC=0, pctD=0, flowRate=100, auxA=b'\xFF', auxB=b'\xFF', auxC=b'\xFF', auxD=b'\xFF')
pumpMethod[2] = pumpPhase(maxPres=100, minPres=10, time=-1, pctA=500, pctB=500, pctC=0, pctD=0, flowRate=100, auxA=b'\xFF', auxB=b'\xFF', auxC=b'\xFF', auxD=b'\xFF')

#for i, pumpPhase in enumerate(pumpMethod):
#    #currentLine = pumpMethod[i]
#    print(pumpMethod[i].time)

lastCommandOut = None
lastPacketIn = None


ser = serial.Serial(
        port='COM7',
        baudrate=460800,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE
    )
serialBuffer = b''
autopollThreadStopEvent = None
serialGoodToWrite = 0

pumpContainer = pumpContainer._replace(status=23)
#pumpContainer.status = 123

def updatePump(**kwargs):
    global pumpContainer
    pumpContainer = pumpContainer._replace(**kwargs)
    #print("updated global")
def updateOven(**kwargs):
    global ovenContainer
    ovenContainer = ovenContainer._replace(**kwargs)
    #print("updated global")
def updateAS(**kwargs):
    global ASContainer
    ASContainer = ASContainer._replace(**kwargs)
    #print("updated global")

def updateDAD(**kwargs):
    global DADContainer
    DADContainer = DADContainer._replace(**kwargs)
    #print("updated global")

def updateSpectra(**kwargs):
    global DADSpectraContainer
    DADSpectraContainer = DADSpectraContainer._replace(**kwargs)
    #print("updated global")

