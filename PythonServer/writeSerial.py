import globals
import time

pauseAS = False
pausePump = False
pauseOven = False


def printBin(byte_obj):
    print(''.join(f'\\x{byte:02x}' for byte in byte_obj))
def calculate_crc(
        data: bytes) -> bytes:  # calculate the CRC for the message and return the original message with the CRC added to it
    """

    :rtype: object
    """
    crc = 0
    for byte in data:
        crc ^= byte
    return data + bytes([crc])



def numberToBin(number, length=3):
    if number < -0x800000 or number > 0x7FFFFF:
        raise ValueError("Number out of range for 3 bytes")
    # Format the number to a 6-digit hexadecimal, padded with zeros
    binary_representation = number.to_bytes(length, byteorder='big', signed=True)
    return binary_representation


def pollInst(command, arg1=0, arg2=0):
    if command == "allClear":
        writeInst(b'\x02\x03\x00\x01\x05')  #clear pump
        writeInst(b'\x0F\x03\x00\x01\x05')  # clear oven?
        writeInst(b'\x0D\x03\x00\x01\x05')  # clear AS?

    if command == "ovenTemp":
        #print("get oven temp")
        #ser.write(calculate_crc(b'\x0F\x03\x00\x02\x10'))  #request oven temp
        writeInst(b'\x0F\x03\x00\x02\x10', 100)

    if command == "ovenStat":
        #print("get oven status")
        #ser.write(calculate_crc(b'\x0F\x03\x00\x02\x01'))  #request oven temp
        writeInst(b'\x0F\x03\x00\x02\x01')
    if command == "ovenOn":
        #ser.write(calculate_crc(b'\x0F\x04\x00\x03\x11\x01'))  # request oven temp
        writeInst(b'\x0F\x04\x00\x03\x11\x01')
    if command == "ovenOff":
        #ser.write(calculate_crc(b'\x0F\x04\x00\x03\x11\x00'))  # request oven temp
        writeInst(b'\x0F\x04\x00\x03\x11\x00')
    if command == "ovenSet":
        calculatedCommand = b'\x0F\x06\x00\x03\x10'
        calculatedCommand += numberToBin(arg1)
        #ser.write(calculate_crc(calculatedCommand))
        writeInst(calculatedCommand)
    if command == "ovenLimit":
        calculatedCommand = b'\x0F\x06\x00\x03\x12'
        calculatedCommand += numberToBin(arg1)
        #ser.write(calculate_crc(calculatedCommand))
        writeInst(calculatedCommand)
    if command == "pumpAll":
        writeInst(b'\x02\x03\x00\x02\x1C', 100) #STA2
        writeInst(b'\x02\x03\x00\x02\x14', 100) # valves
        writeInst(b'\x02\x03\x00\x02\x15', 100) #flow
        writeInst(b'\x02\x03\x00\x02\x16', 100) # pressure
        writeInst(b'\x02\x04\x00\x07\x13\x00', 100)  # MNTE 0
        writeInst(b'\x02\x04\x00\x07\x13\x01', 100) #MNTE 1
        writeInst(b'\x02\x04\x00\x07\x13\x02', 100) #MNTE 2
        writeInst(b'\x02\x04\x00\x07\x13\x03', 100) #MNTE 3



    if command == "pumpStatus":
        writeInst(b'\x02\x03\x00\x02\x1C')
    if command == "pumpOff":
        writeInst(b'\x02\x04\x00\x0A\x11\x00')
    if command == "pumpOn":
        writeInst(b'\x02\x04\x00\x0A\x11\x01')

    if command == "ASStat":
        writeInst(b'\x0D\x03\x00\x02\x1C', 100) # OPTC
        writeInst(b'\x0D\x03\x00\x02\x1B', 100) # OPTC
        writeInst(b'\x0D\x03\x00\x02\x14', 100) # OPTC
        writeInst(b'\x0D\x03\x00\x02\x01', 100) # STAT
        writeInst(b'\x0D\x03\x00\x02\x03', 100) # STAT
        writeInst(b'\x0D\x03\x00\x02\x04', 100) # STAT
        writeInst(b'\x0D\x04\x00\x07\x10\x01', 100) #MNTE 1
        writeInst(b'\x0D\x04\x00\x07\x10\x02', 100) #MNTE 2
        writeInst(b'\x0D\x04\x00\x07\x10\x03', 100) #MNTE 3
        writeInst(b'\x0D\x04\x00\x07\x10\x04', 100) #MNTE 4
        writeInst(b'\x0D\x03\x00\x02\x1D', 100)
    if command == "ASWashInject":
        writeInst(b'\x0D\x04\x00\x0A\x10\x0B')
    if command == "ASWashPump":
        writeInst(b'\x0D\x0D\x00\x0A\x10\x0C\x00\x00\x01\x00\x00\x0A\x00\x00\x05')  #the 01, 0A, and 05 would be worth playing around with
    if command == "ASSample":
        vialNumber=numberToBin(arg1)
        volume=numberToBin(arg2)
        writeInst(b'\x0D\x0C\x00\x03\x10'+vialNumber+volume+b'\x00\x00\x0A')
    if command == "ASSetTray":
        if globals.ASTray.trayReady:
            washStrokes=numberToBin(globals.ASTray.washStrokes)
            injectionWashStrokes = numberToBin(globals.ASTray.injectionWashStrokes)
            needleWashSpeed = numberToBin(globals.ASTray.needleWashSpeed,1)
            injectionWashSpeed = numberToBin(globals.ASTray.injectionWashSpeed,1)
            writeInst(b'\x0D\x0C\x00\x03\x13'+washStrokes+injectionWashStrokes+needleWashSpeed+injectionWashSpeed+b'\x00')

            unknown = numberToBin(globals.ASTray.unknown, 1)
            possibleNeedleVolA = numberToBin(globals.ASTray.possibleNeedleVolA, 1)
            possibleNeedleVolB = numberToBin(globals.ASTray.possibleNeedleVolB, 1)
            writeInst(b'\x0D\x06\x00\x03\x14'+unknown+possibleNeedleVolA+possibleNeedleVolB)

            syringeSpeed = numberToBin(globals.ASTray.syringeSpeed, 1)
            writeInst(b'\x0D\x06\x00\x03\x16' + syringeSpeed)

            injectionMethod = numberToBin(globals.ASTray.injectionMethod, 1)
            cutLeadVol = numberToBin(globals.ASTray.cutLeadVol, 3)
            cutRearVol = numberToBin(globals.ASTray.cutRearVol, 3)
            allFeedVol = numberToBin(globals.ASTray.allFeedVol, 3)
            loopWasteVol = numberToBin(globals.ASTray.loopWasteVol, 3)

            writeInst(b'\x0D\x06\x00\x13\x15'+injectionMethod+cutLeadVol+cutRearVol+allFeedVol+loopWasteVol[0:1])
            writeInst(b'\x0D\x06\x00\x30' + loopWasteVol[2])

            needleDownSpeed = numberToBin(globals.ASTray.needleDownSpeed, 1)
            writeInst(b'\x0D\x04\x00\x03\x17' + needleDownSpeed)
    if command == "writePumpMethod":
        writeInst(b'\x02\x04\x00\x0A\x16\x01', 200)
        writeInst(b'\x02\x05\x00\x06\x01\x01\x02\x03\x06', 100)
        pumpMethod = globals.pumpMethod
        for i, pumpPhase in enumerate(pumpMethod):
            phase=pumpMethod[i]
            #print(phase)
            if phase.time == -1:    # a -1 indicates the end of method
                break
            if i == 0:
                maxPres = numberToBin(phase.maxPres)
                minPres = numberToBin(phase.minPres)
                writeInst(b'\x02\x09\x00\x06\x02'+maxPres+minPres, 100)
                #time.sleep(.05)

            runTime = numberToBin(phase.time, 4)
            pctA = numberToBin(phase.pctA)
            pctB = numberToBin(phase.pctB)
            pctC = numberToBin(phase.pctC)
            pctD = numberToBin(phase.pctD)
            flow = numberToBin(phase.flowRate)
            auxA = phase.auxA
            auxB = phase.auxB
            auxC = phase.auxC
            auxD = phase.auxD

            writeInst(b'\x02\x0F\x00\x16\x03' + runTime + pctA + pctB + pctC[0:2])
            writeInst(b'\x02\x0D\x00\x30' + pctC[2:] + pctD + flow + auxA + auxB + auxC + auxD, 100)
            #writeInst(b'\x02\x1A\x00\x16\x03' + runTime + pctA + pctB + pctC + pctD + flow + auxA + auxB + auxC + auxD)
            #time.sleep(.05)
        writeInst(b'\x02\x03\x00\x06\x04', 100)
        writeInst(b'\x02\x04\x00\x0A\x16\x02', 100)

def initPump():
    print("pump init")
    global pausePump
    pausePump = True

    writeNoPause(b'\x02\x04\x00\x00\x01\x02')
    writeNoPause(b'\x02\x0F\x00\x10\x02\xFF\x02\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF')
    writeNoPause(b'\x02\x05\x00\x30\x0D\xFF\x0F')
    pausePump = False
def initOven():
    print("oven init")
    global pauseOven
    pauseOven = True
    writeNoPause(b'\x0F\x04\x00\x00\x01\x0F')
    writeNoPause(b'\x0F\x0F\x00\x10\x02\xFF\x02\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF')
    writeNoPause(b'\x0F\x05\x00\x30\x0D\xFF\x0F')
    # ser.write(b'\x0F\x09\x00\x00\x03\x00\x01\x00\x00\x00\x00\x04')
    pauseOven = False

def initAS():
    print("AS init")
    global pauseAS
    pauseAS = True
    writeNoPause(b'\x0D\x04\x00\x00\x01\x0D')
    writeNoPause(b'\x0D\x0F\x00\x10\x02\xFF\x02\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF')
    writeNoPause(b'\x0D\x05\x00\x30\x0D\xFF\x0F')
    pauseAS = False
def writeInst(command, wait=0):
    if command[0] == b'02' and pausePump:
        return
    if command[0] == b'0F' and pauseOven:
        return
    if command[0] == b'0D' and pauseAS:
        return
    wait = 0    #disable timeout until I get it working
    fullCommand=calculate_crc(command)
    globals.ser.write(fullCommand)
    if wait:
        waitForAck(wait)
    else:
        time.sleep(.030)  # delay to make sure commands dont stomp over each other
    globals.lastCommandOut=fullCommand
    return fullCommand

def writeNoPause(command, wait=0):
    wait = 0    #disable timeout until I get it working
    fullCommand=calculate_crc(command)
    globals.ser.write(fullCommand)
    if wait:
        waitForAck(wait)
    else:
        time.sleep(.030)  # delay to make sure commands dont stomp over each other
    globals.lastCommandOut=fullCommand
    return fullCommand

def waitForAck(timeout=100):    #very simple packet acknologment. It just checks to see if the destination device sends the next packet, no other checking at the moment
    start_time = time.time()
    while True:
        if time.time() - start_time > (timeout/1000):
            #print("timeout")
            #print(globals.lastCommandOut)
            #print(globals.lastPacketIn)
            globals.lastCommandOut = None
            globals.lastPacketIn = None
            return -1
        if globals.lastCommandOut and globals.lastPacketIn:
            if globals.lastCommandOut[2] == globals.lastPacketIn[0]:
                globals.lastCommandOut = None
                globals.lastPacketIn = None
                return 1


def Autopoll():
    print("autopoll")
    time.sleep(2)
    print("setting master")
    globals.ser.write(b'\x99')
    time.sleep(1)
    #pollInst("allClear")
    #pollInst("ASSample", 1, 10)
    #pollInst("writePumpMethod")
    #writeInst(b'\x0D\x06\x00\x03\x18\x00\x00\x01')  #test setting AS rack
    #writeInst(b'\x0D\x0F\x00\x03\x12\x18\x00\x00\x00\x01\x00\x00\xFA\x00\x05\xBE\x00\x00')
    #writeInst(b'\x0D\x0F\x00\x03\x20\x0A\x00\x00\x00\x00\x05\xBE\x00\x00\x0C\x00\x01\x9A')
    #writeInst(b'\x0D\x03\x00\x03\x31\x01')
    #initAS()
    #initPump()
    #initOven()
    time.sleep(5)
    while not globals.autopollThreadStopEvent.is_set():

        start_time = time.time()
        if not (pauseAS or pausePump or pauseOven):
            pollInst("pumpAll")
            #pollInst("ovenTemp")
            #pollInst("ASStat")
            #writeInst(b'\x02\x03\x00\x02\x14')
            # Perform the task
            #print("Task executed at", time.strftime("%Y-%m-%d %H:%M:%S"))

            # Calculate the time taken for the task
            elapsed_time = time.time() - start_time

            # Wait for the remaining time of the 1-second interval
            globals.autopollThreadStopEvent.wait(max(1.0 - elapsed_time, 0))