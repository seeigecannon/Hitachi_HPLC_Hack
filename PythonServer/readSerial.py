import serial
import time
import globals
from collections import namedtuple
from datetime import datetime
import writeSerial

"""def recieveThread():
    ser = globals.ser
    global serialBuffer
    while True:
        time.sleep(0.025)
        if ser.in_waiting > 0:
            
            serialBuffer = serialBuffer + ser.read(ser.in_waiting)
            data = ser.read(ser.in_waiting)
            # Process the received data
            print(data)
"""
def binToNumber(bytes):
    value = int.from_bytes(bytes, "big", signed=True)
    #print(value)
    return value
def bytes_to_hex_string(byte_array):
    return ''.join(f"{byte:02X}" for byte in byte_array)


def decodeOven(message):
    if (message[2] == 10):  # hex 10, temp packet
        print("incoming auto temp packet")
        strippedMessage = message[3:]
        print(binToNumber(strippedMessage))
        #globals.ovenContainer.ovenTemp = binToNumber(strippedMessage)
        globals.updateOven(ovenTemp = binToNumber(strippedMessage))
        #globals.ovenContainer.timestamp = datetime.now()
        globals.updateOven(timestamp=datetime.now())
        # print(globals.ovenContainer.ovenTemp)
        # print(globals.ovenContainer.timestamp)
        return
    elif (message[2] == 1):  # hex 1, over status packet
        strippedMessage = message[3:]
        #globals.ovenContainer.status = strippedMessage
        globals.updateOven(status=strippedMessage)
        #globals.ovenContainer.statusTimestamp = datetime.now()
        globals.updateOven(statusTimestamp = datetime.now())
    elif (message[2] == 17):  # hex 11, over temp packet
        #globals.ovenContainer.power=message[3]
        globals.updateOven(power=message[3])
        #globals.ovenContainer.powerTimestamp = datetime.now()
        globals.updateOven(powerTimestamp = datetime.now())
    elif (message[2] == 16 or message[2] == 18):  # ack temp or limit setting
        #globals.ovenContainer.tempAckTimestamp = datetime.now()
        globals.updateOven(tempAckTimestamp = datetime.now())
    else:
        print(message[2])
        print("unknown " + message.hex())
def decodePump(message):
    if (message[2] == 28):  # hex 1C, pump status packet
        #print(message[3:])
        globals.updatePump(status=message[3:])
    if (message[2] == 20):  # hex 14, pump valve packet
        #print("first packet")
        #print(message)
        time.sleep(0.015)    #needing to grab second packet for this
        secondPacket = getSerialPacket()
        if not secondPacket:
            time.sleep(0.015)
            secondPacket = getSerialPacket()


        #print("second packet")
        #printBin(secondPacket)
        if secondPacket:
            secondMessage = secondPacket.decode('utf-8', errors='ignore')
            #print(secondMessage)
            if secondMessage.startswith("\rPKT:") and secondMessage.endswith("***"):
                secondPacket = secondPacket[5:-4]  # trim packet encapsulation as well as the CRC byte
            #printBin(secondPacket)
            if (secondPacket[3] == 48): #make sure this is the correct packet
                #print("stripped second packet")
                #print(secondPacket[4:])
                fullPacket = message[4:] + secondPacket[4:]
                #printBin(fullPacket[3:5])
                globals.updatePump(pctA=binToNumber(fullPacket[0:3]))
                globals.updatePump(pctB=binToNumber(fullPacket[3:6]))
                globals.updatePump(pctC=binToNumber(fullPacket[6:9]))
                globals.updatePump(pctD=binToNumber(fullPacket[9:12]))
                globals.updatePump(timestamp=datetime.now())
    if (message[2] == 21):  # hex 14, pump valve packet
        globals.updatePump(flowRate=binToNumber(message[4:6]))
        globals.updatePump(timestamp=datetime.now())
    if (message[2] == 22):  # hex 14, pump valve packet
        globals.updatePump(pressure=binToNumber(message[4:6]))

        globals.updatePump(timestamp=datetime.now())
    if (message[2] == 19 or message[2] == 17):  # hex 14, pump valve packet
        globals.updatePump(power=message[3])
        globals.updatePump(powerAckTimestamp=datetime.now())
    if (message[2] == 19):  # hex 13, MNTE responses
        if (message[3] == 0):
            globals.updatePump(MNTE0 = message[4:12])
        if (message[3] == 1):
            globals.updatePump(MNTE1 = message[4:12])
        if (message[3] == 2):
            globals.updatePump(MNTE2 = message[4:12])
        if (message[3] == 3):
            globals.updatePump(MNTE3 = message[4:12])

def decodeAS(message):
    if (message[2] == 153):  # hex 99, sample valve switched back to column
        globals.updateAS(sampleValveTimestamp = datetime.now())
    if (message[2] == 1):
        globals.updateAS(stat1 = bytes_to_hex_string(message[3:4]))
        globals.updateAS(timestamp = datetime.now())
    if (message[2] == 3):
        globals.updateAS(stat3 = bytes_to_hex_string(message[3:4]))
    if (message[2] == 4):
        globals.updateAS(stat4 = bytes_to_hex_string(message[3:4]))
    if (message[2] == 28):
        globals.updateAS(stat1C = bytes_to_hex_string(message[3:7]))
    if (message[2] == 27):
        globals.updateAS(stat1B = bytes_to_hex_string(message[3:7]))
    if (message[2] == 20):
        globals.updateAS(stat14 = bytes_to_hex_string(message[3:6]))
    if (message[1] == 7):
        if (message[4] == 1):
            globals.updateAS(MNTE1 = bytes_to_hex_string(message[5:10]))
        if (message[4] == 2):
            globals.updateAS(MNTE2 = bytes_to_hex_string(message[5:10]))
        if (message[4] == 3):
            globals.updateAS(MNTE3 = bytes_to_hex_string(message[5:10]))
        if (message[4] == 4):
            globals.updateAS(MNTE4 = bytes_to_hex_string(message[5:10]))
    if (message[2] == 29):
        globals.updateAS(IJCK = bytes_to_hex_string(message[3:5]))
    if (message[1] == 8):
        if(message[2] == 2):
            globals.updateAS(wash = 1)
        if (message[2] == 1):
            globals.updateAS(wash = 0)


def readIncomingPacket(message):
    #print("read incoming packet")
    #print(message)
    if message:

        if(message[0]==15):     #packet from oven
            decodeOven(message)

        if (message[0] == 2):       #packet from pump
            decodePump(message)
        if (message[0] == 13):       #packet from autosampler
            decodeAS(message)

def handle_data(data):
    # Decode the byte data to string
    print(data)

    message = data.decode('utf-8', errors='ignore')

    # Check if the message starts with "PKT:" and ends with "***"
    if message.startswith("\rPKT:") and message.endswith("***"):
        #print(data[5:-4])
        globals.lastPacketIn=data[5:-4]
        perform_action(data[5:-4])  #trim packet encapsulation as well as the CRC byte
    elif (message.startswith("\rDAD") or message.startswith("DAD")) and message.endswith("**-"):
        #print(message)
        print(data)
        #processDADPacket(data[5:-3])  # trim packet encapsulation
    else:
        print("Message does not meet criteria:" + message)
    #print(ovenContainer.ovenTemp)
    #print(ovenContainer.timestamp)

def processDADStatusPacket(packet):
    print(' '.join(format(x, '02x') for x in packet))
    timePacket=packet[-4:]
    timestamp=(timePacket[0] << 24) | (timePacket[1] << 16) | (timePacket[2] << 8) | timePacket[3]
    #print(timestamp)
    #print(packet[4])
    globals.updateSpectra(injectionTimestamp=timestamp)
def calculateNoise(spectra):
    print(spectra)
    mean = sum(spectra) / len(spectra)
    squared_deviations = [(x - mean) ** 2 for x in spectra]
    variance = sum(squared_deviations) / (len(spectra) - 1)  # Sample variance
    return variance



def processDADPacket(packet):
    global DADContainer
    global DADSpectraContainer
    #print(packet)
    #DAD:deadbeefdeadbeef...TTTTTT**-
    #print(packet[0]+packet[1]+packet[2])
    #print(packet)
    processDADStatusPacket(packet[-10:])
    packet=packet[7:1156]
    #print(packet)
    #deadbeefdeadbeef...TTTTTT

    spectra=[]
    for i in range(0, len(packet), 2):
        if i + 1 < len(packet):
            combined_value = (packet[i] << 8) | packet[i + 1]
            spectra.append(combined_value)

    #print(spectra)
    #DADSpectraContainer = DADSpectraConstructor(spectraTimestamp=0, Spectra=0, SpectraDetectorArray=None, SINF=0)
    globals.updateSpectra(spectraTimestamp=datetime.now())
    globals.updateSpectra(SpectraDetectorArray=spectra)
    lowNoise=calculateNoise(spectra[25:65])
    highNoise = calculateNoise(spectra[68:71])
    print(lowNoise)
    print(highNoise)
    globals.updateDAD(noiseLow=lowNoise)
    globals.updateDAD(noiseHigh=highNoise)


def perform_action(message):
    ser=globals.ser
    #ser.write(calculate_crc(b'\x50\x05\x00\x30\x0D\xFF\x0F'))
    # check for INIT packets and respond accordingly
    if message== b'\x00\x04\x02\x00\x01\x02':   # pump init
        writeSerial.initPump()

    if message == b'\x00\x04\x0F\x00\x01\x0F':  # oven init
        writeSerial.initOven()
    if message == b'\x00\x04\x0D\x00\x01\x0D':  # autosampler init
        writeSerial.initAS()
    # print(message)
    # Add the code for the action to be performed here

    if(message[0]==0):
        #print("incoming packet")
        readIncomingPacket(message[2:])



def printBin(byte_obj):
    print(''.join(f'\\x{byte:02x}' for byte in byte_obj))
def getSerialPacket():
    """
    Normal comms packet looks like PKT: 0D 03 00 02 11 1D 06 *** where 0D is the destination, 03 is the length not including the first 3 bytes, and 00 is the sender. The 06 is the checksum, message ends with ***
    A DAD packet looks like DAD: DE AD BE **-

    :return:
    """
    ser = globals.ser
    while True:
        time.sleep(0.012)   #wait one packet length
        #global serialBuffer
        globals.serialBuffer = globals.serialBuffer+ser.read(ser.in_waiting)
        #print(globals.serialBuffer.find(b'DAD'))

        globals.serialBuffer = globals.serialBuffer[globals.serialBuffer.find(b'PKT'):]


        if globals.serialBuffer.startswith(b'PKTDAD'):
            #print("got DAD packet")
            packetEnd=globals.serialBuffer.find(b'**-')
            #print(packetEnd)
            #print(len(globals.serialBuffer))
            if(packetEnd!=-1):
                incomingPacket=globals.serialBuffer[0:packetEnd]
                globals.serialBuffer=globals.serialBuffer[packetEnd:]
                processDADPacket(incomingPacket)
        #globals.serialBuffer=b''

        """
        if ser.in_waiting > 10:
            time.sleep(.001)  # wait a bit to make sure the serial packet has fully moved over
    
            data = ser.read(8)  #get packet header and 2 bytes of packet
            #print("first part of incoming packet")
            #printBin(data)
            if data.startswith(b'DAD'):
                print(data)
            else:
                moreData= ser.read(data[6] + 3)   #know how big packet is by looking at second byte and adding 3 bytes for the ending
                #print("second part of incoming packet")
                #printBin(moreData)
                return data+moreData
    """