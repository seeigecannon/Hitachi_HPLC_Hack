import pyvisa
gpib_card = 'GPIB0::INTFC'
#import constants
import time
from datetime import datetime
import globals
from pyvisa.constants import LineState
import threading

from pyvisa.constants import EventType, EventMechanism


GPIBTimeout = 10000
#import ctypes
#dll_path = 'C:\\Program Files (x86)\\National Instruments\\Shared\\ExternalCompilerSupport\\C\\lib64\\msc\\nicaiu.dll'
#dll_path = 'C:\\Windows\\SysWOW64\\gpib-32.dll'
#gpib = ctypes.CDLL(dll_path)
#gpib = ctypes.windll.Gpib-32


#GPIB0 = 0
#gpib.ibrsp.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_short)]
#gpib.ibrsp.restype = ctypes.c_int
#status_byte = ctypes.c_short()
#result = gpib.ibrsp(GPIB0, ctypes.byref(status_byte))
#if result == 0:  # Check if the operation was successful
#    print(f"Cached status byte: {status_byte.value:#04x}")
#else:
#    print(f"An error occurred: {result}")

def intToPaddedAsciiByte(input, numOfPlaces=3):
    string_value = str(input)
    formatted_string = f"{string_value:>{numOfPlaces}}"
    byte_object = formatted_string.encode('ascii')
    return byte_object


def calculate_crc(
        data: bytes) -> bytes:  # calculate the CRC for the message and return the original message with the CRC added to it
    crc = 0
    for byte in data:
        crc ^= byte
    return data + bytes([crc])
def srq_handler(session, event_type, context):
    print("SRQ event detected!")
    status_byte = session.read_stb()
    print(f'Status Byte: {status_byte}')
    data = session.read_raw()
    print(f'Raw Data: {data}')

def readFile(filename):
    #SPD7455 = r"C:\Win32App\HSM\System\SPD7455.BIN"
    file_chunks = []
    try:
        with open(filename, 'rb') as file:
            while True:
                chunk = file.read(28000)
                if not chunk:
                    break
                file_chunks.append(chunk)
        print(f"Read {len(file_chunks)} chunks from the file.")
    except FileNotFoundError:
        print(f"File not found: {filename}")
        file.close()
        return -1
    except IOError as e:
        print(f"An I/O error occurred: {e}")
        file.close()
        return -1
    file.close()
    return file_chunks

def coldBoot():
    GPIBTimeout = 10000
    print(f"cold boot")
    SPD7455_chunks = readFile(r"C:\Win32App\HSM\System\SPD7455.BIN")
    L7455OS_chunks = readFile(r"C:\Win32App\HSM\System\L7455OS.BIN")

    instrument.write_termination = None
    instrument.read_termination = None
    returnData = sendGPIBCommand(b'\x00\x02\x00\x02')  # get serial number
    globals.updateDAD(SN=returnData[3:-2])
    print(f"DAD SN:{globals.DADContainer.SN}")
    # time.sleep(1)

    # todo make an array of commands and the expected responses with a function to go through them one at a time
    bootCommands = []
    bootCommands.append(b'\x00\x03\x00\xE0\x00\x00\x00\xEF\xFF\xFF\x00\x40\x10\x00\x00\x40\xFF\xFF\x06\x5E')
    bootCommands.append(b'\x00\x04\x00\xE0\x00\x00\x6D\x60\x01\xB1')
    bootCommands.append(SPD7455_chunks[0])
    bootCommands.append(b'\x00\x04\x00\xE0\x6D\x60\x6D\x60\x02\x7E')
    bootCommands.append(SPD7455_chunks[1])
    bootCommands.append(b'\x00\x04\x00\xE0\xDA\xC0\x0F\x64\x02\xF1')
    bootCommands.append(SPD7455_chunks[2])
    bootCommands.append(b'\x00\x04\x00\x40\x10\x00\x6D\x60\x01\x21')
    bootCommands.append(L7455OS_chunks[0])
    bootCommands.append(b'\x00\x04\x00\x40\x7D\x60\x2F\x80\x01\xD0')
    bootCommands.append(L7455OS_chunks[1])
    bootCommands.append(b'\x00\x05\x00\xE0\x00\x00\x00\xE0\xEA\x24\x02\xD3')
    bootCommands.append(b'\x00\x05\x00\x40\x10\x00\x00\x40\xAC\xE0\x02\x21')
    bootCommands.append(b'\00\x06\x00\x06')

    expectedResponses = []
    expectedResponses.append(b'\x00\x03\x00\x01\x00\x00\x04')
    expectedResponses.append(b'\x00\x04\x00\x00\x04')
    expectedResponses.append(b'\x00\x04\x00\x00\x04')
    expectedResponses.append(b'\x00\x04\x00\x00\x04')
    expectedResponses.append(b'\x00\x04\x00\x00\x04')
    expectedResponses.append(b'\x00\x04\x00\x00\x04')
    expectedResponses.append(b'\x00\x04\x00\x00\x04')
    expectedResponses.append(b'\x00\x04\x00\x00\x04')
    expectedResponses.append(b'\x00\x04\x00\x00\x04')
    expectedResponses.append(b'\x00\x04\x00\x00\x04')
    expectedResponses.append(b'\x00\x04\x00\x00\x04')
    expectedResponses.append(b'\x00\x05\x00\\\xd5\x016')
    expectedResponses.append(b'\x00\x05\x00\x1e9\x00\\')
    expectedResponses.append(b'\x00\x06\x00\x00\x06')
    del SPD7455_chunks
    del L7455OS_chunks
    for i in range(len(bootCommands)):
        returnData = sendGPIBCommand(bootCommands[i])
        if returnData != expectedResponses[i]:
            print(f"failed boot. expected {expectedResponses[i]}, got {returnData}")
            return -1
    instrument.write_termination = '\r'
    instrument.read_termination = '\r'
    print("boot delay")
    time.sleep(60)
    GPIBTimeout = 1000
    data = sendGPIBCommand(b'IERR  0')
    if len(data) > 20:
        print("boot succsful")
        return 1
    else:
        return 0




def loadDADCode():
    global GPIBTimeout
    returnData = sendGPIBCommand(b'IERR  0')
    if returnData != -1:
        if len(returnData) > 20:
            print(f"warm boot")
            GPIBTimeout = 1000
            print(sendGPIBCommand(b'SEND  0  0  3  0'))
            return 1
        elif returnData != -1:
            if coldBoot():
                print(sendGPIBCommand(b'SEND  0  0  3  0'))
                setWavelengths()
                return 1
            else:
                return -1
        else:
            print(f"failed boot")
            return -1
    else:
        print(returnData)
        print("DAD not responding")


def sendGPIBCommand(command, receive=1):
    try:
        #stb 64 means incoming data available
        #command=calculate_crc(command)
        globals.updateDAD(lastCommand=command)
        if instrument.read_termination:     #add termination character if enabled
            instrument.write_raw(command+b'\x0d')
        else:
            instrument.write_raw(command)
        #print(f'Command "{command}" sent to GPIB address {gpib_address}')
        try:
            instrument.wait_for_srq(GPIBTimeout)
            #man_wait_for_srq(GPIBTimeout)
            #testHammerSTB(GPIBTimeout)

            #time.sleep(.001)
            if command[0:4] == b'SEND' or receive == 0:
                return 0
            incomingData = instrument.read_raw()
            if incomingData[-1] == 13:      #remove termination character if applicable
                #print(f'Recieved {incomingData[:-1]}')
                return incomingData[:-1]
            else:
                return incomingData
        except:
            print("read failed")
            return -1
    except pyvisa.VisaIOError as e:
        print(f'An error occurred: {e}')
        return -1

def clearGPIB():
    #status = instrument.read_stb()

    #if status > 1:
    try:
        instrument.wait_for_srq(1)
        print(instrument.read_raw())
    except:
        pass

def splitList(byte_object):
    decoded_string = byte_object.decode('utf-8')
    string_list = decoded_string.split()
    #float_list = [float(element) for element in string_list]
    float_list = [float(element) for element in string_list if float(element) != 0.0]
    return float_list

def setWavelengths(table=0, start=0, end=497, bandwidth = 0):
    """
    The DAD can set various wave tables. First, WTBL  0  x  y gets called. X=0 displays the number of entries for Y
      If X = 1 it will display the entries 10 wavelengths at a time for Y. All the wavelengths must be collected to continue
      If X = 2 it will apply that wavelength table for the following WLRG command
    :param table: wavelength table, 0-3, 0 being the biggest range
    :param start: index of starting wavelength
    :param end:     index of ending wavelength
    :return:    none
    WLRG  0  0 497
    """
    returnData = sendGPIBCommand(b'WTBL  0  0'+intToPaddedAsciiByte(table))
    totalEntries = int(returnData[8:].decode('utf-8'))
    #totalEntries = 498
    print(totalEntries)
    returnData = b'\x20'    #replace return data for the += lines coming up next
    for i in range(int((totalEntries + 10 - 1) // 10)):   #cycle through all of the table values and collect for later use, use mod so if there are 270 entries the for loop will run 27 times, but if there are 271 entries it will run 28 times
        returnData += sendGPIBCommand(b'WTBL  0  1'+intToPaddedAsciiByte(table))[11:] + b'\x20'
        #spectra = (splitList(returnData[11:]))
        #print(returnData[11:])
    #print(returnData)
    globals.updateSpectra(SpectraDetectorArray=splitList(returnData))

    if end >= totalEntries - 1:
        end = totalEntries - 1

    sendGPIBCommand(b'WTBL  0  2'+intToPaddedAsciiByte(table), 0)
    #sendGPIBCommand(b'WLRG  0  0 10', 0)    #used to be 497
    sendGPIBCommand(b'WLRG  0'+intToPaddedAsciiByte(start)+intToPaddedAsciiByte(end, 4), 0)  # used to be 497
    sendGPIBCommand(b'BNDP  0'+intToPaddedAsciiByte(bandwidth), 0)   #sets the bandwidth, ignoreing for now
    sendGPIBCommand(b'BNDP  0' + intToPaddedAsciiByte(bandwidth), 0)  # sets the bandwidth, ignoreing for now
    globals.updateSpectra(SINF=sendGPIBCommand(b'SINF  0')[9:])

def getStatus():

    globals.updateDAD(busy=sendGPIBCommand(b'IQEX  0')[9:])
    globals.updateDAD(status=sendGPIBCommand(b'ISTA  0')[9:])
    globals.updateDAD(error=sendGPIBCommand(b'IERR  0')[9:])
    globals.updateDAD(lampStat=sendGPIBCommand(b'GLMP  0')[9:])
    globals.updateDAD(ISLT=sendGPIBCommand(b'ISLT  0')[9:])

    globals.updateDAD(statusTimestamp=datetime.now())
    return int(globals.DADContainer.busy.decode('ascii'))

def getSpectra():
    timestamp = datetime.now()  #get timestamp before GPIB command to make it a few milliseconds more accutate, hopefully
    print(sendGPIBCommand(b'SSPC  0  1'))
    sendGPIBCommand(b'RESP  0  0', 0)
    print(sendGPIBCommand(b'SSPC  0  0'))
    sendGPIBCommand(b'RESP  0  0', 0)  # collect new spectra, the bigger the spectra the longer this takes
    #returnData = sendGPIBCommand(b'AZDT  0')  # get the collected spectra
    #sendGPIBCommand(b'RESP  0  0', 0)   #collect new spectra, the bigger the spectra the longer this takes
    #returnData = sendGPIBCommand(b'AZDT  0')    #get the collected spectra
    #returnData = sendGPIBCommand(b'SSPC  0  1')

    try:
        if len(returnData) > 10:
            globals.updateSpectra(Spectra=returnData[9:])
            globals.updateSpectra(spectraTimestamp=timestamp)
            return returnData

    except:
        globals.updateDAD(Spectra=None)
        globals.updateDAD(spectraTimestamp=timestamp)
        print("get spectra fail")

def testRunSpectra():
    sendGPIBCommand(b'TIMP  0     0.0   600.0', 0)
    sendGPIBCommand(b'TITV  0  1  20', 0)
    sendGPIBCommand(b'SEND  0  0  3  0')
    #sendGPIBCommand(b'CZRO  0')
    sendGPIBCommand(b'GLMP  0')
    sendGPIBCommand(b'SEND  0  0  2  1')
    sendGPIBCommand(b'ERCL  0', 0)
    sendGPIBCommand(b'STRS  0', 0)
    sendGPIBCommand(b'SEND  0  0  1  1')
    sendGPIBCommand(b'SEND  0  0  3  1')
    instrument.wait_for_srq(GPIBTimeout)
    instrument.wait_for_srq(GPIBTimeout)
    #time.sleep(12)
    while(1):
        #time.sleep(1.7)
        try:
            print(instrument.read_raw())
            sendGPIBCommand(b'RESP  0  0', 0)
            instrument.wait_for_srq(GPIBTimeout)
            instrument.wait_for_srq(GPIBTimeout)
        except:
            print("failure")
def setSlit(slit):
    sendGPIBCommand(b'SLIT  0' + intToPaddedAsciiByte(slit), 0)

def autoZero():
    sendGPIBCommand(b'ZERO  0')

def calibrate():
    pass
def testGetSpectra():
    previousTime = time.time() * 1000
    totalTime=0
    for i in range(100):
        # print(getSpectra())
        # time.sleep(delayTime/1000)
        # delayTime = delayTime - 10
        # print(delayTime)
        # time.sleep(0.001)
        # getStatus()
        getSpectra()
        #print(datetime.now() - previousTime)
        totalTime+=(time.time() * 1000) - previousTime
        previousTime = time.time() * 1000
        # print(datetime.now())
        # getStatus()
    return totalTime/100

def testHammerSTB(timeout=10000):
    interval = 100
    start_time = time.time()  # Get the start time
    timeout_seconds = timeout / 1000.0  # Convert timeout to seconds
    interval_seconds = interval / 1000.0  # Convert interval to seconds
    while time.time() - start_time < timeout_seconds:
        print("Executing task...")
        try:
            status = instrument.read_stb()
            print(status)
        except:
            pass
        # Sleep for the specified interval
        time.sleep(interval_seconds)

def man_wait_for_srq():
    """
    Wait for the SRQ line to be asserted and return the status byte.

    Parameters:
    - instrument: The instrument resource object.
    - timeout: Timeout in milliseconds (default 25000 ms).
    """
    # Enable the SRQ event
    while(1):
        instrument.enable_event(EventType.service_request, EventMechanism.queue)

        try:
            # Wait for the SRQ event to be triggered within the specified timeout
            print("wait for srq")
            instrument.wait_on_event(EventType.service_request, 100)
            print("wait for srq event passed")
            # Check the status byte to confirm SRQ (typically bit 6, 0x40)
            status_byte = instrument.stb
            if status_byte & 0x40:
                print(f"SRQ detected! Status Byte: {status_byte:#04x}")
            else:
                print("SRQ not detected, but an event was triggered.")

        except pyvisa.errors.VisaIOError as e:
            pass
            #print(f"An error occurred while waiting for SRQ: {e}")

        finally:
            # Discard the SRQ event to clean up the queue
            #instrument.discard_events(EventType.service_request, EventMechanism.queue)
            pass
def main(gpib_address=7):
    # Initialize the Visa resource manager
    rm = pyvisa.ResourceManager()
    global GPIBTimeout
    try:
        # Open a connection to the GPIB instrument
        global instrument
        instrument = rm.open_resource(f'GPIB::{gpib_address}::INSTR')
        #exit()
        instrument.write_termination = '\r'  # 0x0D in hexadecimal
        instrument.read_termination = '\r'
        #SerialReadThread = threading.Thread(target=man_wait_for_srq, args=())
        #SerialReadThread.daemon = True
        #SerialReadThread.start()


        #instrument.wait_for_srq(1000000)

        #man_wait_for_srq(instrument)
        #input("Press Enter to continue.")
        #print(instrument.read_raw())

        #instrument.set_visa_attribute(VI_ATTR_GPIB_UNADDR_EN, False)
        #print(instrument.read_termination)
        #exit()
        #print(instrument.read_stb())
        #print(instrument.read_raw())
        #print(sendGPIBCommand(b'IERR  0'))
        clearGPIB()
        #input("Press Enter to continue.")
        if loadDADCode() == 1:
            #sendGPIBCommand(b'CALB  0  1', 0)
            while 1:
                input("Press Enter to lamp on")
                sendGPIBCommand(b'OLMP  0  1  1', 0)
                input("Press Enter to lamp off")
                sendGPIBCommand(b'OLMP  0  1  0', 0)

            while False:
                while True:
                    input("Press Enter to SLIT  0  0")
                    sendGPIBCommand(b'SLIT  0  0', 0)
                    input("Press Enter to SLIT  0  1")
                    sendGPIBCommand(b'SLIT  0  1', 0)
                    input("Press Enter to SLIT  0  2")
                    sendGPIBCommand(b'SLIT  0  2', 0)
                    input("Press Enter to SLIT  0  3")
                    sendGPIBCommand(b'SLIT  0  3', 0)
                    input("Press Enter to SLIT  0  4")
                    sendGPIBCommand(b'SLIT  0  4', 0)
                    input("Press Enter to SLIT  0  5")
                    sendGPIBCommand(b'SLIT  0  5', 0)

                input("Press Enter to BPOS 0.")
                print(sendGPIBCommand(b'BPOS  0  0', ))
                input("Press Enter to SLIT  0  0")
                sendGPIBCommand(b'SLIT  0  0', 0)
                input("Press Enter to BPOS 1.")
                print(sendGPIBCommand(b'BPOS  0  1', ))
                input("Press Enter to SLIT  0  1")
                sendGPIBCommand(b'SLIT  0  1', 0)
                input("Press Enter to BPOS 2.")
                print(sendGPIBCommand(b'BPOS  0  2', ))
                input("Press Enter to SLIT  0  2")
                sendGPIBCommand(b'SLIT  0  2', 0)
                input("Press Enter to BPOS.")
                print(sendGPIBCommand(b'BPOS  0  0', ))
            while True:
                input("Press Enter to CALB 1.")
                sendGPIBCommand(b'CALB  0  1', 0)
                input("Press Enter to DMOD  0  1  0  1  1.")
                sendGPIBCommand(b'DMOD  0  1  0  1  1', 0)
                input("Press Enter to CAED  0.")
                sendGPIBCommand(b'CAED  0', 0)
                input("Press Enter to DMOD  0  0  0  1  1.")
                sendGPIBCommand(b'DMOD  0  0  0  1  1', 0)







            input("Press Enter to sendGPIBCommand(b'SEND  0  0  0  0').")
            sendGPIBCommand(b'SEND  0  0  0  0')
            input("Press Enter to sendGPIBCommand(b'SEND  0  0  0  1').")
            sendGPIBCommand(b'SEND  0  0  0  1')
            input("Press Enter to sendGPIBCommand(b'SEND  0  0  0  2').")
            sendGPIBCommand(b'SEND  0  0  0  2')
            input("Press Enter to sendGPIBCommand(b'SEND  0  0  1  0').")
            sendGPIBCommand(b'SEND  0  0  1  0')
            input("Press Enter to sendGPIBCommand(b'SEND  0  0  2  0').")
            sendGPIBCommand(b'SEND  0  0  2  0')
            exit()

            setWavelengths(3,0,9, 0)

            getStatus()
            #getSpectra()
            #sendGPIBCommand(b'CAED  0')

            #exit()
            GPIBTimeout = 30000



            testRunSpectra()
            """sendGPIBCommand(b'TITV  0 16  20', 0)
            sendGPIBCommand(b'TIMP  0       0      50', 0)
            sendGPIBCommand(b'DMOD  0  1  0  1  1', 0)
            while getStatus():
                print(getStatus())
            setSlit(0)
            while getStatus():
                print(getStatus())
            sendGPIBCommand(b'RESP  0  0', 0)
            sendGPIBCommand(b'SEND  0  0  1  1', 0)
            time.sleep(5)
            getSpectra()"""
            """instrument.write_raw(b'SSPC  0  1' + b'\x0d')
            instrument.wait_for_srq(GPIBTimeout)
            instrument.wait_for_srq(GPIBTimeout)
            incomingData = instrument.read_raw()
            globals.updateSpectra(Spectra=incomingData[9:])
            sendGPIBCommand(b'RESP  0  0', 0)
            print(incomingData)
            # todo write RESP and wait for a spectra back after checking for SRQ a couple of times
            instrument.wait_for_srq(GPIBTimeout)
            #instrument.wait_for_srq(GPIBTimeout)
            incomingData = instrument.read_raw()
            print(incomingData)"""
            exit()
            #getSpectra()
            setSlit(3)
            getStatus()
            print(testGetSpectra())

            while 0:
                getStatus()
                getSpectra()
                """sendGPIBCommand(b'SEND  0  0  0  0')
                getSpectra()
                sendGPIBCommand(b'SEND  0  0  1  0')
                getSpectra()
                sendGPIBCommand(b'SEND  0  0  2  0')
                getSpectra()
                sendGPIBCommand(b'SEND  0  0  3  0')
                getSpectra()
                sendGPIBCommand(b'SEND  0  0  0  1')
                getSpectra()
                sendGPIBCommand(b'SEND  0  0  1  1')
                getSpectra()
                sendGPIBCommand(b'SEND  0  0  2  1')
                getSpectra()
                sendGPIBCommand(b'SEND  0  0  3  1')
                getSpectra()"""
            while getStatus():
                print(getStatus())


            while 0:
                sendGPIBCommand(b'SEND  0  0  0  0')
                getSpectra()
                sendGPIBCommand(b'SEND  0  0  1  0')
                getSpectra()
                sendGPIBCommand(b'SEND  0  0  2  0')
                getSpectra()
                sendGPIBCommand(b'SEND  0  0  3  0')
                getSpectra()
                sendGPIBCommand(b'SEND  0  0  0  1')
                getSpectra()
                sendGPIBCommand(b'SEND  0  0  1  1')
                getSpectra()
                sendGPIBCommand(b'SEND  0  0  2  1')
                getSpectra()
                sendGPIBCommand(b'SEND  0  0  3  1')
                getSpectra()

        #print(sendGPIBCommand(b'IERR  0'))
        #sendGPIBCommand(b'STAT  0\x0d\x2f')
        #instrument.write_raw(b'\x49\x45\x52\x52\x20\x20\x30\x0D')
        #print(instrument.read_stb())
        #instrument.wait_for_srq(10000)

        #time.sleep(1)
        #print(instrument.read_stb())
        #print(instrument.read_raw())
        #print(instrument.InterfaceType())
        #print(line_state = interface.get_line_state())
        #print(instrument.LineState())

        #status_byte = instrument.read_stb()

        #data = instrument.read_raw()
        #print(data)
        #sendGPIBCommand(instrument, b'\x00\x02\x00\x02')
        #print("Waiting for SRQ event...")

        # Main loop to keep the program running


    except pyvisa.VisaIOError as e:
        print(f'An error occurred: {e}')

    finally:
        # Disable SRQ event handling
        time.sleep(1)
        instrument.disable_event(pyvisa.constants.EventType.service_request, pyvisa.constants.EventMechanism.queue)

        # Close the instrument connection
        instrument.close()


# Replace with your GPIB address
#gpib_address = 7
main()

"""
pin 1 (far inside of unit) = intensity to 10V, resets every 25ms. Seems to start from the side of chip toward the light, each pixel is on/off for 22ms
2 GND
3 -12V
4 GND
5 +12V
6 
7 GND
8 286 kHz square wave, each pixel is 3 clock pulses with another 3 pulses in the blank period
9 GND
10 5v that pulses to 0V for 14us every every 42us, new pixel starts on falling edge, should be read on rising edge
11 GND
12 5v that pulses to 0V for 21us every every 25ms, the one pixel long pulse indicates the end of what the sensor is able to read. Discard rest of data? Not seeing a start pulse though
13 GND
14 5V supply
15 GND

 IC8 VHC, 244, 9 21
 IC1 PALCE16V8, Z-25SC, 9930DEA
 IC2 same as IC8
 IC5 BB    OPA605KP, 0004 7556
 IC6 HA3, 2525-5, H9926BF
"""