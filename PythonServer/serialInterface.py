#import serial
import threading
import time
from collections import namedtuple
from datetime import datetime
#import socket
#import pickle
import globals
import networkInterface
import readSerial
import writeSerial
import DAD



def main():
    # Configure the serial port
    #global ser
    #ser = serial.Serial(
    #    port='COM7',
    #    baudrate=115200,
    #    bytesize=serial.EIGHTBITS,
    #    parity=serial.PARITY_NONE,
    #    stopbits=serial.STOPBITS_ONE
    #)
    #ser.close()
    #exit()

    #ser.write(b'\x99')
    # Start a thread to read from the serial port
    SerialReadThread = threading.Thread(target=readSerial.getSerialPacket, args=())
    SerialReadThread.daemon = True
    SerialReadThread.start()
    """recieveThreadAsync = threading.Thread(target=readSerial.recieveThread, args=())
    recieveThreadAsync.daemon = True
    recieveThreadAsync.start()"""
    #ExternalCommThread = threading.Thread(target=networkInterface.startServer, args=())
    #ExternalCommThread.daemon = True
    #ExternalCommThread.start()
    globals.autopollThreadStopEvent = threading.Event()
    #serialAutopollThread = threading.Thread(target=writeSerial.Autopoll, args=())
    #serialAutopollThread.daemon = True
    #serialAutopollThread.start()
    DADThread = threading.Thread(target=DAD.main, args=())
    DADThread.daemon = True
    DADThread.start()




    ser = globals.ser


    time.sleep(1)
    #ser.write(readSerial.calculate_crc(
    #    b'\x02\x11\x00\x10\x02\xFF\x02\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\x0F'))  # trying single line init
    input("Press Enter to continue.")
    time.sleep(2)
    #ser.write(readSerial.calculate_crc(b'\x0D\x04\x00\x00\x01\x0D\x05'))
    time.sleep(.050)
    #ser.write(readSerial.calculate_crc(b'\x0D\x12\x00\x10\x02\xFF\x02\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\x0D\xFF\x21'))
    time.sleep(.050)
    #ser.write(calculate_crc(b'\x0D\x05\x00\x30\x0D\xFF\x0F'))
    time.sleep(1.050)
    #ser.write(readSerial.calculate_crc(b'\x0D\x06\x00\x02\x18\x00\x00\x01'))
    exit()


    try:
        while True:
            time.sleep(1)
            # ser.write(calculate_crc(b'\x0F\x07\x00\x03\x01\x00\x00\x00\x32'))
            # pollInst("ovenTemp")
            #pollInst("pump")
            # print("this is a tuple: %s" % (ovenContainer.ovenTemp,))
            # print(f"Oven Temperature: {ovenContainer.ovenTemp}")
            # print(ovenContainer.ovenTemp,)
            # print(ovenContainer.ovenTemp,)
            # print(ovenContainer.timestamp)
            # Main thread can do other tasks or just sleep
            pass
    except KeyboardInterrupt:
        print("Program terminated by user.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
