import pyvisa
#import constants
import time

from pyvisa import ResourceManager, constants

rm = pyvisa.ResourceManager()


def DADTest():
    hexString = '494552522020300D'
    hexBytes = bytes.fromhex(hexString)
    DAD.write_raw(hexBytes)  # works
    #time.sleep(1)
    print(DAD.read_stb())
    #time.sleep(1)
    incomingData = (DAD.read_raw())
    print(incomingData)


def D7000Test():
    hexString = '00020002 '
    hexBytes = bytes.fromhex(hexString)
    D7000.write_raw(hexBytes)  # works
    #time.sleep(1)
    #print(D7000.read_stb())
    #time.sleep(1)
    incomingData = D7000.read_raw()
    #print(incomingData)


def initGPIB(): #check to see if the devices have anything waiting on the port, get rid of it if it does
    #this is where I am able to read the stuff that was generated a while ago without problem, but if I write and read again, even with a delay, nothing comes over
    if (DAD.read_stb()):
        DAD.read_raw()
        print("flushed DAD")
    if (D7000.read_stb()):
        D7000.read_raw()
        print("flushed D7000")
    time.sleep(1)


#print(rm.list_resources())
DAD = rm.open_resource('GPIB0::7::INSTR')
D7000 = rm.open_resource('GPIB0::15::INSTR')

#set up the GPIB with the correct endings
DAD.writeTermination = ''
DAD.readTermination = ''
DAD.send_end = True #this line enables the EOI line to assert when the message is done
D7000.writeTermination = None
D7000.readTermination = None
D7000.send_end = True


#initGPIB()  #flush GPIB

#I was trying the below both within and outside of a function, same result
hexString = '00020002'  #get SN string
hexBytes = bytes.fromhex(hexString)
D7000.write_raw(hexBytes)  #This does send the correct message
#time.sleep(1)
#print(D7000.read_stb())        #added status read to simulate the initGPIB() function
#time.sleep(1)                  #tried a delay before and after the status check
incomingData = D7000.read_raw()     #this times out even though I can see the ServiceRequest pin indicating data on the GPIB status screen, I also tried the normal GPIB.read()
#print(incomingData)


#D7000Test()

quit()  #this is here because Python has no block comment feature


#kind of works, no longer errors out anyway. When I manually force the ServiceRequest pin on the GPIB port nothing happens

def handle_event(resource, event, user_handle):
    resource.called = True
    print(f"Handled event {event.event_type} on {resource}")


with rm.open_resource('GPIB0::7::INSTR') as instr:

    instr.called = False

    # Type of event we want to be notified about
    event_type = constants.EventType.service_request
    # Mechanism by which we want to be notified
    event_mech = constants.EventMechanism.handler

    wrapped = instr.wrap_handler(handle_event)

    user_handle = instr.install_handler(event_type, wrapped, 42)
    instr.enable_event(event_type, event_mech, None)

    # Instrument specific code to enable service request
    # (for example on operation complete OPC)
    #instr.write("*SRE 1")
    #instr.write("INIT")

    while not instr.called:
        time.sleep(1)
        print("test")       #making sure this is even running
    print("called")         #this is never called when I force the SRQ pin
    instr.disable_event(event_type, event_mech)
    instr.uninstall_handler(event_type, wrapped, user_handle)




