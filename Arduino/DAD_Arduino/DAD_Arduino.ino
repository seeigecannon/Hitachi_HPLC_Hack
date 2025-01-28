#include <avr/wdt.h>  // Include watchdog timer library
#include <TimerOne.h>
#include <AccelStepper.h>

#define DAC_Ready_Pin 21
#define hplcDigA 20
#define hplcDigB 19
#define hplcDigC 18
#define newlinePin 20
#define bufferSize 1200 //1800
#define pixelsToSkip 0

byte DADBuffer0[bufferSize];
byte DADBuffer1[bufferSize];
//byte DADBuffer2[bufferSize];
int skipPixelCounter=0;
int DADBufPos=0;
volatile byte useBufferNum = 0;
bool okToTransmit = false;
unsigned long int DADTimeStamp=0;

int inputStatusTest=0;

byte count=0;

const unsigned long TIMEOUT_MS = 3;  // Timeout for reading a packet
const size_t MAX_PACKET_SIZE =30;     // Maximum packet size (adjust as needed)
bool isMaster = 0;  //if true the arduino will acknologe packets and transmit to HPLC devices
bool previousPumpPinState = PINE & _BV(PINE5);
bool previousASPinState = PING & _BV(PING5);

//motion control stuff
const int pulsePin = 2;    // Step pin
const int dirPin = 5;      // Direction pin
const int homeSensorPin = 7; // Optical homing sensor pin (active low)
AccelStepper myStepper(AccelStepper::DRIVER, pulsePin, dirPin);  // Use DRIVER mode (pulse/direction)
#define motorEnPin 8
float targetAngle = 90.0;  // Desired angle in degrees (or any unit for your system)
float currentAngle = 0.0;  // Current angle, initially at 0
// Motor steps per degree (this depends on your motor's configuration)
const float stepsPerDegree = 100.0;  // Adjust to your motor's specifics
volatile bool homingComplete = false;  // Flag to indicate homing status






void setup() {
  

  Serial.begin(460800);

  Serial2.begin(19200);

  clearSerialBuffer();
  clearSerial1Buffer();
  
  DDRA = 0xFF;
  DDRC = 0xFF;


  DDRL = 0x00;  //PORTC and PORTL as inputs
  DDRC = 0x00;
  DDRD = 0x00;

  pinMode(DAC_Ready, INPUT);
  pinMode(newlinePin, INPUT);
  //attachInterrupt(digitalPinToInterrupt(DAC_Ready_Pin), DAC_Ready, RISING);
  //attachInterrupt(digitalPinToInterrupt(newlinePin), DAD_NewLine, FALLING);


  //motion control stuff
  myStepper.setMaxSpeed(1000);  // Max speed
  myStepper.setAcceleration(500);  // Acceleration
  pinMode(homeSensorPin, INPUT_PULLUP);  // Assume sensor is active low
  Timer1.initialize(1000);  // 1000 microseconds = 1ms, adjust for the desired frequency
  Timer1.attachInterrupt(runStepper);  // Calls runStepper() every time interrupt triggers
  // Home the motor (set current position to 0)
  homeMotor();

  wdt_enable(WDTO_1S);    //enable use of WDT
}


  





void loop() {
  float targetPosition = targetAngle * stepsPerDegree;

  // Move the motor to the target position
  myStepper.moveTo(targetPosition);

  // Optionally: Check and print current position
  Serial.print("Current Position (steps): ");
  Serial.println(myStepper.currentPosition());

  // Monitor motor state: If the motor has reached its target, disable the interrupt
  if (myStepper.distanceToGo() == 0) {
    // Motor has reached its target, disable the timer interrupt
    Timer1.detachInterrupt();
    Serial.println("Motor is in position, interrupt disabled.");
  }
  else{
    Timer1.attachInterrupt(runStepper);
  }



  if(DADBuffer1[bufferSize-5]){
    sendBuffer(DADBuffer1);
  }
  if(DADBuffer0[bufferSize-5]){
    sendBuffer(DADBuffer0);
  }


    

  
  
  //Serial2.write(0x61);
  wdt_reset();    //kick the watchdog
  //checkInputPins();
  checkHPLCSerial();
  checkHostSerial();
  //Serial2.write(0x60);
  
}

void runStepper() {
  myStepper.run();  // Call run() to move the motor step by step

  // If the motor has finished its move, stop the interrupt
  if (myStepper.distanceToGo() == 0) {
    Timer1.detachInterrupt();  // Disable the interrupt if the motor is in position
  }
}

void homeMotor() {
  // Move motor in one direction until home sensor is triggered
  Serial.println("Homing motor...");

  myStepper.setMaxSpeed(200);  // Slow speed for homing
  myStepper.setAcceleration(100);  // Low acceleration for homing

  while (digitalRead(homeSensorPin) == HIGH) {  // Active LOW, so HIGH means not yet triggered
    myStepper.moveTo(10000);  
    myStepper.run();
  }

  // Sensor triggered, motor is homed
  Serial.println("Homing complete!");
  myStepper.setCurrentPosition(0);  // Set current position to 0

  // Optionally, reset the motor speed and acceleration
  myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(500);
}


void sendBuffer(byte buffer[]){
  Serial.print("PKTDAD:");
  for(unsigned int i=0; i<bufferSize; i++){
    Serial.write(buffer[i]);
    //Serial.write(0x01);
  }
  //Serial.print("test");
  Serial.print("**-");
  memset(buffer,0,bufferSize);
}






void DAC_Ready(){
  noInterrupts();
  byte highByte = PINL ^=0b10000000;  //signal is aliesing, invert first bit to help somehow

  byte lowByte = PINC;

  if(skipPixelCounter<pixelsToSkip){
    skipPixelCounter++;
    interrupts();
    return;
  }


  if(DADBufPos>bufferSize-7){

    DADBufPos=300;
  }

  switch(useBufferNum){
    case 0:
      PORTA = PORTA | 10100000;   //27 write buffer 0
      DADBuffer0[DADBufPos]=highByte;
      DADBuffer0[DADBufPos+1]=lowByte;

      break;
    case 1:

      DADBuffer1[DADBufPos]=highByte;
      DADBuffer1[DADBufPos+1]=lowByte;

      break;
//    case 2:
//
//      DADBuffer2[DADBufPos]=highByte;
//      DADBuffer2[DADBufPos+1]=lowByte;
//
//      break;
    }
  interrupts();
  DADBufPos=DADBufPos+2;

}
void DAD_NewLine(){
  //Serial.println("new line");
  noInterrupts();
  DADBufPos = 0;
  skipPixelCounter=0;
  unsigned long currentTime = millis();

  useBufferNum++;
  if(useBufferNum==2){
    useBufferNum=0;
  }
  //Serial.print("case");Serial.print(useBufferNum);c:\Users\Casey\Desktop\AccelStepper-master\examples\MultiStepper\MultiStepper.pde
  
  switch(useBufferNum){
    case 0:

      //Serial.print("initA");
      DADBuffer1[bufferSize-5]=0x30;
      DADBuffer0[bufferSize-6]=PIND;
      DADBuffer0[bufferSize-4]=(currentTime >> 24) & 0xFF;
      DADBuffer0[bufferSize-3]=(currentTime >> 16) & 0xFF;
      DADBuffer0[bufferSize-2]=(currentTime >> 8) & 0xFF;
      DADBuffer0[bufferSize-1]=(currentTime) & 0xFF;
      break;
    case 1:

      //Serial.print("initB");
      DADBuffer0[bufferSize-5]=0x31;
      DADBuffer1[bufferSize-4]=(currentTime >> 24) & 0xFF;
      DADBuffer1[bufferSize-3]=(currentTime >> 16) & 0xFF;
      DADBuffer1[bufferSize-2]=(currentTime >> 8) & 0xFF;
      DADBuffer1[bufferSize-1]=(currentTime) & 0xFF;
      break;

  }

  okToTransmit = true;
  interrupts();
}

bool isValidPacket(byte* packet, size_t length) { //check the checksum of the packet. Return true if the calculated and actual CS match
  byte checksum = 0;
  for (size_t i = 0; i < length - 1; i++) {
    //Serial.print(packet[i], HEX);
    checksum ^= packet[i];
  }
  
  //Serial.write(checksum);
  //Serial.println("=checksum");
  if(checksum == packet[length - 1]){
    return 1;
  }
  //else{
  //  Serial2.write(packet, length);
  //  return 0;
  //}
}

void clearSerial1Buffer() {   //read serial buffer until empty
  while (Serial2.available()) {
    Serial2.read();
  }
}
void clearSerialBuffer() {
  while (Serial.available()) {
    Serial.read();
  }
}

void checkHPLCSerial(){             //check for incoming data from HPLC on Serial1. Do stuff if this is the case
  if (Serial2.available()) {        //see if data is available
    if(Serial2.peek()==0x06){   //see if the next byte is an ACK byte, if so, discard it
      Serial2.read();
      return;
    }
    //Serial2.write(0x21);
    //Serial.print("incoming serial");
    unsigned long startTime = millis();
    while (Serial2.available() < 2) {           //wait for at least 2 bytes to be available in the buffer
      if (millis() - startTime >= TIMEOUT_MS) { //timeout, clear the buffer
        //Serial.println("timeout");
        
        clearSerial1Buffer();
        //Serial2.write(0x22);
        return;
      }
    }

    byte firstByte = Serial2.read();  //store first byte
    byte secondByte = Serial2.read(); //store second byte
    size_t packetLength = secondByte+3; //second byte is the packet length. Add 3 because the length byte is the message size instead of the packet size

    if (packetLength > MAX_PACKET_SIZE) { //packet claims to be too big, must have been a bad read, clear buffer and start over
      clearSerial1Buffer();
      return;
    }

    byte packet[MAX_PACKET_SIZE] = {0}; 
    packet[0] = firstByte;
    packet[1] = secondByte;

    startTime = millis();
    for (size_t i = 2; i < packetLength; i++) {
      while (!Serial2.available()) {  //check timeout while no data is available
        if (millis() - startTime >= TIMEOUT_MS) {
          clearSerial1Buffer();
          Serial2.write(0x23);
          return;
        }
      }
      startTime = millis(); //reset the timeout when a byte is available
      packet[i] = Serial2.read();
    }
    //Serial2.write(0x24);
    //Serial.print(packet,HEX);
    if (isValidPacket(packet, packetLength)) {  //check to see if the packet claimed CS matches the calculated CS
      if(isMaster){           // respond with ACK if the packet is detected as being good
        Serial2.write(0x06);
      }
      //Serial.println("CS good");
      Serial.print("PKTCOM:");               //encapulsate packet so it is easy to pull in Python
      Serial.write(packet, packetLength);
      Serial.print("***");
    }
  }
}

void checkHostSerial(){
  if (Serial.available()) {
    //Serial2.write(0x53);
    if(Serial.peek()==0x99){  //check to see if first byte is a 0x99, if so, the Arduino is the communications master and should respond to packets
      isMaster = 1;
      //Serial2.write(0x52);
      clearSerialBuffer();
      return;
    }
    if(Serial.peek()==0x98){  //check to see if first byte is a 0x98, if so, the Arduino is no longer the communications master and should not respond to packets
      isMaster = 0;
      clearSerialBuffer();
      return;
    }
    unsigned long startTime = millis();
    while (Serial.available() < 2) {
      if (millis() - startTime >= TIMEOUT_MS) {
        clearSerialBuffer();
        //Serial2.write(0x50);
        return;
      }
    }
    byte firstByte = Serial.read();
    byte secondByte = Serial.read();
    size_t packetLength = secondByte+3;
    //Serial2.write(0x54);
    if (packetLength > MAX_PACKET_SIZE) {
      clearSerialBuffer();
      //Serial2.write(0x55);
      return;
    }

    byte masterPacket[MAX_PACKET_SIZE] = {0};
    masterPacket[0] = firstByte;
    


    masterPacket[1] = secondByte;

    startTime = millis();
    for (size_t i = 2; i < packetLength; i++) {
      while (!Serial.available()) {
        if (millis() - startTime >= TIMEOUT_MS) {
          clearSerialBuffer();
          //Serial2.write(0x56);
          return;
        }
      }
      masterPacket[i] = Serial.read();
    }

    if (isValidPacket(masterPacket, packetLength)) {  //leave packet validation in as a double check that the PC is acting correctly
      //Serial2.write(masterPacket, packetLength);
      Serial2.write(masterPacket, packetLength);
      Serial2.flush();
      clearSerial1Buffer();
    }
    else{
      //Serial2.write(0x51);
    }
  }
}

void checkInputPins(){
  bool currentPumpPinState = PINE & _BV(PINE5);
  bool currentASPinState = PING & _BV(PING5);
  if (currentPumpPinState && !previousPumpPinState) {
    if(currentASPinState){
      Serial.print("\rPKT:");
      Serial.write(0x0002029900);
      Serial.print("***");
    }
    /*else{
      Serial.print("\rPKT:");
      Serial.print("pump L");
      Serial.print("***");
    }*/

  }
  
  if (currentASPinState && !previousASPinState) {
    if(currentASPinState){
      Serial.print("\rPKT:");
      Serial.write(0x00020D9900);
      Serial.print("***");
    }
    else{
      Serial.print("PKT:");
      Serial.print("AS L");
      Serial.print("***");
    }
  }
  //previousPumpPinState=currentPumpPinState;
  //previousASPinState=currentASPinState;

}
