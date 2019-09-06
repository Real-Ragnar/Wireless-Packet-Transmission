#include <NRF_Module.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CARD_SIZE 128
MPU6050 mpu;
NRF_Module nrf;
#define LED_PIN 13 
#define HI_MPU_THRESHOLD 640
#define LO_MPU_THRESHOLD 560 
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
//volatile becase system cannot control this variable

void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      MPU DMP SETUP                       ===
// ================================================================
int FifoAlive = 0;     // tests if the interrupt is triggering
int IsAlive = -20;      // counts interrupt start at -20 to get 20+ good values before assuming connected
// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector



//mpu global variables
float ax,ay,az;
int array_z[260];
int array_z_raw[260];
unsigned int array_z_tosend[128];             
int acc_val_scaled=0;                         //scaled value of raw acceleration data from mpu6050
dynacard card_main;                           //card holding raw acceleration(128 ints) and load cell data (128 ints)
cardDataMessagePack cardDataMessagePack0;     //object containing array of 18 message-structures
static int state = 0;                                //send one packet/message-structure(32 bytes) per loop 
int counter = 0;                              //counts the number of loops elapsed

void MPU6050Connect() {
static int MPUInitCntr = 0;
  // initialize device
  mpu.initialize(); // same
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();// same

  if (devStatus != 0) 
  {// ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    char * StatStr[5] { "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};    
  //where 3 and 4 are other error outputs of the dmpInitialize() function

    MPUInitCntr++;

    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus]);
    Serial.println(F(")"));

    if (MPUInitCntr >= 10) return; //only try 10 times
    delay(1000);
    MPU6050Connect(); // Lets try again
    return;
  }
  
 
 
  //Calibration values
  mpu.setXAccelOffset(-1863);
  mpu.setYAccelOffset(-22);
  mpu.setZAccelOffset(1006);

  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  
  // enable Arduino interrupt detection
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt pin 2 on the Uno)..."));
  attachInterrupt(0, dmpDataReady, FALLING); //Pin 2 on the mega is an interrupt pin. This corresponds to interrupt vector 0
  //dmpDataReady is the function that is run when this interrupt condition is met
  //FALLING is the pin action that triggers the interrupt

  mpuIntStatus = mpu.getIntStatus(); 
  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  delay(1000); // Let it Stabalize
  mpu.resetFIFO(); // Clear fifo buffer
  mpu.getIntStatus();
  mpuInterrupt = false; // wait for next interrupt

}
// ================================================================
// ===                      i2c SETUP Items                     ===
// ================================================================
void i2cSetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

// ================================================================
// ===                      MPU and radio setup                 ===
// ================================================================
void setup() {
  nrf.txSetup();
  Serial.begin(115200); 
  for(int i=0; i<128;i++)    //filling load array with some numbers for testing purposes
    {
      card_main.load[i]=i;
    }
  while (!Serial);
  Serial.println(F("I2C Setup"));
  i2cSetup();

  Serial.println(F("Alive"));
  MPU6050Connect();
  Serial.println(F("after connect"));
  pinMode(LED_PIN, OUTPUT); // LED Blinks when you are recieving FIFO packets from your MPU6050

  /*fill array with 0 to begin with */
  initializeArr();

  counter = 0;        //initialize counter
}

void loop() {
  static long lInteruptTimeout = millis();
  if (mpuInterrupt ) 
  { // wait for MPU interrupt or extra packet(s) available
    GetDMP(); // Gets the MPU Data and calculates angles
    lInteruptTimeout = millis();
  }
  else
  {
    if ((long)millis() - lInteruptTimeout > 1000)
    {
      MPU6050Connect();
      lInteruptTimeout = millis();
    }
  }
  
// ================================================================
// ===         POLLING MPU6050 AND TRANSMITTING VIA nRF         ===
// ================================================================
  static long QTimer = millis();
  if ((long)( millis() - QTimer ) >= 100)                   //poll for mpu acceleration data every 100 ms
  {
       QTimer = millis();
       counter++;                                           //===========================TODO====================================
       acc_val_scaled = constrain(az,az-700,az+700);        //getting rid of random spikes ==========================="az" in the second and third arguments should be the value of az in the previous cycle
       acc_val_scaled = map(acc_val_scaled,-600,1500,0,2100);   //move the range of values away from 0 in order to get positive value

      if(acc_val_scaled>HI_MPU_THRESHOLD)
      {
        array_z[counter] = acc_val_scaled;                  //this array analyzed to search for beginning and end of stroke
        array_z_raw[counter] = acc_val_scaled;              //this array stores raw values to be sent over nRF module via packets
        printRawAccValue();
      }
      else if (acc_val_scaled<(LO_MPU_THRESHOLD))
      {
        array_z[counter] = -acc_val_scaled;
        array_z_raw[counter] = (acc_val_scaled);
        printRawAccValue();
      }
      else
      {
        array_z[counter] = 0;
        array_z_raw[counter] = acc_val_scaled;
        printRawAccValue();
      }
   
   bool upFlag=false;
   int upStartIndex1 = 0;
   int upStartIndex2 = 0;
   float toSendSize = 0;
   float offset = 0;
   
   //after every 250 values, find where the stroke data occurs then print it out
   if (counter == 250)       //we have enough data now to search through array for upstroke period
   {
    for(int i = 1; i<251; i++)    //scan array for an upstroke and mark the indexes
    {
        if((array_z[i] > 0)&&(upFlag==false)&&(array_z[i-1]==0)&&(i>1))         //if upstroke detected, start storing values into 'array to send'
        { 
            upStartIndex1 = i;                          //mark this point 
            Serial.println("Start of upstroke: ");
            Serial.println(upStartIndex1);
            upFlag = true;
        }  
        if(array_z[i] > 0 && upFlag == true && (i>upStartIndex1 + 25)&&(array_z[i-1]==0))    //this if only reached if second upstroke detected 
          {
            //second upstroke detected
            upStartIndex2 = i;
            Serial.println("Start of the next upstroke: ");
            Serial.println(upStartIndex2);
          }   
    }
  
    //at this point, we now have two indexes where we can extract data
    toSendSize = upStartIndex2 - upStartIndex1;
    //TODO===============================================add if here to check if toSendSize<128
    //here we are assuming it is greater than 128
    
    offset = toSendSize / 128;
    int raw_increment = 0;
    //store every 'offset'th' data point from array_z_raw into array_z_tosend

    Serial.println(toSendSize);
    Serial.println(offset);
    
    for(int i=0; i<128; i++)                                  //fill the 'tosend' array with raw values 
    {
      raw_increment = upStartIndex1+(int)((float)i*offset);   //casting to int truncates the float value
      array_z_tosend[i] = array_z_raw[raw_increment];
      Serial.print(raw_increment);                            //print the index of the array value extracted 
      Serial.print(" ");
    }

    Serial.println(" ");                                      //print the array that is to be sent via nRF module
    for(int i=0; i<128;i++)
    {
      Serial.print(" ");
      Serial.print(array_z_tosend[i]);
      Serial.print(" ");
      Serial.print(i);
      Serial.print(",");
    }

      memcpy(card_main.raw_acceleration,array_z_tosend,256);    //copy 256 bytes of the collection array to the array to be sent over radio...Improvement, to collect data into raw_acceleration w/o intermediate array_z_tosend
      cardDataMessagePack0 = nrf.cardToMessage(card_main);      //result of the break up of the larger array into packets
      state = 1;                                                //initialize to 1 here then use switch statement to send 1 packet per loop
  }//end of if(counter == 250)

  switch(state) {           //if counter is not 250, state will be 0 for all loops
    case 1  :
      counter = 0;
      state = 2;
      Serial.print("inside loop");
      nrf.txLoop(cardDataMessagePack0.dataBlock[0]);
      break; 
    case 2  :
      state = 3;
      Serial.print("inside loop2");
      nrf.txLoop(cardDataMessagePack0.dataBlock[1]);
      break; 
    case 3:
      state = 4;
      Serial.print("inside loop3");
      nrf.txLoop(cardDataMessagePack0.dataBlock[2]);
      break;
    case 4:
      Serial.print("inside loop4");
      state = 5;
      nrf.txLoop(cardDataMessagePack0.dataBlock[3]); 
      break;     
    case 5:     //
      Serial.print("inside loop5");
      state = 6;
      nrf.txLoop(cardDataMessagePack0.dataBlock[4]);
      break; 
    case 6:     //
      Serial.print("inside loop6");
      state = 7;
      nrf.txLoop(cardDataMessagePack0.dataBlock[5]);
      break; 
    case 7:     //
      Serial.print("inside loop7");
      state = 8;
      nrf.txLoop(cardDataMessagePack0.dataBlock[6]);
      break; 
    case 8:     //
      Serial.print("inside loop8");
      state = 9;
      nrf.txLoop(cardDataMessagePack0.dataBlock[7]);
      break; 
    case 9:     //
      Serial.print("inside loop9");
      state = 10;
      nrf.txLoop(cardDataMessagePack0.dataBlock[8]);
      break; 
    case 10:     //
      Serial.print("inside loop10");
      state = 11;
      nrf.txLoop(cardDataMessagePack0.dataBlock[9]);
      break; 
    case 11:     //
      Serial.print("inside loop11");
      state = 12;
      nrf.txLoop(cardDataMessagePack0.dataBlock[10]);
      break;   
    case 12:     //
      Serial.print("inside loop12");
      state = 13;
      nrf.txLoop(cardDataMessagePack0.dataBlock[11]);
      break;   
    case 13:     // 
      Serial.print("inside loop13");
      state = 14;
      nrf.txLoop(cardDataMessagePack0.dataBlock[12]);
      break;   
    case 14:     //
      Serial.print("inside loop14");
      state = 15;
      nrf.txLoop(cardDataMessagePack0.dataBlock[13]);
      break;   
    case 15:     //
      Serial.print("inside loop15");
      state = 16;
      nrf.txLoop(cardDataMessagePack0.dataBlock[14]);
      break;   
    case 16:     //
      Serial.print("inside loop16");
      state = 17;
      nrf.txLoop(cardDataMessagePack0.dataBlock[15]);
      break;   
    case 17:     //
      Serial.print("inside loop17");
      state = 18;
      nrf.txLoop(cardDataMessagePack0.dataBlock[16]);
      break;   
    case 18:     
      Serial.print("inside loop18");
      state = 19;
      nrf.txLoop(cardDataMessagePack0.dataBlock[17]); 
      break; 
    default : 
      Serial.print("");
      
  }//end of switch statement
}//end of 100 ms timer
  

  
  char cInput;
    while(Serial.available() > 0)
    {
      cInput = Serial.read();
      Serial.print(cInput);            
      switch(cInput)
      {
        case 'a':
          Serial.println(": Reset ALL");
          setup();
          break;
        case 'b':
          Serial.println(": Reset I2C");
          i2cSetup();
         break;        
        case 'c':
          Serial.println(": Reset DMP");
          MPU6050Connect();
         break;        
        default:
          Serial.println(": Unknown"); 
          break;
      }
      while(Serial.read()>0);
    }
}





/*
have one main array of size 2000 storing all point

using another array go through all points of the above array and set index at first upstroke detection and another index when the next upstroke is detected
store all points inbetween into array to send
*/
void initializeArr(){
    for(int i=0; i<260; i++)
  {
    array_z[i] = 0;
  }
  for(int i=0; i<260; i++)
  {
    array_z_raw[i] = 0;
  }
  for(int i=0; i<128; i++)
  {
    array_z_tosend[i] = 0;
  }
}


void printRawAccValue(){
        Serial.print(acc_val_scaled); 
        Serial.print(" ");
        Serial.print(counter); 
        Serial.println(" ");
}

void GetDMP() { // Best version I have made so far
  // Serial.println(F("FIFO interrupt at:"));
  // Serial.println(micros());
  mpuInterrupt = false;
  FifoAlive = 1;
  fifoCount = mpu.getFIFOCount();
  /*
  fifoCount is a 16-bit unsigned value. Indicates the number of bytes stored in the FIFO buffer.
  This number is in turn the number of bytes that can be read from the FIFO buffer and it is
  directly proportional to the number of samples available given the set of sensor data bound
  to be stored in the FIFO
  */

  // PacketSize = 42; refference in MPU6050_6Axis_MotionApps20.h Line 527
  // FIFO Buffer Size = 1024;
  uint16_t MaxPackets = 20;// 20*42=840 leaving us with  2 Packets (out of a total of 24 packets) left before we overflow.
  // If we overflow the entire FIFO buffer will be corrupt and we must discard it!

  // At this point in the code FIFO Packets should be at 1 99% of the time if not we need to look to see where we are skipping samples.
  if ((fifoCount % packetSize) || (fifoCount > (packetSize * MaxPackets)) || (fifoCount < packetSize)) { // we have failed Reset and wait till next time!
    digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
    Serial.println(F("Reset FIFO"));
    if (fifoCount % packetSize) Serial.print(F("\t Packet corruption")); // fifoCount / packetSize returns a remainder... Not good! This should never happen if all is well.
    Serial.print(F("\tfifoCount ")); Serial.print(fifoCount);
    Serial.print(F("\tpacketSize ")); Serial.print(packetSize);

    mpuIntStatus = mpu.getIntStatus(); // reads MPU6050_RA_INT_STATUS       0x3A
    Serial.print(F("\tMPU Int Status ")); Serial.print(mpuIntStatus , BIN);
    // MPU6050_RA_INT_STATUS       0x3A
    //
    // Bit7, Bit6, Bit5, Bit4          , Bit3       , Bit2, Bit1, Bit0
    // ----, ----, ----, FIFO_OFLOW_INT, I2C_MST_INT, ----, ----, DATA_RDY_INT

    /*
    Bit4 FIFO_OFLOW_INT: This bit automatically sets to 1 when a FIFO buffer overflow interrupt has been generated.
    Bit3 I2C_MST_INT: This bit automatically sets to 1 when an I2C Master interrupt has been generated. For a list of I2C Master interrupts, please refer to Register 54.
    Bit1 DATA_RDY_INT This bit automatically sets to 1 when a Data Ready interrupt is generated.
    */
    if (mpuIntStatus & B10000) { //FIFO_OFLOW_INT
      Serial.print(F("\tFIFO buffer overflow interrupt "));
    }
    if (mpuIntStatus & B1000) { //I2C_MST_INT
      Serial.print(F("\tSlave I2c Device Status Int "));
    }
    if (mpuIntStatus & B1) { //DATA_RDY_INT
      Serial.print(F("\tData Ready interrupt "));
    }
    Serial.println();
    //I2C_MST_STATUS
    //PASS_THROUGH, I2C_SLV4_DONE,I2C_LOST_ARB,I2C_SLV4_NACK,I2C_SLV3_NACK,I2C_SLV2_NACK,I2C_SLV1_NACK,I2C_SLV0_NACK,
    mpu.resetFIFO();// clear the buffer and start over
    mpu.getIntStatus(); // make sure status is cleared we will read it again.
  } else {
    while (fifoCount  >= packetSize) { // Get the packets until we have the latest!
      if (fifoCount < packetSize) break; // Something is left over and we don't want it!!!
      mpu.getFIFOBytes(fifoBuffer, packetSize); // lets do the magic and get the data
      fifoCount -= packetSize;
    }
    MPUMath(); // <<<<<<<<<<<<<<<<<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<<<<<
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the Light
    if (fifoCount > 0) mpu.resetFIFO(); // clean up any leftovers Should never happen! but lets start fresh if we need to. this should never happen.
  }
}


void MPUMath() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  ax = aaReal.x;
  ay = aaReal.y;
  az = aaReal.z;
}
  
