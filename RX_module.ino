#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <NRF_Module.h>
#include <TD_Modbus.h>
#include <NRF_Module.h>
//#include <TD_Radio.h>

// revision
#define SOFTWARE_MAJOR_REVISION 74
#define SOFTWARE_MINOR_REVISION 100
#define SOFTWARE_BUILD_NUMBER 2

// system variables
unsigned int systemConfig = DISABLED; 
signed long systemStatus = DISABLED; 
unsigned int cycleTime; //scan time in microseconds

// well variables
pump pumps[PUMPS_SUPPORTED];

// class instances
updateSystem tUpdateSystem;
updatePump tUpdatePump;
test tTest;
NRF_Module nrf;

// test variables
static dynacard card0;
dynacard card1;
cardDataMessagePack cardDataMessages0;
int pumpToPublish = INVALID_PUMP;
unsigned int test0;
unsigned int test1;

void setup() {
  // set software revision and config modbus server
  tUpdateSystem.start(SOFTWARE_MAJOR_REVISION, SOFTWARE_MINOR_REVISION, SOFTWARE_BUILD_NUMBER);
  
  nrf.rxSetup();
 
}
void loop() {
  

   // poll modbus
  ModbusRTUServer.poll();



  // cycle time
  //cycleTime = tTest.scanTime();
  //ModbusRTUServer.inputRegisterWrite(8, cycleTime);
  //ModbusRTUServer.inputRegisterWrite(9, random(30,35));
  //Serial.println("blah");
  // end test stuff
  nrf.rxLoop();
}
