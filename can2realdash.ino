 
#include <mcp_can.h>
#include <SPI.h>
#include "emu.h"
bool debug_emu_vals = false;
uint32_t  EMUbase = 0x600;
unsigned long previousMillis = 0;
const long interval = 20;
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
#define CAN0_INT 2                              // Set INT to pin  2
MCP_CAN CAN0(10);                               // Set CS to pin  10

//          UNO   Micro
// INT    |  2 |  |  2 |
// CS     | 10 |  | 10 |
// SI     | 11 |  | 16 |
// SO     | 12 |  | 14 |
// SCK    | 13 |  | 15 |


EMU emu;

void setup() {
  Serial.begin(115200);
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
  
  Serial.println("MCP2515 Library Receive Example...");
}

void loop() {
  CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
  fillEmuData();
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    SendCANFramesToSerial();
  }
}


void fillEmuData(){
  if (rxId == EMUbase) {
    //0-1 RPM in 16Bit unsigned
    emu.DATA.RPM = (rxBuf[1] << 8) + rxBuf[0];
    //2 TPS in /2 %
    emu.DATA.TPS = rxBuf[2] / 2;
    //3 IAT 8bit signed -40-127°C
    emu.DATA.IAT = rxBuf[3];
    //4-5 MAP 16Bit 0-600kpa
    emu.DATA.MAP = (rxBuf[5] << 8) + rxBuf[4];
    //6-7 INJPW 0-50 0.016129ms
    emu.DATA.pulseWidth = ((rxBuf[7] << 8) + rxBuf[6]) * 0.016129;
  }
  //Base +1:
  if (rxId == EMUbase + 1) {
    //AIN in 16Bit unsigned  0.0048828125 V/bit
    emu.DATA.analogIn1 = ((rxBuf[1] << 8) + rxBuf[0]) * 0.0048828125;
    emu.DATA.analogIn2 = ((rxBuf[3] << 8) + rxBuf[2]) * 0.0048828125;
    emu.DATA.analogIn3 = ((rxBuf[5] << 8) + rxBuf[4]) * 0.0048828125;
    emu.DATA.analogIn4 = ((rxBuf[7] << 8) + rxBuf[6]) * 0.0048828125;
  }
  //Base +2:
  if (rxId == EMUbase + 2) {
    //0-1 VSPD in 16Bit unsigned
    emu.DATA.vssSpeed = (rxBuf[1] << 8) + rxBuf[0];
    //2 BARO kPa
    emu.DATA.Baro = rxBuf[2];
    //3 OILT 0-160°C
    emu.DATA.oilTemperature = rxBuf[3];
    //4 OILP BAR 0.0625
    emu.DATA.oilPressure = rxBuf[4] * 0.0625;
    //5 FUELP BAR 0.03125
    emu.DATA.fuelPressure = rxBuf[5] * 0.03125;
    //6-7 CLT 16bit Signed 0.016129ms
    emu.DATA.CLT = ((rxBuf[7] << 8) + rxBuf[6]);
  }
  //Base +3:
  if (rxId == EMUbase + 3) {
    //0 IGNANG in 8Bit signed    -60 60  0.5deg/bit
    emu.DATA.IgnAngle = rxBuf[0] / 2;
    //1 DWELL 0-10ms 0.05ms/bit
    emu.DATA.dwellTime = rxBuf[1] * 0.05;
    //2 LAMBDA 8bit 0-2 0.0078125 L/bit
    emu.DATA.wboLambda = rxBuf[2] * 0.0078125;
    //3 LAMBDACORR 75-125 0.5%
    emu.DATA.LambdaCorrection = rxBuf[3] / 2;
    //4-5 EGT1 16bit °C
    emu.DATA.Egt1 = ((rxBuf[5] << 8) + rxBuf[4]);
    //6-7 EGT2 16bit °C
    emu.DATA.Egt2 = ((rxBuf[7] << 8) + rxBuf[6]);
  }
  //Base +4:
  if (rxId == EMUbase + 4) {
    //0 GEAR
    emu.DATA.gear = rxBuf[0];
    //1 ECUTEMP °C
    emu.DATA.emuTemp = rxBuf[1];
    //2-3 BATT 16bit  0.027 V/bit
    emu.DATA.Batt = ((rxBuf[3] << 8) + rxBuf[2]) * 0.0275;
    //4-5 ERRFLAG 16bit
    emu.DATA.cel = ((rxBuf[5] << 8) + rxBuf[4]);
    //6 FLAGS1 8bit
    emu.DATA.flags1 = rxBuf[6];
    //7 ETHANOL %
    emu.DATA.flexFuelEthanolContent = rxBuf[7];
  }
  //Base +6:
  if (rxId == EMUbase + 6) {
    //AIN in 16Bit unsigned  0.0048828125 V/bit
    emu.DATA.analogIn5 = ((rxBuf[1] << 8) + rxBuf[0]) * 0.0048828125;
    emu.DATA.analogIn6 = ((rxBuf[3] << 8) + rxBuf[2]) * 0.0048828125;
    emu.DATA.outflags1 = rxBuf[4];
    emu.DATA.outflags2 = rxBuf[5];
    emu.DATA.outflags3 = rxBuf[6];
    emu.DATA.outflags4 = rxBuf[7];
  }
  //Base +7:
  if (rxId == EMUbase + 7) {
    //Boost target 16bit 0-600 kPa
    emu.DATA.boostTarget = ((rxBuf[1] << 8) + rxBuf[0]);
    //PWM#1 DC 1%/bit
    emu.DATA.pwm1 = rxBuf[2];
  }
}

void SendCANFramesToSerial()
{
  
      byte buf[8];
      int rpm=emu.DATA.RPM;
      int bar=emu.DATA.MAP*10;
      int clt=emu.DATA.CLT+100;
      int tps=emu.DATA.TPS;
      if(debug_emu_vals){
        Serial.print("RPM:");
        Serial.print(rpm);
        Serial.print(" MAP:");
        Serial.print(bar);
        Serial.print(" CLT:");
        Serial.print(clt);
        Serial.print(" TPS:");
        Serial.println(tps);
      }
      memcpy(buf, &rpm, 2);
      memcpy(buf + 2, &bar, 2);
      memcpy(buf + 4, &clt, 2);
      memcpy(buf + 6, &tps, 2);
      // write first CAN frame to serial
      SendCANFrameToSerial(3200, buf);
    
      int bat = emu.DATA.Batt*10;
      int iat = emu.DATA.IAT*10;
      int afrtarget = emu.DATA.afrTarget*10;
      int afr = emu.DATA.wboAFR*10;
      if(debug_emu_vals){
        Serial.print("BAT:");
        Serial.print(bat);
        Serial.print(" IAT:");
        Serial.print(iat);
        Serial.print(" AFT:");
        Serial.print(afrtarget);
        Serial.print(" AFR:");
        Serial.println(afr);
      }
      memcpy(buf, &bat, 2);
      memcpy(buf + 2, &iat, 2);
      memcpy(buf + 4, &afrtarget, 2);
      memcpy(buf + 6, &afr, 2);
      // write first CAN frame to serial
      SendCANFrameToSerial(3201, buf);
    
      int pulseWidth = emu.DATA.pulseWidth*10;
      int knockV = emu.DATA.knockLevel*10;
      int vss = emu.DATA.vssSpeed*10;
      int gear = emu.DATA.gear+10;
      if(debug_emu_vals){
        Serial.print("PuW:");
        Serial.print(pulseWidth);
        Serial.print(" KNK:");
        Serial.print(knockV);
        Serial.print(" VSS:");
        Serial.print(vss);
        Serial.print(" GAR:");
        Serial.println(gear);
      }
      memcpy(buf, &pulseWidth, 2);
      memcpy(buf + 2, &knockV, 2);
      memcpy(buf + 4, &vss, 2);
      memcpy(buf + 6, &gear, 2);
      // write first CAN frame to serial
      SendCANFrameToSerial(3202, buf);


}


void SendCANFrameToSerial(unsigned long canFrameId, const byte* frameData)
{
  // the 4 byte identifier at the beginning of each CAN frame
  // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
  const byte serialBlockTag[4] = { 0x44, 0x33, 0x22, 0x11 };
  Serial.write(serialBlockTag, 4);

  // the CAN frame id number (as 32bit little endian value)
  Serial.write((const byte*)&canFrameId, 4);

  // CAN frame payload
  Serial.write(frameData, 8);
}
