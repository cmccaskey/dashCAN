//avrdude -c usbtiny -p m328p -U lfuse:w:0b11110111:m

//#define GEAR_ONLY

#include <SPI.h>
#include <SD.h>
#include "MCP/mcp_can.cpp"

#include "PDQ/PDQ_GFX.h"        // PDQ: Core graphics library
#include "PDQ/PDQ_ILI9341_config.h"     // PDQ: ILI9341 pins and other setup for this sketch
#include "PDQ/PDQ_ILI9341.h"      // PDQ: Hardware-specific driver library
#include "PDQ/glcdfont.c"

#include "Fonts/FreeSansBold12pt7b.h"
#ifdef GEAR_ONLY
#include "Fonts/FreeSansBold24pt7b.h"
#endif

#define bootColor ILI9341_BLACK //color of screen while booting

#define rpmRange1 2000 //under => green
#define rpmRange2 4000 //under => yellow
#define rpmRange3 5000 //above => red
#define rpmMax 7000 //sets max rpm the tach will display

#define rpmRange1Color ILI9341_GREEN //color of RPM range 1
#define rpmRange2Color ILI9341_YELLOW //color of RPM range 2
#define rpmRange3Color ILI9341_RED //color of RPM range 3
#define rpmRange4Color ILI9341_RED //color of RPM range 4

#define gearIndiColor ILI9341_BLACK //gear area color
#define speedIndiColor ILI9341_BLACK //speed area color
#define rpmBackColor ILI9341_BLACK //rpm background color
#define warnColor ILI9341_BLACK //warning area color
#define warnLightColor ILI9341_RED //warning light color

#define speedTextColor ILI9341_WHITE //color of text for speed
#define gearTextColor ILI9341_WHITE //color of text for gear

#define rpmBackHeight 100 //height of bar behind rpm //there are 12 segments
#define rpmHeight 80 //height of each rpm segment
#define rpmWidth 10
//width of each rpm segment
#define rpmLeftMargin 1 //left margin of first rpm segment
#define rpmRightMargin 1 //right margin of every rpm segment

#define rpmRes 250
#define ILI9341_LED_PIN 10
//#define ILI9341_RST_PIN 8 //in header?

#define screenWidth 320
#define screenHeight 240

#define MCP_INT 5

#define CAN_ID_PID 0x7DF
#define ENGINE_COOLANT_TEMP 0x05
#define ENGINE_RPM          0x0C
#define VEHICLE_SPEED       0x0D
#define MAF_SENSOR          0x10
#define O2_VOLTAGE          0x14
#define THROTTLE            0x11
#define INTAKE_PRESSURE     0x0B
#define TIMING_ADVANCE      0x0E
#define ENGINE_REF_TORQUE   0x63
#define ENGINE_PER_TORQUE   0x64
#define ENGINE_OIL_TEMP     0x5C
#define FUEL_TANK_LEVEL     0x2F

#define SD_CS 3
#define SPI_CS_PIN 16
MCP_CAN CAN(SPI_CS_PIN);

PDQ_ILI9341 tft;      // PDQ: create LCD object (using pins in "PDQ_ILI9341_config.h")

//2016 VW Jetta S 5 speed
//1st: 3.78
//2nd: 2.12
//3rd: 1.27
//4th: 0.87
//5th: 0.63
//Reverse: 3.60
//Axle ratio: 3.35
//Factory wheel: 205 55 r16 revolutions per mile = 810.68mi (503.73KM)
//const float totalRatio[] = {12.663, 7.102, 4.2545, 2.9145, 2.1105}; //rpm to mph ratio for each forward gear
//const float wheelRevKm = 503.73;
//const float rpmOverKmh[] = {106.31, 59.62, 35.72, 24.47, 17.72};

#define GEAR_HER 5
const float rpmOverKmh[] = {120, 67.8, 40.36, 27.65, 20.02};
//output shaft rpm = Ngear * Axle ratio
//gear is inferred by rounding gear ratio to nearest
//KM/H and engine RPM is read from CAN
//X KM/H * / X RPM * X Rev/KM / 60 = Gear Ratio

int lastRPM = -1; //used to detect if the data needs updated
int lastGear = -1;
int lastSpeed = -1;

const int MPH_CLEAR[] = {10, 170, 100, 40};
const int GEAR_CLEAR[] = {110, 170, 100, 40};
const int PSI_CLEAR[] = {210, 170, 100, 40};
const int THROTTLE_CLEAR[] = {10, 100, 100, 40};
const int COOL_CLEAR[] = {110, 100, 100, 40};
const int FUEL_CLEAR[] = {210, 100, 100, 40};
const int GEAR_LARGE_CLEAR[] = {0, 0, 320, 240};

const int MPH_DRAW[] = {10, 200};
const int GEAR_DRAW[] = {110, 200};
const int PSI_DRAW[] = {205, 200};
const int THROTTLE_DRAW[] = {10, 130};
const int COOL_DRAW[] = {110, 130};
const int FUEL_DRAW[] = {210, 130};
const int GEAR_LARGE_BLUE_DRAW[] = {100, 70};
const int GEAR_LARGE_GREEN_DRAW[] = {100, 70};
const int GEAR_LARGE_YELLOW_DRAW[] = {100, 70};
const int GEAR_LARGE_RED_DRAW[] = {100, 70};

File dataFile;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup(void) {
  delay(1000);
  digitalWrite(ILI9341_LED_PIN, LOW);
  pinMode(ILI9341_LED_PIN, OUTPUT);
  Serial.begin(115200);
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {
    Serial.println("CAN FAIL");
    delay(100);
  }
  if (!SD.begin(SD_CS)) {
    Serial.println("SD FAIL");
    // don't do anything more:
    //while (1);
  }
  for (uint8_t i = 0; i < 255; i ++) {
    if (!SD.exists(i + ".csv")) {
      dataFile = SD.open(i + ".csv");
      break;
    }
  }
  Serial.println("GOOD");
  pinMode(MCP_INT, INPUT_PULLUP);
  set_mask_filt();
  initTFT();
  initTFTDraw();
  //clearData(GEAR_LARGE_CLEAR);
  //drawDataLarge(GEAR_LARGE_BLUE_DRAW, 0, 0);
  //drawSweep();
  //drawLabels();
  dataFile.println("millis(),psi,rpm,mph,coolantTemp,throttle,fuelLevel,gear");
}

int gearLargeLast = 0;
int gearLargeColorLast = 0;
int gearCounter = 0;

int mphLast = -1;
int psiLast = -1;
int fuelLevelLast = -1;
int coolantTempLast = -1;
int throttleLast = -1;

void loop() {
#ifndef GEAR_ONLY
  float psi = ecu_req(INTAKE_PRESSURE);
  psi = psi * 0.145038 - 14.0;
  if ((int) psi != psiLast) {
    clearData(PSI_CLEAR);
    drawData(PSI_DRAW, psi);
    psiLast = psi;
  }

  int rpm = ecu_req(ENGINE_RPM);
  if (rpm >= 0) {
    drawRPM(rpm + rpmRes / 2);
  }

  int kmh = ecu_req(VEHICLE_SPEED);
  int mph = kmh * 0.6214 + 0.5;
  if (mph >= 0 && mph != mphLast) {
    clearData(MPH_CLEAR);
    drawData(MPH_DRAW, mph);
    mphLast = mph;
  }

  float coolantTemp = ecu_req(ENGINE_COOLANT_TEMP);
  coolantTemp = coolantTemp * 9 / 5 + 32;
  if ((int) coolantTemp != coolantTempLast) {
    clearData(COOL_CLEAR);
    drawData(COOL_DRAW, coolantTemp);
    coolantTempLast = coolantTemp;
  }

  int throttle = ecu_req(THROTTLE);
  if (throttle != throttleLast) {
    clearData(THROTTLE_CLEAR);
    drawData(THROTTLE_DRAW, throttle);
    throttleLast = throttle;
  }


  float fuelLevel = ecu_req(FUEL_TANK_LEVEL);
  if ((int) fuelLevel != fuelLevelLast) {
    clearData(FUEL_CLEAR);
    drawData(FUEL_DRAW, fuelLevel);
    fuelLevelLast = fuelLevel;
  }

  float gearRatio = rpm / kmh;
  int gear = 0;
  for (int i = 0; i < sizeof(rpmOverKmh) / sizeof(rpmOverKmh[0]); i ++) {
    if (gearRatio / rpmOverKmh[i] < 1.15 && gearRatio / rpmOverKmh[i] > 0.85) {
      gear = i + 1;
      gearCounter = 3;
      break;
    }
  }
  if (gear == 0 && gearCounter <= 2) {
    gearCounter ++;
  }
  if (gearRatio != -1 && gearCounter > 2) {
    gearCounter = 0;
    clearData(GEAR_CLEAR);
    if (gear != 0) {
      drawData(GEAR_DRAW, gear);
    }
  }
  dataFile.print(millis());
  dataFile.print(",");
  dataFile.print(psi);
  dataFile.print(",");
  dataFile.print(rpm);
  dataFile.print(",");
  dataFile.print(mph);
  dataFile.print(",");
  dataFile.print(coolantTemp);
  dataFile.print(",");
  dataFile.print(throttle);
  dataFile.print(",");
  dataFile.print(fuelLevel);
  dataFile.print(",");
  dataFile.println(gear);
#else
  int kmh = ecu_req(VEHICLE_SPEED);
  int rpm = ecu_req(ENGINE_RPM);
  float gearRatio = rpm / kmh;
  int gear = 0;
  for (int i = 0; i < sizeof(rpmOverKmh) / sizeof(rpmOverKmh[0]); i ++) {
    if (gearRatio / rpmOverKmh[i] < 1.15 && gearRatio / rpmOverKmh[i] > 0.85) {
      gear = i + 1;
      gearCounter = 3;
      break;
    }
  }
  if (gear == 0 && gearCounter <= 2) {
    gearCounter ++;
  }
  if (gearRatio != -1 && gearCounter > 2) {
    gearCounter = 0;

    if (gear != 0) {
      if (rpm < 2000) {
        if (gearLargeColorLast != 0 || gearLargeLast != gear) {
          gearLargeColorLast = 0;
          gearLargeLast = gear;
          drawDataLarge(GEAR_LARGE_BLUE_DRAW, gear, 0);
        }
      } else if (rpm < 3000) {
        if (gearLargeColorLast != 1 || gearLargeLast != gear) {
          gearLargeColorLast = 1;
          gearLargeLast = gear;
          gearLargeColorLast = 1;
          drawDataLarge(GEAR_LARGE_GREEN_DRAW, gear, 1);
        }
      } else if (rpm < 4000) {
        if (gearLargeColorLast != 2 || gearLargeLast != gear) {
          gearLargeColorLast = 2;
          gearLargeLast = gear;
          drawDataLarge(GEAR_LARGE_YELLOW_DRAW, gear, 2);
        }
      } else {
        if (gearLargeColorLast != 3 || gearLargeLast != gear) {
          gearLargeColorLast = 3;
          gearLargeLast = gear;
          gearLargeColorLast = 3;
          drawDataLarge(GEAR_LARGE_RED_DRAW, gear, 3);
        }
      }
    }
#endif
}

void drawLabels() {
  tft.fillRect(0, 0, 320, 240, ILI9341_BLACK);
  tft.setFont(&FreeSansBold12pt7b);
  tft.setTextColor(ILI9341_WHITE);
  //tft.setTextSize(1);
  tft.setCursor(30, 230);
  tft.print("MPH");
  tft.setCursor(125, 230);
  tft.print("GEAR");
  tft.setCursor(240, 230);
  tft.print("PSI");
  tft.setCursor(225, 160);
  tft.print("FUEL");
  tft.setCursor(125, 160);
  tft.print("COOL");
  tft.setCursor(30, 160);
  tft.print("TB%");
}

void clearData(int * rect) {
  tft.fillRect(rect[0], rect[1], rect[2], rect[3], ILI9341_BLACK);
}

#ifdef GEAR_ONLY
void drawDataLarge(int * rect, int data, int color) {
  clearData(GEAR_LARGE_CLEAR);
  tft.setFont(&FreeSansBold24pt7b);
  tft.setTextSize(5);
  tft.setRotation(3);
  tft.setCursor(rect[0], rect[1]);
  switch (color) {
    case 0:
      tft.setTextColor(ILI9341_BLUE);
      break;
    case 1:
      tft.setTextColor(ILI9341_GREEN);
      break;
    case 2:
      tft.setTextColor(ILI9341_YELLOW);
      break;
    case 3:
      tft.setTextColor(ILI9341_RED);
      break;
  }
  tft.print(String(data));
}
#endif

void drawData(int * rect, int data) {
  int charOffset;
  if (data < 10) //single digit
    charOffset = 40;
  if (data >= 10) //double digit
    charOffset = 35;
  if (data >= 100) //triple digit
    charOffset = 30;
  tft.setFont(&FreeSansBold12pt7b);
  tft.setTextSize(1);
  tft.setRotation(3);
  tft.setCursor(rect[0] + charOffset, rect[1]);
  tft.setTextColor(ILI9341_WHITE);
  tft.print(String(data));
}

void drawSweep() {
  for (int i = 0; i < rpmMax; i += 200) {
    drawRPM(i);
    delay(10);
  }
  for (int i = rpmMax; i > 0; i -= 200) {
    drawRPM(i);
    delay(10);
  }
}

void initTFTDraw() {
  tft.setRotation(3);
  tft.fillScreen(bootColor);
#ifndef GEAR_ONLY
  drawLabels();
  drawSweep();
#endif
  digitalWrite(ILI9341_LED_PIN, HIGH);
}

void drawRPM(int rpm) {
  int loops = (rpm - lastRPM) / rpmRes;
  if (rpm > lastRPM + rpmRes) {
    for (int i = lastRPM; i <= rpm; i += rpmRes) {
      drawRPMBars(i);
    }
  } else {
    drawRPMBars(rpm);
  }
  lastRPM = rpm;
}

void drawRPMBars(int rpm) {
  int color; //don't ask why this is an int
  rpm = int(rpm / rpmRes) * rpmRes - 1; //rounds rpm
  if (rpm != lastRPM) { //only updates tach if rpm changed
    //lastRPM = rpm;
    if (rpm < rpmRange1) {
      color = rpmRange1Color;
    } else if (rpm < rpmRange2) {
      color = rpmRange2Color;
    } else if (rpm < rpmRange3) {
      color = rpmRange3Color;
    } else {
      color = rpmRange4Color;
      if (rpm > rpmMax)
        rpm = rpmMax;
    }
    int xOffset = int(rpm / rpmRes) * (rpmWidth + rpmRightMargin) + rpmLeftMargin;
    tft.setRotation(3);
    tft.fillRect(xOffset, 2, rpmWidth, rpmHeight, color);
    tft.fillRect(xOffset + rpmWidth + rpmRightMargin, 0, screenWidth - xOffset, rpmBackHeight, ILI9341_BLACK);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set_mask_filt()
{
  /*
     set mask, set both the mask to 0x3ff
  */
  CAN.init_Mask(0, 0, 0x7FC);
  CAN.init_Mask(1, 0, 0x7FC);
  /*
     set filter, we can receive id from 0x04 ~ 0x09
  */
  CAN.init_Filt(0, 0, 0x7E8);
  CAN.init_Filt(1, 0, 0x7E8);

  CAN.init_Filt(2, 0, 0x7E8);
  CAN.init_Filt(3, 0, 0x7E8);
  CAN.init_Filt(4, 0, 0x7E8);
  CAN.init_Filt(5, 0, 0x7E8);
}

float ecu_req(unsigned char pid) {
  float engine_data;
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned char tmp[8] = {0x02, 0x01, pid, 0, 0, 0, 0, 0};
  CAN.sendMsgBuf(CAN_ID_PID, 0, 8, tmp);
  int timeout = 0;
  while (digitalRead(MCP_INT) && timeout < 50) {
    timeout ++;
    if (timeout < 50) {
      Serial.print(".");
    } else {
      Serial.println(".");
    }
    delay(1);
  }
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, buf);
    switch (pid) {
      case ENGINE_RPM:        //   ((A*256)+B)/4    [RPM]
        engine_data =  ((buf[3] * 256) + buf[4]) / 4;
        return engine_data;

      case ENGINE_COOLANT_TEMP:   //  A-40        [degree C]
        engine_data =  buf[3] - 40;
        return engine_data;

      case VEHICLE_SPEED:     // A          [km]
        engine_data =  buf[3];
        return engine_data;

      case MAF_SENSOR:        // ((256*A)+B) / 100  [g/s]
        engine_data =  ((buf[3] * 256) + buf[4]) / 100;
        return engine_data;

      case O2_VOLTAGE:        // A * 0.005   (B-128) * 100/128 (if B==0xFF, sensor is not used in trim calc)
        engine_data = buf[3] * 0.005;
        return engine_data;

      case THROTTLE:        // Throttle Position
        engine_data = (buf[3] * 100) / 255;
        return engine_data;

      case INTAKE_PRESSURE:        // Intake Pressure A kPa
        engine_data = buf[3];
        return engine_data;

      case TIMING_ADVANCE:         //Timing advance deg. before TDC
        engine_data = buf[3] / 2 - 64;
        return engine_data;

      case ENGINE_REF_TORQUE:         //Engine Torque Nm
        engine_data = 256 * buf[3] + buf[4];
        return engine_data;

      case ENGINE_PER_TORQUE:         //% Engine Torque
        engine_data = buf[3];
        return engine_data;

      case ENGINE_OIL_TEMP:         //Oil Temp deg. C
        engine_data = buf[3] - 40;
        return engine_data;

      case FUEL_TANK_LEVEL:         //Fuel Level Percentage
        engine_data = buf[3] * 100 / 255;
        return engine_data;
    }
  }
}

void initTFT() {
  FastPin<ILI9341_RST_PIN>::setOutput();
  FastPin<ILI9341_RST_PIN>::hi();
  FastPin<ILI9341_RST_PIN>::lo();
  delay(1);
  FastPin<ILI9341_RST_PIN>::hi();
  tft.begin();
}
