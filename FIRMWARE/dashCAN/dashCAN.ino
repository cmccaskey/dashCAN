//avrdude -c usbtiny -p m328p -U lfuse:w:0b11110111:m -U
//avrdude -c usbtiny -p m328p -U efuse:w:0b00000111:m

//#define GEAR_ONLY

#include <SPI.h>
#include <SdFat.h>
#include "MCP/mcp_can.cpp"
#include "PCINT/pcint.h"

#include "PCINT/pcint.cpp"


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

const int MPH_DRAW[] = {10, 200};
const int GEAR_DRAW[] = {110, 200};
const int PSI_DRAW[] = {205, 200};
const int THROTTLE_DRAW[] = {10, 130};
const int COOL_DRAW[] = {110, 130};
const int TIMING_DRAW[] = {110, 50};
const int FUEL_DRAW[] = {210, 130};
const int GEAR_LARGE_BLUE_DRAW[] = {100, 70};
const int GEAR_LARGE_GREEN_DRAW[] = {100, 70};
const int GEAR_LARGE_YELLOW_DRAW[] = {100, 70};
const int GEAR_LARGE_RED_DRAW[] = {100, 70};

File imageFile;
File dataFile;
File configFile;

SdFat SD;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define BUTTON 4

#define HEIGHT 40

char logFileName[] = "L_01.CSV";

uint8_t bufferTemp[HEIGHT];

void setup(void) {
  digitalWrite(ILI9341_LED_PIN, LOW);
  pinMode(ILI9341_LED_PIN, OUTPUT);

  Serial.begin(115200);
  Serial.println("Booting");
  if (CAN_OK != CAN.begin(CAN_500KBPS)) {
    Serial.println("CAN FAIL");
  }

  SD.begin(SD_CS);
  char configFileName[] = "CONFIG.CSV";
  configFile = SD.open(configFileName, FILE_READ);

  char configBuffer[3] = { -1, -1, -1};
  int configBufferCounter = 0;
  while (configFile.peek() != -1) {
    char in = configFile.read();
    if (in != ',' && in != '\n' && in != '\r') {
      configBuffer[configBufferCounter] = in - 48; //ASCII to decimal
      configBufferCounter ++;
    } else {
      int configBufferInt = 0;
      if (configBuffer[1] == -1) {
        configBufferInt = configBuffer[0];
      } else if (configBuffer[1] == -1) {
        configBufferInt = configBuffer[0] * 10 + configBuffer[1];
      } else {
        configBufferInt = configBuffer[0] * 100 + configBuffer[1] * 10 + configBuffer[2];
      }
      Serial.print("configBufferInt: ");
      Serial.println(configBufferInt);
      configBufferCounter = 0;
      for (int i = 0; i < 3; i ++) {
        configBuffer[i] = -1;
      }
    }
  }


  for (uint8_t i = 1; i < 99; i ++) {
    if (i <= 9) {
      logFileName[3] = i + 48;
    } else if (i <= 99) {
      logFileName[3] = i % 10 + 48;
      logFileName[2] = i / 10 + 48;
    }

    if (!SD.exists(logFileName)) {
      Serial.print("Logging to: ");
      Serial.println(logFileName);
      break;
    }
  }
  dataFile = SD.open(logFileName, FILE_WRITE);
  dataFile.println("millis(),psi,rpm,mph,coolantTemp,throttle,fuelLevel,gear");
  dataFile.flush();
  pinMode(MCP_INT, INPUT_PULLUP);
  set_mask_filt();

  initTFT();
  drawImage("Mask.xbml", ILI9341_BLACK);
  drawImage("Loading.xbml", ILI9341_RED);
  initTFTDraw();
  //dataFile.close();
  delay(1000);
  drawImage("Mask.xbml", ILI9341_BLACK);
  drawImage("Menu1.xbml", ILI9341_WHITE);
}



char SPCR_SD;

void swapSPCR() {
  char temp = SPCR;
  SPCR = SPCR_SD;
  SPCR_SD = temp;
}


void drawImage(String fileName, int color) {
  //digitalWrite(ILI9341_LED_PIN, LOW); 
  dataFile.close();
  unsigned long totalTFT = 0;
  unsigned long totalSDbegin = 0;
  unsigned long totalSDread = 0;
  SD.begin(SD_CS);
  imageFile = SD.open(fileName, FILE_READ);
  if (imageFile.available()) {
    for (int j = 0; j < 9600 / HEIGHT; j ++) {
      unsigned long totalSDbeginTemp = millis();
      SD.begin(SD_CS);
      totalSDbeginTemp = millis() - totalSDbeginTemp;
      totalSDbegin += totalSDbeginTemp;

      unsigned long totalSDreadTemp = millis();
      imageFile.read(bufferTemp, HEIGHT);
      totalSDreadTemp = millis() - totalSDreadTemp;
      totalSDread += totalSDreadTemp;

      unsigned long totalTFTtemp = millis();

      tft.drawYBitmap(0, j * HEIGHT / 40, bufferTemp, 320, HEIGHT / 40, color);
      totalTFTtemp = millis() - totalTFTtemp;
      totalTFT += totalTFTtemp;
    }
  } else {
    Serial.println("image not available");
  }
  //dataFile = SD.open(logFileName, FILE_WRITE);
  Serial.print("SD Begin: ");
  Serial.println(totalSDbegin);
  Serial.print("SD Read: ");
  Serial.println(totalSDread);
  Serial.print("TFT Draw: ");
  Serial.println(totalTFT);
  Serial.println();
  digitalWrite(ILI9341_LED_PIN, HIGH);
}

int gearLargeLast = 0;
int gearLargeColorLast = 0;
int gearCounter = 0;

int mphLast = -1;
int psiLast = -1;
int fuelLevelLast = -1;
int coolantTempLast = -1;
int throttleLast = -1;
int gearLast = -1;
int timingLast = -1;

void loop() {
  float psi = ecu_req(INTAKE_PRESSURE);
  psi = psi * 0.145038 - 14.0;
  if ((int) psi != psiLast) {
    drawData(PSI_DRAW, psi, psiLast);
    psiLast = psi;
  }

  int rpm = ecu_req(ENGINE_RPM);
  if (rpm >= 0) {
    //drawRPM(rpm + rpmRes / 2);
  }

  int kmh = ecu_req(VEHICLE_SPEED);
  int mph = kmh * 0.6214 + 0.5;
  if (mph >= 0 && mph != mphLast) {
    drawData(MPH_DRAW, mph, mphLast);
    mphLast = mph;
  }

  float coolantTemp = ecu_req(ENGINE_COOLANT_TEMP);
  coolantTemp = coolantTemp * 9 / 5 + 32;
  if ((int) coolantTemp != coolantTempLast) {
    drawData(COOL_DRAW, coolantTemp, coolantTempLast);
    coolantTempLast = coolantTemp;
  }

  int throttle = ecu_req(THROTTLE);
  if (throttle != throttleLast) {
    drawData(THROTTLE_DRAW, throttle, throttleLast);
    throttleLast = throttle;
  }


  float fuelLevel = ecu_req(FUEL_TANK_LEVEL);
  if ((int) fuelLevel != fuelLevelLast) {
    drawData(FUEL_DRAW, fuelLevel, fuelLevelLast);
    fuelLevelLast = fuelLevel;
  }

  int timing = ecu_req(TIMING_ADVANCE);
  if (timing != timingLast) {
    drawData(TIMING_DRAW, timing, timingLast);
    timingLast = timing;
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
  if (gearRatio != -1 && gearCounter > 2 && gear != gearLast) {
    gearCounter = 0;
    if (gear != 0) {
      drawData(GEAR_DRAW, gear, gearLast);
      gearLast = gear;
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
  dataFile.flush();
}

void drawData(int * rect, int data, int dataLast) {
  putData(rect, dataLast, 0);
  putData(rect, data, 1);
}

void putData(int * rect, int data, char color) {
  int charOffset;
  if (abs(data) < 10) //single digit
    charOffset = 40;
  if (abs(data) >= 10) //double digit
    charOffset = 35;
  if (abs(data) >= 100) //triple digit
    charOffset = 30;
  tft.setFont(&FreeSansBold12pt7b);
  tft.setTextSize(1);
  tft.setRotation(3);
  tft.setCursor(rect[0] + charOffset, rect[1]);
  if (color == 0) {
    tft.setTextColor(ILI9341_BLACK);
  } else {
    tft.setTextColor(ILI9341_WHITE);
  }
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
  //tft.fillScreen(bootColor);
#ifndef GEAR_ONLY
  //drawLabels();
  //drawSweep();
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
  while (digitalRead(MCP_INT) && timeout < 100) {
    timeout ++;
    if (timeout < 100) {
      //Serial.print(".");
    } else {
      //Serial.println(".");
      //Serial.println();
      //Serial.println("Entering Sleep");
      //delay(10);
      //enterSleep();
      //return;
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
  tft.setRotation(3);
}
