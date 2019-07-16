#include <Esp.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <spo2_algorithm.h>

#define DEBUG

#define GYRO_INT1_PIN 23
#define GYRO_INT2_PIN 4
#define GYRO_CS_PIN 15
#define MANUAL_BUTTON_PIN 25
#define VIBERATOR_PIN 26

enum GYRO_INT_SOURCE {
  GYRO_INT_NONE = 0,
  GYRO_INT_FREEFALL,
  GYRO_INT_ACT,
  GYRO_INT_INACT,
};

unsigned long lastAccelReachTime;
sensors_event_t initialAccel;
unsigned long lastFallDetectTime = 0;

enum ALARM_STATUS {
  ALARM_NONE = 0,
  ALARM_FREEFALL,
  ALARM_FALL,
  ALARM_CRITICAL_FALL,
  ALARM_MANUAL,
};

uint8_t currentAlarmStatus;
unsigned long lastReportTime = 0;

unsigned long lastAlivePacketPublishTime = 0;

HardwareSerial A9(1);
Adafruit_ADXL345_Unified gyro = Adafruit_ADXL345_Unified(12345);

void debug(String);

void initA9(void);
void connectMqtt(void);

void initGyro(void);
uint8_t getGyroIntSource(void);
double accelDifference(sensors_event_t, sensors_event_t);

void viberate(int);

unsigned long detectFall(void);

unsigned long lastManualButtonTriggered = 0;
void detectManual(void);

MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

void initMAX30102(void);
int getHeartBeat(void);
int getSPO2(void);

void report(void);
void resetAlarm(void);

bool shouldSendResetMessage = false;

void setup(void) {
  currentAlarmStatus = ALARM_NONE;

  pinMode(GYRO_INT1_PIN, INPUT);
  pinMode(GYRO_INT2_PIN, INPUT);
  pinMode(GYRO_CS_PIN, OUTPUT);
  pinMode(MANUAL_BUTTON_PIN, INPUT);
  pinMode(VIBERATOR_PIN, OUTPUT);

  attachInterrupt(GYRO_INT2_PIN, resetAlarm, RISING);

  A9.begin(115200, SERIAL_8N1, 16, 17);
#ifdef DEBUG
  Serial.begin(115200);
#endif
  delay(100);

  initA9();
  initGyro();
  initMAX30102();
}

void loop(void) {
  if (Serial.available()) {
    String msg = Serial.readString();
    A9.println(msg);
    while(!A9.available());
    msg = A9.readString();
    Serial.print(msg);
  }

  if (millis() - lastAlivePacketPublishTime > 3000) {
    A9.println("at+mqttpub=\"alive\",\"1\",0,0,1");
    lastAlivePacketPublishTime = millis();
  }

  if (shouldSendResetMessage) {
    shouldSendResetMessage = false;
    debug("double tap. reset");
    A9.println("at+mqttpub=\"alarm_status\",\"0\",0,0,1");
  }

  if (A9.available()) {
    String msg = A9.readString();
    if (msg.indexOf("+MQTTDISCONN") != -1) {
      debug("mqtt connection lost! trying to reconnect...");
      delay(5000);
      connectMqtt();
    }
    if (msg.indexOf("+MQTTPUBLISH") != -1) {
      debug("reset alarm packet recieved");
      currentAlarmStatus = ALARM_NONE;
    }
  }
  if (!currentAlarmStatus) {
    // no alarm, detecting
    lastReportTime = 0;
    // detect manual
    if (analogRead(MANUAL_BUTTON_PIN) < 1400) {
      debug("pressed");
      if (!lastManualButtonTriggered) {
        lastManualButtonTriggered = millis();
      } else if (millis() - lastManualButtonTriggered > 5000) {
        detectManual();
      }
    } else if (lastManualButtonTriggered && millis() - lastManualButtonTriggered > 50) {
      lastManualButtonTriggered = 0; // reset
    }
    // detect fall
    uint8_t gyroInt1Status =  digitalRead(GYRO_INT1_PIN);
    if (gyroInt1Status == HIGH) {
      // int triggered
      uint8_t intType = getGyroIntSource();
      if (intType == GYRO_INT_FREEFALL) {
        lastFallDetectTime = detectFall();
      }
    } else {
      if (millis() - lastAccelReachTime > 100 || millis() - lastAccelReachTime < 0) {
          gyro.getEvent(&initialAccel);
          lastAccelReachTime = millis();
      }
    }
  } else {
    // alarm occurred, reporting
    if (currentAlarmStatus == ALARM_FALL) {
      // detect critical fall
      if (lastFallDetectTime > 0) {
        if (millis() - lastFallDetectTime < 15000) {
          if (digitalRead(GYRO_INT1_PIN) == HIGH && getGyroIntSource() == GYRO_INT_INACT) {
            debug("critical fall");
            currentAlarmStatus = ALARM_CRITICAL_FALL;
          }
        } else {
          // time out
          debug("critical fall time out");
          lastFallDetectTime = 0;
        }
      }
    }

    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++) {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++) {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    if (!lastReportTime || millis() - lastReportTime > 1000 || millis() - lastReportTime < 0) {
      report();
      lastReportTime = millis();
    }
  }
}

void debug(String msg) {
#ifdef DEBUG
  Serial.print("[DEBUG] ");
  Serial.println(msg);
#endif
}

void initA9() {
  delay(1000);
  debug("reset");
  A9.println("at+rst=1");
  while(1) {
    while(!A9.available());
    String msg = A9.readString();
    debug(msg);
    if (msg.indexOf("READY") != -1) {
      debug("ready");
      break;
    }
  }
  delay(500);
  if (A9.available()) {
    while(A9.read());
  }
  A9.println("at+cgdcont=1,\"IP\",\"CMNET\"");
  while (!A9.available());
  String msg = A9.readString();
  if (msg.indexOf("OK") == -1) {
    debug("cgdcont not ok");
    debug(msg);
    while (1);
  }
  A9.println("at+cgact=1,1");
  while (!A9.available());
  msg = A9.readString();
  if (msg.indexOf("OK") == -1) {
    debug("cgact not ok");
    debug(msg);
    while (1);
  }
  debug("network set");
  A9.println("at+gps=1");
  while (!A9.available());
  msg = A9.readString();
  if (msg.indexOf("OK") == -1) {
    debug("gps not ok");
    debug(msg);
    while (1);
  }
  debug("gps set");
  connectMqtt();
  A9.println("at+mqttpub=\"alive\",\"1\",1,0,1");
  while (!A9.available());
  msg = A9.readString();
  if (msg.indexOf("OK") == -1) {
    debug("cannot pub mqtt");
    debug(msg);
    while (1);
  }
  debug("mqtt first alive packet published");
  lastAlivePacketPublishTime = millis();
  A9.println("at+mqttsub=\"reset_alarm\",1,1");
  while (!A9.available());
  msg = A9.readString();
  if (msg.indexOf("OK") == -1) {
    debug("cannot sub mqtt");
    debug(msg);
    while (1);
  }
  debug("mqtt channel subscribed");
}

void connectMqtt(void) {
  String msg;
  while (1) {
    A9.println("at+mqttconn=\"bingring.leaferx.ink\",1883,\"device\",300,0");
    while (!A9.available());
    msg = A9.readString();
    if (msg.indexOf("OK") == -1) {
      if (msg.indexOf("58") != -1) {
        A9.println();
      }
      debug("cannot connect mqtt broker. retry after 5s");
      debug(msg);
      delay(5000);
    } else {
      break;
    }
  }
  debug("mqtt broker connected");
}

void initGyro(void) {
  digitalWrite(GYRO_CS_PIN, HIGH);
  delay(200);

  if(!gyro.begin()) {
    Serial.println("Failed to init ADXL345");
    while(1);
  }
  gyro.writeRegister(ADXL345_REG_DATA_FORMAT, 0x0B); // 16g, 13-bit right, High trigger, I2C
  gyro.writeRegister(ADXL345_REG_THRESH_ACT, 0X20); // 2g
  gyro.writeRegister(ADXL345_REG_THRESH_INACT, 0x03); // 0.1875g
  gyro.writeRegister(ADXL345_REG_TIME_INACT, 0x02); // 2s
  gyro.writeRegister(ADXL345_REG_ACT_INACT_CTL, 0x7F); // INACT AC, ACT DC
  gyro.writeRegister(ADXL345_REG_THRESH_FF, 0x0C); // 0.75g
  gyro.writeRegister(ADXL345_REG_TIME_FF, 0x06); // 30ms
  gyro.writeRegister(ADXL345_REG_THRESH_TAP, 0x32);
  gyro.writeRegister(ADXL345_REG_DUR, 0x30);
  gyro.writeRegister(ADXL345_REG_LATENT, 0x22);
  gyro.writeRegister(ADXL345_REG_WINDOW, 0xFF);
  gyro.writeRegister(ADXL345_REG_TAP_AXES, 0x07);
  gyro.writeRegister(ADXL345_REG_INT_ENABLE, 0x3C); // DoubleTap, Activity, Inactivity, Freefall
  gyro.writeRegister(ADXL345_REG_INT_MAP, 0x20); // Activity, Inactivity, Freefall => INT1; DoubleTap => INT2

  gyro.getEvent(&initialAccel);
  lastAccelReachTime = millis();
}

uint8_t getGyroIntSource(void) {
  uint8_t source = gyro.readRegister(ADXL345_REG_INT_SOURCE);
  if (source & 0x04) return GYRO_INT_FREEFALL;
  if (source & 0x08) return GYRO_INT_INACT;
  if (source & 0x10) return GYRO_INT_ACT;
  return GYRO_INT_NONE;
}

double accelDifference(sensors_event_t event1, sensors_event_t event2) {
  double diff = sqrt(
    sq(event1.acceleration.x - event2.acceleration.x) +
    sq(event1.acceleration.y - event2.acceleration.y) +
    sq(event1.acceleration.z - event2.acceleration.z)
  );

  return diff;
}

unsigned long detectFall(void) {
  debug("freefall int");
  unsigned long timer = millis();
  while (digitalRead(GYRO_INT1_PIN) == HIGH && getGyroIntSource() == GYRO_INT_FREEFALL) {
    if (millis() - timer > 400) {
      // Freefall!
      currentAlarmStatus = ALARM_FREEFALL;
      return 0;
    }
  }
  debug("wait for act");
  gyro.writeRegister(ADXL345_REG_THRESH_ACT, 0X20); // 2g
  gyro.writeRegister(ADXL345_REG_ACT_INACT_CTL, 0x7F); // INACT AC, ACT DC
  timer = millis();
  while (digitalRead(GYRO_INT1_PIN) == LOW || getGyroIntSource() != GYRO_INT_ACT) {
    if (millis() - timer > 200) {
      debug("time out");
      return 0;
    } 
  }
  debug("act! wait for inact");
  gyro.writeRegister(ADXL345_REG_TIME_INACT, 0x02); // 2s
  timer = millis();
  while (digitalRead(GYRO_INT1_PIN) == LOW || getGyroIntSource() != GYRO_INT_INACT) {
    if (millis() - timer > 3500) {
      debug("time out");
      return 0;
    } 
  }
  debug("inact! comparing");
  sensors_event_t currentAccel;
  gyro.getEvent(&currentAccel);
  double diff = accelDifference(currentAccel, initialAccel);
  debug(String(diff));
  if (diff < 6) {
    // may not be falling
    debug("less difference");
    return 0;
  }
  debug("fall");
  currentAlarmStatus = ALARM_FALL;

  // prepare for critical fall detect
  gyro.writeRegister(ADXL345_REG_THRESH_ACT, 0X0A); // 0.5g
  gyro.writeRegister(ADXL345_REG_TIME_INACT, 0x0A); // 10s
  gyro.writeRegister(ADXL345_REG_ACT_INACT_CTL, 0xFF); // INACT AC, ACT AC
  return millis();
}

void viberate(int lastTime) {
  digitalWrite(VIBERATOR_PIN, HIGH);
  delay(lastTime);
  digitalWrite(VIBERATOR_PIN, LOW);
}

void detectManual(void) {
  debug("begin manual detect");
  viberate(100);

  unsigned long timer = millis();
  while (analogRead(MANUAL_BUTTON_PIN) < 1400) {
    if (millis() - timer > 1500) {
      viberate(100);
      lastManualButtonTriggered = 0;
      return;
    }
  }
  delay(100);
  while (analogRead(MANUAL_BUTTON_PIN) > 3000) {
    if (millis() - timer > 1500) {
      viberate(100);
      lastManualButtonTriggered = 0;
      return;
    }
  }
  viberate(500);
  currentAlarmStatus = ALARM_MANUAL;
}

void initMAX30102(void) {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) { //Use default I2C port, 400kHz speed
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++) {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
}

int getHeartBeat(void) {
  return heartRate;
}

int getSPO2(void) {
  return spo2;
}

void report(void) {
  if (currentAlarmStatus == ALARM_NONE) return;
  String alarmTypeText;
  switch (currentAlarmStatus){
    case ALARM_FALL:
      alarmTypeText = "FALL";
      break;
    case ALARM_CRITICAL_FALL:
      alarmTypeText = "CRITICAL FALL";
      break;
    case ALARM_FREEFALL:
      alarmTypeText = "FREEFALL";
      break;
    case ALARM_MANUAL:
      alarmTypeText = "MANUAL";
  }
  debug("REPORT: " + alarmTypeText);

  A9.println("at+mqttpub=\"alarm_status\",\"" + String(currentAlarmStatus) + "\",0,0,1");

  String alarmData = String(getHeartBeat()) + "," + String(getSPO2());
  A9.println("at+mqttpub=\"alarm_data\",\"" + alarmData + "\",0,0,1");
}

void resetAlarm(void) {
  currentAlarmStatus = ALARM_NONE;
  shouldSendResetMessage = true;
}