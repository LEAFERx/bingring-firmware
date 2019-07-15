#include <Esp.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#define DEBUG

#define GYRO_INT1_PIN 15
#define GYRO_INT2_PIN 00
#define MANUAL_BUTTON_PIN 00
#define VIBERATOR_PIN 00

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

void initMAX30102(void);
int getHeartBeat(void);
int getSPO2(void);

void report(void);
void resetAlarm(void);

bool shouldSendResetMessage = false;

unsigned long asdasd = 0;

void setup(void) {
  currentAlarmStatus = ALARM_NONE;

  pinMode(GYRO_INT1_PIN, INPUT);
  pinMode(GYRO_INT2_PIN, INPUT);
  attachInterrupt(GYRO_INT2_PIN, resetAlarm, RISING);

  A9.begin(115200, SERIAL_8N1, 16, 17);
#ifdef DEBUG
  Serial.begin(115200);
#endif
  delay(100);

  initA9();
  //initGyro();
  //initMAX30102();

  asdasd = millis();
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
  /*
  if (millis() - asdasd > 5000) {
    debug("int1status");
    debug(String(digitalRead(GYRO_INT1_PIN)));
    asdasd = millis();
  }
  if (!currentAlarmStatus) {
    // no alarm, detecting
    lastReportTime = 0;
    // detect manual
    if (digitalRead(MANUAL_BUTTON_PIN) == HIGH) {
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
      debug("inttype");
      uint8_t intType = getGyroIntSource();
      debug(String(intType));
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

    if (!lastReportTime || millis() - lastReportTime > 1000 || millis() - lastReportTime < 0) {
      report();
    }
  }
  */
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
  while (1) {
    A9.println("at+mqttconn=\"bingring.leaferx.ink\",1883,\"device\",300,0");
    while (!A9.available());
    msg = A9.readString();
    if (msg.indexOf("OK") == -1) {
      if (msg.indexOf("58" != -1)) {
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
  debug("source");
  debug(String(source));
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
      // time out
      return 0;
    } 
  }
  debug("act! wait for inact");
  gyro.writeRegister(ADXL345_REG_TIME_INACT, 0x02); // 2s
  timer = millis();
  while (digitalRead(GYRO_INT1_PIN) == LOW || getGyroIntSource() != GYRO_INT_INACT) {
    if (millis() - timer > 3500) {
      // time out
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
  viberate(100);

  unsigned long timer = millis();
  while (digitalRead(MANUAL_BUTTON_PIN) == HIGH) {
    if (millis() - timer > 1000) {
      viberate(100);
      return;
    }
  }
  delay(100);
  while (digitalRead(MANUAL_BUTTON_PIN) == LOW) {
    if (millis() - timer > 1000) {
      viberate(100);
      return;
    }
  }
  viberate(500);
  currentAlarmStatus = ALARM_MANUAL;
}

void initMAX30102(void) {
  return;
}

int getHeartBeat(void) {
  return 0;
}

int getSPO2(void) {
  return 0;
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