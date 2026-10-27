#include <Servo.h>
#include <string.h>

// Base and tilt servos
Servo baseD11;
Servo baseD10;
Servo tiltD9;

// Servo pins
const byte BASE1_PIN = 11;
const byte BASE2_PIN = 10;
const byte TILT_PIN  = 9;

// HC-SR04 pins
const byte TRIG_PIN = 6;
const byte ECHO_PIN = 7;

// Scanner settings
const int AZ_STEP = 15;
const int TILT_MIN = 0;
const int TILT_MAX = 180;
const int TILT_STEP = 5;

// Timing
const unsigned long BASE_SETTLE_MS = 250;
const unsigned long TILT_SETTLE_MS = 30;

bool tiltUp = true;
bool scanPaused = false;
bool manualGoto = false;
int currentAzimuth = 0;
int currentTilt = 0;
int resumeAzimuth = 0;
int resumeTilt = 0;
char commandBuffer[20];
byte commandLength = 0;

void readSerialCommands();
void waitIfPaused();
void waitWithPause(unsigned long durationMs);
void writeAzimuth(int azimuth);
bool parseCoordinate(char *text, int *result);
long readDistanceCM();
void setAzimuth(int azimuth);
void reportPoint(int azimuth, int tiltAngle);
void scanTilt(int azimuth);

// Read PAUSE/RESUME at any point during the scan.
void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (commandLength == 0) continue;
      commandBuffer[commandLength] = '\0';
      commandLength = 0;

      // Convert the command to uppercase, so pause/resume are case-insensitive.
      for (byte i = 0; commandBuffer[i] != '\0'; i++) {
        if (commandBuffer[i] >= 'a' && commandBuffer[i] <= 'z') {
          commandBuffer[i] -= ('a' - 'A');
        }
      }

      if (strcmp(commandBuffer, "PAUSE") == 0 || strcmp(commandBuffer, "STOP") == 0) {
        if (!scanPaused) {
          scanPaused = true;
          Serial.println(F("# PAUSED at current scan point"));
        }
      } else if (strcmp(commandBuffer, "RESUME") == 0 || strcmp(commandBuffer, "START") == 0) {
        if (scanPaused) {
          if (manualGoto) {
            writeAzimuth(resumeAzimuth);
            tiltD9.write(resumeTilt);
            manualGoto = false;
          }
          scanPaused = false;
          Serial.println(F("# RESUMED; continuing scan"));
        }
      } else if (strncmp(commandBuffer, "GOTO ", 5) == 0) {
        char *coordinates = commandBuffer + 5;
        char *comma = strchr(coordinates, ',');
        if (comma == NULL) {
          Serial.println(F("# Use GOTO azimuth,tilt (example: GOTO 120,45)"));
        } else {
          *comma = '\0';
          int azimuth;
          int elevation;
          if (!parseCoordinate(coordinates, &azimuth) ||
              !parseCoordinate(comma + 1, &elevation) ||
              azimuth < 0 || azimuth > 360 ||
              elevation < TILT_MIN || elevation > TILT_MAX) {
            Serial.println(F("# GOTO range: azimuth 0-360, tilt 0-180"));
          } else {
            if (!manualGoto) {
              resumeAzimuth = currentAzimuth;
              resumeTilt = currentTilt;
            }
            manualGoto = true;
            scanPaused = true;
            writeAzimuth(azimuth);
            tiltD9.write(elevation);
            Serial.print(F("# GOTO "));
            Serial.print(azimuth);
            Serial.print(',');
            Serial.print(elevation);
            Serial.println(F("; scan paused; send RESUME to continue"));
          }
        }
      } else {
        Serial.println(F("# Commands: PAUSE, RESUME, GOTO azimuth,tilt"));
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    } else {
      commandLength = 0;
      Serial.println(F("# Command too long; send PAUSE or RESUME"));
    }
  }
}

// While paused, keep checking serial. The scan loop and its current angles stay intact.
void waitIfPaused() {
  readSerialCommands();
  while (scanPaused) {
    readSerialCommands();
    delay(2);
  }
}

// A pause does not consume the remaining settling time.
void waitWithPause(unsigned long durationMs) {
  while (durationMs > 0) {
    waitIfPaused();
    delay(1);
    durationMs--;
  }
}

bool parseCoordinate(char *text, int *result) {
  while (*text == ' ' || *text == '\t') text++;
  if (*text == '\0') return false;
  long value = 0;
  bool foundDigit = false;
  while (*text != '\0') {
    if (*text == ' ' || *text == '\t') {
      while (*text == ' ' || *text == '\t') text++;
      if (*text != '\0') return false;
      break;
    }
    if (*text < '0' || *text > '9') return false;
    foundDigit = true;
    value = value * 10 + (*text - '0');
    if (value > 10000) return false;
    text++;
  }
  if (!foundDigit) return false;
  *result = (int)value;
  return true;
}

void writeAzimuth(int azimuth) {
  if (azimuth <= 180) {
    baseD10.write(0);
    baseD11.write(azimuth);
  } else {
    baseD11.write(180);
    baseD10.write(azimuth - 180);
  }
}

long readDistanceCM() {
  waitIfPaused();
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000UL);
  waitIfPaused();

  if (duration == 0) return -1;
  long distance = duration / 58;
  if (distance < 2 || distance > 400) return -1;
  return distance;
}

void setAzimuth(int azimuth) {
  waitIfPaused();
  currentAzimuth = azimuth;
  writeAzimuth(azimuth);
  waitWithPause(BASE_SETTLE_MS);
}

void reportPoint(int azimuth, int tiltAngle) {
  waitWithPause(TILT_SETTLE_MS);
  long distance = readDistanceCM();

  Serial.print(azimuth);
  Serial.print(',');
  Serial.print(tiltAngle);
  Serial.print(',');
  Serial.print(distance);
  Serial.print(',');
  if (distance >= 0) Serial.println(F("ECHO_RECEIVED"));
  else Serial.println(F("NO_ECHO"));
}

void scanTilt(int azimuth) {
  if (tiltUp) {
    for (int el = TILT_MIN; el <= TILT_MAX; el += TILT_STEP) {
      waitIfPaused();
      currentTilt = el;
      tiltD9.write(el);
      reportPoint(azimuth, el);
    }
  } else {
    for (int el = TILT_MAX; el >= TILT_MIN; el -= TILT_STEP) {
      waitIfPaused();
      currentTilt = el;
      tiltD9.write(el);
      reportPoint(azimuth, el);
    }
  }
  tiltUp = !tiltUp;
}

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  baseD11.attach(BASE1_PIN);
  baseD10.attach(BASE2_PIN);
  tiltD9.attach(TILT_PIN);
  baseD11.write(0);
  baseD10.write(0);
  tiltD9.write(TILT_MIN);
  delay(700);

  Serial.println(F("AZ_CMD_DEG,TILT_CMD_DEG,DISTANCE_CM,STATUS"));
  Serial.println(F("# Send PAUSE to pause; RESUME to continue"));
}

void loop() {
  // Forward scan: 0 to 345 degrees.
  for (int az = 0; az < 360; az += AZ_STEP) {
    waitIfPaused();
    setAzimuth(az);
    scanTilt(az);
  }

  // Reverse scan: 330 back to 15 degrees.
  for (int az = 360 - 2 * AZ_STEP; az >= AZ_STEP; az -= AZ_STEP) {
    waitIfPaused();
    setAzimuth(az);
    scanTilt(az);
  }

  // Return home without another tilt scan.
  waitIfPaused();
  setAzimuth(0);
}
