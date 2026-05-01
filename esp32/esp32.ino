#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// I2C
#define SDA_PIN 8
#define SCL_PIN 9

// Channels
#define CH_BASE      11
#define CH_SHOULDER  12
#define CH_ELBOW     13
#define CH_WRIST     14
#define CH_CLAW      15

// PWM limits
#define PWM_MIN 102
#define PWM_MAX 512
#define CLAW_MIN 125

// More samples than max |ΔPWM| so smoothstep + integer rounding does not stair-step.
#define MOTION_OVERSAMPLE 3.0f

// Wi-Fi / TCP
struct WiFiCredential {
  const char* ssid;
  const char* pass;
};

const WiFiCredential WIFI_CREDENTIALS[] = {
  {"Park East", "SilverGoldJackal"},
  {"TTUguest", "fearthestache"},
};
const int WIFI_CREDENTIALS_COUNT = sizeof(WIFI_CREDENTIALS) / sizeof(WIFI_CREDENTIALS[0]);
const uint16_t TCP_PORT = 9000;

WiFiServer server(TCP_PORT);
WiFiClient client;

// UART control link from Raspberry Pi (TX/RX).
// Adjust pins to match your ESP32-S3 board wiring if needed.
HardwareSerial ArmSerial(0);
const int ARM_UART_RX_PIN = 44;  // ESP RX <- Pi TX (J3 RX / U0RXD)
const int ARM_UART_TX_PIN = 43;  // ESP TX -> Pi RX (J3 TX / U0TXD)
const uint32_t ARM_UART_BAUD = 115200;

struct ArmPose {
  int base;
  int shoulder;
  int elbow;
  int wrist;
  int claw;
};

ArmPose currentPose;
ArmPose poseHome;
ArmPose poseReady;
ArmPose poseReach;

struct JointMotion {
  bool active;
  int target;
  int step;
  int intervalMs;
  unsigned long lastMs;
  int channel;
};

// Non-blocking per-joint motion state (immediate stop/override behavior).
JointMotion baseMotion = {false, 0, 4, 10, 0, CH_BASE};
JointMotion shoulderMotion = {false, 0, 4, 10, 0, CH_SHOULDER};
JointMotion elbowMotion = {false, 0, 4, 10, 0, CH_ELBOW};
JointMotion wristMotion = {false, 0, 4, 10, 0, CH_WRIST};
JointMotion clawMotion = {false, 0, 4, 10, 0, CH_CLAW};

// ---------- Helpers ----------
int clamp(int val, int minv, int maxv) {
  if (val < minv) return minv;
  if (val > maxv) return maxv;
  return val;
}

int minForChannel(int ch) {
  if (ch == CH_CLAW) return CLAW_MIN;
  return PWM_MIN;
}

int maxForChannel(int ch) {
  (void)ch;
  return PWM_MAX;
}

int clampForChannel(int ch, int val) {
  return clamp(val, minForChannel(ch), maxForChannel(ch));
}

void writeJoint(int ch, int val) {
  val = clampForChannel(ch, val);
  pwm.setPWM(ch, 0, val);
}

void writePose(const ArmPose &p) {
  writeJoint(CH_BASE, p.base);
  writeJoint(CH_SHOULDER, p.shoulder);
  writeJoint(CH_ELBOW, p.elbow);
  writeJoint(CH_WRIST, p.wrist);
  writeJoint(CH_CLAW, p.claw);
}

int* posePtrByChannel(int ch) {
  if (ch == CH_BASE) return &currentPose.base;
  if (ch == CH_SHOULDER) return &currentPose.shoulder;
  if (ch == CH_ELBOW) return &currentPose.elbow;
  if (ch == CH_WRIST) return &currentPose.wrist;
  if (ch == CH_CLAW) return &currentPose.claw;
  return nullptr;
}

JointMotion* motionByChannel(int ch) {
  if (ch == CH_BASE) return &baseMotion;
  if (ch == CH_SHOULDER) return &shoulderMotion;
  if (ch == CH_ELBOW) return &elbowMotion;
  if (ch == CH_WRIST) return &wristMotion;
  if (ch == CH_CLAW) return &clawMotion;
  return nullptr;
}

void configureMotionProfile(String speed, int &step, int &intervalMs) {
  speed.trim();
  speed.toLowerCase();
  if (speed == "slow") {
    step = 2;
    intervalMs = 18;
  } else if (speed == "fast") {
    step = 8;
    intervalMs = 6;
  } else {
    // "medium" default
    step = 4;
    intervalMs = 10;
  }
}

void configureBaseMotionProfile(String speed) {
  configureMotionProfile(speed, baseMotion.step, baseMotion.intervalMs);
}

void configureWristMotionProfile(String speed) {
  configureMotionProfile(speed, wristMotion.step, wristMotion.intervalMs);
}

void configureJointMotionProfile(JointMotion &jm, String speed) {
  configureMotionProfile(speed, jm.step, jm.intervalMs);
}

void stopJointMotion(JointMotion &jm) {
  jm.active = false;
  int* p = posePtrByChannel(jm.channel);
  if (p) jm.target = *p;
}

void startJointMotion(JointMotion &jm, int target, String speed = "medium") {
  int* p = posePtrByChannel(jm.channel);
  if (!p) return;
  target = clampForChannel(jm.channel, target);
  configureJointMotionProfile(jm, speed);
  jm.target = target;
  jm.lastMs = 0;
  jm.active = (jm.target != *p);
}

void updateJointMotion(JointMotion &jm) {
  if (!jm.active) return;
  int* p = posePtrByChannel(jm.channel);
  if (!p) {
    jm.active = false;
    return;
  }

  unsigned long now = millis();
  if (jm.lastMs != 0 && (now - jm.lastMs) < (unsigned long)jm.intervalMs) {
    return;
  }
  jm.lastMs = now;

  int cur = *p;
  if (cur == jm.target) {
    stopJointMotion(jm);
    return;
  }

  int dir = (jm.target > cur) ? 1 : -1;
  int next = cur + dir * jm.step;
  if ((dir > 0 && next > jm.target) || (dir < 0 && next < jm.target)) {
    next = jm.target;
  }

  *p = next;
  writeJoint(jm.channel, *p);

  if (*p == jm.target) {
    stopJointMotion(jm);
  }
}

void stopAllJointMotions() {
  stopJointMotion(baseMotion);
  stopJointMotion(shoulderMotion);
  stopJointMotion(elbowMotion);
  stopJointMotion(wristMotion);
  stopJointMotion(clawMotion);
}

void updateAllJointMotions() {
  updateJointMotion(baseMotion);
  updateJointMotion(shoulderMotion);
  updateJointMotion(elbowMotion);
  updateJointMotion(wristMotion);
  updateJointMotion(clawMotion);
}

void stopBaseMotion() {
  stopJointMotion(baseMotion);
}

void stopWristMotion() {
  stopJointMotion(wristMotion);
}

void startBaseMotion(int target, String speed = "medium") {
  startJointMotion(baseMotion, target, speed);
}

void startWristMotion(int target, String speed = "medium") {
  startJointMotion(wristMotion, target, speed);
}

void updateBaseMotion() {
  updateJointMotion(baseMotion);
}

void updateWristMotion() {
  updateJointMotion(wristMotion);
}

// Ease-in-out in [0,1] — zero velocity at ends (smoother than linear 1-PWM stepping).
static float smoothstep01(float t) {
  if (t <= 0.0f) return 0.0f;
  if (t >= 1.0f) return 1.0f;
  return t * t * (3.0f - 2.0f * t);
}

// How many interpolation samples: at least proportional to largest joint travel.
static int motionStepsForPose(const ArmPose &from, const ArmPose &to) {
  int db = abs(to.base - from.base);
  int ds = abs(to.shoulder - from.shoulder);
  int de = abs(to.elbow - from.elbow);
  int dw = abs(to.wrist - from.wrist);
  int dc = abs(to.claw - from.claw);
  int m = db;
  if (ds > m) m = ds;
  if (de > m) m = de;
  if (dw > m) m = dw;
  if (dc > m) m = dc;
  return m;
}

// Interpolated move: same endpoints as before, smoother velocity profile (ESP-side).
void moveSmooth(const ArmPose &target, int delayMs = 16) {
  ArmPose start = currentPose;

  int stepsRaw = motionStepsForPose(start, target);
  if (stepsRaw < 1) {
    currentPose = target;
    writePose(currentPose);
    return;
  }

  int steps = (int)ceilf((float)stepsRaw * MOTION_OVERSAMPLE);
  if (steps < 1) steps = 1;
  int perDelay = 0;
  if (delayMs > 0) {
    perDelay = (int)roundf((float)delayMs / MOTION_OVERSAMPLE);
    if (perDelay < 1) perDelay = 1;
  }

  int db = target.base - start.base;
  int ds = target.shoulder - start.shoulder;
  int de = target.elbow - start.elbow;
  int dw = target.wrist - start.wrist;
  int dc = target.claw - start.claw;

  for (int i = 1; i <= steps; i++) {
    float t = (float)i / (float)steps;
    t = smoothstep01(t);

    currentPose.base = start.base + (int)roundf((float)db * t);
    currentPose.shoulder = start.shoulder + (int)roundf((float)ds * t);
    currentPose.elbow = start.elbow + (int)roundf((float)de * t);
    currentPose.wrist = start.wrist + (int)roundf((float)dw * t);
    currentPose.claw = start.claw + (int)roundf((float)dc * t);

    writePose(currentPose);
    if (perDelay > 0) {
      delay(perDelay);
    }
  }

  currentPose = target;
  writePose(currentPose);
}

void replyLine(const String &msg) {
  Serial.println(msg);
  if (client && client.connected()) {
    client.println(msg);
  }
  ArmSerial.println(msg);
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "home") {
    stopAllJointMotions();
    moveSmooth(poseHome);
    replyLine("OK home");
  }
  else if (cmd == "ready") {
    stopAllJointMotions();
    moveSmooth(poseReady);
    replyLine("OK ready");
  }
  else if (cmd == "reach") {
    stopAllJointMotions();
    moveSmooth(poseReach);
    replyLine("OK reach");
  }
  else if (cmd == "open") {
    stopAllJointMotions();
    ArmPose p = currentPose;
    p.claw = 140;
    moveSmooth(p, 10);
    replyLine("OK open");
  }
  else if (cmd == "close") {
    stopAllJointMotions();
    ArmPose p = currentPose;
    p.claw = 320;
    moveSmooth(p, 10);
    replyLine("OK close");
  }
  else if (cmd == "status") {
    String s = "STATUS ";
    s += String(currentPose.base) + " ";
    s += String(currentPose.shoulder) + " ";
    s += String(currentPose.elbow) + " ";
    s += String(currentPose.wrist) + " ";
    s += String(currentPose.claw);
    replyLine(s);
  }
  else if (cmd == "stop") {
    stopAllJointMotions();
    replyLine("OK stop");
  }
  else if (cmd.startsWith("movebase")) {
    int target = 0;
    char speedBuf[16] = "medium";
    int parsed = sscanf(cmd.c_str(), "movebase %d %15s", &target, speedBuf);
    if (parsed >= 1) {
      String speed = (parsed == 2) ? String(speedBuf) : String("medium");
      startBaseMotion(target, speed);
      replyLine("OK movebase");
    } else {
      replyLine("ERR movebase");
    }
  }
  else if (cmd == "basestatus") {
    String s = "BASESTATUS ";
    s += String(currentPose.base) + " ";
    s += String(baseMotion.target) + " ";
    s += String(baseMotion.active ? 1 : 0);
    replyLine(s);
  }
  else if (cmd.startsWith("movewrist")) {
    int target = 0;
    char speedBuf[16] = "medium";
    int parsed = sscanf(cmd.c_str(), "movewrist %d %15s", &target, speedBuf);
    if (parsed >= 1) {
      String speed = (parsed == 2) ? String(speedBuf) : String("medium");
      startWristMotion(target, speed);
      replyLine("OK movewrist");
    } else {
      replyLine("ERR movewrist");
    }
  }
  else if (cmd == "wriststatus") {
    String s = "WRISTSTATUS ";
    s += String(currentPose.wrist) + " ";
    s += String(wristMotion.target) + " ";
    s += String(wristMotion.active ? 1 : 0);
    replyLine(s);
  }
  else if (cmd.startsWith("movejoint")) {
    int ch = 0;
    int target = 0;
    char speedBuf[16] = "medium";
    int parsed = sscanf(cmd.c_str(), "movejoint %d %d %15s", &ch, &target, speedBuf);
    JointMotion* jm = motionByChannel(ch);
    if (parsed >= 2 && jm != nullptr) {
      String speed = (parsed == 3) ? String(speedBuf) : String("medium");
      startJointMotion(*jm, target, speed);
      replyLine("OK movejoint");
    } else {
      replyLine("ERR movejoint");
    }
  }
  else if (cmd.startsWith("moveshoulder")) {
    int target = 0;
    char speedBuf[16] = "medium";
    int parsed = sscanf(cmd.c_str(), "moveshoulder %d %15s", &target, speedBuf);
    if (parsed >= 1) {
      String speed = (parsed == 2) ? String(speedBuf) : String("medium");
      startJointMotion(shoulderMotion, target, speed);
      replyLine("OK moveshoulder");
    } else {
      replyLine("ERR moveshoulder");
    }
  }
  else if (cmd.startsWith("moveelbow")) {
    int target = 0;
    char speedBuf[16] = "medium";
    int parsed = sscanf(cmd.c_str(), "moveelbow %d %15s", &target, speedBuf);
    if (parsed >= 1) {
      String speed = (parsed == 2) ? String(speedBuf) : String("medium");
      startJointMotion(elbowMotion, target, speed);
      replyLine("OK moveelbow");
    } else {
      replyLine("ERR moveelbow");
    }
  }
  else if (cmd.startsWith("moveclaw")) {
    int target = 0;
    char speedBuf[16] = "medium";
    int parsed = sscanf(cmd.c_str(), "moveclaw %d %15s", &target, speedBuf);
    if (parsed >= 1) {
      String speed = (parsed == 2) ? String(speedBuf) : String("medium");
      startJointMotion(clawMotion, target, speed);
      replyLine("OK moveclaw");
    } else {
      replyLine("ERR moveclaw");
    }
  }
  else if (cmd == "jointstatus") {
    String s = "JOINTSTATUS ";
    s += String(currentPose.base) + " " + String(baseMotion.target) + " " + String(baseMotion.active ? 1 : 0) + " ";
    s += String(currentPose.shoulder) + " " + String(shoulderMotion.target) + " " + String(shoulderMotion.active ? 1 : 0) + " ";
    s += String(currentPose.elbow) + " " + String(elbowMotion.target) + " " + String(elbowMotion.active ? 1 : 0) + " ";
    s += String(currentPose.wrist) + " " + String(wristMotion.target) + " " + String(wristMotion.active ? 1 : 0) + " ";
    s += String(currentPose.claw) + " " + String(clawMotion.target) + " " + String(clawMotion.active ? 1 : 0);
    replyLine(s);
  }
  else if (cmd.startsWith("setall")) {
    int b, s, e, w, c;
    if (sscanf(cmd.c_str(), "setall %d %d %d %d %d", &b, &s, &e, &w, &c) == 5) {
      stopAllJointMotions();
      b = clampForChannel(CH_BASE, b);
      s = clampForChannel(CH_SHOULDER, s);
      e = clampForChannel(CH_ELBOW, e);
      w = clampForChannel(CH_WRIST, w);
      c = clampForChannel(CH_CLAW, c);
      writeJoint(CH_BASE, b);
      writeJoint(CH_SHOULDER, s);
      writeJoint(CH_ELBOW, e);
      writeJoint(CH_WRIST, w);
      writeJoint(CH_CLAW, c);

      currentPose.base = b;
      currentPose.shoulder = s;
      currentPose.elbow = e;
      currentPose.wrist = w;
      currentPose.claw = c;

      replyLine("OK setall");
    } else {
      replyLine("ERR setall");
    }
  }
  else if (cmd.startsWith("set")) {
    int ch, val;
    if (sscanf(cmd.c_str(), "set %d %d", &ch, &val) == 2) {
      JointMotion* jm = motionByChannel(ch);
      if (jm != nullptr) {
        stopJointMotion(*jm);
      }
      val = clampForChannel(ch, val);
      writeJoint(ch, val);

      if (ch == 11) currentPose.base = val;
      if (ch == 12) currentPose.shoulder = val;
      if (ch == 13) currentPose.elbow = val;
      if (ch == 14) currentPose.wrist = val;
      if (ch == 15) currentPose.claw = val;

      replyLine("OK set");
    } else {
      replyLine("ERR set");
    }
  }
  else {
    replyLine("ERR unknown");
  }
}

bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  const unsigned long maxConnectMs = 15000;

  for (int i = 0; i < WIFI_CREDENTIALS_COUNT; ++i) {
    const char* ssid = WIFI_CREDENTIALS[i].ssid;
    const char* pass = WIFI_CREDENTIALS[i].pass;

    Serial.print("Connecting WiFi SSID: ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);

    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      if (millis() - t0 > maxConnectMs) {
        Serial.println("\nWiFi attempt timeout.");
        WiFi.disconnect(true, true);
        delay(300);
        break;
      }
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.print("WiFi connected (");
      Serial.print(ssid);
      Serial.print("), IP: ");
      Serial.println(WiFi.localIP());
      return true;
    }
  }

  Serial.println("WiFi connect failed for all SSIDs, continuing with UART control only.");
  return false;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  ArmSerial.begin(ARM_UART_BAUD, SERIAL_8N1, ARM_UART_RX_PIN, ARM_UART_TX_PIN);
  Wire.begin(SDA_PIN, SCL_PIN);

  pwm.begin();
  pwm.setPWMFreq(50);

  delay(500);

  // Poses (home adjusted to match Pi reset/default pose)
  poseHome = {307, 440, 150, 196, 320};

  poseReady = {
    307,
    440,
    150,
    180,
    140
  };

  poseReach = {
    307,
    420,
    200,
    220,
    140
  };

  currentPose = poseHome;
  writePose(currentPose);

  bool wifiOk = connectWiFi();
  if (wifiOk) {
    server.begin();
    server.setNoDelay(true);
    Serial.print("TCP server listening on port ");
    Serial.println(TCP_PORT);
  } else {
    Serial.println("TCP server disabled (no WiFi), UART control still active.");
  }
  Serial.print("UART control ready on RX=");
  Serial.print(ARM_UART_RX_PIN);
  Serial.print(" TX=");
  Serial.println(ARM_UART_TX_PIN);
  Serial.println("READY");
}

// ---------- Loop ----------
void loop() {
  updateAllJointMotions();

  // Accept one client at a time
  if (!client || !client.connected()) {
    WiFiClient newClient = server.available();
    if (newClient) {
      if (client && client.connected()) {
        client.stop();
      }
      client = newClient;
      client.setNoDelay(true);
      Serial.print("Client connected: ");
      Serial.println(client.remoteIP());
      client.println("READY");
    }
  }

  // Network command input (serial command input intentionally disabled)
  if (client && client.connected() && client.available()) {
    String cmd = client.readStringUntil('\n');
    handleCommand(cmd);
  }

  // UART command input from Raspberry Pi
  if (ArmSerial.available()) {
    String cmd = ArmSerial.readStringUntil('\n');
    handleCommand(cmd);
  }
}
