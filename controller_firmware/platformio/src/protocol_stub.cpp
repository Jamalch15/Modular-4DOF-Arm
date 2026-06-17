#include <Arduino.h>

// Safe full-arm protocol stub for PC dashboard integration.
// This intentionally does not drive motors. It only validates and reports
// protocol-level state until hardware pins, drivers, gearing, limits, and
// homing details are known.

// Default to the CH343 USB-UART bridge used by the current ESP32-S3 setup.
// Define ARM_PROTOCOL_NATIVE_USB to use the ESP32-S3 native USB CDC port.
#if defined(ARM_PROTOCOL_NATIVE_USB)
#define ARM_SERIAL Serial
#else
#define ARM_SERIAL Serial0
#endif

#ifndef ESP_RGB_LED_PIN
#define ESP_RGB_LED_PIN 48
#endif

namespace {
constexpr unsigned long kSerialWaitMs = 3000;
constexpr uint32_t kStatusIntervalMs = 1000;
constexpr float kHomePose[4] = {0.0f, 20.0f, 20.0f, 0.0f};
constexpr float kJointMinDeg[4] = {-160.0f, -30.0f, -120.0f, -120.0f};
constexpr float kJointMaxDeg[4] = {160.0f, 115.0f, 120.0f, 120.0f};

enum class ControllerState {
  Idle,
  Moving,
  Stopped,
  Estop,
  Fault,
};

ControllerState controllerState = ControllerState::Idle;
bool homed = false;
bool armed = false;
bool configInProgress = false;
float currentJointsDeg[4] = {kHomePose[0], kHomePose[1], kHomePose[2],
                             kHomePose[3]};
float targetJointsDeg[4] = {kHomePose[0], kHomePose[1], kHomePose[2],
                            kHomePose[3]};
float lastSpeedDegS = 0.0f;
float lastAccelDegS2 = 0.0f;
char faultText[32] = "OK";
char toolState[12] = "unknown";
char poseSourceText[12] = "unknown";
float toolValue = 0.0f;
String commandLine;
uint32_t lastStatusMs = 0;

const char* stateName() {
  switch (controllerState) {
    case ControllerState::Idle:
      return "idle";
    case ControllerState::Moving:
      return "moving";
    case ControllerState::Stopped:
      return "stopped";
    case ControllerState::Estop:
      return "estop";
    case ControllerState::Fault:
      return "fault";
  }
  return "fault";
}

void setFault(const char* message) {
  controllerState = ControllerState::Fault;
  strlcpy(faultText, message, sizeof(faultText));
}

void clearFaultText() {
  strlcpy(faultText, "OK", sizeof(faultText));
}

void printHello() {
  ARM_SERIAL.println("HELLO name=esp32s3-arm firmware=protocol_stub protocol=1");
}

void printStatus() {
  ARM_SERIAL.printf(
      "STATUS state=%s homed=%d known=%d pose_source=%s armed=%d hw=simulated enabled=0000 enc=0000 e1=0.00 e2=0.00 "
      "j1=%.2f j2=%.2f j3=%.2f j4=%.2f closed_loop=off tool_type=generic tool=%s tool_value=%.3f fault=%s\r\n",
      stateName(), homed ? 1 : 0, homed ? 1 : 0, poseSourceText, armed ? 1 : 0, currentJointsDeg[0],
      currentJointsDeg[1], currentJointsDeg[2], currentJointsDeg[3], toolState, toolValue, faultText);
}

void printError(const char* code, const char* message) {
  ARM_SERIAL.printf("ERR code=%s message=%s\r\n", code, message);
}

bool validateJoints(const float joints[4]) {
  for (int i = 0; i < 4; i++) {
    if (joints[i] < kJointMinDeg[i] || joints[i] > kJointMaxDeg[i]) {
      ARM_SERIAL.printf("ERR code=LIMIT message=j%d_out_of_range\r\n", i + 1);
      return false;
    }
  }
  return true;
}

void handleMoveJ(const char* buffer) {
  if (controllerState == ControllerState::Estop) {
    printError("ESTOP", "emergency_stop_active");
    return;
  }

  float requested[4] = {};
  float speed = 0.0f;
  float accel = 0.0f;
  const int parsed =
      sscanf(buffer, "%*s %f %f %f %f %f %f", &requested[0], &requested[1],
             &requested[2], &requested[3], &speed, &accel);
  if (parsed != 6) {
    printError("USAGE", "MOVEJ_requires_j1_j2_j3_j4_speed_accel");
    return;
  }
  if (speed <= 0.0f || accel <= 0.0f) {
    printError("LIMIT", "speed_and_accel_must_be_positive");
    return;
  }
  if (!validateJoints(requested)) {
    return;
  }

  for (int i = 0; i < 4; i++) {
    targetJointsDeg[i] = requested[i];
    // Stub behavior: report the commanded pose immediately without moving
    // hardware. Real firmware should replace this with measured/executed pose.
    currentJointsDeg[i] = requested[i];
  }
  lastSpeedDegS = speed;
  lastAccelDegS2 = accel;
  controllerState = ControllerState::Idle;
  clearFaultText();
  ARM_SERIAL.println("OK command=MOVEJ");
  printStatus();
}

void handleArm(const char* buffer) {
  const int requested = String(buffer).substring(String(buffer).indexOf(' ') + 1).toInt();
  if (requested != 0 && controllerState == ControllerState::Estop) {
    printError("ESTOP", "emergency_stop_active");
    return;
  }
  armed = requested != 0;
  if (!armed && controllerState != ControllerState::Estop) {
    controllerState = ControllerState::Stopped;
  }
  ARM_SERIAL.printf("OK command=ARM armed=%d\r\n", armed ? 1 : 0);
  printStatus();
}

void handleSetPose(const char* buffer) {
  if (armed) {
    printError("ARM", "setpose_requires_disarmed");
    return;
  }
  float requested[4] = {};
  const int parsed = sscanf(buffer, "%*s %f %f %f %f", &requested[0], &requested[1], &requested[2], &requested[3]);
  if (parsed != 4) {
    printError("USAGE", "SETPOSE_requires_j1_j2_j3_j4");
    return;
  }
  if (!validateJoints(requested)) {
    return;
  }
  for (int i = 0; i < 4; i++) {
    targetJointsDeg[i] = requested[i];
    currentJointsDeg[i] = requested[i];
  }
  homed = true;
  strlcpy(poseSourceText, "setpose", sizeof(poseSourceText));
  controllerState = ControllerState::Stopped;
  clearFaultText();
  ARM_SERIAL.println("OK command=SETPOSE");
  printStatus();
}

float tokenFloat(const String& line, const char* key, float fallback) {
  const String prefix = String(key) + "=";
  int start = line.indexOf(prefix);
  if (start < 0) {
    return fallback;
  }
  start += prefix.length();
  int end = line.indexOf(' ', start);
  if (end < 0) {
    end = line.length();
  }
  return line.substring(start, end).toFloat();
}

void handleTool(const String& rawCommand, const String& upperCommand) {
  if (controllerState == ControllerState::Estop) {
    printError("ESTOP", "emergency_stop_active");
    return;
  }
  if (upperCommand.startsWith("TOOL OPEN")) {
    strlcpy(toolState, "open", sizeof(toolState));
    toolValue = 0.0f;
  } else if (upperCommand.startsWith("TOOL CLOSE")) {
    strlcpy(toolState, "closed", sizeof(toolState));
    toolValue = 1.0f;
  } else if (upperCommand.startsWith("TOOL SET")) {
    toolValue = max(0.0f, min(1.0f, tokenFloat(rawCommand, "value", toolValue)));
    strlcpy(toolState, "set", sizeof(toolState));
  } else if (upperCommand.startsWith("TOOL ON")) {
    strlcpy(toolState, "on", sizeof(toolState));
    toolValue = 1.0f;
  } else if (upperCommand.startsWith("TOOL OFF")) {
    strlcpy(toolState, "off", sizeof(toolState));
    toolValue = 0.0f;
  } else {
    printError("USAGE", "TOOL_requires_OPEN_CLOSE_ON_OFF_or_SET_value");
    return;
  }
  ARM_SERIAL.printf("OK command=TOOL state=%s value=%.3f\r\n", toolState, toolValue);
  printStatus();
}

void handleConfig(const String& upperCommand) {
  if (upperCommand.startsWith("CONFIG BEGIN")) {
    if (controllerState == ControllerState::Moving) {
      printError("CONFIG", "stop_motion_before_config");
      return;
    }
    configInProgress = true;
    return;
  }
  if (upperCommand.startsWith("CONFIG JOINT")) {
    if (!configInProgress) {
      printError("CONFIG", "begin_required");
    }
    return;
  }
  if (upperCommand.startsWith("CONFIG TOOL")) {
    if (!configInProgress) {
      printError("CONFIG", "begin_required");
    }
    return;
  }
  if (upperCommand.startsWith("CONFIG END")) {
    if (!configInProgress) {
      printError("CONFIG", "begin_required");
      return;
    }
    configInProgress = false;
    ARM_SERIAL.println("OK command=CONFIG axes=4 hw=simulated enabled=0000");
    printStatus();
    return;
  }
  printError("CONFIG", "unknown_config_command");
}

void handleHome() {
  if (controllerState == ControllerState::Estop) {
    printError("ESTOP", "emergency_stop_active");
    return;
  }

  for (int i = 0; i < 4; i++) {
    targetJointsDeg[i] = kHomePose[i];
    currentJointsDeg[i] = kHomePose[i];
  }
  homed = true;
  strlcpy(poseSourceText, "home", sizeof(poseSourceText));
  controllerState = ControllerState::Idle;
  clearFaultText();
  ARM_SERIAL.println("OK command=HOME");
  printStatus();
}

void handleStop() {
  for (int i = 0; i < 4; i++) {
    targetJointsDeg[i] = currentJointsDeg[i];
  }
  if (controllerState != ControllerState::Estop) {
    controllerState = ControllerState::Stopped;
    clearFaultText();
  }
  ARM_SERIAL.println("OK command=STOP");
  printStatus();
}

void handleEstop() {
  for (int i = 0; i < 4; i++) {
    targetJointsDeg[i] = currentJointsDeg[i];
  }
  controllerState = ControllerState::Estop;
  strlcpy(faultText, "ESTOP", sizeof(faultText));
  ARM_SERIAL.println("OK command=ESTOP");
  printStatus();
}

void handleCommand(String rawCommand) {
  rawCommand.trim();
  if (rawCommand.length() == 0) {
    return;
  }

  char buffer[160] = {};
  rawCommand.toCharArray(buffer, sizeof(buffer));

  char command[24] = {};
  sscanf(buffer, "%23s", command);
  String upperCommand = rawCommand;
  upperCommand.toUpperCase();

  if (strcasecmp(command, "HELLO") == 0) {
    printHello();
  } else if (strcasecmp(command, "STATUS") == 0) {
    printStatus();
  } else if (strcasecmp(command, "CONFIG") == 0) {
    handleConfig(upperCommand);
  } else if (strcasecmp(command, "MOVEJ") == 0) {
    handleMoveJ(buffer);
  } else if (strcasecmp(command, "ARM") == 0) {
    handleArm(buffer);
  } else if (strcasecmp(command, "SETPOSE") == 0) {
    handleSetPose(buffer);
  } else if (strcasecmp(command, "STOP") == 0) {
    handleStop();
  } else if (strcasecmp(command, "ESTOP") == 0) {
    handleEstop();
  } else if (strcasecmp(command, "HOME") == 0) {
    handleHome();
  } else if (strcasecmp(command, "TOOL") == 0) {
    handleTool(rawCommand, upperCommand);
  } else {
    printError("UNKNOWN", command);
  }
}

void readSerialCommands() {
  while (ARM_SERIAL.available() > 0) {
    const char incoming = static_cast<char>(ARM_SERIAL.read());
    if (incoming == '\n' || incoming == '\r') {
      handleCommand(commandLine);
      commandLine = "";
    } else if (commandLine.length() < 159) {
      commandLine += incoming;
    }
  }
}

void turnOffOnboardRgbLed() {
#if defined(ESP32)
  pinMode(ESP_RGB_LED_PIN, OUTPUT);
  digitalWrite(ESP_RGB_LED_PIN, LOW);
  neopixelWrite(ESP_RGB_LED_PIN, 0, 0, 0);
#endif
}
}  // namespace

void setup() {
  turnOffOnboardRgbLed();

  ARM_SERIAL.begin(115200);
  const unsigned long startMs = millis();
  while (!ARM_SERIAL && millis() - startMs < kSerialWaitMs) {
    delay(10);
  }

  clearFaultText();
  lastStatusMs = millis();
  printHello();
  printStatus();
}

void loop() {
  readSerialCommands();

  const uint32_t nowMs = millis();
  if (nowMs - lastStatusMs >= kStatusIntervalMs) {
    lastStatusMs = nowMs;
    printStatus();
  }

  (void)lastSpeedDegS;
  (void)lastAccelDegS2;
}
