#include <Arduino.h>
#include <math.h>

#include <SPI.h>

// Full-arm ESP32-S3 controller for the PC dashboard protocol.
// Current assumption: the PC plans trajectories and uploads timed joint points.
// The ESP follows the queued path open-loop, drives only configured hardware
// axes, and simulates disabled axes in its reported pose.

#if defined(ARM_CONTROLLER_NATIVE_USB) || defined(ARM_PROTOCOL_NATIVE_USB)
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
constexpr int kJointCount = 4;
constexpr int kMaxLineLength = 320;
constexpr int kMaxTrajectoryPoints = 240;
constexpr uint32_t kJogWatchdogMs = 350;
constexpr int kServoPwmResolutionBits = 14;
constexpr int kServoPwmChannelBase = 0;
constexpr uint32_t kServoPwmMaxDuty = (1UL << kServoPwmResolutionBits) - 1UL;
constexpr float kDefaultHome[kJointCount] = {0.0f, 20.0f, 20.0f, 0.0f};
constexpr float kDefaultMin[kJointCount] = {-160.0f, -30.0f, -120.0f, -120.0f};
constexpr float kDefaultMax[kJointCount] = {160.0f, 115.0f, 120.0f, 120.0f};
const char* kDefaultNames[kJointCount] = {"base", "shoulder", "elbow", "wrist"};

constexpr uint32_t kEncoderSpiClockHz = 1000000;
constexpr uint16_t kAs5048AngleAddress = 0x3FFF;
constexpr uint16_t kAs5048DiagnosticsAddress = 0x3FFD;
constexpr uint16_t kAs5048ClearErrorCommand = 0x4001;
constexpr uint16_t kAs5048AngleMask = 0x3FFF;
constexpr uint16_t kAs5048ErrorFlag = 0x4000;

enum class ControllerState {
  Idle,
  Moving,
  Stopped,
  Estop,
  Fault,
};

enum class ActuatorType {
  Stepper,
  Servo,
  Unknown,
};

enum class AxisState {
  Simulated,
  Hardware,
  Invalid,
};

enum class ToolType {
  Generic,
  ServoGripper,
  Electromagnet,
};

struct StepperConfig {
  int stepPin = -1;
  int dirPin = -1;
  int enablePin = -1;
  bool enableActiveLow = true;
  int fullStepsPerRev = 200;
  int microsteps = 16;
  float gearRatio = 1.0f;
};

struct ServoConfig {
  int pwmPin = -1;
  int pulseMinUs = 500;
  int pulseMaxUs = 2500;
  int pwmFrequencyHz = 50;
  float rangeDeg = 270.0f;
  float neutralDeg = 135.0f;
  float gearRatio = 1.0f;
};

struct JointConfig {
  char name[20] = "";
  ActuatorType actuator = ActuatorType::Unknown;
  bool enabled = false;
  bool received = false;
  float zeroOffsetDeg = 0.0f;
  int directionSign = 1;
  float minDeg = -180.0f;
  float maxDeg = 180.0f;
  float homeDeg = 0.0f;
  float maxSpeedDegS = 45.0f;
  float maxAccelDegS2 = 120.0f;
  StepperConfig stepper;
  ServoConfig servo;
  AxisState axisState = AxisState::Simulated;
};

struct ToolConfig {
  char name[20] = "generic";
  ToolType type = ToolType::Generic;
  bool received = false;
  int pwmPin = -1;
  int pulseMinUs = 500;
  int pulseMaxUs = 2500;
  int pwmFrequencyHz = 50;
  float openValue = 0.0f;
  float closedValue = 1.0f;
  int gpioPin = -1;
  bool activeHigh = true;
};

struct EncoderBusConfig {
  bool enabled = false;
  int sckPin = 12;
  int misoPin = 13;
  int mosiPin = 14;
  uint32_t clockHz = kEncoderSpiClockHz;
  uint32_t sampleIntervalMs = 100;
};

struct EncoderConfig {
  bool received = false;
  bool enabled = false;
  int jointIndex = -1;
  int csPin = -1;
  float referenceRawDeg = 0.0f;
  float referenceJointDeg = 0.0f;
  int directionSign = 1;
  float wrapPeriodDeg = 360.0f;
  float sensorTurnsPerJointTurn = 1.0f;
  uint32_t freshnessTimeoutMs = 500;
  float maxNoiseDeg = 0.5f;
  bool calibrationValidated = false;
  char mounting[20] = "joint_output";
  char calibrationId[40] = "none";
};

struct EncoderPolicy {
  char mode[24] = "diagnostic";
  char verificationPolicy[16] = "diagnostic";
  uint32_t settleDelayMs = 300;
  int requiredStableSamples = 3;
  float warningToleranceDeg = 2.0f;
  float faultToleranceDeg = 5.0f;
  float hysteresisDeg = 0.25f;
  bool requireEncoder = false;
  bool correctionEnabled = false;
  char validationId[40] = "none";
  float maxCorrectionDeltaDeg = 1.0f;
  float correctionJointLimitMarginDeg = 2.0f;
  float correctionSpeedDegS = 2.0f;
  float correctionAccelDegS2 = 10.0f;
  int maxCorrectionAttempts = 2;
};

struct StepperRuntime {
  long currentSteps = 0;
  long targetSteps = 0;
  unsigned long lastStepUs = 0;
};

struct ServoRuntime {
  int pulseUs = 1500;
  int channel = -1;
  bool attached = false;
  float velocityDegS = 0.0f;
  unsigned long lastUpdateUs = 0;
};

struct ToolRuntime {
  int channel = 6;
  bool attached = false;
  int attachedPwmPin = -1;
  int gpioPin = -1;
  bool gpioActiveHigh = true;
};

struct EncoderRuntime {
  bool available = false;
  bool valid = false;
  uint16_t rawCount = 0;
  float rawAngleDeg = 0.0f;
  float measuredJointDeg = 0.0f;
  float noiseDeg = 0.0f;
  float previousRawAngleDeg = 0.0f;
  bool hasPrevious = false;
  uint16_t diagnostics = 0;
  uint32_t lastAttemptMs = 0;
  uint32_t lastValidMs = 0;
  int consecutiveValidSamples = 0;
  char flags[48] = "none";
};

struct TrajectoryPoint {
  float timeS = 0.0f;
  float jointsDeg[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f};
};

struct TrajectoryRuntime {
  TrajectoryPoint points[kMaxTrajectoryPoints];
  int expectedCount = 0;
  int count = 0;
  bool receiving = false;
  bool ready = false;
  bool active = false;
  unsigned long startUs = 0;
  float durationS = 0.0f;
};

struct PositionStreamRuntime {
  bool active = false;
  unsigned long startUs = 0;
  float durationS = 0.0f;
  float startDeg[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f};
  float targetDeg[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f};
};

ControllerState controllerState = ControllerState::Idle;
JointConfig joints[kJointCount];
JointConfig draftJoints[kJointCount];
ToolConfig activeTool;
ToolConfig draftTool;
EncoderBusConfig encoderBus;
EncoderBusConfig draftEncoderBus;
EncoderConfig encoderConfigs[kJointCount];
EncoderConfig draftEncoderConfigs[kJointCount];
EncoderPolicy encoderPolicy;
EncoderPolicy draftEncoderPolicy;
StepperRuntime stepperRuntime[kJointCount];
ServoRuntime servoRuntime[kJointCount];
ToolRuntime toolRuntime;
EncoderRuntime encoderRuntime[kJointCount];
TrajectoryRuntime trajectoryRuntime;
PositionStreamRuntime positionStreamRuntime;
float currentJointsDeg[kJointCount] = {kDefaultHome[0], kDefaultHome[1], kDefaultHome[2], kDefaultHome[3]};
float targetJointsDeg[kJointCount] = {kDefaultHome[0], kDefaultHome[1], kDefaultHome[2], kDefaultHome[3]};
float lastSpeedDegS = 25.0f;
float lastAccelDegS2 = 120.0f;
bool armed = false;
bool homed = false;
bool knownPose = false;
bool configInProgress = false;
bool jogActive = false;
bool jogVelocityMode = false;
bool jogStopRequested = false;
uint32_t lastJogMs = 0;
unsigned long jogLastUpdateUs = 0;
float jogTargetVelocityDegS[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f};
float jogCurrentVelocityDegS[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f};
float jogStepperRemainderSteps[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f};
char faultText[40] = "OK";
char toolState[12] = "unknown";
char poseSourceText[24] = "unknown";
float toolValue = 0.0f;
bool encoderAvailable[kJointCount] = {false, false, false, false};
float encoderAnglesDeg[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f};
float correctionBiasDeg[kJointCount] = {0.0f, 0.0f, 0.0f, 0.0f};
char correctionState[20] = "idle";
char correctionTransactionId[40] = "none";
int correctionAttempts = 0;
bool correctionActive = false;
int correctionJointIndex = -1;
float pendingCorrectionBiasDeltaDeg = 0.0f;
long correctionStartSteps = 0;
long lastCorrectionEmittedSteps = 0;
float lastCorrectionRequestedDeltaDeg = 0.0f;
unsigned long correctionStartUs = 0;
bool startupAlignmentActive = false;
bool startupAlignmentHold = false;
String commandLine;
uint32_t lastStatusMs = 0;
bool encoderSpiStarted = false;

float clampFloat(float value, float minValue, float maxValue) {
  return min(max(value, minValue), maxValue);
}

bool validPinOrUnused(int pin) {
  return pin == -1 || ((pin >= 0 && pin <= 21) || (pin >= 26 && pin <= 48));
}

ActuatorType parseActuator(const String& value) {
  if (value.equalsIgnoreCase("stepper")) {
    return ActuatorType::Stepper;
  }
  if (value.equalsIgnoreCase("servo")) {
    return ActuatorType::Servo;
  }
  return ActuatorType::Unknown;
}

const char* actuatorName(ActuatorType actuator) {
  switch (actuator) {
    case ActuatorType::Stepper:
      return "stepper";
    case ActuatorType::Servo:
      return "servo";
    case ActuatorType::Unknown:
      return "unknown";
  }
  return "unknown";
}

ToolType parseToolType(const String& value) {
  if (value.equalsIgnoreCase("servo_gripper")) {
    return ToolType::ServoGripper;
  }
  if (value.equalsIgnoreCase("electromagnet")) {
    return ToolType::Electromagnet;
  }
  return ToolType::Generic;
}

const char* toolTypeName(ToolType type) {
  switch (type) {
    case ToolType::ServoGripper:
      return "servo_gripper";
    case ToolType::Electromagnet:
      return "electromagnet";
    case ToolType::Generic:
      return "generic";
  }
  return "generic";
}

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

JointConfig defaultJoint(int index) {
  JointConfig joint;
  snprintf(joint.name, sizeof(joint.name), "%s", kDefaultNames[index]);
  joint.actuator = index < 2 ? ActuatorType::Stepper : ActuatorType::Servo;
  joint.enabled = false;
  joint.received = false;
  joint.zeroOffsetDeg = 0.0f;
  joint.directionSign = 1;
  joint.minDeg = kDefaultMin[index];
  joint.maxDeg = kDefaultMax[index];
  joint.homeDeg = kDefaultHome[index];
  joint.maxSpeedDegS = index < 2 ? 45.0f : 60.0f;
  joint.maxAccelDegS2 = index < 2 ? 120.0f : 180.0f;
  joint.axisState = AxisState::Simulated;
  return joint;
}

ToolConfig defaultTool() {
  ToolConfig tool;
  snprintf(tool.name, sizeof(tool.name), "generic");
  tool.type = ToolType::Generic;
  tool.received = false;
  return tool;
}

EncoderConfig defaultEncoder(int index) {
  EncoderConfig encoder;
  encoder.jointIndex = index;
  encoder.received = false;
  encoder.enabled = false;
  encoder.csPin = -1;
  return encoder;
}

void resetDraftConfig() {
  for (int i = 0; i < kJointCount; i++) {
    draftJoints[i] = defaultJoint(i);
    draftEncoderConfigs[i] = defaultEncoder(i);
  }
  draftTool = defaultTool();
  draftEncoderBus = EncoderBusConfig();
  draftEncoderPolicy = EncoderPolicy();
}

void clearFaultText() {
  strlcpy(faultText, "OK", sizeof(faultText));
}

void setFault(const char* message) {
  controllerState = ControllerState::Fault;
  strlcpy(faultText, message, sizeof(faultText));
}

String tokenValue(const String& line, const char* key, const String& fallback) {
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
  return line.substring(start, end);
}

int tokenInt(const String& line, const char* key, int fallback) {
  String value = tokenValue(line, key, "");
  if (value.length() == 0) {
    return fallback;
  }
  return value.toInt();
}

float tokenFloat(const String& line, const char* key, float fallback) {
  String value = tokenValue(line, key, "");
  if (value.length() == 0) {
    return fallback;
  }
  return value.toFloat();
}

String tokenString(const String& line, const char* key, const String& fallback) {
  return tokenValue(line, key, fallback);
}

void clearTrajectory() {
  trajectoryRuntime.expectedCount = 0;
  trajectoryRuntime.count = 0;
  trajectoryRuntime.receiving = false;
  trajectoryRuntime.ready = false;
  trajectoryRuntime.active = false;
  trajectoryRuntime.startUs = 0;
  trajectoryRuntime.durationS = 0.0f;
}

void clearJogMotion(bool freezeTarget = false) {
  positionStreamRuntime.active = false;
  positionStreamRuntime.startUs = 0;
  positionStreamRuntime.durationS = 0.0f;
  jogActive = false;
  jogVelocityMode = false;
  jogStopRequested = false;
  lastJogMs = 0;
  jogLastUpdateUs = 0;
  for (int i = 0; i < kJointCount; i++) {
    jogTargetVelocityDegS[i] = 0.0f;
    jogCurrentVelocityDegS[i] = 0.0f;
    jogStepperRemainderSteps[i] = 0.0f;
  }
  if (!freezeTarget) {
    return;
  }
  const unsigned long nowUs = micros();
  for (int i = 0; i < kJointCount; i++) {
    targetJointsDeg[i] = currentJointsDeg[i];
    if (joints[i].actuator == ActuatorType::Stepper) {
      stepperRuntime[i].targetSteps = stepperRuntime[i].currentSteps;
    } else if (joints[i].actuator == ActuatorType::Servo) {
      servoRuntime[i].velocityDegS = 0.0f;
      servoRuntime[i].lastUpdateUs = nowUs;
    }
  }
  if (controllerState == ControllerState::Moving) {
    controllerState = ControllerState::Idle;
  }
}

String validationErrorForJoint(const JointConfig& joint, int index) {
  if (!joint.enabled) {
    return "";
  }
  if (joint.actuator == ActuatorType::Stepper) {
    if (joint.stepper.stepPin < 0 || !validPinOrUnused(joint.stepper.stepPin)) {
      return "joint_" + String(index + 1) + "_missing_step_pin";
    }
    if (joint.stepper.dirPin < 0 || !validPinOrUnused(joint.stepper.dirPin)) {
      return "joint_" + String(index + 1) + "_missing_dir_pin";
    }
    if (!validPinOrUnused(joint.stepper.enablePin)) {
      return "joint_" + String(index + 1) + "_invalid_optional_pin";
    }
    if (joint.stepper.fullStepsPerRev <= 0) {
      return "joint_" + String(index + 1) + "_invalid_full_steps";
    }
    if (joint.stepper.microsteps <= 0) {
      return "joint_" + String(index + 1) + "_invalid_microsteps";
    }
    if (joint.stepper.gearRatio <= 0.0f) {
      return "joint_" + String(index + 1) + "_invalid_gear";
    }
    return "";
  }
  if (joint.actuator == ActuatorType::Servo) {
    if (joint.servo.pwmPin < 0 || !validPinOrUnused(joint.servo.pwmPin)) {
      return "joint_" + String(index + 1) + "_missing_pwm_pin";
    }
    if (joint.servo.pulseMinUs <= 0 || joint.servo.pulseMaxUs <= joint.servo.pulseMinUs) {
      return "joint_" + String(index + 1) + "_invalid_pulse_range";
    }
    if (joint.servo.pwmFrequencyHz <= 0 || joint.servo.rangeDeg <= 0.0f || joint.servo.gearRatio <= 0.0f) {
      return "joint_" + String(index + 1) + "_invalid_servo_mapping";
    }
    return "";
  }
  return "joint_" + String(index + 1) + "_unsupported_actuator";
}

String validationErrorForTool(const ToolConfig& tool) {
  if (tool.type == ToolType::Generic) {
    return "";
  }
  if (tool.type == ToolType::ServoGripper) {
    if (tool.pwmPin < 0 || !validPinOrUnused(tool.pwmPin)) {
      return "active_tool_missing_pwm_pin";
    }
    if (tool.pulseMinUs <= 0 || tool.pulseMaxUs <= tool.pulseMinUs) {
      return "active_tool_invalid_pulse_range";
    }
    if (tool.pwmFrequencyHz <= 0) {
      return "active_tool_invalid_pwm_frequency";
    }
    if (tool.openValue < 0.0f || tool.openValue > 1.0f || tool.closedValue < 0.0f || tool.closedValue > 1.0f) {
      return "active_tool_open_close_values_out_of_range";
    }
    return "";
  }
  if (tool.type == ToolType::Electromagnet) {
    if (tool.gpioPin < 0 || !validPinOrUnused(tool.gpioPin)) {
      return "active_tool_missing_gpio_pin";
    }
    return "";
  }
  return "";
}

String validationErrorForEncoders(
    const EncoderBusConfig& bus,
    EncoderConfig configs[kJointCount],
    const EncoderPolicy& policy,
    JointConfig jointConfigs[kJointCount],
    const ToolConfig& tool) {
  if (!bus.enabled) {
    if (policy.correctionEnabled) {
      return "encoder_correction_requires_bus";
    }
    return "";
  }
  if (!validPinOrUnused(bus.sckPin) || !validPinOrUnused(bus.misoPin) || !validPinOrUnused(bus.mosiPin) ||
      bus.sckPin < 0 || bus.misoPin < 0 || bus.mosiPin < 0) {
    return "encoder_bus_invalid_pins";
  }
  if (bus.clockHz == 0 || bus.sampleIntervalMs == 0) {
    return "encoder_bus_invalid_timing";
  }

  int enabledCount = 0;
  for (int i = 0; i < kJointCount; i++) {
    const EncoderConfig& encoder = configs[i];
    if (!encoder.enabled) {
      continue;
    }
    enabledCount++;
    if (i != 1 || encoder.jointIndex != 1) {
      return "only_shoulder_encoder_supported";
    }
    if (!validPinOrUnused(encoder.csPin) || encoder.csPin < 0) {
      return "shoulder_encoder_invalid_cs";
    }
    if (encoder.directionSign != -1 && encoder.directionSign != 1) {
      return "shoulder_encoder_invalid_sign";
    }
    if (encoder.wrapPeriodDeg <= 0.0f || encoder.sensorTurnsPerJointTurn <= 0.0f ||
        encoder.freshnessTimeoutMs == 0 || encoder.maxNoiseDeg < 0.0f) {
      return "shoulder_encoder_invalid_calibration";
    }
    if (encoder.referenceJointDeg < jointConfigs[i].minDeg || encoder.referenceJointDeg > jointConfigs[i].maxDeg) {
      return "shoulder_encoder_reference_out_of_range";
    }
  }
  if (enabledCount > 1) {
    return "only_one_shoulder_encoder_supported";
  }
  if (policy.warningToleranceDeg <= 0.0f || policy.faultToleranceDeg <= policy.warningToleranceDeg ||
      policy.requiredStableSamples < 1) {
    return "encoder_policy_invalid_thresholds";
  }
  if (policy.correctionEnabled) {
    const EncoderConfig& shoulder = configs[1];
    if (!shoulder.enabled || !shoulder.calibrationValidated || strcmp(shoulder.mounting, "joint_output") != 0 ||
        jointConfigs[1].actuator != ActuatorType::Stepper || !jointConfigs[1].enabled ||
        strcmp(policy.validationId, "none") == 0 || policy.maxCorrectionDeltaDeg <= 0.0f ||
        policy.correctionJointLimitMarginDeg < 0.0f ||
        policy.correctionSpeedDegS <= 0.0f || policy.correctionAccelDegS2 <= 0.0f ||
        policy.maxCorrectionAttempts < 1) {
      return "encoder_correction_prerequisites_missing";
    }
  }

  int pins[24] = {};
  String labels[24];
  int pinCount = 0;
  auto claimPin = [&](int pin, const String& label) -> String {
    if (pin < 0) {
      return "";
    }
    for (int index = 0; index < pinCount; index++) {
      if (pins[index] == pin) {
        return "gpio_" + String(pin) + "_conflict_" + labels[index] + "_" + label;
      }
    }
    if (pinCount < 24) {
      pins[pinCount] = pin;
      labels[pinCount] = label;
      pinCount++;
    }
    return "";
  };

  for (int i = 0; i < kJointCount; i++) {
    if (!jointConfigs[i].enabled) {
      continue;
    }
    String error;
    if (jointConfigs[i].actuator == ActuatorType::Stepper) {
      error = claimPin(jointConfigs[i].stepper.stepPin, String(jointConfigs[i].name) + "_step");
      if (error.length() == 0) {
        error = claimPin(jointConfigs[i].stepper.dirPin, String(jointConfigs[i].name) + "_dir");
      }
      if (error.length() == 0) {
        error = claimPin(jointConfigs[i].stepper.enablePin, String(jointConfigs[i].name) + "_enable");
      }
    } else if (jointConfigs[i].actuator == ActuatorType::Servo) {
      error = claimPin(jointConfigs[i].servo.pwmPin, String(jointConfigs[i].name) + "_pwm");
    }
    if (error.length() > 0) {
      return error;
    }
  }
  String error = claimPin(bus.sckPin, "encoder_sck");
  if (error.length() == 0) {
    error = claimPin(bus.misoPin, "encoder_miso");
  }
  if (error.length() == 0) {
    error = claimPin(bus.mosiPin, "encoder_mosi");
  }
  if (error.length() == 0 && configs[1].enabled) {
    error = claimPin(configs[1].csPin, "shoulder_encoder_cs");
  }
  if (error.length() == 0 && tool.type == ToolType::ServoGripper) {
    error = claimPin(tool.pwmPin, "tool_pwm");
  }
  if (error.length() == 0 && tool.type == ToolType::Electromagnet) {
    error = claimPin(tool.gpioPin, "tool_gpio");
  }
  return error;
}

String classifyConfig(JointConfig config[kJointCount]) {
  for (int i = 0; i < kJointCount; i++) {
    config[i].axisState = AxisState::Simulated;
    const String error = validationErrorForJoint(config[i], i);
    if (error.length() > 0) {
      config[i].axisState = AxisState::Invalid;
      return error;
    }
    if (config[i].enabled) {
      config[i].axisState = AxisState::Hardware;
    }
  }
  return "";
}

String hardwareMode() {
  int hardwareCount = 0;
  for (int i = 0; i < kJointCount; i++) {
    if (joints[i].axisState == AxisState::Invalid) {
      return "invalid";
    }
    if (joints[i].axisState == AxisState::Hardware) {
      hardwareCount++;
    }
  }
  if (hardwareCount == kJointCount) {
    return "hardware";
  }
  if (hardwareCount > 0) {
    return "mixed";
  }
  return "simulated";
}

String enabledBits() {
  String bits;
  for (int i = 0; i < kJointCount; i++) {
    bits += joints[i].axisState == AxisState::Hardware ? "1" : "0";
  }
  return bits;
}

String encoderBits() {
  String bits;
  for (int i = 0; i < kJointCount; i++) {
    bits += encoderAvailable[i] ? "1" : "0";
  }
  return bits;
}

String encoderValidBits() {
  String bits;
  const uint32_t nowMs = millis();
  for (int i = 0; i < kJointCount; i++) {
    const bool fresh =
        encoderRuntime[i].valid &&
        encoderRuntime[i].lastValidMs > 0 &&
        nowMs - encoderRuntime[i].lastValidMs <= encoderConfigs[i].freshnessTimeoutMs;
    bits += fresh ? "1" : "0";
  }
  return bits;
}

const char* poseSourceName() {
  return poseSourceText;
}

void markOpenLoopEstimate() {
  if (knownPose) {
    strlcpy(poseSourceText, "open_loop_estimate", sizeof(poseSourceText));
  }
}

bool hasKnownPoseAuthority() {
  return knownPose;
}

const char* closedLoopModeName() {
  if (!encoderBus.enabled) {
    return "off";
  }
  if (encoderPolicy.correctionEnabled) {
    return "bounded_correction";
  }
  return encoderPolicy.mode;
}

bool configHasInvalidAxis() {
  for (int i = 0; i < kJointCount; i++) {
    if (joints[i].axisState == AxisState::Invalid) {
      return true;
    }
  }
  return false;
}

float stepperStepsPerDegree(int index) {
  const StepperConfig& stepper = joints[index].stepper;
  return (static_cast<float>(stepper.fullStepsPerRev) * static_cast<float>(stepper.microsteps) * stepper.gearRatio) /
         360.0f;
}

long jointDegToSteps(int index, float jointDeg) {
  const float physicalJointDeg = jointDeg + correctionBiasDeg[index];
  const float signedDeg =
      (physicalJointDeg + joints[index].zeroOffsetDeg) * static_cast<float>(joints[index].directionSign);
  return lroundf(signedDeg * stepperStepsPerDegree(index));
}

float stepsToJointDeg(int index, long steps) {
  const float scale = stepperStepsPerDegree(index);
  if (scale <= 0.0f) {
    return currentJointsDeg[index];
  }
  const float signedDeg = static_cast<float>(steps) / scale;
  return signedDeg / static_cast<float>(joints[index].directionSign) -
         joints[index].zeroOffsetDeg - correctionBiasDeg[index];
}

int servoPulseForJoint(int index, float jointDeg) {
  const ServoConfig& servo = joints[index].servo;
  const float servoDeg =
      servo.neutralDeg + static_cast<float>(joints[index].directionSign) * (jointDeg + joints[index].zeroOffsetDeg) *
                             servo.gearRatio;
  const float clampedDeg = clampFloat(servoDeg, 0.0f, servo.rangeDeg);
  const float fraction = clampedDeg / servo.rangeDeg;
  return static_cast<int>(roundf(static_cast<float>(servo.pulseMinUs) +
                                 fraction * static_cast<float>(servo.pulseMaxUs - servo.pulseMinUs)));
}

uint32_t servoDutyForPulse(int index, int pulseUs) {
  const uint32_t frequency = static_cast<uint32_t>(max(1, joints[index].servo.pwmFrequencyHz));
  const float periodUs = 1000000.0f / static_cast<float>(frequency);
  const float duty = clampFloat(static_cast<float>(pulseUs) / periodUs, 0.0f, 1.0f);
  return static_cast<uint32_t>(roundf(duty * static_cast<float>(kServoPwmMaxDuty)));
}

void writeServoPwm(int index, bool enabled) {
  ServoRuntime& runtime = servoRuntime[index];
  if (!runtime.attached || runtime.channel < 0) {
    return;
  }
  ledcWrite(runtime.channel, enabled ? servoDutyForPulse(index, runtime.pulseUs) : 0);
}

void writeStepperEnable(int index, bool enabled) {
  const StepperConfig& stepper = joints[index].stepper;
  if (stepper.enablePin < 0) {
    return;
  }
  const bool activeLevel = stepper.enableActiveLow ? LOW : HIGH;
  digitalWrite(stepper.enablePin, enabled ? activeLevel : !activeLevel);
}

int toolPulseForValue(float value) {
  const float clamped = clampFloat(value, 0.0f, 1.0f);
  return static_cast<int>(roundf(static_cast<float>(activeTool.pulseMinUs) +
                                 clamped * static_cast<float>(activeTool.pulseMaxUs - activeTool.pulseMinUs)));
}

uint32_t toolDutyForPulse(int pulseUs) {
  const uint32_t frequency = static_cast<uint32_t>(max(1, activeTool.pwmFrequencyHz));
  const float periodUs = 1000000.0f / static_cast<float>(frequency);
  const float duty = clampFloat(static_cast<float>(pulseUs) / periodUs, 0.0f, 1.0f);
  return static_cast<uint32_t>(roundf(duty * static_cast<float>(kServoPwmMaxDuty)));
}

void writeToolOutput() {
  if (activeTool.type == ToolType::ServoGripper && toolRuntime.attached) {
    ledcWrite(toolRuntime.channel, toolDutyForPulse(toolPulseForValue(toolValue)));
  } else if (activeTool.type == ToolType::Electromagnet && activeTool.gpioPin >= 0) {
    const bool on = toolValue >= 0.5f;
    const int activeLevel = activeTool.activeHigh ? HIGH : LOW;
    digitalWrite(activeTool.gpioPin, on ? activeLevel : !activeLevel);
  }
}

void setToolSafe() {
  if (activeTool.type == ToolType::ServoGripper) {
    toolValue = clampFloat(activeTool.openValue, 0.0f, 1.0f);
    strlcpy(toolState, "open", sizeof(toolState));
    writeToolOutput();
  } else if (activeTool.type == ToolType::Electromagnet) {
    toolValue = 0.0f;
    strlcpy(toolState, "off", sizeof(toolState));
    writeToolOutput();
  } else {
    toolValue = 0.0f;
    strlcpy(toolState, "unknown", sizeof(toolState));
  }
}

void releaseToolOutputs() {
  if (toolRuntime.attached && toolRuntime.attachedPwmPin >= 0) {
    ledcWrite(toolRuntime.channel, 0);
    ledcDetachPin(toolRuntime.attachedPwmPin);
  }
  if (toolRuntime.gpioPin >= 0) {
    const int activeLevel = toolRuntime.gpioActiveHigh ? HIGH : LOW;
    digitalWrite(toolRuntime.gpioPin, !activeLevel);
  }
  toolRuntime.attached = false;
  toolRuntime.attachedPwmPin = -1;
  toolRuntime.gpioPin = -1;
  toolRuntime.gpioActiveHigh = true;
}

void configureToolPins() {
  releaseToolOutputs();
  if (activeTool.type == ToolType::ServoGripper && activeTool.pwmPin >= 0) {
    toolRuntime.channel = kServoPwmChannelBase + kJointCount + 1;
    ledcSetup(toolRuntime.channel, activeTool.pwmFrequencyHz, kServoPwmResolutionBits);
    ledcAttachPin(activeTool.pwmPin, toolRuntime.channel);
    toolRuntime.attached = true;
    toolRuntime.attachedPwmPin = activeTool.pwmPin;
  } else if (activeTool.type == ToolType::Electromagnet && activeTool.gpioPin >= 0) {
    pinMode(activeTool.gpioPin, OUTPUT);
    toolRuntime.gpioPin = activeTool.gpioPin;
    toolRuntime.gpioActiveHigh = activeTool.activeHigh;
  }
  setToolSafe();
}

void disableHardwareOutputs() {
  startupAlignmentHold = false;
  for (int i = 0; i < kJointCount; i++) {
    if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Stepper) {
      writeStepperEnable(i, false);
    }
    if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Servo &&
        joints[i].servo.pwmPin >= 0) {
      writeServoPwm(i, false);
    }
  }
  setToolSafe();
}

void configurePins() {
  for (int i = 0; i < kJointCount; i++) {
    if (joints[i].axisState != AxisState::Hardware) {
      continue;
    }
    if (joints[i].actuator == ActuatorType::Stepper) {
      const StepperConfig& stepper = joints[i].stepper;
      pinMode(stepper.stepPin, OUTPUT);
      pinMode(stepper.dirPin, OUTPUT);
      digitalWrite(stepper.stepPin, LOW);
      digitalWrite(stepper.dirPin, LOW);
      if (stepper.enablePin >= 0) {
        pinMode(stepper.enablePin, OUTPUT);
        writeStepperEnable(i, false);
      }
    } else if (joints[i].actuator == ActuatorType::Servo) {
      servoRuntime[i].channel = kServoPwmChannelBase + i;
      ledcSetup(servoRuntime[i].channel, joints[i].servo.pwmFrequencyHz, kServoPwmResolutionBits);
      ledcAttachPin(joints[i].servo.pwmPin, servoRuntime[i].channel);
      servoRuntime[i].attached = true;
      writeServoPwm(i, false);
    }
  }
}

void syncRuntimeFromCurrentPose() {
  const unsigned long nowUs = micros();
  for (int i = 0; i < kJointCount; i++) {
    targetJointsDeg[i] = currentJointsDeg[i];
    if (joints[i].actuator == ActuatorType::Stepper) {
      stepperRuntime[i].currentSteps = jointDegToSteps(i, currentJointsDeg[i]);
      stepperRuntime[i].targetSteps = stepperRuntime[i].currentSteps;
    } else if (joints[i].actuator == ActuatorType::Servo) {
      servoRuntime[i].velocityDegS = 0.0f;
      servoRuntime[i].lastUpdateUs = nowUs;
      servoRuntime[i].pulseUs = servoPulseForJoint(i, currentJointsDeg[i]);
      writeServoPwm(i, armed && joints[i].axisState == AxisState::Hardware);
    }
  }
}

bool poseMappingChanged(const JointConfig& previous, const JointConfig& next) {
  if (previous.actuator != next.actuator || previous.zeroOffsetDeg != next.zeroOffsetDeg ||
      previous.directionSign != next.directionSign || previous.homeDeg != next.homeDeg) {
    return true;
  }
  if (next.actuator == ActuatorType::Stepper) {
    return previous.stepper.fullStepsPerRev != next.stepper.fullStepsPerRev ||
           previous.stepper.microsteps != next.stepper.microsteps ||
           previous.stepper.gearRatio != next.stepper.gearRatio;
  }
  if (next.actuator == ActuatorType::Servo) {
    return previous.servo.pulseMinUs != next.servo.pulseMinUs ||
           previous.servo.pulseMaxUs != next.servo.pulseMaxUs ||
           previous.servo.rangeDeg != next.servo.rangeDeg ||
           previous.servo.neutralDeg != next.servo.neutralDeg ||
           previous.servo.gearRatio != next.servo.gearRatio;
  }
  return false;
}

uint16_t encoderTransfer16(int csPin, uint16_t value) {
  SPISettings encoderSpiSettings(encoderBus.clockHz, MSBFIRST, SPI_MODE1);
  SPI.beginTransaction(encoderSpiSettings);
  digitalWrite(csPin, LOW);
  delayMicroseconds(1);
  const uint16_t response = SPI.transfer16(value);
  delayMicroseconds(1);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  delayMicroseconds(1);
  return response;
}

uint16_t withEvenParity(uint16_t value) {
  value &= 0x7FFF;
  uint16_t bits = value;
  bool odd = false;
  while (bits != 0) {
    odd = !odd;
    bits &= bits - 1;
  }
  return odd ? value | 0x8000 : value;
}

bool hasEvenParity(uint16_t value) {
  bool odd = false;
  while (value != 0) {
    odd = !odd;
    value &= value - 1;
  }
  return !odd;
}

uint16_t readCommand(uint16_t address) {
  return withEvenParity(0x4000 | (address & 0x3FFF));
}

bool readAs5048Register(int csPin, uint16_t address, uint16_t& data, bool& frameError) {
  encoderTransfer16(csPin, readCommand(address));
  const uint16_t response = encoderTransfer16(csPin, 0x0000);
  frameError = (response & kAs5048ErrorFlag) != 0;
  const bool parityOk = hasEvenParity(response);
  if (frameError) {
    encoderTransfer16(csPin, kAs5048ClearErrorCommand);
    encoderTransfer16(csPin, 0x0000);
  }
  data = response & kAs5048AngleMask;
  return parityOk && !frameError;
}

float wrappedEncoderDelta(float valueDeg, float referenceDeg, float periodDeg) {
  const float period = max(0.001f, periodDeg);
  const float half = period * 0.5f;
  float delta = fmodf(valueDeg - referenceDeg + half, period);
  if (delta < 0.0f) {
    delta += period;
  }
  return delta - half;
}

float calibratedEncoderJointDeg(const EncoderConfig& config, float rawAngleDeg) {
  const float turns = max(0.000001f, config.sensorTurnsPerJointTurn);
  return config.referenceJointDeg +
         static_cast<float>(config.directionSign) *
             wrappedEncoderDelta(rawAngleDeg, config.referenceRawDeg, config.wrapPeriodDeg) / turns;
}

void configureEncoderPins() {
  if (encoderSpiStarted) {
    SPI.end();
    encoderSpiStarted = false;
  }
  for (int i = 0; i < kJointCount; i++) {
    encoderRuntime[i] = EncoderRuntime();
    encoderAvailable[i] = false;
  }
  if (!encoderBus.enabled) {
    return;
  }
  SPI.begin(encoderBus.sckPin, encoderBus.misoPin, encoderBus.mosiPin);
  encoderSpiStarted = true;
  for (int i = 0; i < kJointCount; i++) {
    if (encoderConfigs[i].enabled && encoderConfigs[i].csPin >= 0) {
      pinMode(encoderConfigs[i].csPin, OUTPUT);
      digitalWrite(encoderConfigs[i].csPin, HIGH);
    }
  }
}

void updateEncoderReadback() {
  if (!encoderBus.enabled || !encoderSpiStarted) {
    for (int i = 0; i < kJointCount; i++) {
      encoderAvailable[i] = false;
      encoderRuntime[i].available = false;
      encoderRuntime[i].valid = false;
    }
    return;
  }
  const uint32_t nowMs = millis();
  for (int i = 0; i < kJointCount; i++) {
    const EncoderConfig& config = encoderConfigs[i];
    EncoderRuntime& runtime = encoderRuntime[i];
    if (!config.enabled || config.csPin < 0) {
      encoderAvailable[i] = false;
      runtime.available = false;
      runtime.valid = false;
      continue;
    }
    if (runtime.lastAttemptMs > 0 && nowMs - runtime.lastAttemptMs < encoderBus.sampleIntervalMs) {
      continue;
    }
    runtime.lastAttemptMs = nowMs;

    uint16_t raw = 0;
    uint16_t diagnostics = 0;
    bool angleFrameError = false;
    bool diagnosticFrameError = false;
    const bool angleOk = readAs5048Register(config.csPin, kAs5048AngleAddress, raw, angleFrameError);
    const bool diagnosticsOk =
        readAs5048Register(config.csPin, kAs5048DiagnosticsAddress, diagnostics, diagnosticFrameError);
    runtime.available = angleOk || diagnosticsOk || angleFrameError || diagnosticFrameError;
    encoderAvailable[i] = runtime.available;
    runtime.diagnostics = diagnostics;

    const bool ocf = (diagnostics & (1U << 8)) != 0;
    const bool cof = (diagnostics & (1U << 9)) != 0;
    const bool magnetLow = (diagnostics & (1U << 10)) != 0;
    const bool magnetHigh = (diagnostics & (1U << 11)) != 0;
    runtime.valid = angleOk && diagnosticsOk && ocf && !cof && !magnetLow && !magnetHigh;
    strlcpy(runtime.flags, "none", sizeof(runtime.flags));
    if (!angleOk || !diagnosticsOk) {
      strlcpy(runtime.flags, "spi_or_parity", sizeof(runtime.flags));
    } else if (!ocf) {
      strlcpy(runtime.flags, "offset_pending", sizeof(runtime.flags));
    } else if (cof) {
      strlcpy(runtime.flags, "cordic_overflow", sizeof(runtime.flags));
    } else if (magnetLow) {
      strlcpy(runtime.flags, "magnet_low", sizeof(runtime.flags));
    } else if (magnetHigh) {
      strlcpy(runtime.flags, "magnet_high", sizeof(runtime.flags));
    }
    if (!runtime.valid) {
      runtime.consecutiveValidSamples = 0;
      continue;
    }

    runtime.consecutiveValidSamples = min(runtime.consecutiveValidSamples + 1, 32767);
    runtime.rawCount = raw;
    runtime.rawAngleDeg = static_cast<float>(raw) * 360.0f / 16384.0f;
    runtime.measuredJointDeg = calibratedEncoderJointDeg(config, runtime.rawAngleDeg);
    encoderAnglesDeg[i] = runtime.measuredJointDeg;
    if (controllerState == ControllerState::Moving) {
      runtime.hasPrevious = false;
      runtime.noiseDeg = 0.0f;
    } else if (runtime.hasPrevious) {
      const float sampleDeltaJointDeg =
          fabsf(wrappedEncoderDelta(runtime.rawAngleDeg, runtime.previousRawAngleDeg, 360.0f)) /
          max(0.000001f, config.sensorTurnsPerJointTurn);
      runtime.noiseDeg = runtime.noiseDeg * 0.8f + sampleDeltaJointDeg * 0.2f;
    }
    runtime.previousRawAngleDeg = runtime.rawAngleDeg;
    runtime.hasPrevious = true;
    runtime.lastValidMs = nowMs;
  }
}

bool validateJointLimits(const float requested[kJointCount]) {
  for (int i = 0; i < kJointCount; i++) {
    if (requested[i] < joints[i].minDeg || requested[i] > joints[i].maxDeg) {
      ARM_SERIAL.printf("ERR code=LIMIT message=j%d_out_of_range\r\n", i + 1);
      return false;
    }
  }
  return true;
}

void finalizeCorrection(const char* stateText) {
  const bool wasStartupAlignment = startupAlignmentActive;
  if (correctionJointIndex >= 0 && correctionJointIndex < kJointCount) {
    const float scale = stepperStepsPerDegree(correctionJointIndex);
    if (scale > 0.0f) {
      const long emittedSteps = stepperRuntime[correctionJointIndex].currentSteps - correctionStartSteps;
      lastCorrectionEmittedSteps = emittedSteps;
      const float physicalDelta =
          static_cast<float>(emittedSteps) / scale /
          static_cast<float>(joints[correctionJointIndex].directionSign);
      correctionBiasDeg[correctionJointIndex] += physicalDelta;
    }
    stepperRuntime[correctionJointIndex].targetSteps = stepperRuntime[correctionJointIndex].currentSteps;
    if (wasStartupAlignment && !startupAlignmentHold && !armed) {
      writeStepperEnable(correctionJointIndex, false);
    }
  }
  correctionActive = false;
  correctionJointIndex = -1;
  pendingCorrectionBiasDeltaDeg = 0.0f;
  correctionStartUs = 0;
  startupAlignmentActive = false;
  strlcpy(correctionState, stateText, sizeof(correctionState));
}

void clearCorrectionBias(bool preserveStartupHold = false) {
  const bool keepStartupHold = preserveStartupHold && startupAlignmentHold && !armed;
  if (startupAlignmentHold && !keepStartupHold && !armed) {
    writeStepperEnable(1, false);
  }
  correctionActive = false;
  correctionJointIndex = -1;
  pendingCorrectionBiasDeltaDeg = 0.0f;
  correctionStartSteps = 0;
  lastCorrectionEmittedSteps = 0;
  lastCorrectionRequestedDeltaDeg = 0.0f;
  correctionStartUs = 0;
  correctionAttempts = 0;
  startupAlignmentActive = false;
  startupAlignmentHold = keepStartupHold;
  for (int i = 0; i < kJointCount; i++) {
    correctionBiasDeg[i] = 0.0f;
  }
  strlcpy(correctionState, "idle", sizeof(correctionState));
  strlcpy(correctionTransactionId, "none", sizeof(correctionTransactionId));
}

bool hasHardwareMotionPending() {
  if (trajectoryRuntime.active || positionStreamRuntime.active) {
    return true;
  }
  for (int i = 0; i < kJointCount; i++) {
    if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Stepper &&
        stepperRuntime[i].currentSteps != stepperRuntime[i].targetSteps) {
      return true;
    }
    if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Servo &&
        (fabsf(targetJointsDeg[i] - currentJointsDeg[i]) > 0.05f || fabsf(servoRuntime[i].velocityDegS) > 0.05f)) {
      return true;
    }
  }
  return false;
}

void setTargetPose(const float requested[kJointCount]) {
  const unsigned long nowUs = micros();
  for (int i = 0; i < kJointCount; i++) {
    targetJointsDeg[i] = requested[i];
    if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Stepper) {
      stepperRuntime[i].targetSteps = jointDegToSteps(i, requested[i]);
    } else if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Servo) {
      servoRuntime[i].lastUpdateUs = nowUs;
    } else {
      currentJointsDeg[i] = requested[i];
      if (joints[i].actuator == ActuatorType::Servo) {
        servoRuntime[i].velocityDegS = 0.0f;
        servoRuntime[i].lastUpdateUs = nowUs;
        servoRuntime[i].pulseUs = servoPulseForJoint(i, requested[i]);
        writeServoPwm(i, armed && joints[i].axisState == AxisState::Hardware);
      }
    }
  }
  markOpenLoopEstimate();
  controllerState = hasHardwareMotionPending() ? ControllerState::Moving : ControllerState::Idle;
  clearFaultText();
}

void setTrajectoryTargetPose(const float requested[kJointCount]) {
  for (int i = 0; i < kJointCount; i++) {
    targetJointsDeg[i] = requested[i];
    if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Stepper) {
      stepperRuntime[i].targetSteps = jointDegToSteps(i, requested[i]);
    } else if (joints[i].axisState != AxisState::Hardware) {
      currentJointsDeg[i] = requested[i];
      if (joints[i].actuator == ActuatorType::Servo) {
        servoRuntime[i].velocityDegS = 0.0f;
        servoRuntime[i].pulseUs = servoPulseForJoint(i, requested[i]);
      }
    }
  }
  controllerState = ControllerState::Moving;
  clearFaultText();
}

float trajectoryDerivative(int pointIndex, int jointIndex) {
  const int count = trajectoryRuntime.count;
  if (count < 2) {
    return 0.0f;
  }
  if (pointIndex <= 0) {
    const float dt = max(0.001f, trajectoryRuntime.points[1].timeS - trajectoryRuntime.points[0].timeS);
    return (trajectoryRuntime.points[1].jointsDeg[jointIndex] - trajectoryRuntime.points[0].jointsDeg[jointIndex]) / dt;
  }
  if (pointIndex >= count - 1) {
    const float dt = max(0.001f, trajectoryRuntime.points[count - 1].timeS - trajectoryRuntime.points[count - 2].timeS);
    return (trajectoryRuntime.points[count - 1].jointsDeg[jointIndex] -
            trajectoryRuntime.points[count - 2].jointsDeg[jointIndex]) /
           dt;
  }
  const float dt =
      max(0.001f, trajectoryRuntime.points[pointIndex + 1].timeS - trajectoryRuntime.points[pointIndex - 1].timeS);
  return (trajectoryRuntime.points[pointIndex + 1].jointsDeg[jointIndex] -
          trajectoryRuntime.points[pointIndex - 1].jointsDeg[jointIndex]) /
         dt;
}

void interpolateTrajectory(float elapsedS, float out[kJointCount]) {
  const int count = trajectoryRuntime.count;
  if (count <= 0) {
    for (int i = 0; i < kJointCount; i++) {
      out[i] = currentJointsDeg[i];
    }
    return;
  }
  if (elapsedS <= trajectoryRuntime.points[0].timeS || count == 1) {
    for (int i = 0; i < kJointCount; i++) {
      out[i] = trajectoryRuntime.points[0].jointsDeg[i];
    }
    return;
  }
  if (elapsedS >= trajectoryRuntime.points[count - 1].timeS) {
    for (int i = 0; i < kJointCount; i++) {
      out[i] = trajectoryRuntime.points[count - 1].jointsDeg[i];
    }
    return;
  }

  int segment = 0;
  while (segment < count - 2 && elapsedS > trajectoryRuntime.points[segment + 1].timeS) {
    segment++;
  }
  const TrajectoryPoint& p0 = trajectoryRuntime.points[segment];
  const TrajectoryPoint& p1 = trajectoryRuntime.points[segment + 1];
  const float dt = max(0.001f, p1.timeS - p0.timeS);
  const float u = clampFloat((elapsedS - p0.timeS) / dt, 0.0f, 1.0f);
  const float u2 = u * u;
  const float u3 = u2 * u;
  const float h00 = 2.0f * u3 - 3.0f * u2 + 1.0f;
  const float h10 = u3 - 2.0f * u2 + u;
  const float h01 = -2.0f * u3 + 3.0f * u2;
  const float h11 = u3 - u2;

  for (int joint = 0; joint < kJointCount; joint++) {
    const float m0 = trajectoryDerivative(segment, joint);
    const float m1 = trajectoryDerivative(segment + 1, joint);
    const float raw = h00 * p0.jointsDeg[joint] + h10 * dt * m0 + h01 * p1.jointsDeg[joint] + h11 * dt * m1;
    const float low = min(p0.jointsDeg[joint], p1.jointsDeg[joint]);
    const float high = max(p0.jointsDeg[joint], p1.jointsDeg[joint]);
    out[joint] = clampFloat(raw, low, high);
  }
}

void updateTrajectoryFollower(unsigned long nowUs) {
  if (!trajectoryRuntime.active) {
    return;
  }
  if (!armed || controllerState == ControllerState::Estop || controllerState == ControllerState::Stopped ||
      controllerState == ControllerState::Fault) {
    trajectoryRuntime.active = false;
    trajectoryRuntime.ready = false;
    return;
  }

  const float elapsedS = static_cast<float>(nowUs - trajectoryRuntime.startUs) / 1000000.0f;
  float requested[kJointCount] = {};
  interpolateTrajectory(elapsedS, requested);
  setTrajectoryTargetPose(requested);

  if (elapsedS >= trajectoryRuntime.durationS) {
    for (int i = 0; i < kJointCount; i++) {
      requested[i] = trajectoryRuntime.points[trajectoryRuntime.count - 1].jointsDeg[i];
    }
    trajectoryRuntime.active = false;
    trajectoryRuntime.ready = false;
    setTargetPose(requested);
  }
}

void updatePositionStream(unsigned long nowUs) {
  if (!positionStreamRuntime.active) {
    return;
  }
  if (!armed || controllerState == ControllerState::Estop || controllerState == ControllerState::Stopped ||
      controllerState == ControllerState::Fault) {
    clearJogMotion(true);
    return;
  }

  const float durationS = max(0.005f, positionStreamRuntime.durationS);
  const float elapsedS = static_cast<float>(nowUs - positionStreamRuntime.startUs) / 1000000.0f;
  const float progress = clampFloat(elapsedS / durationS, 0.0f, 1.0f);
  bool hardwarePending = false;

  for (int i = 0; i < kJointCount; i++) {
    const float desired =
        positionStreamRuntime.startDeg[i] +
        (positionStreamRuntime.targetDeg[i] - positionStreamRuntime.startDeg[i]) * progress;
    targetJointsDeg[i] = desired;
    if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Stepper) {
      stepperRuntime[i].targetSteps = jointDegToSteps(i, desired);
      hardwarePending = hardwarePending || stepperRuntime[i].currentSteps != stepperRuntime[i].targetSteps;
    } else if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Servo) {
      currentJointsDeg[i] = desired;
      if (servoRuntime[i].attached && nowUs - servoRuntime[i].lastUpdateUs >= 20000UL) {
        servoRuntime[i].lastUpdateUs = nowUs;
        servoRuntime[i].velocityDegS = 0.0f;
        servoRuntime[i].pulseUs = servoPulseForJoint(i, desired);
        writeServoPwm(i, true);
      }
    } else {
      currentJointsDeg[i] = desired;
    }
  }

  if (progress >= 1.0f) {
    positionStreamRuntime.active = false;
    for (int i = 0; i < kJointCount; i++) {
      targetJointsDeg[i] = positionStreamRuntime.targetDeg[i];
      if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Stepper) {
        stepperRuntime[i].targetSteps = jointDegToSteps(i, targetJointsDeg[i]);
        hardwarePending = hardwarePending || stepperRuntime[i].currentSteps != stepperRuntime[i].targetSteps;
      } else {
        currentJointsDeg[i] = targetJointsDeg[i];
      }
    }
  }

  controllerState = positionStreamRuntime.active || hardwarePending ? ControllerState::Moving : ControllerState::Idle;
  clearFaultText();
}

bool anyJogVelocityPending() {
  for (int i = 0; i < kJointCount; i++) {
    if (fabsf(jogTargetVelocityDegS[i]) > 0.001f || fabsf(jogCurrentVelocityDegS[i]) > 0.001f ||
        fabsf(jogStepperRemainderSteps[i]) >= 1.0f) {
      return true;
    }
  }
  return false;
}

void requestJogVelocityStop() {
  if (!jogVelocityMode) {
    clearJogMotion(true);
    return;
  }
  for (int i = 0; i < kJointCount; i++) {
    jogTargetVelocityDegS[i] = 0.0f;
  }
  jogStopRequested = true;
  jogActive = true;
  lastJogMs = millis();
  if (controllerState == ControllerState::Moving || anyJogVelocityPending()) {
    controllerState = ControllerState::Moving;
  }
}

void emitJogStepperSteps(int index, long stepsToEmit) {
  if (stepsToEmit == 0) {
    return;
  }
  StepperRuntime& runtime = stepperRuntime[index];
  const int direction = stepsToEmit > 0 ? 1 : -1;
  long remaining = stepsToEmit > 0 ? stepsToEmit : -stepsToEmit;
  digitalWrite(joints[index].stepper.dirPin, direction > 0 ? HIGH : LOW);
  while (remaining > 0) {
    digitalWrite(joints[index].stepper.stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(joints[index].stepper.stepPin, LOW);
    runtime.currentSteps += direction;
    remaining--;
  }
  currentJointsDeg[index] = stepsToJointDeg(index, runtime.currentSteps);
  targetJointsDeg[index] = currentJointsDeg[index];
}

void updateJogVelocity(unsigned long nowUs) {
  if (!jogActive || !jogVelocityMode) {
    return;
  }
  if (!armed || controllerState == ControllerState::Estop || controllerState == ControllerState::Stopped ||
      controllerState == ControllerState::Fault) {
    clearJogMotion(true);
    return;
  }
  if (jogLastUpdateUs == 0) {
    jogLastUpdateUs = nowUs;
    return;
  }

  const unsigned long elapsedUs = nowUs - jogLastUpdateUs;
  if (elapsedUs < 1000UL) {
    return;
  }
  jogLastUpdateUs = nowUs;
  const float dtS = clampFloat(static_cast<float>(elapsedUs) / 1000000.0f, 0.001f, 0.05f);
  const float accelLimit = max(0.1f, lastAccelDegS2);
  bool moving = false;

  for (int i = 0; i < kJointCount; i++) {
    const float axisSpeedLimit = max(0.1f, joints[i].maxSpeedDegS);
    jogTargetVelocityDegS[i] = clampFloat(jogTargetVelocityDegS[i], -axisSpeedLimit, axisSpeedLimit);
    const float maxVelocityStep = accelLimit * dtS;
    const float velocityDelta = jogTargetVelocityDegS[i] - jogCurrentVelocityDegS[i];
    if (velocityDelta > maxVelocityStep) {
      jogCurrentVelocityDegS[i] += maxVelocityStep;
    } else if (velocityDelta < -maxVelocityStep) {
      jogCurrentVelocityDegS[i] -= maxVelocityStep;
    } else {
      jogCurrentVelocityDegS[i] = jogTargetVelocityDegS[i];
    }

    if (fabsf(jogCurrentVelocityDegS[i]) <= 0.001f && fabsf(jogTargetVelocityDegS[i]) <= 0.001f) {
      jogCurrentVelocityDegS[i] = 0.0f;
      targetJointsDeg[i] = currentJointsDeg[i];
      continue;
    }

    float nextDeg = currentJointsDeg[i] + jogCurrentVelocityDegS[i] * dtS;
    if (nextDeg > joints[i].maxDeg) {
      nextDeg = joints[i].maxDeg;
      jogCurrentVelocityDegS[i] = 0.0f;
      jogTargetVelocityDegS[i] = 0.0f;
    } else if (nextDeg < joints[i].minDeg) {
      nextDeg = joints[i].minDeg;
      jogCurrentVelocityDegS[i] = 0.0f;
      jogTargetVelocityDegS[i] = 0.0f;
    }

    if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Stepper) {
      const float stepDelta = (nextDeg - currentJointsDeg[i]) * static_cast<float>(joints[i].directionSign) *
                              stepperStepsPerDegree(i);
      const float accumulatedSteps = jogStepperRemainderSteps[i] + stepDelta;
      long stepsToEmit = static_cast<long>(accumulatedSteps);
      stepsToEmit = max(-80L, min(80L, stepsToEmit));
      jogStepperRemainderSteps[i] = accumulatedSteps - static_cast<float>(stepsToEmit);
      emitJogStepperSteps(i, stepsToEmit);
    } else {
      currentJointsDeg[i] = nextDeg;
      targetJointsDeg[i] = nextDeg;
      if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Servo &&
          servoRuntime[i].attached && nowUs - servoRuntime[i].lastUpdateUs >= 20000UL) {
        servoRuntime[i].lastUpdateUs = nowUs;
        servoRuntime[i].pulseUs = servoPulseForJoint(i, currentJointsDeg[i]);
        writeServoPwm(i, true);
      }
    }

    if (fabsf(jogTargetVelocityDegS[i]) > 0.001f || fabsf(jogCurrentVelocityDegS[i]) > 0.001f ||
        fabsf(jogStepperRemainderSteps[i]) >= 1.0f) {
      moving = true;
    }
  }

  if (moving) {
    controllerState = ControllerState::Moving;
    clearFaultText();
  } else if (jogStopRequested) {
    clearJogMotion(true);
  } else {
    controllerState = ControllerState::Idle;
  }
}

void updateJogWatchdog(uint32_t nowMs) {
  if (!jogActive) {
    return;
  }
  if (nowMs - lastJogMs > kJogWatchdogMs) {
    requestJogVelocityStop();
  }
}

void printHello() {
  ARM_SERIAL.println("HELLO name=esp32s3-arm firmware=arm_controller protocol=4 config=1 encoder=1 alignj=1");
}

void printStatus() {
  updateEncoderReadback();
  const bool statusKnownPose = hasKnownPoseAuthority();
  const uint32_t nowMs = millis();
  const int shoulderAgeMs =
      encoderRuntime[1].lastValidMs > 0 ? static_cast<int>(nowMs - encoderRuntime[1].lastValidMs) : -1;
  ARM_SERIAL.printf(
      "STATUS state=%s homed=%d known=%d known_mask=%s pose_source=%s armed=%d hw=%s enabled=%s "
      "enc=%s enc_valid=%s e2=%.3f er2=%u ea2=%.3f em2=%.3f eage2=%d enoise2=%.4f "
      "evalidn2=%d ef2=%s "
      "j1=%.3f j2=%.3f j3=%.3f j4=%.3f closed_loop=%s correction=%s correction_id=%s "
      "correction_delta=%.6f correction_steps=%ld correction_attempts=%d "
      "cb1=%.4f cb2=%.4f cb3=%.4f cb4=%.4f align_hold=%d "
      "tool_type=%s tool=%s tool_value=%.3f fault=%s\r\n",
      stateName(), homed ? 1 : 0, statusKnownPose ? 1 : 0, statusKnownPose ? "1111" : "0000",
      poseSourceName(), armed ? 1 : 0, hardwareMode().c_str(), enabledBits().c_str(), encoderBits().c_str(),
      encoderValidBits().c_str(), encoderRuntime[1].measuredJointDeg, encoderRuntime[1].rawCount,
      encoderRuntime[1].rawAngleDeg, encoderRuntime[1].measuredJointDeg, shoulderAgeMs,
      encoderRuntime[1].noiseDeg, encoderRuntime[1].consecutiveValidSamples,
      encoderRuntime[1].flags, currentJointsDeg[0], currentJointsDeg[1],
      currentJointsDeg[2], currentJointsDeg[3], closedLoopModeName(), correctionState,
      correctionTransactionId, lastCorrectionRequestedDeltaDeg, lastCorrectionEmittedSteps, correctionAttempts,
      correctionBiasDeg[0], correctionBiasDeg[1], correctionBiasDeg[2], correctionBiasDeg[3],
      startupAlignmentHold ? 1 : 0,
      toolTypeName(activeTool.type), toolState, toolValue, faultText);
}

void printError(const char* code, const String& message) {
  ARM_SERIAL.printf("ERR code=%s message=%s\r\n", code, message.c_str());
}

void handleConfig(const String& rawCommand, const String& upperCommand) {
  if (upperCommand.startsWith("CONFIG BEGIN")) {
    if (armed) {
      printError("CONFIG", "config_requires_disarmed");
      return;
    }
    if (controllerState == ControllerState::Moving) {
      printError("CONFIG", "stop_motion_before_config");
      return;
    }
    clearTrajectory();
    clearJogMotion(true);
    const int axes = tokenInt(rawCommand, "axes", 0);
    if (axes != kJointCount) {
      printError("CONFIG", "axes_must_be_4");
      return;
    }
    resetDraftConfig();
    configInProgress = true;
    return;
  }

  if (upperCommand.startsWith("CONFIG JOINT")) {
    if (!configInProgress) {
      printError("CONFIG", "begin_required");
      return;
    }
    const int index = tokenInt(rawCommand, "index", 0) - 1;
    if (index < 0 || index >= kJointCount) {
      printError("CONFIG", "invalid_joint_index");
      return;
    }
    JointConfig joint = defaultJoint(index);
    const String name = tokenString(rawCommand, "name", kDefaultNames[index]);
    snprintf(joint.name, sizeof(joint.name), "%s", name.c_str());
    joint.actuator = parseActuator(tokenString(rawCommand, "actuator", actuatorName(joint.actuator)));
    joint.enabled = tokenInt(rawCommand, "enabled", 0) != 0;
    joint.zeroOffsetDeg = tokenFloat(rawCommand, "zero", 0.0f);
    joint.directionSign = tokenInt(rawCommand, "sign", 1) < 0 ? -1 : 1;
    joint.minDeg = tokenFloat(rawCommand, "min", joint.minDeg);
    joint.maxDeg = tokenFloat(rawCommand, "max", joint.maxDeg);
    joint.homeDeg = tokenFloat(rawCommand, "home", joint.homeDeg);
    joint.maxSpeedDegS = tokenFloat(rawCommand, "max_speed", joint.maxSpeedDegS);
    joint.maxAccelDegS2 = tokenFloat(rawCommand, "max_accel", joint.maxAccelDegS2);
    if (joint.actuator == ActuatorType::Stepper) {
      joint.stepper.stepPin = tokenInt(rawCommand, "step", -1);
      joint.stepper.dirPin = tokenInt(rawCommand, "dir", -1);
      joint.stepper.enablePin = tokenInt(rawCommand, "enable", -1);
      joint.stepper.enableActiveLow = tokenInt(rawCommand, "enable_low", 1) != 0;
      joint.stepper.fullStepsPerRev = tokenInt(rawCommand, "full_steps", 200);
      joint.stepper.microsteps = tokenInt(rawCommand, "microsteps", 16);
      joint.stepper.gearRatio = tokenFloat(rawCommand, "gear", 1.0f);
    } else if (joint.actuator == ActuatorType::Servo) {
      joint.servo.pwmPin = tokenInt(rawCommand, "pwm", -1);
      joint.servo.pulseMinUs = tokenInt(rawCommand, "min_us", 500);
      joint.servo.pulseMaxUs = tokenInt(rawCommand, "max_us", 2500);
      joint.servo.pwmFrequencyHz = tokenInt(rawCommand, "freq", 50);
      joint.servo.rangeDeg = tokenFloat(rawCommand, "servo_range", 270.0f);
      joint.servo.neutralDeg = tokenFloat(rawCommand, "neutral", 135.0f);
      joint.servo.gearRatio = tokenFloat(rawCommand, "gear", 1.0f);
    }
    joint.received = true;
    draftJoints[index] = joint;
    return;
  }

  if (upperCommand.startsWith("CONFIG ENCODER_BUS")) {
    if (!configInProgress) {
      printError("CONFIG", "begin_required");
      return;
    }
    draftEncoderBus.enabled = tokenInt(rawCommand, "enabled", 0) != 0;
    draftEncoderBus.sckPin = tokenInt(rawCommand, "sck", 12);
    draftEncoderBus.misoPin = tokenInt(rawCommand, "miso", 13);
    draftEncoderBus.mosiPin = tokenInt(rawCommand, "mosi", 14);
    draftEncoderBus.clockHz = static_cast<uint32_t>(max(0, tokenInt(rawCommand, "clock", kEncoderSpiClockHz)));
    draftEncoderBus.sampleIntervalMs =
        static_cast<uint32_t>(max(0, tokenInt(rawCommand, "sample_ms", 100)));
    return;
  }

  if (upperCommand.startsWith("CONFIG ENCODER_POLICY")) {
    if (!configInProgress) {
      printError("CONFIG", "begin_required");
      return;
    }
    snprintf(
        draftEncoderPolicy.mode,
        sizeof(draftEncoderPolicy.mode),
        "%s",
        tokenString(rawCommand, "mode", "diagnostic").c_str());
    snprintf(
        draftEncoderPolicy.verificationPolicy,
        sizeof(draftEncoderPolicy.verificationPolicy),
        "%s",
        tokenString(rawCommand, "policy", "diagnostic").c_str());
    draftEncoderPolicy.settleDelayMs =
        static_cast<uint32_t>(max(0, tokenInt(rawCommand, "settle_ms", 300)));
    draftEncoderPolicy.requiredStableSamples = tokenInt(rawCommand, "samples", 3);
    draftEncoderPolicy.warningToleranceDeg = tokenFloat(rawCommand, "warn", 2.0f);
    draftEncoderPolicy.faultToleranceDeg = tokenFloat(rawCommand, "fault", 5.0f);
    draftEncoderPolicy.hysteresisDeg = tokenFloat(rawCommand, "hysteresis", 0.25f);
    draftEncoderPolicy.requireEncoder = tokenInt(rawCommand, "require", 0) != 0;
    draftEncoderPolicy.correctionEnabled = tokenInt(rawCommand, "correction", 0) != 0;
    snprintf(
        draftEncoderPolicy.validationId,
        sizeof(draftEncoderPolicy.validationId),
        "%s",
        tokenString(rawCommand, "validation_id", "none").c_str());
    draftEncoderPolicy.maxCorrectionDeltaDeg = tokenFloat(rawCommand, "max_delta", 1.0f);
    draftEncoderPolicy.correctionJointLimitMarginDeg =
        tokenFloat(rawCommand, "limit_margin", 2.0f);
    draftEncoderPolicy.correctionSpeedDegS = tokenFloat(rawCommand, "correction_speed", 2.0f);
    draftEncoderPolicy.correctionAccelDegS2 = tokenFloat(rawCommand, "correction_accel", 10.0f);
    draftEncoderPolicy.maxCorrectionAttempts = tokenInt(rawCommand, "attempts", 2);
    return;
  }

  if (upperCommand.startsWith("CONFIG ENCODER")) {
    if (!configInProgress) {
      printError("CONFIG", "begin_required");
      return;
    }
    const int index = tokenInt(rawCommand, "joint", 0) - 1;
    if (index < 0 || index >= kJointCount) {
      printError("CONFIG", "invalid_encoder_joint");
      return;
    }
    EncoderConfig encoder = defaultEncoder(index);
    encoder.received = true;
    encoder.enabled = tokenInt(rawCommand, "enabled", 0) != 0;
    encoder.csPin = tokenInt(rawCommand, "cs", -1);
    encoder.referenceRawDeg = tokenFloat(rawCommand, "reference_raw", 0.0f);
    encoder.referenceJointDeg = tokenFloat(rawCommand, "reference_joint", 0.0f);
    encoder.directionSign = tokenInt(rawCommand, "sign", 1) < 0 ? -1 : 1;
    encoder.wrapPeriodDeg = tokenFloat(rawCommand, "wrap", 360.0f);
    encoder.sensorTurnsPerJointTurn = tokenFloat(rawCommand, "turns", 1.0f);
    encoder.freshnessTimeoutMs =
        static_cast<uint32_t>(max(0, tokenInt(rawCommand, "freshness_ms", 500)));
    encoder.maxNoiseDeg = tokenFloat(rawCommand, "max_noise", 0.5f);
    encoder.calibrationValidated = tokenInt(rawCommand, "calibrated", 0) != 0;
    snprintf(
        encoder.mounting,
        sizeof(encoder.mounting),
        "%s",
        tokenString(rawCommand, "mounting", "joint_output").c_str());
    snprintf(
        encoder.calibrationId,
        sizeof(encoder.calibrationId),
        "%s",
        tokenString(rawCommand, "calibration_id", "none").c_str());
    draftEncoderConfigs[index] = encoder;
    return;
  }

  if (upperCommand.startsWith("CONFIG TOOL")) {
    if (!configInProgress) {
      printError("CONFIG", "begin_required");
      return;
    }
    const bool active = tokenInt(rawCommand, "active", 0) != 0;
    if (!active) {
      return;
    }
    ToolConfig tool = defaultTool();
    const String name = tokenString(rawCommand, "name", "tool");
    snprintf(tool.name, sizeof(tool.name), "%s", name.c_str());
    tool.type = parseToolType(tokenString(rawCommand, "type", "generic"));
    tool.pwmPin = tokenInt(rawCommand, "pwm", -1);
    tool.pulseMinUs = tokenInt(rawCommand, "min_us", 500);
    tool.pulseMaxUs = tokenInt(rawCommand, "max_us", 2500);
    tool.pwmFrequencyHz = tokenInt(rawCommand, "freq", 50);
    tool.openValue = clampFloat(tokenFloat(rawCommand, "open", 0.0f), 0.0f, 1.0f);
    tool.closedValue = clampFloat(tokenFloat(rawCommand, "close", 1.0f), 0.0f, 1.0f);
    tool.gpioPin = tokenInt(rawCommand, "pin", -1);
    tool.activeHigh = tokenInt(rawCommand, "active_high", 1) != 0;
    tool.received = true;
    draftTool = tool;
    return;
  }

  if (upperCommand.startsWith("CONFIG END")) {
    if (!configInProgress) {
      printError("CONFIG", "begin_required");
      return;
    }
    if (armed) {
      configInProgress = false;
      printError("CONFIG", "config_requires_disarmed");
      return;
    }
    configInProgress = false;
    for (int i = 0; i < kJointCount; i++) {
      if (!draftJoints[i].received) {
        printError("CONFIG", "missing_joint_" + String(i + 1));
        return;
      }
      if (draftJoints[i].minDeg >= draftJoints[i].maxDeg) {
        printError("CONFIG", "joint_" + String(i + 1) + "_invalid_limits");
        return;
      }
      if (draftJoints[i].homeDeg < draftJoints[i].minDeg || draftJoints[i].homeDeg > draftJoints[i].maxDeg) {
        printError("CONFIG", "joint_" + String(i + 1) + "_home_out_of_range");
        return;
      }
    }
    const String error = classifyConfig(draftJoints);
    if (error.length() > 0) {
      printError("CONFIG", error);
      return;
    }
    const String toolError = validationErrorForTool(draftTool);
    if (toolError.length() > 0) {
      printError("CONFIG", toolError);
      return;
    }
    const String encoderError =
        validationErrorForEncoders(draftEncoderBus, draftEncoderConfigs, draftEncoderPolicy, draftJoints, draftTool);
    if (encoderError.length() > 0) {
      printError("CONFIG", encoderError);
      return;
    }
    bool invalidatesKnownPose = false;
    for (int i = 0; i < kJointCount; i++) {
      invalidatesKnownPose = invalidatesKnownPose || poseMappingChanged(joints[i], draftJoints[i]);
    }
    disableHardwareOutputs();
    for (int i = 0; i < kJointCount; i++) {
      joints[i] = draftJoints[i];
    }
    activeTool = draftTool;
    encoderBus = draftEncoderBus;
    encoderPolicy = draftEncoderPolicy;
    for (int i = 0; i < kJointCount; i++) {
      encoderConfigs[i] = draftEncoderConfigs[i];
    }
    clearCorrectionBias();
    configurePins();
    configureToolPins();
    configureEncoderPins();
    syncRuntimeFromCurrentPose();
    if (invalidatesKnownPose) {
      knownPose = false;
      homed = false;
      strlcpy(poseSourceText, "unknown", sizeof(poseSourceText));
    }
    clearFaultText();
    ARM_SERIAL.printf(
        "OK command=CONFIG axes=4 hw=%s enabled=%s pose_invalidated=%d\r\n",
        hardwareMode().c_str(), enabledBits().c_str(), invalidatesKnownPose ? 1 : 0);
    printStatus();
    return;
  }

  printError("CONFIG", "unknown_config_command");
}

void handleArm(const String& rawCommand) {
  const int requested = rawCommand.substring(rawCommand.indexOf(' ') + 1).toInt();
  if (requested != 0) {
    if (controllerState == ControllerState::Estop) {
      printError("ESTOP", "reset_required");
      return;
    }
    if (configHasInvalidAxis()) {
      printError("CONFIG", "hardware_config_invalid");
      return;
    }
    if (!hasKnownPoseAuthority()) {
      printError("POSE", "setpose_required_before_arming");
      return;
    }
    armed = true;
    startupAlignmentHold = false;
    for (int i = 0; i < kJointCount; i++) {
      if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Stepper) {
        writeStepperEnable(i, true);
      } else if (joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Servo) {
        writeServoPwm(i, true);
      }
    }
    if (controllerState == ControllerState::Stopped) {
      controllerState = ControllerState::Idle;
    }
    clearFaultText();
  } else {
    armed = false;
    if (correctionActive) {
      finalizeCorrection("aborted_disarm");
    }
    clearTrajectory();
    clearJogMotion(true);
    const unsigned long nowUs = micros();
    for (int i = 0; i < kJointCount; i++) {
      targetJointsDeg[i] = currentJointsDeg[i];
      if (joints[i].actuator == ActuatorType::Stepper) {
        stepperRuntime[i].targetSteps = stepperRuntime[i].currentSteps;
      } else if (joints[i].actuator == ActuatorType::Servo) {
        servoRuntime[i].velocityDegS = 0.0f;
        servoRuntime[i].lastUpdateUs = nowUs;
      }
    }
    disableHardwareOutputs();
    if (controllerState != ControllerState::Estop) {
      controllerState = ControllerState::Stopped;
      clearFaultText();
    }
  }
  ARM_SERIAL.printf("OK command=ARM armed=%d\r\n", armed ? 1 : 0);
  printStatus();
}

void handleMoveJ(const char* buffer) {
  if (controllerState == ControllerState::Estop) {
    printError("ESTOP", "reset_required");
    return;
  }
  if (!armed) {
    printError("ARM", "not_armed");
    return;
  }
  if (!hasKnownPoseAuthority()) {
    printError("POSE", "motion_requires_known_pose");
    return;
  }
  if (configHasInvalidAxis()) {
    printError("CONFIG", "hardware_config_invalid");
    return;
  }

  float requested[kJointCount] = {};
  float speed = 0.0f;
  float accel = 0.0f;
  const int parsed =
      sscanf(buffer, "%*s %f %f %f %f %f %f", &requested[0], &requested[1], &requested[2], &requested[3], &speed, &accel);
  if (parsed != 6) {
    printError("USAGE", "MOVEJ_requires_j1_j2_j3_j4_speed_accel");
    return;
  }
  if (speed <= 0.0f || accel <= 0.0f) {
    printError("LIMIT", "speed_and_accel_must_be_positive");
    return;
  }
  if (!validateJointLimits(requested)) {
    return;
  }

  lastSpeedDegS = speed;
  lastAccelDegS2 = accel;
  clearTrajectory();
  clearJogMotion(false);
  setTargetPose(requested);
  ARM_SERIAL.printf("OK command=MOVEJ hw=%s\r\n", hardwareMode().c_str());
  printStatus();
}

void handleJog(const String& rawCommand, const String& upperCommand, const char* buffer) {
  (void)rawCommand;
  if (upperCommand.startsWith("JOG STOP")) {
    if (positionStreamRuntime.active) {
      clearJogMotion(true);
    } else {
      requestJogVelocityStop();
    }
    ARM_SERIAL.println("OK command=JOG_STOP");
    printStatus();
    return;
  }
  if (controllerState == ControllerState::Estop) {
    printError("ESTOP", "reset_required");
    return;
  }
  if (!armed) {
    printError("ARM", "not_armed");
    return;
  }
  if (configHasInvalidAxis()) {
    printError("CONFIG", "hardware_config_invalid");
    return;
  }
  if (trajectoryRuntime.active || trajectoryRuntime.receiving) {
    printError("STATE", "trajectory_active");
    return;
  }

  if (upperCommand.startsWith("SERVOJ")) {
    float requested[kJointCount] = {};
    float durationS = 0.0f;
    const int parsed =
        sscanf(buffer, "%*s %f %f %f %f %f", &requested[0], &requested[1], &requested[2], &requested[3], &durationS);
    if (parsed != 5) {
      printError("USAGE", "SERVOJ_requires_j1_j2_j3_j4_duration");
      return;
    }
    if (durationS < 0.005f || durationS > 0.250f) {
      printError("LIMIT", "SERVOJ_duration_out_of_range");
      return;
    }
    if (!validateJointLimits(requested)) {
      return;
    }

    clearTrajectory();
    clearJogMotion(false);
    positionStreamRuntime.active = true;
    positionStreamRuntime.startUs = micros();
    positionStreamRuntime.durationS = durationS;
    lastSpeedDegS = 0.1f;
    for (int i = 0; i < kJointCount; i++) {
      positionStreamRuntime.startDeg[i] = currentJointsDeg[i];
      positionStreamRuntime.targetDeg[i] = requested[i];
      const float requiredSpeed = fabsf(requested[i] - currentJointsDeg[i]) / durationS;
      lastSpeedDegS = max(lastSpeedDegS, requiredSpeed);
    }
    controllerState = ControllerState::Moving;
    markOpenLoopEstimate();
    clearFaultText();
    ARM_SERIAL.printf("OK command=SERVOJ hw=%s\r\n", hardwareMode().c_str());
    return;
  }

  if (upperCommand.startsWith("JOGV")) {
    float requestedVelocity[kJointCount] = {};
    float accel = 0.0f;
    const int parsed =
        sscanf(buffer, "%*s %f %f %f %f %f", &requestedVelocity[0], &requestedVelocity[1], &requestedVelocity[2],
               &requestedVelocity[3], &accel);
    if (parsed != 5) {
      printError("USAGE", "JOGV_requires_v1_v2_v3_v4_accel");
      return;
    }
    if (accel <= 0.0f) {
      printError("LIMIT", "accel_must_be_positive");
      return;
    }

    clearTrajectory();
    lastAccelDegS2 = accel;
    lastSpeedDegS = 0.0f;
    bool moving = false;
    for (int i = 0; i < kJointCount; i++) {
      const float speedLimit = max(0.1f, joints[i].maxSpeedDegS);
      float velocity = clampFloat(requestedVelocity[i], -speedLimit, speedLimit);
      if ((currentJointsDeg[i] >= joints[i].maxDeg && velocity > 0.0f) ||
          (currentJointsDeg[i] <= joints[i].minDeg && velocity < 0.0f)) {
        velocity = 0.0f;
      }
      jogTargetVelocityDegS[i] = velocity;
      lastSpeedDegS = max(lastSpeedDegS, fabsf(velocity));
      if (fabsf(velocity) > 0.001f) {
        moving = true;
      }
    }
    jogActive = true;
    jogVelocityMode = true;
    jogStopRequested = false;
    lastJogMs = millis();
    jogLastUpdateUs = micros();
    controllerState = moving || anyJogVelocityPending() ? ControllerState::Moving : ControllerState::Idle;
    markOpenLoopEstimate();
    clearFaultText();
    ARM_SERIAL.printf("OK command=JOGV hw=%s\r\n", hardwareMode().c_str());
    return;
  }

  float requested[kJointCount] = {};
  float speed = 0.0f;
  float accel = 0.0f;
  const int parsed =
      sscanf(buffer, "%*s %f %f %f %f %f %f", &requested[0], &requested[1], &requested[2], &requested[3], &speed, &accel);
  if (parsed != 6) {
    printError("USAGE", "JOGJ_requires_j1_j2_j3_j4_speed_accel");
    return;
  }
  if (speed <= 0.0f || accel <= 0.0f) {
    printError("LIMIT", "speed_and_accel_must_be_positive");
    return;
  }
  if (!validateJointLimits(requested)) {
    return;
  }

  clearTrajectory();
  clearJogMotion(false);
  lastSpeedDegS = speed;
  lastAccelDegS2 = accel;
  jogActive = true;
  jogVelocityMode = false;
  lastJogMs = millis();
  setTargetPose(requested);
  ARM_SERIAL.printf("OK command=JOGJ hw=%s\r\n", hardwareMode().c_str());
}

void handleTrajectory(const String& rawCommand, const String& upperCommand) {
  if (upperCommand.startsWith("TRAJ CLEAR")) {
    clearTrajectory();
    clearJogMotion(false);
    ARM_SERIAL.println("OK command=TRAJ_CLEAR");
    return;
  }
  if (controllerState == ControllerState::Estop) {
    printError("ESTOP", "reset_required");
    return;
  }
  if (!armed) {
    printError("ARM", "not_armed");
    return;
  }
  if (configHasInvalidAxis()) {
    printError("CONFIG", "hardware_config_invalid");
    return;
  }

  if (upperCommand.startsWith("TRAJ BEGIN")) {
    if (controllerState == ControllerState::Moving || trajectoryRuntime.active) {
      printError("STATE", "trajectory_requires_idle");
      return;
    }
    const int count = tokenInt(rawCommand, "count", 0);
    const float durationS = tokenFloat(rawCommand, "duration", 0.0f);
    const float speed = tokenFloat(rawCommand, "speed", 0.0f);
    const float accel = tokenFloat(rawCommand, "accel", 0.0f);
    if (count < 2 || count > kMaxTrajectoryPoints) {
      printError("LIMIT", "trajectory_count_out_of_range");
      return;
    }
    if (durationS <= 0.0f || speed <= 0.0f || accel <= 0.0f) {
      printError("LIMIT", "trajectory_duration_speed_accel_must_be_positive");
      return;
    }
    clearTrajectory();
    clearJogMotion(false);
    trajectoryRuntime.expectedCount = count;
    trajectoryRuntime.durationS = durationS;
    trajectoryRuntime.receiving = true;
    lastSpeedDegS = speed;
    lastAccelDegS2 = accel;
    ARM_SERIAL.printf("OK command=TRAJ_BEGIN count=%d duration=%.3f\r\n", count, durationS);
    return;
  }

  if (upperCommand.startsWith("TRAJ POINT")) {
    if (!trajectoryRuntime.receiving || trajectoryRuntime.active) {
      printError("STATE", "trajectory_begin_required");
      return;
    }
    const int index = tokenInt(rawCommand, "index", -1);
    if (index != trajectoryRuntime.count || index < 0 || index >= trajectoryRuntime.expectedCount) {
      printError("USAGE", "trajectory_point_index_must_be_sequential");
      return;
    }
    TrajectoryPoint point;
    point.timeS = tokenFloat(rawCommand, "t", -1.0f);
    for (int i = 0; i < kJointCount; i++) {
      const String key = String("j") + String(i + 1);
      point.jointsDeg[i] = tokenFloat(rawCommand, key.c_str(), NAN);
    }
    if (!isfinite(point.timeS)) {
      printError("USAGE", "trajectory_point_requires_time");
      return;
    }
    if (index == 0 && fabsf(point.timeS) > 0.001f) {
      printError("USAGE", "trajectory_first_point_time_must_be_zero");
      return;
    }
    if (index > 0 && point.timeS <= trajectoryRuntime.points[index - 1].timeS) {
      printError("USAGE", "trajectory_point_times_must_increase");
      return;
    }
    if (index == trajectoryRuntime.expectedCount - 1 && fabsf(point.timeS - trajectoryRuntime.durationS) > 0.05f) {
      printError("USAGE", "trajectory_last_point_time_must_match_duration");
      return;
    }
    for (int i = 0; i < kJointCount; i++) {
      if (!isfinite(point.jointsDeg[i])) {
        printError("USAGE", "trajectory_point_requires_j1_j2_j3_j4");
        return;
      }
    }
    if (!validateJointLimits(point.jointsDeg)) {
      return;
    }
    trajectoryRuntime.points[index] = point;
    trajectoryRuntime.count++;
    trajectoryRuntime.ready = trajectoryRuntime.count == trajectoryRuntime.expectedCount;
    ARM_SERIAL.printf("OK command=TRAJ_POINT index=%d\r\n", index);
    return;
  }

  if (upperCommand.startsWith("TRAJ START")) {
    if (!trajectoryRuntime.ready || trajectoryRuntime.count != trajectoryRuntime.expectedCount) {
      printError("STATE", "trajectory_not_ready");
      return;
    }
    trajectoryRuntime.receiving = false;
    trajectoryRuntime.active = true;
    trajectoryRuntime.startUs = micros();
    controllerState = ControllerState::Moving;
    markOpenLoopEstimate();
    clearFaultText();
    ARM_SERIAL.printf(
        "OK command=TRAJ_START count=%d duration=%.3f\r\n", trajectoryRuntime.count, trajectoryRuntime.durationS);
    return;
  }

  printError("USAGE", "TRAJ_requires_BEGIN_POINT_START_or_CLEAR");
}

void handleHome() {
  if (controllerState == ControllerState::Estop) {
    printError("ESTOP", "reset_required");
    return;
  }
  if (!armed) {
    printError("ARM", "not_armed");
    return;
  }
  if (!hasKnownPoseAuthority()) {
    printError("POSE", "home_requires_known_pose");
    return;
  }
  float requested[kJointCount] = {};
  for (int i = 0; i < kJointCount; i++) {
    requested[i] = joints[i].homeDeg;
  }
  clearTrajectory();
  clearJogMotion(false);
  setTargetPose(requested);
  homed = false;
  ARM_SERIAL.println("OK command=HOME");
  printStatus();
}

void handleSetPose(const char* buffer) {
  if (armed) {
    printError("ARM", "setpose_requires_disarmed");
    return;
  }
  if (controllerState == ControllerState::Moving) {
    printError("STATE", "setpose_requires_stopped");
    return;
  }
  float requested[kJointCount] = {};
  const int parsed = sscanf(buffer, "%*s %f %f %f %f", &requested[0], &requested[1], &requested[2], &requested[3]);
  if (parsed != 4) {
    printError("USAGE", "SETPOSE_requires_j1_j2_j3_j4");
    return;
  }
  if (!validateJointLimits(requested)) {
    return;
  }
  clearCorrectionBias(true);
  for (int i = 0; i < kJointCount; i++) {
    currentJointsDeg[i] = requested[i];
    targetJointsDeg[i] = requested[i];
  }
  clearTrajectory();
  clearJogMotion(false);
  syncRuntimeFromCurrentPose();
  homed = false;
  knownPose = true;
  strlcpy(poseSourceText, "setpose", sizeof(poseSourceText));
  controllerState = ControllerState::Stopped;
  clearFaultText();
  ARM_SERIAL.println("OK command=SETPOSE");
  printStatus();
}

void handleStop() {
  if (correctionActive) {
    finalizeCorrection("aborted_stop");
  }
  clearTrajectory();
  clearJogMotion(true);
  const unsigned long nowUs = micros();
  for (int i = 0; i < kJointCount; i++) {
    targetJointsDeg[i] = currentJointsDeg[i];
    if (joints[i].actuator == ActuatorType::Stepper) {
      stepperRuntime[i].targetSteps = stepperRuntime[i].currentSteps;
    } else if (joints[i].actuator == ActuatorType::Servo) {
      servoRuntime[i].velocityDegS = 0.0f;
      servoRuntime[i].lastUpdateUs = nowUs;
    }
  }
  if (controllerState != ControllerState::Estop) {
    controllerState = ControllerState::Stopped;
    clearFaultText();
  }
  setToolSafe();
  ARM_SERIAL.println("OK command=STOP");
  printStatus();
}

void handleEstop() {
  armed = false;
  if (correctionActive) {
    finalizeCorrection("aborted_estop");
  }
  clearTrajectory();
  clearJogMotion(true);
  const unsigned long nowUs = micros();
  for (int i = 0; i < kJointCount; i++) {
    targetJointsDeg[i] = currentJointsDeg[i];
    if (joints[i].actuator == ActuatorType::Stepper) {
      stepperRuntime[i].targetSteps = stepperRuntime[i].currentSteps;
    } else if (joints[i].actuator == ActuatorType::Servo) {
      servoRuntime[i].velocityDegS = 0.0f;
      servoRuntime[i].lastUpdateUs = nowUs;
    }
  }
  disableHardwareOutputs();
  controllerState = ControllerState::Estop;
  strlcpy(faultText, "ESTOP", sizeof(faultText));
  ARM_SERIAL.println("OK command=ESTOP");
  printStatus();
}

void handleCorrectJ(const String& rawCommand) {
  const int jointIndex = tokenInt(rawCommand, "joint", 0) - 1;
  const float deltaDeg = tokenFloat(rawCommand, "delta", NAN);
  const float speedDegS = tokenFloat(rawCommand, "speed", 0.0f);
  const float accelDegS2 = tokenFloat(rawCommand, "accel", 0.0f);
  const String transactionId = tokenString(rawCommand, "id", "none");
  if (jointIndex != 1) {
    printError("CORRECTION", "only_shoulder_supported");
    return;
  }
  if (!armed) {
    printError("ARM", "not_armed");
    return;
  }
  if (!knownPose) {
    printError("POSE", "correction_requires_known_pose");
    return;
  }
  if (controllerState != ControllerState::Idle && controllerState != ControllerState::Stopped) {
    printError("STATE", "correction_requires_idle");
    return;
  }
  if (trajectoryRuntime.active || trajectoryRuntime.receiving || positionStreamRuntime.active || jogActive) {
    printError("STATE", "correction_requires_no_active_motion");
    return;
  }
  if (!encoderPolicy.correctionEnabled || !encoderBus.enabled || !encoderConfigs[jointIndex].enabled ||
      !encoderConfigs[jointIndex].calibrationValidated ||
      strcmp(encoderConfigs[jointIndex].mounting, "joint_output") != 0 ||
      joints[jointIndex].actuator != ActuatorType::Stepper ||
      joints[jointIndex].axisState != AxisState::Hardware) {
    printError("CORRECTION", "correction_not_validated");
    return;
  }
  const uint32_t nowMs = millis();
  const bool fresh =
      encoderRuntime[jointIndex].valid &&
      encoderRuntime[jointIndex].lastValidMs > 0 &&
      nowMs - encoderRuntime[jointIndex].lastValidMs <= encoderConfigs[jointIndex].freshnessTimeoutMs;
  if (!fresh || encoderRuntime[jointIndex].noiseDeg > encoderConfigs[jointIndex].maxNoiseDeg) {
    printError("ENCODER", "shoulder_measurement_not_fresh_stable");
    return;
  }
  if (!isfinite(deltaDeg) || fabsf(deltaDeg) <= 0.0001f ||
      fabsf(deltaDeg) > encoderPolicy.maxCorrectionDeltaDeg) {
    printError("CORRECTION", "delta_out_of_range");
    return;
  }
  if (speedDegS <= 0.0f || speedDegS > encoderPolicy.correctionSpeedDegS ||
      accelDegS2 <= 0.0f || accelDegS2 > encoderPolicy.correctionAccelDegS2) {
    printError("CORRECTION", "speed_or_accel_out_of_range");
    return;
  }
  if (transactionId.length() == 0 || transactionId == "none") {
    printError("CORRECTION", "transaction_id_required");
    return;
  }
  if (strcmp(correctionTransactionId, transactionId.c_str()) != 0) {
    correctionAttempts = 0;
  }
  if (correctionAttempts >= encoderPolicy.maxCorrectionAttempts) {
    printError("CORRECTION", "attempt_limit_reached");
    return;
  }
  const float correctedPhysicalAngle = encoderRuntime[jointIndex].measuredJointDeg + deltaDeg;
  const float limitMargin = encoderPolicy.correctionJointLimitMarginDeg;
  if (correctedPhysicalAngle < joints[jointIndex].minDeg + limitMargin ||
      correctedPhysicalAngle > joints[jointIndex].maxDeg - limitMargin) {
    printError("LIMIT", "correction_would_cross_joint_limit");
    return;
  }

  const long stepDelta = lroundf(
      deltaDeg * static_cast<float>(joints[jointIndex].directionSign) * stepperStepsPerDegree(jointIndex));
  if (stepDelta == 0) {
    printError("CORRECTION", "delta_below_one_step");
    return;
  }
  correctionActive = true;
  correctionJointIndex = jointIndex;
  correctionStartSteps = stepperRuntime[jointIndex].currentSteps;
  pendingCorrectionBiasDeltaDeg = deltaDeg;
  lastCorrectionRequestedDeltaDeg = deltaDeg;
  lastCorrectionEmittedSteps = 0;
  correctionStartUs = micros();
  stepperRuntime[jointIndex].targetSteps = correctionStartSteps + stepDelta;
  correctionAttempts++;
  lastSpeedDegS = speedDegS;
  lastAccelDegS2 = accelDegS2;
  snprintf(correctionTransactionId, sizeof(correctionTransactionId), "%s", transactionId.c_str());
  strlcpy(correctionState, "executing", sizeof(correctionState));
  controllerState = ControllerState::Moving;
  clearFaultText();
  ARM_SERIAL.printf(
      "OK command=CORRECTJ joint=2 delta=%.6f steps=%ld attempt=%d id=%s\r\n",
      deltaDeg,
      stepDelta,
      correctionAttempts,
      correctionTransactionId);
}

void handleAlignJ(const String& rawCommand) {
  const int jointIndex = tokenInt(rawCommand, "joint", 0) - 1;
  const float deltaDeg = tokenFloat(rawCommand, "delta", NAN);
  const float speedDegS = tokenFloat(rawCommand, "speed", 0.0f);
  const float accelDegS2 = tokenFloat(rawCommand, "accel", 0.0f);
  const int holdAfter = tokenInt(rawCommand, "hold", 1);
  const String transactionId = tokenString(rawCommand, "id", "none");
  if (jointIndex != 1) {
    printError("ALIGN", "only_shoulder_supported");
    return;
  }
  if (controllerState == ControllerState::Estop) {
    printError("ESTOP", "reset_required");
    return;
  }
  if (controllerState != ControllerState::Idle && controllerState != ControllerState::Stopped) {
    printError("STATE", "align_requires_idle_or_stopped");
    return;
  }
  if (trajectoryRuntime.active || trajectoryRuntime.receiving || positionStreamRuntime.active || jogActive) {
    printError("STATE", "align_requires_no_active_motion");
    return;
  }
  if (configHasInvalidAxis()) {
    printError("CONFIG", "hardware_config_invalid");
    return;
  }
  if (!encoderPolicy.correctionEnabled || !encoderBus.enabled || !encoderConfigs[jointIndex].enabled ||
      !encoderConfigs[jointIndex].calibrationValidated ||
      strcmp(encoderConfigs[jointIndex].mounting, "joint_output") != 0 ||
      joints[jointIndex].actuator != ActuatorType::Stepper ||
      joints[jointIndex].axisState != AxisState::Hardware) {
    printError("ALIGN", "align_not_validated");
    return;
  }
  const uint32_t nowMs = millis();
  const bool fresh =
      encoderRuntime[jointIndex].valid &&
      encoderRuntime[jointIndex].lastValidMs > 0 &&
      nowMs - encoderRuntime[jointIndex].lastValidMs <= encoderConfigs[jointIndex].freshnessTimeoutMs;
  if (!fresh || encoderRuntime[jointIndex].noiseDeg > encoderConfigs[jointIndex].maxNoiseDeg) {
    printError("ENCODER", "shoulder_measurement_not_fresh_stable");
    return;
  }
  if (!isfinite(deltaDeg) || fabsf(deltaDeg) <= 0.0001f ||
      fabsf(deltaDeg) > encoderPolicy.maxCorrectionDeltaDeg) {
    printError("ALIGN", "delta_out_of_range");
    return;
  }
  if (speedDegS <= 0.0f || speedDegS > joints[jointIndex].maxSpeedDegS ||
      accelDegS2 <= 0.0f || accelDegS2 > joints[jointIndex].maxAccelDegS2) {
    printError("ALIGN", "speed_or_accel_out_of_range");
    return;
  }
  if (transactionId.length() == 0 || transactionId == "none") {
    printError("ALIGN", "transaction_id_required");
    return;
  }

  const float targetMeasuredAngle = encoderRuntime[jointIndex].measuredJointDeg + deltaDeg;
  const float limitMargin = encoderPolicy.correctionJointLimitMarginDeg;
  if (targetMeasuredAngle < joints[jointIndex].minDeg + limitMargin ||
      targetMeasuredAngle > joints[jointIndex].maxDeg - limitMargin) {
    printError("LIMIT", "align_would_cross_joint_limit");
    return;
  }

  const long stepDelta = lroundf(
      deltaDeg * static_cast<float>(joints[jointIndex].directionSign) * stepperStepsPerDegree(jointIndex));
  if (stepDelta == 0) {
    printError("ALIGN", "delta_below_one_step");
    return;
  }

  clearTrajectory();
  clearJogMotion(false);
  for (int i = 0; i < kJointCount; i++) {
    if (i != jointIndex && joints[i].axisState == AxisState::Hardware && joints[i].actuator == ActuatorType::Stepper) {
      stepperRuntime[i].targetSteps = stepperRuntime[i].currentSteps;
    }
  }

  correctionActive = true;
  startupAlignmentActive = true;
  startupAlignmentHold = holdAfter != 0 && !armed;
  correctionJointIndex = jointIndex;
  correctionStartSteps = stepperRuntime[jointIndex].currentSteps;
  pendingCorrectionBiasDeltaDeg = deltaDeg;
  lastCorrectionRequestedDeltaDeg = deltaDeg;
  lastCorrectionEmittedSteps = 0;
  correctionStartUs = micros();
  stepperRuntime[jointIndex].targetSteps = correctionStartSteps + stepDelta;
  correctionAttempts = 1;
  lastSpeedDegS = speedDegS;
  lastAccelDegS2 = accelDegS2;
  snprintf(correctionTransactionId, sizeof(correctionTransactionId), "%s", transactionId.c_str());
  strlcpy(correctionState, "aligning", sizeof(correctionState));
  writeStepperEnable(jointIndex, true);
  controllerState = ControllerState::Moving;
  clearFaultText();
  ARM_SERIAL.printf(
      "OK command=ALIGNJ joint=2 delta=%.6f steps=%ld id=%s hold=%d\r\n",
      deltaDeg,
      stepDelta,
      correctionTransactionId,
      startupAlignmentHold ? 1 : 0);
}

void handleTool(const String& rawCommand, const String& upperCommand) {
  if (controllerState == ControllerState::Estop) {
    printError("ESTOP", "reset_required");
    return;
  }
  if (!armed) {
    printError("ARM", "not_armed");
    return;
  }
  if (activeTool.type == ToolType::ServoGripper && upperCommand.startsWith("TOOL OPEN")) {
    strlcpy(toolState, "open", sizeof(toolState));
    toolValue = clampFloat(activeTool.openValue, 0.0f, 1.0f);
  } else if (activeTool.type == ToolType::ServoGripper && upperCommand.startsWith("TOOL CLOSE")) {
    strlcpy(toolState, "closed", sizeof(toolState));
    toolValue = clampFloat(activeTool.closedValue, 0.0f, 1.0f);
  } else if (activeTool.type == ToolType::ServoGripper && upperCommand.startsWith("TOOL SET")) {
    toolValue = clampFloat(tokenFloat(rawCommand, "value", toolValue), 0.0f, 1.0f);
    strlcpy(toolState, "set", sizeof(toolState));
  } else if (activeTool.type == ToolType::Electromagnet && upperCommand.startsWith("TOOL ON")) {
    strlcpy(toolState, "on", sizeof(toolState));
    toolValue = 1.0f;
  } else if (activeTool.type == ToolType::Electromagnet && upperCommand.startsWith("TOOL OFF")) {
    strlcpy(toolState, "off", sizeof(toolState));
    toolValue = 0.0f;
  } else {
    printError("USAGE", "tool_command_not_supported_by_active_tool");
    return;
  }
  writeToolOutput();
  ARM_SERIAL.printf("OK command=TOOL state=%s value=%.3f\r\n", toolState, toolValue);
  printStatus();
}

unsigned long stepIntervalUs(int index, unsigned long nowUs) {
  float speed = max(1.0f, min(lastSpeedDegS, joints[index].maxSpeedDegS));
  if (correctionActive && index == correctionJointIndex) {
    const float scale = max(0.0001f, stepperStepsPerDegree(index));
    const float elapsedS = static_cast<float>(nowUs - correctionStartUs) / 1000000.0f;
    const float acceleratingSpeed = max(0.1f, lastAccelDegS2 * elapsedS);
    const float remainingDeg =
        fabsf(static_cast<float>(stepperRuntime[index].targetSteps - stepperRuntime[index].currentSteps)) /
        scale;
    const float brakingSpeed = sqrtf(max(0.0f, 2.0f * lastAccelDegS2 * remainingDeg));
    speed = max(0.1f, min(speed, min(acceleratingSpeed, brakingSpeed)));
  }
  const float stepRate = max(1.0f, speed * stepperStepsPerDegree(index));
  return static_cast<unsigned long>(1000000.0f / stepRate);
}

void updateSteppers(unsigned long nowUs) {
  if (jogActive && jogVelocityMode) {
    return;
  }
  if ((!armed && !startupAlignmentActive) || controllerState == ControllerState::Estop ||
      controllerState == ControllerState::Fault ||
      (controllerState == ControllerState::Stopped && !startupAlignmentActive)) {
    return;
  }
  for (int i = 0; i < kJointCount; i++) {
    if (joints[i].axisState != AxisState::Hardware || joints[i].actuator != ActuatorType::Stepper) {
      continue;
    }
    StepperRuntime& runtime = stepperRuntime[i];
    const long delta = runtime.targetSteps - runtime.currentSteps;
    if (delta == 0) {
      continue;
    }
    const unsigned long interval = stepIntervalUs(i, nowUs);
    if (nowUs - runtime.lastStepUs < interval) {
      continue;
    }
    runtime.lastStepUs = nowUs;
    digitalWrite(joints[i].stepper.dirPin, delta > 0 ? HIGH : LOW);
    digitalWrite(joints[i].stepper.stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(joints[i].stepper.stepPin, LOW);
    runtime.currentSteps += delta > 0 ? 1 : -1;
    if (!correctionActive || i != correctionJointIndex) {
      currentJointsDeg[i] = stepsToJointDeg(i, runtime.currentSteps);
    }
  }

  if (correctionActive && correctionJointIndex >= 0 &&
      stepperRuntime[correctionJointIndex].currentSteps == stepperRuntime[correctionJointIndex].targetSteps) {
    finalizeCorrection("completed");
  }
  if (controllerState == ControllerState::Moving && !hasHardwareMotionPending()) {
    controllerState = ControllerState::Idle;
    clearFaultText();
  }
}

void updateServoPwm(unsigned long nowUs) {
  if ((jogActive && jogVelocityMode) || positionStreamRuntime.active) {
    return;
  }
  if (!armed || controllerState == ControllerState::Estop || controllerState == ControllerState::Stopped ||
      controllerState == ControllerState::Fault) {
    for (int i = 0; i < kJointCount; i++) {
      if (joints[i].actuator == ActuatorType::Servo) {
        servoRuntime[i].velocityDegS = 0.0f;
        servoRuntime[i].lastUpdateUs = nowUs;
      }
    }
    return;
  }

  for (int i = 0; i < kJointCount; i++) {
    if (joints[i].axisState != AxisState::Hardware || joints[i].actuator != ActuatorType::Servo) {
      continue;
    }
    ServoRuntime& runtime = servoRuntime[i];
    if (!runtime.attached) {
      continue;
    }
    if (runtime.lastUpdateUs == 0) {
      runtime.lastUpdateUs = nowUs;
      continue;
    }

    const unsigned long elapsedUs = nowUs - runtime.lastUpdateUs;
    if (elapsedUs < 20000UL) {
      continue;
    }
    runtime.lastUpdateUs = nowUs;

    const float dtS = static_cast<float>(elapsedUs) / 1000000.0f;
    const float delta = targetJointsDeg[i] - currentJointsDeg[i];
    if (fabsf(delta) <= 0.05f) {
      currentJointsDeg[i] = targetJointsDeg[i];
      runtime.velocityDegS = 0.0f;
      runtime.pulseUs = servoPulseForJoint(i, currentJointsDeg[i]);
      writeServoPwm(i, true);
      continue;
    }

    const float direction = delta > 0.0f ? 1.0f : -1.0f;
    const float speedLimit = max(0.1f, min(lastSpeedDegS, joints[i].maxSpeedDegS));
    const float accelLimit = max(0.1f, min(lastAccelDegS2, joints[i].maxAccelDegS2));
    const float stoppingDistance = (runtime.velocityDegS * runtime.velocityDegS) / (2.0f * accelLimit);
    float desiredVelocity = direction * speedLimit;
    if (runtime.velocityDegS * direction > 0.0f && stoppingDistance >= fabsf(delta)) {
      desiredVelocity = 0.0f;
    }

    const float maxVelocityStep = accelLimit * dtS;
    const float velocityDelta = desiredVelocity - runtime.velocityDegS;
    if (velocityDelta > maxVelocityStep) {
      runtime.velocityDegS += maxVelocityStep;
    } else if (velocityDelta < -maxVelocityStep) {
      runtime.velocityDegS -= maxVelocityStep;
    } else {
      runtime.velocityDegS = desiredVelocity;
    }

    const float step = runtime.velocityDegS * dtS;
    if (fabsf(step) >= fabsf(delta)) {
      currentJointsDeg[i] = targetJointsDeg[i];
      runtime.velocityDegS = 0.0f;
    } else {
      currentJointsDeg[i] += step;
    }
    runtime.pulseUs = servoPulseForJoint(i, currentJointsDeg[i]);
    writeServoPwm(i, true);
  }

  if (controllerState == ControllerState::Moving && !hasHardwareMotionPending()) {
    controllerState = ControllerState::Idle;
    clearFaultText();
  }
}

void handleCommand(String rawCommand) {
  rawCommand.trim();
  if (rawCommand.length() == 0) {
    return;
  }

  String upperCommand = rawCommand;
  upperCommand.toUpperCase();

  char buffer[kMaxLineLength] = {};
  rawCommand.toCharArray(buffer, sizeof(buffer));
  char command[24] = {};
  sscanf(buffer, "%23s", command);

  if (strcasecmp(command, "HELLO") == 0) {
    printHello();
  } else if (strcasecmp(command, "STATUS") == 0) {
    printStatus();
  } else if (strcasecmp(command, "CONFIG") == 0) {
    handleConfig(rawCommand, upperCommand);
  } else if (strcasecmp(command, "ARM") == 0) {
    handleArm(rawCommand);
  } else if (strcasecmp(command, "SETPOSE") == 0) {
    handleSetPose(buffer);
  } else if (strcasecmp(command, "CORRECTJ") == 0) {
    handleCorrectJ(rawCommand);
  } else if (strcasecmp(command, "ALIGNJ") == 0) {
    handleAlignJ(rawCommand);
  } else if (strcasecmp(command, "MOVEJ") == 0) {
    handleMoveJ(buffer);
  } else if (strcasecmp(command, "JOGJ") == 0 || strcasecmp(command, "JOGV") == 0 ||
             strcasecmp(command, "SERVOJ") == 0 || strcasecmp(command, "JOG") == 0) {
    handleJog(rawCommand, upperCommand, buffer);
  } else if (strcasecmp(command, "TRAJ") == 0) {
    handleTrajectory(rawCommand, upperCommand);
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
    } else if (commandLine.length() < kMaxLineLength - 1) {
      commandLine += incoming;
    }
  }
}

void turnOffOnboardRgbLed() {
#if defined(ESP32) && defined(ARM_CONTROLLER_ENABLE_ONBOARD_RGB_OFF)
  pinMode(ESP_RGB_LED_PIN, OUTPUT);
  digitalWrite(ESP_RGB_LED_PIN, LOW);
  neopixelWrite(ESP_RGB_LED_PIN, 0, 0, 0);
#endif
}
}  // namespace

void setup() {
  turnOffOnboardRgbLed();

  for (int i = 0; i < kJointCount; i++) {
    joints[i] = defaultJoint(i);
    encoderConfigs[i] = defaultEncoder(i);
    currentJointsDeg[i] = kDefaultHome[i];
    targetJointsDeg[i] = kDefaultHome[i];
  }
  activeTool = defaultTool();
  resetDraftConfig();
  clearTrajectory();
  clearJogMotion(false);
  classifyConfig(joints);
  syncRuntimeFromCurrentPose();
  configureEncoderPins();
  clearFaultText();

  ARM_SERIAL.begin(115200);
  const unsigned long startMs = millis();
  while (!ARM_SERIAL && millis() - startMs < kSerialWaitMs) {
    delay(10);
  }

  lastStatusMs = millis();
  printHello();
  printStatus();
}

void loop() {
  readSerialCommands();
  const unsigned long nowUs = micros();
  const uint32_t nowMs = millis();
  updateJogWatchdog(nowMs);
  updateJogVelocity(nowUs);
  updateTrajectoryFollower(nowUs);
  updatePositionStream(nowUs);
  updateSteppers(nowUs);
  updateServoPwm(nowUs);
  updateEncoderReadback();

  if (nowMs - lastStatusMs >= kStatusIntervalMs) {
    lastStatusMs = nowMs;
    printStatus();
  }
}
